#include "lazy_theta_planner/lazy_theta_planner.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <algorithm>

namespace lazy_theta_planner
{

double round_to_3dp(double val) {
    return std::round(val * 1000.0) / 1000.0;
}


void LazyThetaPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock lifecycle node");
  }

  logger_ = node->get_logger();
  costmap_ros_ = costmap_ros;

  // Declare parameters safely (Nav2 utility checks if already declared in yaml)
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".los_shortcut_cost_limit", rclcpp::ParameterValue(50));

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".cost_travel_multiplier", rclcpp::ParameterValue(2.0));

  // Retrieve values
  node->get_parameter(name + ".los_shortcut_cost_limit", los_shortcut_cost_limit_);
  node->get_parameter(name + ".cost_travel_multiplier", cost_travel_multiplier_);

  los_shortcut_cost_limit_ = std::clamp(los_shortcut_cost_limit_, 1, 100);
}





void LazyThetaPlanner::activate()
{
  if (!costmap_ros_) {
    return;
  }

  auto costmap = costmap_ros_->getCostmap();
  costmap_meta_.update(costmap);

  RCLCPP_INFO_STREAM(logger_, "Plugin Activated Successfully");
}




void LazyThetaPlanner::deactivate() {}




void LazyThetaPlanner::cleanup()
{
  costmap_ros_.reset();

  RCLCPP_INFO_STREAM(logger_, "Plugin Cleaned Up Successfully");
}





nav_msgs::msg::Path LazyThetaPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  // auto start_time = std::chrono::steady_clock::now();

  if (!costmap_ros_) {
    return nav_msgs::msg::Path();
  }

  auto costmap = costmap_ros_->getCostmap();
  if (!costmap) {
    RCLCPP_ERROR(logger_, "Costmap not available");
    return nav_msgs::msg::Path();
  }

  // Lock the costmap mutex to ensure thread-safe access to raw map memory during string pulling
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  // Atomically refresh metadata struct
  costmap_meta_.update(costmap);
  const unsigned char* char_map = costmap->getCharMap();

  auto start_node = std::make_shared<GridNode>(poseToGrid(start.pose));
  auto goal_node = std::make_shared<GridNode>(poseToGrid(goal.pose));

  // Execute Search
  std::shared_ptr<GridNode> best_goal = nullptr;

  best_goal = runLazyThetaStarPlan(
    start_node, 
    goal_node, 
    cancel_checker, 
    char_map, 
    costmap_meta_.size_x
  );

  // Reconstruction
  nav_msgs::msg::Path path;
  path.header.frame_id = costmap_ros_->getGlobalFrameID();

  if (!best_goal) {
    RCLCPP_INFO_STREAM(logger_, "No Path Generated");
    return path;
  }

  auto curr_node = best_goal;
  while (curr_node) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = path.header.frame_id;
    pose_stamped.header.stamp = path.header.stamp;
    pose_stamped.pose = gridToPose(*curr_node);
    
    path.poses.push_back(pose_stamped);
    curr_node = curr_node->prev; // Traces shared_ptr back safely
  }

  std::reverse(path.poses.begin(), path.poses.end());

  // auto end_time = std::chrono::steady_clock::now();
  // std::chrono::duration<double> diff_sec = end_time - start_time;
  // RCLCPP_INFO_STREAM(logger_, "planning_time = " << round_to_3dp(diff_sec.count()*1000.0) << " ms");

  return fillUpPath(path, goal); 
}



//--------------- LAZY THETA STAR ------------------------------

std::shared_ptr<GridNode> LazyThetaPlanner::runLazyThetaStarPlan(
  std::shared_ptr<GridNode> start_node,
  std::shared_ptr<GridNode> goal_node,
  const std::function<bool()>& cancel_checker,
  const unsigned char* char_map,
  unsigned int size_x)
{
  std::vector<DirNode> explore_directions = {
      DirNode({-1, 0}, 1.0), 
      DirNode({1, 0}, 1.0), 
      DirNode({0, 1}, 1.0), 
      DirNode({0, -1}, 1.0),
      DirNode({-1, 1}, 1.4142), 
      DirNode({1, -1}, 1.4142), 
      DirNode({1, 1}, 1.4142), 
      DirNode({-1, -1}, 1.4142),
  };

  auto comp = [](const std::shared_ptr<GridNode>& a, const std::shared_ptr<GridNode>& b) {
    return *a > *b;
  };

  std::priority_queue<
    std::shared_ptr<GridNode>, 
    std::vector<std::shared_ptr<GridNode>>, 
    decltype(comp)
  > nodes_to_explore(comp);

  int map_size = costmap_meta_.size_x * costmap_meta_.size_y;
  std::vector<bool> visited(map_size, false);
  std::vector<std::shared_ptr<GridNode>> node_lookup(map_size, nullptr);

  start_node->g_cost = 0.0;
  start_node->h_cost = euclidean_distance(*start_node, *goal_node);
  start_node->prev = nullptr; // Start node has no parent

  int start_idx = gridToMapIndex(*start_node);
  node_lookup[start_idx] = start_node;
  nodes_to_explore.push(start_node);

  while (!nodes_to_explore.empty()) {
    if (cancel_checker && cancel_checker()) {
      return nullptr;
    }

    std::shared_ptr<GridNode> active_node = nodes_to_explore.top();
    int active_idx = gridToMapIndex(*active_node);
    nodes_to_explore.pop();

    // Skip closed nodes or stale pointers
    if (visited[active_idx]) {
      continue;
    }

    if (node_lookup[active_idx] && active_node->g_cost > node_lookup[active_idx]->g_cost) {
      continue;
    }

    // --------------------------------------------------
    // 1. LAZY VALIDATION / SETVERTEX PHASE
    // --------------------------------------------------
    if (active_node->prev != nullptr) {
      // Check if optimistic shortcut to parent is valid
      if (!lineOfSight(*(active_node->prev), *active_node, char_map, size_x)) {
        // Line of sight failed: Repair active_node inline using VISITED neighbors
        double min_g = std::numeric_limits<double>::infinity();
        std::shared_ptr<GridNode> best_parent = nullptr;

        for (const auto &dir : explore_directions) {
          GridNode neighbor_pos = *active_node + dir.dir;
          int neigbhor_idx = gridToMapIndex(neighbor_pos);

          if (visited[neigbhor_idx] && isGridOnMap(neighbor_pos) && isMapCellFree(neighbor_pos, char_map)) {
            if (node_lookup[neigbhor_idx] != nullptr) {
              auto neighbor_node = node_lookup[neigbhor_idx];

              if (lineOfSight(*neighbor_node, *active_node, char_map, size_x, true)) {
                double dist = euclidean_distance(*active_node, *neighbor_node);
                double candidate_g = neighbor_node->g_cost + (dist * getGridCost(*active_node, char_map));

                if (candidate_g < min_g) {
                  min_g = candidate_g;
                  best_parent = neighbor_node;
                }
              }
            }
          }
        }

        if (best_parent) {
          active_node->g_cost = min_g;
          active_node->prev = best_parent;
        } else {
          // Unreachable cell; discard and close
          visited[active_idx] = true;
          continue;
        }
      }
    }

    // Mark node as closed/visited
    visited[active_idx] = true;

    // --------------------------------------------------
    // 2. GOAL CHECK
    // --------------------------------------------------
    if (*active_node == *goal_node) {
      return active_node;
    }

    // --------------------------------------------------
    // 3. EXPAND NEIGHBORS (OPTIMISTIC UPDATE)
    // --------------------------------------------------
    for (const auto &dir : explore_directions) {
      GridNode neighbor_pos = *active_node + dir.dir;
      int neighbor_idx = gridToMapIndex(neighbor_pos);

      if (!visited[neighbor_idx] && isGridOnMap(neighbor_pos) && isMapCellFree(neighbor_pos, char_map)){
        // Optimistic assumption: Try active_node's parent if present, else active_node
        std::shared_ptr<GridNode> optimistic_parent = (active_node->prev != nullptr) 
                                                      ? active_node->prev 
                                                      : active_node;

        double dist = euclidean_distance(*optimistic_parent, neighbor_pos);
        double grid_cost_factor = getGridCost(neighbor_pos, char_map);
        double new_cost = optimistic_parent->g_cost + (dist * grid_cost_factor);

        auto neighbor_node = node_lookup[neighbor_idx];

        if (!neighbor_node || new_cost < neighbor_node->g_cost) {
          if (!neighbor_node) {
            neighbor_node = std::make_shared<GridNode>(neighbor_pos);
          }

          neighbor_node->g_cost = new_cost;
          neighbor_node->h_cost = euclidean_distance(*neighbor_node, *goal_node);
          neighbor_node->prev = optimistic_parent;

          node_lookup[neighbor_idx] = neighbor_node;
          nodes_to_explore.push(neighbor_node);
        }
      }
    }
  }

  return nullptr;
}

//--------------------------------------------------------------


GridNode LazyThetaPlanner::poseToGrid(const geometry_msgs::msg::Pose &pose)
{
  int gx = static_cast<int>((pose.position.x - costmap_meta_.origin_x) * costmap_meta_.inv_resolution);
  int gy = static_cast<int>((pose.position.y - costmap_meta_.origin_y) * costmap_meta_.inv_resolution);

  return GridNode(gx, gy);
}

geometry_msgs::msg::Pose LazyThetaPlanner::gridToPose(const GridNode &grid)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = grid.x * costmap_meta_.resolution + costmap_meta_.origin_x;
  pose.position.y = grid.y * costmap_meta_.resolution + costmap_meta_.origin_y;
  pose.position.z = 0.0;

  return pose;
}

int LazyThetaPlanner::gridToMapIndex(const GridNode &grid_node)
{
  return static_cast<int>(grid_node.y * costmap_meta_.size_x + grid_node.x);
}

bool LazyThetaPlanner::isGridOnMap(const GridNode &grid)
{
  return (grid.x >= 0 && grid.x < costmap_meta_.size_x &&
          grid.y >= 0 && grid.y < costmap_meta_.size_y);
}

double LazyThetaPlanner::getGridCost(const GridNode &grid, const unsigned char* char_map)
{
  return  1.0+(cost_travel_multiplier_ * std::clamp(static_cast<double>(char_map[gridToMapIndex(grid)]) / 252.0, 0.0, 1.0));
}

bool LazyThetaPlanner::isMapCellFree(const GridNode &grid, const unsigned char* char_map)
{
  return /*(char_map[gridToMapIndex(grid)] >= 0) &&*/ (char_map[gridToMapIndex(grid)] < static_cast<unsigned char>(los_shortcut_cost_limit_+120));
}

double LazyThetaPlanner::euclidean_distance(const GridNode &a, const GridNode &b){
  return std::hypot(a.x - b.x, a.y - b.y);
}


bool LazyThetaPlanner::lineOfSight(
  const GridNode &start, 
  const GridNode &end,
  const unsigned char* char_map,
  unsigned int size_x,
  bool relax) const
{
  int x0 = start.x; int y0 = start.y;
  int x1 = end.x; int y1 = end.y;

  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  int stride_x = sx; 
  int stride_y = sy * static_cast<int>(size_x);

  int current_idx = y0 * size_x + x0;

  int max_x = static_cast<int>(costmap_meta_.size_x);
  int max_y = static_cast<int>(costmap_meta_.size_y);

  while (true)
  {
    // Safety Guard: Check map boundaries before reading char_map
    if (x0 < 0 || x0 >= max_x || y0 < 0 || y0 >= max_y) {
      return false;
    }

    if(relax){
      if (char_map[current_idx] > static_cast<unsigned char>(los_shortcut_cost_limit_+120)) {
        return false;
      }
    }
    else {
      if (char_map[current_idx] > static_cast<unsigned char>(los_shortcut_cost_limit_)) {
        return false;
      }
    }

    if (x0 == x1 && y0 == y1) {
      break;
    }

    int e2 = 2 * err;
    if (e2 > -dy) { 
      err -= dy; 
      x0 += sx; 
      current_idx += stride_x; 
    }
    if (e2 < dx)  { 
      err += dx; 
      y0 += sy; 
      current_idx += stride_y; 
    }
  }

  return true;
}


std::vector<geometry_msgs::msg::PoseStamped>
LazyThetaPlanner::addStraightLinePoses(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & end,
  double resolution) const
{
  std::vector<geometry_msgs::msg::PoseStamped> out;

  double dx = end.pose.position.x - start.pose.position.x;
  double dy = end.pose.position.y - start.pose.position.y;
  double dist = std::hypot(dx, dy);

  if (dist == 0)
    return {};

  int steps = std::max(1, static_cast<int>(dist / resolution));

  for (int i = 1; i <= steps; i++)
  {
    geometry_msgs::msg::PoseStamped ps = start;
    ps.pose.position.x = start.pose.position.x + dx * (i / (double)steps);
    ps.pose.position.y = start.pose.position.y + dy * (i / (double)steps);
    out.push_back(ps);
  }

  return out;
}



nav_msgs::msg::Path
LazyThetaPlanner::fillUpPath(
  const nav_msgs::msg::Path & path, 
  const geometry_msgs::msg::PoseStamped & goal) const
{
  nav_msgs::msg::Path filled;
  filled.header = path.header;

  if (path.poses.empty())
    return filled;

  // 1. Interpolate and densify the positions
  filled.poses.push_back(path.poses.front());
  for (size_t i = 1; i < path.poses.size(); i++)
  {
    auto seg = addStraightLinePoses(
      path.poses[i - 1],
      path.poses[i],
      costmap_ros_->getCostmap()->getResolution());

    filled.poses.insert(filled.poses.end(), seg.begin(), seg.end());
  }

  // 2. Calculate Yaw Orientations for intermediate waypoints
  for (size_t i = 0; i < filled.poses.size() - 1; i++)
  {
    double dx = filled.poses[i+1].pose.position.x - filled.poses[i].pose.position.x;
    double dy = filled.poses[i+1].pose.position.y - filled.poses[i].pose.position.y;
    double yaw = std::atan2(dy, dx);

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    filled.poses[i].pose.orientation.x = q.x();
    filled.poses[i].pose.orientation.y = q.y();
    filled.poses[i].pose.orientation.z = q.z();
    filled.poses[i].pose.orientation.w = q.w();
  }

  // 3. Force the absolute last waypoint to match the exact goal orientation
  filled.poses.back().pose.orientation = goal.pose.orientation;

  return filled;
}

}  // namespace theta_star_smooth_planner_plugin

PLUGINLIB_EXPORT_CLASS(
  lazy_theta_planner::LazyThetaPlanner,
  nav2_core::GlobalPlanner)
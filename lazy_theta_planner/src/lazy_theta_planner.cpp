#include "lazy_theta_planner/lazy_theta_planner.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <algorithm>

namespace lazy_theta_planner
{

double round_to_3dp(double val) {
    return std::round(val * 1000.0) / 1000.0;
}


void LazyThetaPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & optimistic_parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = optimistic_parent.lock();
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

  unsigned int map_size = costmap_meta_.size_x * costmap_meta_.size_y;

  // Pre-allocate vector pools once on activation
  node_pool_.resize(map_size);
  node_initialized_.resize(map_size, false);
  g_score_cache_.resize(map_size, -1.0);
  closed_cache_.resize(map_size, 0);

  RCLCPP_INFO_STREAM(logger_, "Plugin Activated Successfully");
}




void LazyThetaPlanner::deactivate() {}




void LazyThetaPlanner::cleanup()
{
  node_pool_.clear();
  node_pool_.shrink_to_fit();

  node_initialized_.clear();
  node_initialized_.shrink_to_fit();

  g_score_cache_.clear();
  g_score_cache_.shrink_to_fit();

  closed_cache_.clear();
  closed_cache_.shrink_to_fit();

  costmap_ros_.reset();

  RCLCPP_INFO_STREAM(logger_, "Plugin Cleaned Up Successfully");
}





nav_msgs::msg::Path LazyThetaPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
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

  unsigned int current_map_size = costmap_meta_.size_x * costmap_meta_.size_y;

  // Fallback memory check in case the costmap dynamically resizes during runtime
  if (node_pool_.size() != current_map_size) {
    node_pool_.resize(current_map_size);
    node_initialized_.resize(current_map_size);
    g_score_cache_.resize(current_map_size);
    closed_cache_.resize(current_map_size);
  }

  // --- FAST FLAT MEMORY RESETS ---
  std::fill(node_initialized_.begin(), node_initialized_.end(), false);
  std::fill(closed_cache_.begin(), closed_cache_.end(), 0);
  std::fill(g_score_cache_.begin(), g_score_cache_.end(), -1.0);

  auto get_node_from_pool = [this](int x, int y, int index) -> GridNode* {
    GridNode* node = &node_pool_[index];
    if (!node_initialized_[index]) {
      node->x = x;
      node->y = y;
      node->g_cost = std::numeric_limits<double>::max();
      node->h_cost = 0.0;
      node->prev = nullptr;
      node_initialized_[index] = true;
    }
    return node;
  };

  const unsigned char* char_map = costmap->getCharMap();

  // Start & Goal Node Setup
  GridNode raw_start = poseToGrid(start.pose);
  int start_idx = gridToMapIndex(raw_start);
  GridNode* start_node = get_node_from_pool(raw_start.x, raw_start.y, start_idx);

  GridNode raw_goal = poseToGrid(goal.pose);
  int goal_idx = gridToMapIndex(raw_goal);
  GridNode* goal_node = get_node_from_pool(raw_goal.x, raw_goal.y, goal_idx);

  // Execute Search
  GridNode* best_goal = runLazyThetaStarPlan(
    start_node, 
    goal_node, 
    cancel_checker, 
    char_map, 
    costmap_meta_.size_x
  );

  // Path Reconstruction
  nav_msgs::msg::Path path;
  path.header.frame_id = costmap_ros_->getGlobalFrameID();

  if (!best_goal) {
    return path;
  }

  GridNode* node = best_goal;
  while (node)
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = path.header.frame_id;
    ps.pose = gridToPose(*node);
    path.poses.push_back(ps);

    if (node->prev == node) {
      break;
    }
    node = node->prev;
  }

  std::reverse(path.poses.begin(), path.poses.end());
  return fillUpPath(path, goal);
}



//--------------- LAZY THETA STAR ------------------------------

GridNode* LazyThetaPlanner::runLazyThetaStarPlan(
  GridNode* start_node,
  GridNode* goal_node,
  const std::function<bool()>& cancel_checker,
  const unsigned char* char_map,
  unsigned int size_x)
{
  std::priority_queue<
    GridNode*,
    std::vector<GridNode*>,
    LazyThetaPlanner::CompareNode
  > open;

  int start_idx = gridToMapIndex(*start_node);

  start_node->g_cost = 0.0;
  start_node->h_cost = euclidean_distance(*start_node, *goal_node);
  start_node->prev = start_node;

  open.push(start_node);
  g_score_cache_[start_idx] = 0.0;

  // Directions with pre-calculated step distances for accurate grid traversal weighting
  static const struct { int dx; int dy; double dist; } dirs[] = {
    {-1,  0, 1.0},
    {1,  0, 1.0},
    {0, -1, 1.0},
    {0,  1, 1.0},
    {-1, -1, 1.4142},
    {-1, 1, 1.4142},
    {1, -1, 1.4142},
    {1,  1, 1.4142}
  };

  auto get_node_from_pool = [this](int x, int y, int index) -> GridNode* {
    GridNode* node = &node_pool_[index];
    if (!node_initialized_[index]) {
      node->x = x;
      node->y = y;
      node->g_cost = std::numeric_limits<double>::max();
      node->h_cost = 0.0;
      node->prev = nullptr;
      node_initialized_[index] = true;
    }
    return node;
  };

  while (!open.empty() /*&& rclcpp::ok()*/)
  {
    if (cancel_checker && cancel_checker()) {
      return nullptr;
    }

    GridNode* active = open.top();
    open.pop();

    int active_idx = gridToMapIndex(*active);

    if (closed_cache_[active_idx] == 1) {
      continue;
    }

    // --------------------------------------------------
    // 1. LAZY VALIDATION / SETVERTEX PHASE
    // --------------------------------------------------
    if (active->prev && active->prev != active)
    {
      if (!lineOfSight(active->prev->x, active->prev->y, active->x, active->y, char_map, size_x))
      {
        double min_g = std::numeric_limits<double>::infinity();
        GridNode* best_parent = nullptr;

        for (const auto & d : dirs)
        {
          int nx = active->x + d.dx;
          int ny = active->y + d.dy;
          GridNode nbr_pos(nx, ny);
          int nbr_idx = gridToMapIndex(nbr_pos);

          if ((closed_cache_[nbr_idx] == 1) && isGridOnMap(nbr_pos) && isMapCellFree(nbr_pos, char_map))
          {
            GridNode* nbr_node = &node_pool_[nbr_idx];

            if (lineOfSight(nbr_node->x, nbr_node->y, active->x, active->y, char_map, size_x, true))
            // double g_val = g_score_cache_[nbr_idx];
            // if (g_val >= 0.0)
            {
              // Include step heuristic + costmap terrain cost during optimistic_parent repair
              double g_val = g_score_cache_[nbr_idx];
              double dist = euclidean_distance(*nbr_node, *active);
              double cost_to_current = g_val + (dist * getGridCost(*active, char_map));
              if (cost_to_current < min_g) {
                min_g = cost_to_current;
                best_parent = nbr_node;
              }
            }
          }
        }

        if (best_parent) {
          active->g_cost = min_g;
          g_score_cache_[active_idx] = min_g;
          active->prev = best_parent;
        } else {
          // FIX: Mark node closed before skipping to prevent queue pollution/loops
          closed_cache_[active_idx] = 1;
          continue; 
        }
      }
    }

    // --------------------------------------------------
    // GOAL CHECK
    // --------------------------------------------------
    if (active->x == goal_node->x && active->y == goal_node->y) {
      return active; // Path extraction target
    }

    closed_cache_[active_idx] = 1;

    // --------------------------------------------------
    // 3. EXPAND NEIGHBORS (OPTIMISTIC UPDATE)
    // --------------------------------------------------
    for (const auto & d : dirs)
    {
      int nx = active->x + d.dx;
      int ny = active->y + d.dy;

      GridNode nbr_pos(nx, ny);
      int nbr_idx = gridToMapIndex(nbr_pos);

      if (!(closed_cache_[nbr_idx] == 1) && isGridOnMap(nbr_pos) && isMapCellFree(nbr_pos, char_map)) {
        GridNode* optimistic_parent = active->prev ? active->prev : active;
        GridNode* neighbor_node = get_node_from_pool(nx, ny, nbr_idx);

        double optimistic_parent_g = g_score_cache_[gridToMapIndex(*optimistic_parent)];

        // Combine geometric distance from line-of-sight optimistic_parent with local terrain penalty
        double dist = euclidean_distance(*optimistic_parent, nbr_pos);
        double grid_cost_factor = getGridCost(nbr_pos, char_map);
        double new_cost = optimistic_parent_g + (dist * grid_cost_factor);

        double current_neighbor_g = g_score_cache_[nbr_idx];
        if (current_neighbor_g < 0.0 || new_cost < current_neighbor_g)
        {
          g_score_cache_[nbr_idx] = new_cost;

          neighbor_node->g_cost = new_cost;
          neighbor_node->h_cost = euclidean_distance(*neighbor_node, *goal_node);
          neighbor_node->prev = optimistic_parent;

          open.push(neighbor_node);
        }
      }
    }
  }

  return nullptr; // Goal unreachable
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
  double dx = static_cast<double>(a.x - b.x);
  double dy = static_cast<double>(a.y - b.y);
  return std::sqrt(dx * dx + dy * dy);
}


bool LazyThetaPlanner::lineOfSight(
  int x0, int y0, 
  int x1, int y1,
  const unsigned char* char_map,
  unsigned int size_x,
  bool relax) const
{

  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  int stride_x = sx; 
  int stride_y = sy * static_cast<int>(size_x);

  int active_idx = y0 * size_x + x0;

  int max_x = static_cast<int>(costmap_meta_.size_x);
  int max_y = static_cast<int>(costmap_meta_.size_y);

  while (true)
  {
    // Safety Guard: Check map boundaries before reading char_map
    if (x0 < 0 || x0 >= max_x || y0 < 0 || y0 >= max_y) {
      return false;
    }

    if(relax){
      if (char_map[active_idx] > static_cast<unsigned char>(los_shortcut_cost_limit_+120)) {
        return false;
      }
    }
    else {
      if (char_map[active_idx] > static_cast<unsigned char>(los_shortcut_cost_limit_)) {
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
      active_idx += stride_x; 
    }
    if (e2 < dx)  { 
      err += dx; 
      y0 += sy; 
      active_idx += stride_y; 
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
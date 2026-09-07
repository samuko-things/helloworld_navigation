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
  g_cost_cache_.resize(map_size, -1.0);
  visited_.resize(map_size, false);

  RCLCPP_INFO_STREAM(logger_, "Plugin Activated Successfully");
}




void LazyThetaPlanner::deactivate() {}




void LazyThetaPlanner::cleanup()
{
  node_pool_.clear();
  node_pool_.shrink_to_fit();

  node_initialized_.clear();
  node_initialized_.shrink_to_fit();

  g_cost_cache_.clear();
  g_cost_cache_.shrink_to_fit();

  visited_.clear();
  visited_.shrink_to_fit();

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
    g_cost_cache_.resize(current_map_size);
    visited_.resize(current_map_size);
  }

  // --- FAST FLAT MEMORY RESETS ---
  std::fill(node_initialized_.begin(), node_initialized_.end(), false);
  std::fill(visited_.begin(), visited_.end(), false);
  std::fill(g_cost_cache_.begin(), g_cost_cache_.end(), -1.0);

  const unsigned char* char_map = costmap->getCharMap();

  // Start & Goal Node Setup
  GridNode raw_start = poseToGrid(start.pose);
  int start_idx = gridToMapIndex(raw_start);
  GridNode* start_node = get_node_from_pool(raw_start.x, raw_start.y, start_idx);

  GridNode raw_goal = poseToGrid(goal.pose);
  int goal_idx = gridToMapIndex(raw_goal);
  GridNode* goal_node = get_node_from_pool(raw_goal.x, raw_goal.y, goal_idx);

  // Execute Search

  // GridNode* best_goal = runLazyThetaStarPlan(
  //   start_node, 
  //   goal_node, 
  //   cancel_checker, 
  //   char_map, 
  //   costmap_meta_.size_x
  // );

  GridNode* best_goal = runDRSPPlan(
    start_node, 
    goal_node, 
    cancel_checker, 
    char_map, 
    costmap_meta_.size_x
  );

  // GridNode* best_goal = runAStarPlan(
  //   start_node, 
  //   goal_node, 
  //   cancel_checker, 
  //   char_map,
  //   costmap_meta_.size_x
  // );

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

  // std::reverse(path.poses.begin(), path.poses.end());
  // return path;

  std::reverse(path.poses.begin(), path.poses.end());
  return fillUpPath(path, goal);
}


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
  > nodes_to_explore;

  int start_idx = gridToMapIndex(*start_node);

  start_node->g_cost = 0.0;
  start_node->h_cost = euclidean_distance(*start_node, *goal_node);
  start_node->prev = start_node;

  nodes_to_explore.push(start_node);
  g_cost_cache_[start_idx] = 0.0;

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

  while (!nodes_to_explore.empty() /*&& rclcpp::ok()*/)
  {
    if (cancel_checker && cancel_checker()) {
      return nullptr;
    }

    GridNode* active = nodes_to_explore.top();
    nodes_to_explore.pop();

    int active_idx = gridToMapIndex(*active);

    if (visited_[active_idx]) {
      continue;
    }

    if (active->g_cost > g_cost_cache_[active_idx]) {
      continue;
    }

    if (active->x == goal_node->x && active->y == goal_node->y) {
      return active; // Path extraction target
    }

    // --------------------------------------------------
    // LAZY VALIDATION / SETVERTEX PHASE
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

          if (isGridOnMap(nbr_pos) && isMapCellFree(nbr_pos, char_map) && visited_[nbr_idx])
          {
            GridNode* nbr_node = &node_pool_[nbr_idx];
            double g_val = g_cost_cache_[nbr_idx];
            double cost_to_active = g_val + (d.dist * getGridCost(*active, char_map));
            if (cost_to_active < min_g) {
              min_g = cost_to_active;
              best_parent = nbr_node;
            }
          }
        }

        if (best_parent) {
          active->g_cost = min_g;
          g_cost_cache_[active_idx] = min_g;
          active->prev = best_parent;
        } 
        else {
          visited_[active_idx] = true;
          continue; 
        }
      }
    }

    visited_[active_idx] = true;

    // --------------------------------------------------
    // EXPAND NEIGHBORS (OPTIMISTIC UPDATE)
    // --------------------------------------------------
    for (const auto & d : dirs)
    {
      int nx = active->x + d.dx;
      int ny = active->y + d.dy;

      GridNode nbr_pos(nx, ny);
      int nbr_idx = gridToMapIndex(nbr_pos);

      if (isGridOnMap(nbr_pos) && isMapCellFree(nbr_pos, char_map) && !visited_[nbr_idx]) {
        GridNode* optimistic_parent = active->prev ? active->prev : active;
        GridNode* neighbor_node = get_node_from_pool(nx, ny, nbr_idx);

        double optimistic_parent_g = g_cost_cache_[gridToMapIndex(*optimistic_parent)];

        double dist = euclidean_distance(*optimistic_parent, *neighbor_node);
        double grid_cost_factor = getGridCost(*neighbor_node, char_map);
        double new_cost = optimistic_parent_g + (dist * grid_cost_factor);

        double current_neighbor_g = g_cost_cache_[nbr_idx];
        if (current_neighbor_g < 0.0 || new_cost < current_neighbor_g)
        {
          g_cost_cache_[nbr_idx] = new_cost;

          neighbor_node->g_cost = new_cost;
          neighbor_node->h_cost = euclidean_distance(*neighbor_node, *goal_node);
          neighbor_node->prev = optimistic_parent;

          nodes_to_explore.push(neighbor_node);
        }
      }
    }
  }

  return nullptr;
}



GridNode* LazyThetaPlanner::runDRSPPlan(
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
  > nodes_to_explore;

  int start_idx = gridToMapIndex(*start_node);

  start_node->g_cost = 0.0;
  start_node->h_cost = euclidean_distance(*start_node, *goal_node);
  start_node->prev = start_node;

  nodes_to_explore.push(start_node);
  g_cost_cache_[start_idx] = 0.0;

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

  while (!nodes_to_explore.empty() /*&& rclcpp::ok()*/)
  {
    if (cancel_checker && cancel_checker()) {
      return nullptr;
    }

    GridNode* active = nodes_to_explore.top();
    nodes_to_explore.pop();

    int active_idx = gridToMapIndex(*active);

    if (visited_[active_idx]) {
      continue;
    }

    if (active->g_cost > g_cost_cache_[active_idx]) {
      continue;
    }

    if (active->prev && active->prev->prev)
    {
      auto actual_grandparent = active->prev->prev;
      if (lineOfSight(active->x, active->y, actual_grandparent->x, actual_grandparent->y, char_map, size_x))
      {
        active->prev = actual_grandparent;
      }
      else
      {
        double min_g = std::numeric_limits<double>::infinity();
        GridNode* best_parent = nullptr;

        for (const auto & d : dirs)
        {
          int nx = active->x + d.dx;
          int ny = active->y + d.dy;
          GridNode nbr_pos(nx, ny);
          int nbr_idx = gridToMapIndex(nbr_pos);

          if (isGridOnMap(nbr_pos) && isMapCellFree(nbr_pos, char_map) && visited_[nbr_idx])
          {
            GridNode* nbr_node = &node_pool_[nbr_idx];
            double g_val = g_cost_cache_[nbr_idx];
            double cost_to_active = g_val + (d.dist * getGridCost(*active, char_map));
            if (cost_to_active < min_g) {
              min_g = cost_to_active;
              best_parent = nbr_node;
            }
          }
        }

        if (best_parent) {
          active->g_cost = min_g;
          g_cost_cache_[active_idx] = min_g;
          active->prev = best_parent;
        }
      }
    }

    if (active->x == goal_node->x && active->y == goal_node->y) {
      return active;
    }

    visited_[active_idx] = true;

    for (const auto & d : dirs)
    {
      int nx = active->x + d.dx;
      int ny = active->y + d.dy;

      GridNode nbr_pos(nx, ny);
      int nbr_idx = gridToMapIndex(nbr_pos);

      if (isGridOnMap(nbr_pos) && isMapCellFree(nbr_pos, char_map) && !visited_[nbr_idx]) {
        GridNode* assumed_grandparent = active->prev ? active->prev : active;
        GridNode* neighbor_node = get_node_from_pool(nx, ny, nbr_idx);

        double new_g_cost = assumed_grandparent->g_cost + (euclidean_distance(nbr_pos, *(assumed_grandparent)) * getGridCost(nbr_pos, char_map));
        double nbr_g_cost = g_cost_cache_[nbr_idx];

        if (nbr_g_cost < 0.0 || new_g_cost < nbr_g_cost)
        {
          g_cost_cache_[nbr_idx] = new_g_cost;

          neighbor_node->g_cost = new_g_cost;
          neighbor_node->h_cost = euclidean_distance(*neighbor_node, *goal_node);
          // update prev node as the actual parent, not the assumed grandparent
          neighbor_node->prev = active;

          nodes_to_explore.push(neighbor_node);
        }
      }
    }
  }

  return nullptr;
}


GridNode* LazyThetaPlanner::runAStarPlan(
  GridNode* start_node,
  GridNode* goal_node,
  const std::function<bool()>& cancel_checker,
  const unsigned char* char_map,
  unsigned int size_x,
  bool smooth)
{
  std::priority_queue<
    GridNode*,
    std::vector<GridNode*>,
    LazyThetaPlanner::CompareNode
  > nodes_to_explore;

  int start_idx = gridToMapIndex(*start_node);

  start_node->g_cost = 0.0;
  start_node->h_cost = euclidean_distance(*start_node, *goal_node);
  start_node->prev = start_node;

  nodes_to_explore.push(start_node);
  g_cost_cache_[start_idx] = 0.0;

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

  while (!nodes_to_explore.empty() /*&& rclcpp::ok()*/)
  {
    if (cancel_checker && cancel_checker()) {
      return nullptr;
    }

    GridNode* active = nodes_to_explore.top();
    nodes_to_explore.pop();

    int active_idx = gridToMapIndex(*active);

    if (visited_[active_idx]) {
      continue;
    }

    if (active->g_cost > g_cost_cache_[active_idx]) {
      continue;
    }

    if (active->x == goal_node->x && active->y == goal_node->y) {
      if(smooth){
        return greedyStringPullSmooth(
          active,
          cancel_checker, 
          char_map, 
          size_x);
      }
      return active;
    }

    visited_[active_idx] = true;

    for (const auto & d : dirs)
    {
      int nx = active->x + d.dx;
      int ny = active->y + d.dy;

      GridNode nbr_pos(nx, ny);
      int nbr_idx = gridToMapIndex(nbr_pos);

      if (isGridOnMap(nbr_pos) && isMapCellFree(nbr_pos, char_map) && !visited_[nbr_idx]) {
        double new_cost;
        GridNode* neighbor_node = get_node_from_pool(nx, ny, nbr_idx);
        new_cost = active->g_cost + (d.dist * getGridCost(nbr_pos, char_map));

        double current_neighbor_g = g_cost_cache_[nbr_idx];
        if (current_neighbor_g < 0.0 || new_cost < current_neighbor_g)
        {
          g_cost_cache_[nbr_idx] = new_cost;

          neighbor_node->g_cost = new_cost;
          neighbor_node->h_cost = euclidean_distance(*neighbor_node, *goal_node);
          neighbor_node->prev = active;

          nodes_to_explore.push(neighbor_node);
        }
      }
    }
  }

  return nullptr;
}


GridNode* LazyThetaPlanner::greedyStringPullSmooth(
  GridNode* grid_node_path,
  const std::function<bool()>& cancel_checker,
  const unsigned char* char_map,
  unsigned int size_x)
{
  if (!grid_node_path) {
    return nullptr;
  }

  // 1. Unwind the linked list path from Goal -> Start into a vector
  std::vector<GridNode*> poses;
  GridNode* current = grid_node_path;
  
  while (current != nullptr) {
    poses.push_back(current);
    
    if (current->prev == current || current->prev == nullptr) {
      break;
    }
    current = current->prev;
  }

  std::reverse(poses.begin(), poses.end());

  int n = static_cast<int>(poses.size());

  if (n <= 2) {
    return grid_node_path;
  }

  int i = 0;
  int j = 2;

  std::vector<GridNode*> smoothed_poses;
  smoothed_poses.push_back(poses[0]);

  while (true) {
    if (cancel_checker && cancel_checker()) {
      return nullptr;
    }

    if (!(j < n)) {
      break;
    }
    else if (lineOfSight(poses[i]->x, poses[i]->y, poses[j]->x, poses[j]->y, char_map, size_x)) {
      j = j;  
    } else {
      i = j - 1;
      
      poses[i]->prev = smoothed_poses.back();
      smoothed_poses.push_back(poses[i]);
    }
    j += 1;
  }

  i = j - 1;
  poses[i]->prev = smoothed_poses.back();
  smoothed_poses.push_back(poses[i]);

  GridNode* smoothed_grid_node_path = smoothed_poses.back();
  return smoothed_grid_node_path;
}


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


// bool LazyThetaPlanner::lineOfSight(
//   int x0, int y0, 
//   int x1, int y1,
//   const unsigned char* char_map,
//   unsigned int size_x,
//   bool relax) const
// {

//   int dx = std::abs(x1 - x0);
//   int dy = std::abs(y1 - y0);
//   int sx = (x0 < x1) ? 1 : -1;
//   int sy = (y0 < y1) ? 1 : -1;
//   int err = dx - dy;

//   int stride_x = sx; 
//   int stride_y = sy * static_cast<int>(size_x);

//   int active_idx = y0 * size_x + x0;

//   int max_x = static_cast<int>(costmap_meta_.size_x);
//   int max_y = static_cast<int>(costmap_meta_.size_y);

//   while (true)
//   {
//     // Safety Guard: Check map boundaries before reading char_map
//     if (x0 < 0 || x0 >= max_x || y0 < 0 || y0 >= max_y) {
//       return false;
//     }

//     if(relax){
//       if (char_map[active_idx] > static_cast<unsigned char>(los_shortcut_cost_limit_+120)) {
//         return false;
//       }
//     }
//     else {
//       if (char_map[active_idx] > static_cast<unsigned char>(los_shortcut_cost_limit_)) {
//         return false;
//       }
//     }

//     if (x0 == x1 && y0 == y1) {
//       break;
//     }

//     int e2 = 2 * err;
//     if (e2 > -dy) { 
//       err -= dy; 
//       x0 += sx; 
//       active_idx += stride_x; 
//     }
//     if (e2 < dx)  { 
//       err += dx; 
//       y0 += sy; 
//       active_idx += stride_y; 
//     }
//   }

//   return true;
// }


bool LazyThetaPlanner::lineOfSight(
  int x0, int y0, 
  int x1, int y1,
  const unsigned char* char_map,
  unsigned int size_x,
  bool relax) const
{
  int dx = std::abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
  int dy = std::abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  int max_x = static_cast<int>(costmap_meta_.size_x);
  int max_y = static_cast<int>(costmap_meta_.size_y);

  // Compute cost threshold based on the relax flag
  const auto threshold = static_cast<unsigned char>(
    los_shortcut_cost_limit_ + (relax ? 120 : 0));

  // Helper lambda to encapsulate boundary and cost checks
  auto isSafe = [&](int x, int y) -> bool {
    if (x < 0 || x >= max_x || y < 0 || y >= max_y) {
      return false;
    }
    int idx = y * static_cast<int>(size_x) + x;
    return char_map[idx] <= threshold;
  };

  while (true) {
    if (!isSafe(x0, y0)) {
      return false;
    }

    if (x0 == x1 && y0 == y1) {
      break;
    }

    int e2 = 2 * err;
    if (e2 > -dy) { 
      err -= dy; 
      x0 += sx; 
    }
    if (e2 < dx) { 
      err += dx; 
      y0 += sy; 
    }
  }

  return true;
}


GridNode* LazyThetaPlanner::get_node_from_pool(int x, int y, int index) {
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
LazyThetaPlanner::densifyPath(
  const nav_msgs::msg::Path & path, 
  const geometry_msgs::msg::PoseStamped & goal) const
{
  nav_msgs::msg::Path dense_path;
  dense_path.header = path.header;

  if (path.poses.empty())
    return dense_path;

  // 1. Interpolate and densify the positions
  dense_path.poses.push_back(path.poses.front());
  for (size_t i = 1; i < path.poses.size(); i++)
  {
    auto seg = addStraightLinePoses(
      path.poses[i - 1],
      path.poses[i],
      costmap_meta_.resolution);

    dense_path.poses.insert(dense_path.poses.end(), seg.begin(), seg.end());
  }

  // 2. Calculate Yaw Orientations for intermediate waypoints
  for (size_t i = 0; i < dense_path.poses.size() - 1; i++)
  {
    double dx = dense_path.poses[i+1].pose.position.x - dense_path.poses[i].pose.position.x;
    double dy = dense_path.poses[i+1].pose.position.y - dense_path.poses[i].pose.position.y;
    double yaw = std::atan2(dy, dx);

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    dense_path.poses[i].pose.orientation.x = q.x();
    dense_path.poses[i].pose.orientation.y = q.y();
    dense_path.poses[i].pose.orientation.z = q.z();
    dense_path.poses[i].pose.orientation.w = q.w();
  }

  // 3. Force the absolute last waypoint to match the exact goal orientation
  dense_path.poses.back().pose.orientation = goal.pose.orientation;

  return dense_path;
}


nav_msgs::msg::Path LazyThetaPlanner::smoothPath(
  const nav_msgs::msg::Path & path,
  double w_data,
  double w_smooth,
  int max_iterations,
  double tolerance) const
{
  nav_msgs::msg::Path smoothed = path;
  const size_t num_points = smoothed.poses.size();

  // Paths with fewer than 3 points cannot be smoothed (start and goal remain fixed)
  if (num_points < 3) {
    return smoothed;
  }

  // Cache original positions for data fidelity term (w_data)
  std::vector<geometry_msgs::msg::Point> orig_poses(num_points);
  for (size_t i = 0; i < num_points; ++i) {
    orig_poses[i] = path.poses[i].pose.position;
  }

  int iteration = 0;
  double change = tolerance;

  // Working copy to hold iteration updates securely
  nav_msgs::msg::Path temp_path = smoothed;

  while (iteration < max_iterations && change >= tolerance) {
    change = 0.0;

    // Interior points only (preserve index 0 and num_points - 1)
    for (size_t i = 1; i < num_points - 1; ++i) {
      const auto & prev = smoothed.poses[i - 1].pose.position;
      const auto & curr = smoothed.poses[i].pose.position;
      const auto & next = smoothed.poses[i + 1].pose.position;
      const auto & orig = orig_poses[i];

      // Calculate Gradient Descent displacement terms
      double rx = orig.x - curr.x;
      double ry = orig.y - curr.y;

      double sx = prev.x + next.x - (2.0 * curr.x);
      double sy = prev.y + next.y - (2.0 * curr.y);

      double update_x = curr.x + (w_data * rx) + (w_smooth * sx);
      double update_y = curr.y + (w_data * ry) + (w_smooth * sy);

      // Simple collision check before accepting node displacement
      // Replace 'inCollision' with your costmap validation logic if available
      // if (inCollision(update_x, update_y)) {
      //   continue; // Skip moving this node into an obstacle
      // }

      // Track magnitude of coordinate shift for convergence
      change += std::abs(update_x - curr.x) + std::abs(update_y - curr.y);

      temp_path.poses[i].pose.position.x = update_x;
      temp_path.poses[i].pose.position.y = update_y;
    }

    smoothed = temp_path;
    iteration++;
  }

  return smoothed;
}


nav_msgs::msg::Path
LazyThetaPlanner::fillUpPath(
  const nav_msgs::msg::Path & path, 
  const geometry_msgs::msg::PoseStamped & goal,
  bool smooth) const
{
  nav_msgs::msg::Path smoothed = densifyPath(path, goal);

  if (smooth){
    smoothed = smoothPath(smoothed);

    for (size_t i = 0; i < smoothed.poses.size() - 1; i++)
    {
      double dx = smoothed.poses[i+1].pose.position.x - smoothed.poses[i].pose.position.x;
      double dy = smoothed.poses[i+1].pose.position.y - smoothed.poses[i].pose.position.y;
      double yaw = std::atan2(dy, dx);

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      smoothed.poses[i].pose.orientation = tf2::toMsg(q);
    }

    smoothed.poses.back().pose.orientation = goal.pose.orientation;
  }

  return smoothed;
}


}  // namespace theta_star_smooth_planner_plugin

PLUGINLIB_EXPORT_CLASS(
  lazy_theta_planner::LazyThetaPlanner,
  nav2_core::GlobalPlanner)
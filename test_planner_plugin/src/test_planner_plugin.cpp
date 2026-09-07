#include "test_planner_plugin/test_planner_plugin.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <algorithm>

namespace test_planner_plugin
{

double round_to_3dp(double val) {
    return std::round(val * 1000.0) / 1000.0;
}


void TestPlanner::configure(
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
    node, name + ".los_shortcut_cost_limit", rclcpp::ParameterValue(10));

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".cost_travel_multiplier", rclcpp::ParameterValue(3.0));

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".planner_id", rclcpp::ParameterValue(0));

  // Retrieve values
  node->get_parameter(name + ".los_shortcut_cost_limit", los_shortcut_cost_limit_);
  node->get_parameter(name + ".cost_travel_multiplier", cost_travel_multiplier_);
  node->get_parameter(name + ".planner_id", planner_id_);

  los_shortcut_cost_limit_ = std::clamp(los_shortcut_cost_limit_, 10, 100);

  RCLCPP_INFO_STREAM(
    logger_,
    "Configured Test Planner Plugin with: " <<
    "\n  planner_id               : " << planner_id_ <<
    "\n  los_shortcut_cost_limit  : " << los_shortcut_cost_limit_ <<
    "\n  cost_travel_multiplier   : " << cost_travel_multiplier_);
}





void TestPlanner::activate()
{
  if (!costmap_ros_) {
    return;
  }

  auto costmap = costmap_ros_->getCostmap();
  costmap_meta_.update(costmap);

  unsigned int map_size = costmap_meta_.size_x * costmap_meta_.size_y;

  // Pre-allocate vector pools once on activation
  node_pool_.resize(map_size);
  node_visited_id_.resize(map_size);

  RCLCPP_INFO_STREAM(logger_, "Plugin Activated Successfully");
}




void TestPlanner::deactivate() {}




void TestPlanner::cleanup()
{
  node_pool_.clear();
  node_pool_.shrink_to_fit();

  node_visited_id_.clear();
  node_visited_id_.shrink_to_fit();

  costmap_ros_.reset();

  RCLCPP_INFO_STREAM(logger_, "Plugin Cleaned Up Successfully");
}





nav_msgs::msg::Path TestPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  auto start_time = std::chrono::steady_clock::now();

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
    node_visited_id_.resize(current_map_size);
  }

  // --- FAST FLAT MEMORY RESETS ---
  std::fill(node_visited_id_.begin(),node_visited_id_.end(), 0);

  const unsigned char* char_map = costmap->getCharMap();

  run_id_++;
  obs_dir_ = generateDirectionRing(4);

  // Start & Goal Node Setup
  GridNode raw_start = poseToGrid(start.pose);
  int start_idx = gridToMapIndex(raw_start);
  GridNode* start_node = get_node_from_pool(raw_start.x, raw_start.y, start_idx);

  GridNode raw_goal = poseToGrid(goal.pose);
  int goal_idx = gridToMapIndex(raw_goal);
  GridNode* goal_node = get_node_from_pool(raw_goal.x, raw_goal.y, goal_idx);

  // Execute Search

  size_t los_checks;
  size_t node_expansions;
  size_t fallback_count;
  size_t successful_parent_collapses;

  GridNode* best_goal;

  if (planner_id_ == 0) {
    best_goal = runLazyThetaStarPlan(
      start_node, 
      goal_node,
      cancel_checker,
      char_map, 
      costmap_meta_.size_x,
      los_checks,
      node_expansions,
      fallback_count,
      successful_parent_collapses
    );
  }
  else {
    best_goal = runLazyThetaStarPlanTest(
      start_node, 
      goal_node,
      cancel_checker,
      char_map, 
      costmap_meta_.size_x,
      los_checks,
      node_expansions,
      fallback_count,
      successful_parent_collapses
    );
  }


  // Path Reconstruction
  nav_msgs::msg::Path path;
  path.header.frame_id = costmap_ros_->getGlobalFrameID();

  if (!best_goal) {
    RCLCPP_INFO_STREAM(logger_, "No Path Generated");
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

  auto end_time = std::chrono::steady_clock::now();
  std::chrono::duration<double> diff_sec = end_time - start_time;
  RCLCPP_INFO_STREAM(logger_, "planning_time" <<"[" << planner_id_ << "] = " << round_to_3dp(diff_sec.count()*1000.0) << " ms");

  return fillUpPath(path, goal);

}






GridNode* TestPlanner::runLazyThetaStarPlan(
  GridNode* start_node,
  GridNode* goal_node,
  const std::function<bool()>& cancel_checker,
  const unsigned char* char_map,
  unsigned int size_x,
  size_t & los_checks,
  size_t & node_expansions,
  size_t & fallback_count,
  size_t & successful_parent_collapses)
{
  // Reset counters for this run
  los_checks = 0;
  node_expansions = 0;
  fallback_count = 0;
  successful_parent_collapses = 0; // Always 0 for Lazy Theta* (Grandparent collapsing is DRSP-specific)

  start_node->g_cost = 0.0;
  start_node->h_cost = euclidean_distance(*start_node, *goal_node);
  start_node->f_cost = start_node->g_cost + start_node->h_cost;
  start_node->is_in_queue = true;
  start_node->prev = start_node;

  open_queue_.push(start_node);

  while (!open_queue_.empty() /*&& rclcpp::ok()*/) 
  {
    if (cancel_checker && cancel_checker()) {
      clearQueue();
      return nullptr;
    }

    GridNode* current = open_queue_.top();
    open_queue_.pop();

    current->is_in_queue = false;

    // Lazy Theta*: SetVertex Step (Verify line-of-sight to parent upon pop)
    if (current->prev && current->prev->prev)
    {
      GridNode *maybe_parent = current->prev->prev;
      los_checks++; // LOS metric counter
      if (lineOfSight(current, maybe_parent, char_map, size_x))
      {
        double current_g_cost = maybe_parent->g_cost + (euclidean_distance(*current, *maybe_parent) * getGridCost(*current, char_map));
        if (current_g_cost < current->g_cost)
        {
          current->prev = maybe_parent;
          current->g_cost = current_g_cost;
          current->f_cost = current_g_cost + current->h_cost;
          successful_parent_collapses++;
        }
      }
    }

    if (current->x == goal_node->x && current->y == goal_node->y) {
      clearQueue();
      return current;
    }

    node_expansions++; // Node Expansion metric counter

    // UpdateVertex Expansion Phase
    for (const auto & d : dirs_)
    {
      int nx = current->x + d.dx;
      int ny = current->y + d.dy;

      GridNode nbr_pos(nx, ny);
      int nbr_idx = gridToMapIndex(nbr_pos);

      if (isGridOnMap(nbr_pos) && isMapCellFree(nbr_pos, char_map)) {
        GridNode* nbr_node = get_node_from_pool(nx, ny, nbr_idx);

        double nbr_g_cost = current->g_cost + (d.dist * getGridCost(*nbr_node, char_map));
        double nbr_h_cost = euclidean_distance(*nbr_node, *goal_node);
        double nbr_f_cost = nbr_g_cost + nbr_h_cost;

        if (nbr_node->f_cost > nbr_f_cost)
        {
          nbr_node->g_cost = nbr_g_cost;
          nbr_node->h_cost = nbr_h_cost;
          nbr_node->f_cost = nbr_f_cost;
          nbr_node->prev = current;

          if(!nbr_node->is_in_queue)
          {
            nbr_node->is_in_queue = true;
            open_queue_.push(nbr_node);
          }
        }
      }
    }

  }

  return nullptr;
}








GridNode* TestPlanner::runLazyThetaStarPlanTest(
  GridNode* start_node,
  GridNode* goal_node,
  const std::function<bool()>& cancel_checker,
  const unsigned char* char_map,
  unsigned int size_x,
  size_t & los_checks,
  size_t & node_expansions,
  size_t & fallback_count,
  size_t & successful_parent_collapses)
{
  // Reset counters for this run
  los_checks = 0;
  node_expansions = 0;
  fallback_count = 0;
  successful_parent_collapses = 0; // Always 0 for Lazy Theta* (Grandparent collapsing is DRSP-specific)

  start_node->g_cost = 0.0;
  start_node->h_cost = euclidean_distance(*start_node, *goal_node);
  start_node->f_cost = start_node->g_cost + start_node->h_cost;
  start_node->is_in_queue = true;
  start_node->prev = start_node;
  start_node->parent = start_node;

  open_queue_.push(start_node);

  while (!open_queue_.empty() /*&& rclcpp::ok()*/)
  {
    if (cancel_checker && cancel_checker()) {
      clearQueue();
      return nullptr;
    }
    GridNode* current = open_queue_.top();
    open_queue_.pop();

    current->is_in_queue = false;

    // Lazy Theta*: SetVertex Step (Verify line-of-sight to parent upon pop)
    if (current->parent != current && current->prev && current->prev->prev)
    {
      // if(isNodeCloseToObstacle(char_map, *current))
      if(isCloseToObstacle(*current, char_map))
      {
        GridNode *maybe_parent = current->prev->prev;
        los_checks++; // LOS metric counter
        if (lineOfSight(current, maybe_parent, char_map, size_x))
        {
          double current_g_cost = maybe_parent->g_cost + (euclidean_distance(*current, *maybe_parent) * getGridCost(*current, char_map));
          if (current_g_cost < current->g_cost)
          {
            current->prev = maybe_parent;
            // current->parent = maybe_parent;
            current->g_cost = current_g_cost;
            current->f_cost = current_g_cost + current->h_cost;
            successful_parent_collapses++;
          }
        }
        else
        {
          fallback_count++;
          los_checks++;
          if (lineOfSight(current->parent, maybe_parent, char_map, size_x))
          {
            double parent_g_cost = maybe_parent->g_cost + (euclidean_distance(*(current->parent), *maybe_parent) * getGridCost(*(current->parent), char_map));
            if (parent_g_cost < current->parent->g_cost)
            {
              current->parent->g_cost = parent_g_cost;
              current->parent->f_cost = parent_g_cost + current->parent->h_cost;
            }
          }

          double current_g_cost = current->parent->g_cost + (euclidean_distance(*current, *(current->parent)) * getGridCost(*current, char_map));
          current->g_cost = current_g_cost;
          current->f_cost = current_g_cost + current->h_cost;

          current->prev = current->parent;
        }
      }
    }

    if (current->x == goal_node->x && current->y == goal_node->y) {
      clearQueue();
      return current;
    }
    
    node_expansions++; // Node Expansion metric counter

    // UpdateVertex Expansion Phase
    for (const auto & d : dirs_)
    {
      int nx = current->x + d.dx;
      int ny = current->y + d.dy;

      GridNode nbr_pos(nx, ny);
      int nbr_idx = gridToMapIndex(nbr_pos);

      if (isGridOnMap(nbr_pos) && isMapCellFree(nbr_pos, char_map)) {
        GridNode* nbr_node = get_node_from_pool(nx, ny, nbr_idx);

        double nbr_g_cost = current->g_cost + (d.dist * getGridCost(*nbr_node, char_map));
        double nbr_h_cost = euclidean_distance(*nbr_node, *goal_node);
        double nbr_f_cost = nbr_g_cost + nbr_h_cost;

        if (nbr_node->f_cost > nbr_f_cost)
        {
          nbr_node->g_cost = nbr_g_cost;
          nbr_node->h_cost = nbr_h_cost;
          nbr_node->f_cost = nbr_f_cost;
          nbr_node->parent = current;
          nbr_node->prev = current->prev;

          if(!nbr_node->is_in_queue)
          {
            nbr_node->is_in_queue = true;
            open_queue_.push(nbr_node);
          }
        }
      }
    }

  }

  return nullptr;
}







GridNode TestPlanner::poseToGrid(const geometry_msgs::msg::Pose &pose) const
{
  int gx = static_cast<int>((pose.position.x - costmap_meta_.origin_x) * costmap_meta_.inv_resolution);
  int gy = static_cast<int>((pose.position.y - costmap_meta_.origin_y) * costmap_meta_.inv_resolution);

  return GridNode(gx, gy);
}

geometry_msgs::msg::Pose TestPlanner::gridToPose(const GridNode &grid) const
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = grid.x * costmap_meta_.resolution + costmap_meta_.origin_x;
  pose.position.y = grid.y * costmap_meta_.resolution + costmap_meta_.origin_y;
  pose.position.z = 0.0;

  return pose;
}

int TestPlanner::gridToMapIndex(const GridNode &grid_node) const
{
  return static_cast<int>(grid_node.y * costmap_meta_.size_x + grid_node.x);
}

int TestPlanner::gridToMapIndex(const int x, const int y) const
{
  return static_cast<int>(y * costmap_meta_.size_x + x);
}

bool TestPlanner::isGridOnMap(const GridNode &grid) const
{
  return (grid.x >= 0 && grid.x < costmap_meta_.size_x &&
          grid.y >= 0 && grid.y < costmap_meta_.size_y);
}

// double TestPlanner::getGridCost(const GridNode &grid, const unsigned char* char_map) const
// {
//   return  1.0+(cost_travel_multiplier_ * std::clamp(static_cast<double>(char_map[gridToMapIndex(grid)]) / 252.0, 0.0, 1.0));
// }

double TestPlanner::getGridCost(const GridNode &grid, const unsigned char* char_map) const
{
  if(static_cast<int>(char_map[gridToMapIndex(grid)])<=los_shortcut_cost_limit_)
    return 1.0;
  else
    return  1.0+(cost_travel_multiplier_ * std::clamp(static_cast<double>(char_map[gridToMapIndex(grid)]) / 252.0, 0.0, 1.0));
}

bool TestPlanner::isMapCellFree(const GridNode &grid, const unsigned char* char_map) const
{
  // RCLCPP_INFO_STREAM(
  //   logger_, "cell_cost = " << static_cast<int>(char_map[gridToMapIndex(grid)]));
  return /*(char_map[gridToMapIndex(grid)] >= 0) &&*/ (char_map[gridToMapIndex(grid)] < static_cast<unsigned char>(los_shortcut_cost_limit_+120));
}


double TestPlanner::euclidean_distance(const GridNode &a, const GridNode &b) const
{
  return std::hypot(a.x - b.x, a.y - b.y);
}


bool TestPlanner::isCloseToObstacle(const GridNode &node, const unsigned char* char_map) const
{
  return char_map[gridToMapIndex(node)] > 0;
  // for (const auto & d : obs_dir_)
  // {
  //   int nx = node.x + d.dx;
  //   int ny = node.y + d.dy;

  //   if (char_map[gridToMapIndex(nx, ny)] > 0) {
  //       return true;
  //   }

  // }

  // return false;
}


// bool TestPlanner::lineOfSight(
//   GridNode *start, 
//   GridNode *end,
//   const unsigned char* char_map,
//   unsigned int size_x,
//   bool relax) const
// {
//   int x0 = start->x; int y0 = start->y;
//   int x1 = end->x; int y1 = end->y;

//   int dx = std::abs(x1 - x0);
//   int dy = std::abs(y1 - y0);
//   int sx = (x0 < x1) ? 1 : -1;
//   int sy = (y0 < y1) ? 1 : -1;
//   int err = dx - dy;

//   int stride_x = sx; 
//   int stride_y = sy * static_cast<int>(size_x);

//   int current_idx = y0 * size_x + x0;

//   int max_x = static_cast<int>(costmap_meta_.size_x);
//   int max_y = static_cast<int>(costmap_meta_.size_y);

//   while (true)
//   {
//     // Safety Guard: Check map boundaries before reading char_map
//     if (x0 < 0 || x0 >= max_x || y0 < 0 || y0 >= max_y) {
//       return false;
//     }

//     if(relax){
//       if (char_map[current_idx] > static_cast<unsigned char>(los_shortcut_cost_limit_+120)) {
//         return false;
//       }
//     }
//     else {
//       if (char_map[current_idx] > static_cast<unsigned char>(los_shortcut_cost_limit_)) {
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
//       current_idx += stride_x; 
//     }
//     if (e2 < dx)  { 
//       err += dx; 
//       y0 += sy; 
//       current_idx += stride_y; 
//     }
//   }

//   return true;
// }


bool TestPlanner::lineOfSight(
  GridNode *start, 
  GridNode *end,
  const unsigned char* char_map,
  unsigned int size_x,
  bool relax) const
{
  int x0 = start->x, y0 = start->y;
  int x1 = end->x, y1 = end->y;

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

GridNode* TestPlanner::get_node_from_pool(int x, int y, int index) {
  GridNode* node = &node_pool_[index];
  if (node_visited_id_[index] != run_id_) 
  {
    node->x = x;
    node->y = y;
    node->g_cost = std::numeric_limits<double>::infinity();
    node->h_cost = 0.0;
    node->f_cost = std::numeric_limits<double>::infinity();
    node->prev = nullptr;
    node->is_in_queue = false;

    node_visited_id_[index] = run_id_;
  }
  return node;
};

void TestPlanner::clearQueue() {
  open_queue_ = std::priority_queue<GridNode*, std::vector<GridNode*>, CompareNode>();
}


std::vector<Dir> TestPlanner::generateDirections(int grid_radius) {
  std::vector<Dir> dirs;
  dirs.reserve((2 * grid_radius + 1) * (2 * grid_radius + 1) - 1);

  for (int dx = -grid_radius; dx <= grid_radius; ++dx) {
      for (int dy = -grid_radius; dy <= grid_radius; ++dy) {
          if (dx == 0 && dy == 0) continue; // Skip center cell
          dirs.push_back({dx, dy, 0.0});
      }
  }

  // Sort by Euclidean distance so immediate neighbors come first
  std::sort(dirs.begin(), dirs.end(), [](const Dir& a, const Dir& b) {
      return (a.dx * a.dx + a.dy * a.dy) < (b.dx * b.dx + b.dy * b.dy);
  });

  return dirs;
};

std::vector<Dir> TestPlanner::generateDirectionRayCasts(int grid_radius) {
  if (grid_radius <= 0) return {};

  std::vector<Dir> dirs;

  // Standard 8 directions: Up, Down, Left, Right + 4 Diagonals
  const Dir baseDirs[8] = {
      { 0,  1, 0.0}, { 0, -1, 0.0}, {-1,  0, 0.0}, { 1,  0, 0.0}, // Orthogonal
      {-1,  1, 0.0}, { 1,  1, 0.0}, {-1, -1, 0.0}, { 1, -1, 0.0}  // Diagonal
  };

  // For Radius 1, return the standard 8 immediate neighbors
  if (grid_radius == 1) {
      return std::vector<Dir>(baseDirs, baseDirs + 8);
  }

  // For Radius R >= 2, fan out each of the 8 main ray axes
  for (const auto& base : baseDirs) {
      // Calculate the core line position at this radius
      int cx = base.dx * grid_radius;
      int cy = base.dy * grid_radius;

      // Add the primary ray tip
      dirs.push_back({cx, cy, 0.0});

      // Add side-spread offsets to widen the ray sweep as it goes deeper
      if (base.dx == 0) {
          // Vertical ray: spread left and right (-X, +X)
          dirs.push_back({cx - 1, cy, 0.0});
      } else if (base.dy == 0) {
          // Horizontal ray: spread up and down (-Y, +Y)
          dirs.push_back({cx, cy - 1, 0.0});
      } else {
          // Diagonal ray: spread horizontally or vertically relative to main angle
          dirs.push_back({cx - base.dx, cy, 0.0}); 
      }
  }

  return dirs;
}


std::vector<Dir> TestPlanner::generateDirectionRing(int grid_radius) {
  // R1: 8cells, R2: 12cells, R3: 16cells, R4: 24cells, R5: 32cells, R10: ~64 cells
  if (grid_radius <= 0) return {};

  std::vector<Dir> dirs;
  int maxR = static_cast<int>(std::ceil(grid_radius));

  // Tolerance range for 1-cell thick circle shell
  float minDistSq = (grid_radius - 0.5f) * (grid_radius - 0.5f);
  float maxDistSq = (grid_radius + 0.5f) * (grid_radius + 0.5f);

  for (int dx = -maxR; dx <= maxR; ++dx) {
      for (int dy = -maxR; dy <= maxR; ++dy) {
          float distSq = static_cast<float>(dx * dx + dy * dy);
          if (distSq >= minDistSq && distSq < maxDistSq) {
              dirs.push_back({dx, dy, 0.0});
          }
      }
  }

  // Sort radially (-PI to +PI) around the center
  std::sort(dirs.begin(), dirs.end(), [](const Dir& a, const Dir& b) {
      return std::atan2(a.dy, a.dx) < std::atan2(b.dy, b.dx);
  });

  return dirs;
}

std::vector<geometry_msgs::msg::PoseStamped>
TestPlanner::addStraightLinePoses(
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
TestPlanner::densifyPath(
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


nav_msgs::msg::Path TestPlanner::smoothPath(
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
TestPlanner::fillUpPath(
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
  test_planner_plugin::TestPlanner,
  nav2_core::GlobalPlanner)
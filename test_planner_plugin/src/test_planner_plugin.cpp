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
    node, name + ".cost_limit", rclcpp::ParameterValue(50));

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".cost_travel_multiplier", rclcpp::ParameterValue(2.0));

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".planner_name", rclcpp::ParameterValue("test"));

  // Retrieve values
  node->get_parameter(name + ".cost_limit", cost_limit_);
  node->get_parameter(name + ".cost_travel_multiplier", cost_travel_multiplier_);
  node->get_parameter(name + ".planner_name", planner_name_);

  cost_limit_ = std::clamp(cost_limit_, 0, 100);

  RCLCPP_INFO_STREAM(
    logger_, 
    "Configured Test Planner Plugin with \ncost_limit=" << cost_limit_ 
    << "\nplanner_name=" << planner_name_);
}





void TestPlanner::activate()
{
  if (!costmap_ros_) {
    return;
  }

  auto costmap = costmap_ros_->getCostmap();
  costmap_meta_.update(costmap);

  RCLCPP_INFO_STREAM(logger_, "Plugin Activated Successfully");
}




void TestPlanner::deactivate() {}




void TestPlanner::cleanup()
{
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
  const unsigned char* char_map = costmap->getCharMap();

  auto start_node = std::make_shared<GridNode>(poseToGrid(start.pose));
  auto goal_node = std::make_shared<GridNode>(poseToGrid(goal.pose));

  // Execute Search

  std::shared_ptr<GridNode> best_goal = nullptr;

 if(planner_name_ == "theta"){
  best_goal = runThetaStarPlan(
    start_node, 
    goal_node, 
    cancel_checker, 
    char_map, 
    costmap_meta_.size_x
  );
 }
 else if(planner_name_ == "lazy-theta"){
  best_goal = runLazyThetaStarPlan(
    start_node, 
    goal_node, 
    cancel_checker, 
    char_map, 
    costmap_meta_.size_x
  );
 }
 else if(planner_name_ == "astar"){
  best_goal = runAStarPlan(
    start_node, 
    goal_node, 
    cancel_checker, 
    char_map, 
    costmap_meta_.size_x,
    false
  );
 }
 else if(planner_name_ == "astar-smooth"){
  best_goal = runAStarPlan(
    start_node, 
    goal_node, 
    cancel_checker, 
    char_map, 
    costmap_meta_.size_x
  );
 }
 else {
  best_goal = runTestPlan(
    start_node, 
    goal_node, 
    cancel_checker, 
    char_map, 
    costmap_meta_.size_x
  );
 }


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

  auto end_time = std::chrono::steady_clock::now();
  std::chrono::duration<double> diff_sec = end_time - start_time;
  RCLCPP_INFO_STREAM(logger_, "planning_time" <<"[" << planner_name_ << "] = " << round_to_3dp(diff_sec.count()*1000.0) << " ms");

  if(planner_name_=="astar") {
    return path;
  }
  else {
    return fillUpPath(path, goal);
  }  
}








std::shared_ptr<GridNode> TestPlanner::runTestPlan(
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

  // Comparator comparing dereferenced shared pointers using your > operator
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

  start_node->h_cost = euclidean_distance(*start_node, *goal_node);
  nodes_to_explore.push(start_node);

  std::shared_ptr<GridNode> active_node = nullptr;

  while (!nodes_to_explore.empty() /*&& rclcpp::ok()*/) {
    if (cancel_checker && cancel_checker()) {
      return nullptr;
    }

    active_node = nodes_to_explore.top();
    nodes_to_explore.pop();
    
    if (visited[gridToMapIndex(*active_node)]) {
      continue;
    }

    if(active_node->prev && active_node->prev->prev){
      auto grandparent = active_node->prev->prev;
      if (lineOfSight(*active_node, *grandparent, char_map, size_x)) {
          active_node->prev = grandparent;
          active_node->g_cost = grandparent->g_cost + euclidean_distance(*active_node, *grandparent);
      }
    }

    visited[gridToMapIndex(*active_node)] = true;

    if (*active_node == *goal_node) {
      return active_node;
    }

    for (const auto &dir : explore_directions) {
      GridNode neighbor_pos = *active_node + dir.dir; 

      if (!visited[gridToMapIndex(neighbor_pos)] && isGridOnMap(neighbor_pos) && isMapCellFree(neighbor_pos, char_map)) {
        
        auto new_node = std::make_shared<GridNode>(neighbor_pos);
        new_node->prev = active_node;

        if (active_node->prev){
          new_node->g_cost = active_node->prev->g_cost 
                            + (euclidean_distance(*new_node, *(active_node->prev)) * getGridCost(neighbor_pos, char_map));
        } else {
          new_node->g_cost = active_node->g_cost 
                            + (dir.t_cost * getGridCost(neighbor_pos, char_map));
        }

        new_node->h_cost = euclidean_distance(*new_node, *goal_node);
        nodes_to_explore.push(new_node);
      }
    }
  }

  return nullptr;
}






std::shared_ptr<GridNode> TestPlanner::runThetaStarPlan(
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

  // Comparator comparing dereferenced shared pointers using your > operator
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

  start_node->h_cost = euclidean_distance(*start_node, *goal_node);
  nodes_to_explore.push(start_node);

  std::shared_ptr<GridNode> active_node = nullptr;

  while (!nodes_to_explore.empty() /*&& rclcpp::ok()*/) {
    if (cancel_checker && cancel_checker()) {
      return nullptr;
    }

    active_node = nodes_to_explore.top();
    nodes_to_explore.pop();
    
    if (visited[gridToMapIndex(*active_node)]) {
      continue;
    }
    visited[gridToMapIndex(*active_node)] = true;

    if (*active_node == *goal_node) {
      return active_node;
    }

    for (const auto &dir : explore_directions) {
      GridNode neighbor_pos = *active_node + dir.dir; 

      if (!visited[gridToMapIndex(neighbor_pos)] && isGridOnMap(neighbor_pos) && isMapCellFree(neighbor_pos, char_map)) {
        
        auto new_node = std::make_shared<GridNode>(neighbor_pos);

        // Check Theta* Line of Sight
        if (active_node->prev && lineOfSight(*new_node, *(active_node->prev), char_map, size_x)) {
          // Point directly to grandparent (preserves real memory pointer)
          new_node->prev = active_node->prev;
          new_node->g_cost = active_node->prev->g_cost 
                           + (euclidean_distance(*new_node, *(active_node->prev)) * getGridCost(neighbor_pos, char_map));
        } else {
          // Standard step to parent
          new_node->prev = active_node;
          new_node->g_cost = active_node->g_cost 
                           + (dir.t_cost * getGridCost(neighbor_pos, char_map));
        }

        new_node->h_cost = euclidean_distance(*new_node, *goal_node);
        nodes_to_explore.push(new_node);
      }
    }
  }

  return nullptr;
}



std::shared_ptr<GridNode> TestPlanner::runAStarPlan(
  std::shared_ptr<GridNode> start_node,
  std::shared_ptr<GridNode> goal_node,
  const std::function<bool()>& cancel_checker,
  const unsigned char* char_map,
  unsigned int size_x,
  bool smooth)
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

  // Comparator comparing dereferenced shared pointers using your > operator
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

  start_node->h_cost = euclidean_distance(*start_node, *goal_node);
  nodes_to_explore.push(start_node);

  std::shared_ptr<GridNode> active_node = nullptr;

  while (!nodes_to_explore.empty() /*&& rclcpp::ok()*/) {
    if (cancel_checker && cancel_checker()) {
      return nullptr;
    }

    active_node = nodes_to_explore.top();
    nodes_to_explore.pop();
    
    if (visited[gridToMapIndex(*active_node)]) {
      continue;
    }
    visited[gridToMapIndex(*active_node)] = true;

    if (*active_node == *goal_node) {
      if (smooth){
        return greedyStringPullSmooth(
          active_node,
          cancel_checker, 
          char_map, 
          size_x);
      }
      else {
        return active_node;
      }
    }

    for (const auto &dir : explore_directions) {
      GridNode neighbor_pos = *active_node + dir.dir; 

      if (!visited[gridToMapIndex(neighbor_pos)] && isGridOnMap(neighbor_pos) && isMapCellFree(neighbor_pos, char_map)) {
        
        auto new_node = std::make_shared<GridNode>(neighbor_pos);

        new_node->prev = active_node;
        new_node->g_cost = active_node->g_cost 
                          + (dir.t_cost * getGridCost(neighbor_pos, char_map));
        new_node->h_cost = euclidean_distance(*new_node, *goal_node);
        nodes_to_explore.push(new_node);
      }
    }
  }

  return nullptr;
}




std::shared_ptr<GridNode> TestPlanner::greedyStringPullSmooth(
  std::shared_ptr<GridNode> grid_node_path,
  const std::function<bool()>& cancel_checker,
  const unsigned char* char_map,
  unsigned int size_x)
{
  if (!grid_node_path) {
    return nullptr;
  }

  // 1. Unwind the linked list path from Goal -> Start into a vector
  std::vector<std::shared_ptr<GridNode>> poses;
  std::shared_ptr<GridNode> current = grid_node_path;
  
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

  std::vector<std::shared_ptr<GridNode>> smoothed_poses;
  smoothed_poses.push_back(poses[0]);

  while (true) {
    if (cancel_checker && cancel_checker()) {
      return nullptr;
    }

    if (!(j < n)) {
      break;
    }
    else if (lineOfSight(*poses[i], *poses[j], char_map, size_x)) {
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

  std::shared_ptr<GridNode> smoothed_grid_node_path = smoothed_poses.back();
  return smoothed_grid_node_path;
}






//--------------- LAZY THETA STAR ------------------------------


std::shared_ptr<GridNode> TestPlanner::runLazyThetaStarPlan(
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
    nodes_to_explore.pop();

    // Skip closed nodes or stale pointers
    if (visited[gridToMapIndex(*active_node)]) {
      continue;
    }

    if (node_lookup[gridToMapIndex(*active_node)] && active_node->g_cost > node_lookup[gridToMapIndex(*active_node)]->g_cost) {
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

          if (visited[gridToMapIndex(neighbor_pos)] && isGridOnMap(neighbor_pos) && isMapCellFree(neighbor_pos, char_map)) {
            if (node_lookup[gridToMapIndex(neighbor_pos)] != nullptr) {
              auto neighbor_node = node_lookup[gridToMapIndex(neighbor_pos)];

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
          visited[gridToMapIndex(*active_node)] = true;
          continue;
        }
      }
    }

    // Mark node as closed/visited
    visited[gridToMapIndex(*active_node)] = true;

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

      if (!visited[gridToMapIndex(neighbor_pos)] && isGridOnMap(neighbor_pos) && isMapCellFree(neighbor_pos, char_map)){
        // Optimistic assumption: Try active_node's parent if present, else active_node
        std::shared_ptr<GridNode> optimistic_parent = (active_node->prev != nullptr) 
                                                      ? active_node->prev 
                                                      : active_node;

        double dist = euclidean_distance(*optimistic_parent, neighbor_pos);
        double grid_cost_factor = getGridCost(neighbor_pos, char_map);
        double new_cost = optimistic_parent->g_cost + (dist * grid_cost_factor);

        auto neighbor_node = node_lookup[gridToMapIndex(neighbor_pos)];

        if (!neighbor_node || new_cost < neighbor_node->g_cost) {
          if (!neighbor_node) {
            neighbor_node = std::make_shared<GridNode>(neighbor_pos);
          }

          neighbor_node->g_cost = new_cost;
          neighbor_node->h_cost = euclidean_distance(*neighbor_node, *goal_node);
          neighbor_node->prev = optimistic_parent;

          node_lookup[gridToMapIndex(neighbor_pos)] = neighbor_node;
          nodes_to_explore.push(neighbor_node);
        }
      }
    }
  }

  return nullptr;
}

//--------------------------------------------------------------








GridNode TestPlanner::poseToGrid(const geometry_msgs::msg::Pose &pose)
{
  int gx = static_cast<int>((pose.position.x - costmap_meta_.origin_x) * costmap_meta_.inv_resolution);
  int gy = static_cast<int>((pose.position.y - costmap_meta_.origin_y) * costmap_meta_.inv_resolution);

  return GridNode(gx, gy);
}

geometry_msgs::msg::Pose TestPlanner::gridToPose(const GridNode &grid)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = grid.x * costmap_meta_.resolution + costmap_meta_.origin_x;
  pose.position.y = grid.y * costmap_meta_.resolution + costmap_meta_.origin_y;
  pose.position.z = 0.0;

  return pose;
}

int TestPlanner::gridToMapIndex(const GridNode &grid_node)
{
  return static_cast<int>(grid_node.y * costmap_meta_.size_x + grid_node.x);
}

bool TestPlanner::isGridOnMap(const GridNode &grid)
{
  return (grid.x >= 0 && grid.x < costmap_meta_.size_x &&
          grid.y >= 0 && grid.y < costmap_meta_.size_y);
}

double TestPlanner::getGridCost(const GridNode &grid, const unsigned char* char_map)
{
  return  1.0+(cost_travel_multiplier_ * std::clamp(static_cast<double>(char_map[gridToMapIndex(grid)]) / 252.0, 0.0, 1.0));
}

bool TestPlanner::isMapCellFree(const GridNode &grid, const unsigned char* char_map)
{
  // RCLCPP_INFO_STREAM(
  //   logger_, "cell_cost = " << static_cast<int>(char_map[gridToMapIndex(grid)]));
  return /*(char_map[gridToMapIndex(grid)] >= 0) &&*/ (char_map[gridToMapIndex(grid)] < static_cast<unsigned char>(cost_limit_+120));
}


double TestPlanner::euclidean_distance(const GridNode &a, const GridNode &b){
  return std::hypot(a.x - b.x, a.y - b.y);
}


bool TestPlanner::lineOfSight(
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
      if (char_map[current_idx] > static_cast<unsigned char>(cost_limit_+120)) {
        return false;
      }
    }
    else {
      if (char_map[current_idx] > static_cast<unsigned char>(cost_limit_)) {
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
TestPlanner::fillUpPath(
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
  test_planner_plugin::TestPlanner,
  nav2_core::GlobalPlanner)
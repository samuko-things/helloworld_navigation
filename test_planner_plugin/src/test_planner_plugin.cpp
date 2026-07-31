#include "test_planner_plugin/test_planner_plugin.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <algorithm>

namespace test_planner_plugin
{


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
    node, name + ".cost_limit", rclcpp::ParameterValue(99.0));

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".planner_name", rclcpp::ParameterValue("test"));

  // Retrieve values
  node->get_parameter(name + ".cost_limit", cost_limit_);
  node->get_parameter(name + ".planner_name", planner_name_);

  RCLCPP_INFO_STREAM(
    logger_, 
    "Configured Test Planner Plugin with cost_limit=" << cost_limit_ 
    << ", planner_name=" << planner_name_);
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

  // Atomically refresh metadata struct
  costmap_meta_.update(costmap);
  const unsigned char* char_map = costmap->getCharMap();

  auto start_node = std::make_shared<GridNode>(poseToGrid(start.pose));
  auto goal_node = std::make_shared<GridNode>(poseToGrid(goal.pose));

  // Execute Search

  std::shared_ptr<GridNode> best_goal = nullptr;

//  if(planner_name_ == "lazy-theta"){
//   best_goal = runLazyThetaStarPlan(
//     start_node, 
//     goal_node, 
//     cancel_checker, 
//     char_map, 
//     costmap_meta_.size_x
//   );
//  }
 if(planner_name_ == "theta"){
  best_goal = runThetaStarPlan(
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
  RCLCPP_INFO_STREAM(logger_, "planning_time" <<"[" << planner_name_ << "] = " << static_cast<int>(diff_sec.count()*1000000) << " us");

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
    
    int active_idx = gridToMapIndex(*active_node);
    if (visited[active_idx]) {
      continue;
    }

    if(active_node->prev && active_node->prev->prev){
      auto grandparent = active_node->prev->prev;
      if (lineOfSight(*active_node, *grandparent, char_map, size_x)) {
          active_node->prev = grandparent;
          active_node->g_cost = grandparent->g_cost + euclidean_distance(*active_node, *grandparent);
      }
    }

    visited[active_idx] = true;

    if (*active_node == *goal_node) {
      return active_node;
    }

    for (const auto &dir : explore_directions) {
      GridNode neighbor_pos = *active_node + dir.dir; 
      int neighbor_idx = gridToMapIndex(neighbor_pos);

      if (!visited[neighbor_idx] && isGridOnMap(neighbor_pos) && isMapCellFree(neighbor_pos, char_map)) {
        
        auto new_node = std::make_shared<GridNode>(neighbor_pos);
        new_node->prev = active_node;

        if (active_node->prev){
          new_node->g_cost = active_node->prev->g_cost 
                            + euclidean_distance(*new_node, *(active_node->prev)) 
                            + char_map[neighbor_idx];
        } else {
          new_node->g_cost = active_node->g_cost 
                            + dir.t_cost
                            + char_map[neighbor_idx];
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
    
    int active_idx = gridToMapIndex(*active_node);
    if (visited[active_idx]) {
      continue;
    }
    visited[active_idx] = true;

    if (*active_node == *goal_node) {
      return active_node;
    }

    for (const auto &dir : explore_directions) {
      GridNode neighbor_pos = *active_node + dir.dir; 
      int neighbor_idx = gridToMapIndex(neighbor_pos);

      if (!visited[neighbor_idx] && isGridOnMap(neighbor_pos) && isMapCellFree(neighbor_pos, char_map)) {
        
        auto new_node = std::make_shared<GridNode>(neighbor_pos);

        // Check Theta* Line of Sight
        if (active_node->prev && lineOfSight(*new_node, *(active_node->prev), char_map, size_x)) {
          // Point directly to grandparent (preserves real memory pointer)
          new_node->prev = active_node->prev;
          new_node->g_cost = active_node->prev->g_cost 
                           + euclidean_distance(*new_node, *(active_node->prev)) 
                           + char_map[neighbor_idx];
        } else {
          // Standard step to parent
          new_node->prev = active_node;
          new_node->g_cost = active_node->g_cost 
                           + dir.t_cost 
                           + char_map[neighbor_idx];
        }

        new_node->h_cost = euclidean_distance(*new_node, *goal_node);
        nodes_to_explore.push(new_node);
      }
    }
  }

  return nullptr;
}











// std::shared_ptr<GridNode> TestPlanner::runLazyThetaStarPlan(
//   std::shared_ptr<GridNode> start_node,
//   std::shared_ptr<GridNode> goal_node,
//   const std::function<bool()>& cancel_checker,
//   const unsigned char* char_map,
//   unsigned int size_x)
// {
//   std::vector<DirNode> explore_directions = {
//       DirNode({-1, 0}, 1.0), 
//       DirNode({1, 0}, 1.0), 
//       DirNode({0, 1}, 1.0), 
//       DirNode({0, -1}, 1.0),
//       DirNode({-1, 1}, 1.4142), 
//       DirNode({1, -1}, 1.4142), 
//       DirNode({1, 1}, 1.4142), 
//       DirNode({-1, -1}, 1.4142),
//   };

//   auto comp = [](const std::shared_ptr<GridNode>& a, const std::shared_ptr<GridNode>& b) {
//     return *a > *b;
//   };

//   std::priority_queue<
//     std::shared_ptr<GridNode>, 
//     std::vector<std::shared_ptr<GridNode>>, 
//     decltype(comp)
//   > nodes_to_explore(comp);

//   int map_size = costmap_meta_.size_x * costmap_meta_.size_y;
//   std::vector<bool> visited(map_size, false);

//   // Map to store persistent node instances across grid locations (matches nodes_map in Python)
//   std::unordered_map<int, std::shared_ptr<GridNode>> nodes_map;

//   start_node->g_cost = 0.0;
//   start_node->h_cost = euclidean_distance(*start_node, *goal_node);
//   start_node->prev = start_node; // Self-reference for root

//   int start_idx = gridToMapIndex(*start_node);
//   nodes_map[start_idx] = start_node;
//   nodes_to_explore.push(start_node);

//   std::shared_ptr<GridNode> active_node = nullptr;

//   while (!nodes_to_explore.empty() /*&& rclcpp::ok()*/) {
//     if (cancel_checker && cancel_checker()) {
//       return nullptr;
//     }

//     active_node = nodes_to_explore.top();
//     nodes_to_explore.pop();

//     int active_idx = gridToMapIndex(*active_node);

//     if (visited[active_idx]) {
//       continue;
//     }

//     // --------------------------------------------------
//     // LAZY VALIDATION PHASE
//     // --------------------------------------------------
//     if (active_node->prev && active_node->prev != active_node) {
//       // Check if line of sight to optimistic parent is invalid
//       if (!lineOfSight(*active_node, *(active_node->prev), char_map, size_x)) {
//         double min_g = std::numeric_limits<double>::infinity();
//         std::shared_ptr<GridNode> best_parent = nullptr;

//         // Recalculate cost using valid, already-explored (visited) neighbors
//         for (const auto &dir : explore_directions) {
//           GridNode neighbor_pos = *active_node + dir.dir;
//           int neighbor_idx = gridToMapIndex(neighbor_pos);

//           auto it = nodes_map.find(neighbor_idx);
//           if (it != nodes_map.end()) {
//             auto neighbor_node = it->second;
//             if (visited[neighbor_idx]) {
//               double cost_to_current = neighbor_node->g_cost + dir.t_cost;
//               if (cost_to_current < min_g) {
//                 min_g = cost_to_current;
//                 best_parent = neighbor_node;
//               }
//             }
//           }
//         }

//         if (best_parent) {
//           active_node->g_cost = min_g;
//           active_node->prev = best_parent;
//           // Re-queue so it re-sorts in the priority queue under its corrected cost
//           nodes_to_explore.push(active_node);
//           continue;
//         } else {
//           visited[active_idx] = true;
//           continue;
//         }
//       }
//     }

//     // --------------------------------------------------
//     // GOAL CHECK
//     // --------------------------------------------------
//     if (*active_node == *goal_node) {
//       return active_node;
//     }

//     visited[active_idx] = true;

//     // --------------------------------------------------
//     // EXPAND NEIGHBORS (OPTIMISTIC STEP)
//     // --------------------------------------------------
//     for (const auto &dir : explore_directions) {
//       GridNode neighbor_pos = *active_node + dir.dir;
//       int neighbor_idx = gridToMapIndex(neighbor_pos);

//       // Retrieve existing persistent node or initialize a new one
//       if (nodes_map.find(neighbor_idx) == nodes_map.end()) {
//         auto new_node = std::make_shared<GridNode>(neighbor_pos);
//         new_node->g_cost = std::numeric_limits<double>::infinity();
//         nodes_map[neighbor_idx] = new_node;
//       }

//       auto neighbor_node = nodes_map[neighbor_idx];

//       if (!visited[neighbor_idx] && 
//           isGridOnMap(neighbor_pos) && 
//           isMapCellFree(neighbor_pos, char_map)) 
//       {
//         // Optimistic assumption: line of sight from parent to neighbor exists
//         auto parent = active_node->prev ? active_node->prev : active_node;
//         double new_cost = parent->g_cost + euclidean_distance(*parent, *neighbor_node);

//         if (new_cost < neighbor_node->g_cost) {
//           neighbor_node->g_cost = new_cost;
//           neighbor_node->h_cost = euclidean_distance(*neighbor_node, *goal_node);
//           neighbor_node->prev = parent;

//           nodes_to_explore.push(neighbor_node);
//         }
//       }
//     }
//   }

//   return nullptr;
// }





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
    
    int active_idx = gridToMapIndex(*active_node);
    if (visited[active_idx]) {
      continue;
    }
    visited[active_idx] = true;

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
      int neighbor_idx = gridToMapIndex(neighbor_pos);

      if (!visited[neighbor_idx] && isGridOnMap(neighbor_pos) && isMapCellFree(neighbor_pos, char_map)) {
        
        auto new_node = std::make_shared<GridNode>(neighbor_pos);

        new_node->prev = active_node;
        new_node->g_cost = active_node->g_cost 
                          + dir.t_cost 
                          + char_map[neighbor_idx];
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

bool TestPlanner::isMapCellFree(const GridNode &grid, const unsigned char* char_map)
{
  return /*(char_map[gridToMapIndex(grid)] >= 0) &&*/ (char_map[gridToMapIndex(grid)] < static_cast<int>(cost_limit_));
}


double TestPlanner::euclidean_distance(const GridNode &a, const GridNode &b){
  return std::hypot(a.x - b.x, a.y - b.y);
}


bool TestPlanner::lineOfSight(
  const GridNode &start, 
  const GridNode &end,
  const unsigned char* char_map,
  unsigned int size_x) const
{
  int x0 = start.x; int y0 = start.y;
  int x1 = end.x; int y1 = end.y;

  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  // Calculate pre-computed memory offsets (strides)
  int stride_x = sx; 
  int stride_y = sy * static_cast<int>(size_x);

  // Compute the starting flat memory index
  int current_idx = y0 * size_x + x0;

  while (true)
  {
    // Direct, ultra-fast array lookup bypassing getCost() overhead
    if (char_map[current_idx] > static_cast<int>(cost_limit_))
      return false;

    if (x0 == x1 && y0 == y1)
      break;

    int e2 = 2 * err;
    if (e2 > -dy) { 
      err -= dy; 
      x0 += sx; 
      current_idx += stride_x; // Direct index stride addition
    }
    if (e2 < dx)  { 
      err += dx; 
      y0 += sy; 
      current_idx += stride_y; // Direct index stride addition
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
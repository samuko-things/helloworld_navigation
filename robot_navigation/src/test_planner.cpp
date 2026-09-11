#include "robot_navigation/test_planner.hpp"
#include <algorithm>

#include <chrono>
#include <cmath>

namespace test_planner
{




double round_to_3dp(double val) {
    return std::round(val * 1000.0) / 1000.0;
}


TestPlanner::TestPlanner() : Node("test_planner")
{
  declare_parameter<int>("planner_id", planner_id_);
  planner_id_ = get_parameter("planner_id").as_int();

  declare_parameter<double>("dist_to_obstacle_check", dist_to_obstacle_check_);
  dist_to_obstacle_check_ = get_parameter("dist_to_obstacle_check").as_double();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  rclcpp::QoS default_qos(10);

  rclcpp::QoS map_qos(10);
  map_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  map_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map",
    map_qos,
    std::bind(&TestPlanner::mapCallback, this, std::placeholders::_1)
  );

  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose",
    default_qos,
    std::bind(&TestPlanner::goalCallback, this, std::placeholders::_1)
  );

  path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "/test_planner/path",
    default_qos
  );

  map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/test_planner/visited_map",
    default_qos
  );

  // Pre-allocate vector pools once on activation
  node_pool_.resize(10);
  node_visited_id_.resize(10, 0);

  node2_pool_.resize(10);
  node2_initialized_.resize(10);
  g_cost_cache_.resize(10);
  visited_.resize(10);

  RCLCPP_INFO_STREAM(logger_, "TestPlanner Node Has Started Successfully");
}








void TestPlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg){
  map_ = map_msg;
  map_meta_data_.update(map_);

  visited_map_.header.frame_id = map_->header.frame_id;
  visited_map_.info = map_->info;
  visited_map_.data = std::vector<int8_t>(map_meta_data_.size_x*map_meta_data_.size_y, -1);

  const int8_t* char_map = map_->data.data();
  preprocessObstacleProximity(char_map, dist_to_obstacle_check_);

  RCLCPP_INFO_STREAM(logger_, "Map Loaded Successfully");

}










void TestPlanner::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal){
  
  if(!map_){
    RCLCPP_ERROR(logger_, "No Map Received");
    return;
  }

  // get the current pose of the robot i the map
  geometry_msgs::msg::TransformStamped robot_pose_in_map_tf;
  try {
    robot_pose_in_map_tf = tf_buffer_->lookupTransform(
      map_->header.frame_id,
      "base_link",
      tf2::TimePointZero
    );
  }
  catch(tf2::TransformException & e) {
    RCLCPP_ERROR_STREAM(get_logger(), 
    "Could not get robot_pose(base_link) in map transformation: " << e.what());
    return;
  }

  // convert transfrm to pose_msg
  geometry_msgs::msg::PoseStamped robot_pose_in_map;
  robot_pose_in_map.header.frame_id = goal->header.frame_id;

  robot_pose_in_map.pose.position.x = robot_pose_in_map_tf.transform.translation.x;
  robot_pose_in_map.pose.position.y = robot_pose_in_map_tf.transform.translation.y;
  robot_pose_in_map.pose.position.z = robot_pose_in_map_tf.transform.translation.z;
  robot_pose_in_map.pose.orientation = robot_pose_in_map_tf.transform.rotation;


  nav_msgs::msg::Path path = plan(robot_pose_in_map, *goal);

  if(!path.poses.empty()){
    RCLCPP_INFO_STREAM(get_logger(), "Shortest Path Found");
    path_pub_->publish(path);
  }
  else {
    RCLCPP_WARN_STREAM(get_logger(), "No Path Found To Goal");
  }

}











nav_msgs::msg::Path TestPlanner::plan(
  const geometry_msgs::msg::PoseStamped &start,
  const geometry_msgs::msg::PoseStamped &goal)
{
  if(!map_){
    RCLCPP_ERROR(logger_, "No Map Received");
    return nav_msgs::msg::Path();
  }

  map_meta_data_.update(map_);
  size_t current_map_size = map_meta_data_.size_x * map_meta_data_.size_y;

  visited_map_.data = std::vector<int8_t>(current_map_size, 0);

  // Fallback memory check
  if (node_pool_.size() != current_map_size) {
    node_pool_.resize(current_map_size);
    node_visited_id_.resize(current_map_size);

    node2_pool_.resize(current_map_size);
    node2_initialized_.resize(current_map_size);
    g_cost_cache_.resize(current_map_size);
    visited_.resize(current_map_size);
  }

  // --- FAST FLAT MEMORY RESETS ---
  std::fill(node_visited_id_.begin(),node_visited_id_.end(), 0);

  std::fill(node2_initialized_.begin(), node2_initialized_.end(), false);
  std::fill(visited_.begin(), visited_.end(), false);
  std::fill(g_cost_cache_.begin(), g_cost_cache_.end(), -1.0);

  const int8_t* char_map = map_->data.data();

  run_id_++;

  // Start & Goal Node Setup
  GridNode raw_start = poseToGrid(start.pose);
  GridNode* start_node = get_node_from_pool(raw_start.x, raw_start.y);

  GridNode raw_goal = poseToGrid(goal.pose);
  GridNode* goal_node = get_node_from_pool(raw_goal.x, raw_goal.y);

  auto start_time = std::chrono::high_resolution_clock::now();

  size_t los_checks;
  size_t los_checks_attempted;
  double los_check_time;
  size_t node_expansions;
  size_t fallback_count;
  size_t successful_parent_collapses;


  GridNode* best_goal;

  if (planner_id_ == 0) {
    best_goal = runLazyTheta(
      start_node, 
      goal_node, 
      char_map, 
      map_meta_data_.size_x,
      los_check_time,
      los_checks,
      los_checks_attempted,
      node_expansions,
      fallback_count,
      successful_parent_collapses
    );

    RCLCPP_INFO_STREAM(
    logger_,
    "\n================ LAZY THETA BENCHMARK ================");
  }

  else {
    best_goal = runLazyThetaSkipLOS(
      start_node, 
      goal_node, 
      char_map, 
      map_meta_data_.size_x,
      los_check_time,
      los_checks,
      los_checks_attempted,
      node_expansions,
      fallback_count,
      successful_parent_collapses
    );

    RCLCPP_INFO_STREAM(
    logger_,
    "\n================ LAZY THETA WITH LOS SKIP BENCHMARK ================");
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  double execution_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

  double path_length = 0.0;
  size_t waypoints_count = 0;

  if (best_goal != nullptr)
  {
    GridNode* curr = best_goal;
    waypoints_count = 1; // Count goal node

    while (curr->parent && curr->parent != curr)
    {
      GridNode* parent = curr->parent;

      // Euclidean distance between consecutive waypoints in the parent chain
      double dx = static_cast<double>(curr->x - parent->x);
      double dy = static_cast<double>(curr->y - parent->y);
      path_length += std::hypot(dx, dy);

      waypoints_count++;
      curr = parent;
    }
  }

  RCLCPP_INFO_STREAM(
  logger_,
  "\n  Execution Time               : " << execution_time_ms << " ms" <<
  "\n  Path Length (Grid Units)     : " << path_length <<
  "\n  Waypoints Count              : " << waypoints_count <<
  "\n  Line-of-Sight Attempted      : " << los_checks_attempted <<
  "\n  Line-of-Sight Checks         : " << los_checks <<
  "\n  Line-of-Sight Checks Time    : " << los_check_time << "ms" <<
  "\n  Node Expansions              : " << node_expansions <<
  "\n  Fallback Trigger Count       : " << fallback_count <<
  "\n  Successful Parent Collapses  : " << successful_parent_collapses <<
  "\n========================================================");


  // Path Reconstruction
  nav_msgs::msg::Path path;
  path.header.frame_id = map_->header.frame_id;

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

    if (node->parent == node) {
      break;
    }
    node = node->parent;
  }

  std::reverse(path.poses.begin(), path.poses.end());
  return fillUpPath(path, goal, false);
}










GridNode* TestPlanner::runLazyTheta(
  GridNode* start_node,
  GridNode* goal_node,
  const int8_t* char_map,
  unsigned int size_x,
  double & los_check_time,
  size_t & los_checks,
  size_t & los_checks_attempted,
  size_t & node_expansions,
  size_t & fallback_count,
  size_t & successful_parent_collapses)
{
  // Reset counters for this run
  los_check_time = 0.0;
  los_checks = 0;
  los_checks_attempted = 0;
  node_expansions = 0;
  fallback_count = 0;
  successful_parent_collapses = 0; // Always 0 for Lazy Theta* (Grandparent collapsing is DRSP-specific)

  start_node->g_cost = 0.0;
  start_node->h_cost = euclidean_distance(*start_node, *goal_node);
  start_node->is_in_queue = true;
  start_node->grid_parent = start_node;
  start_node->parent = start_node;

  open_queue_.push(start_node);

  while (!open_queue_.empty())
  {
    GridNode* current = open_queue_.top();
    open_queue_.pop();

    current->is_in_queue = false;

    // Lazy Theta*: SetVertex Step (Verify line-of-sight to parent upon pop)
    if (current->parent && current->parent != current)
    {
      auto start_time = std::chrono::high_resolution_clock::now();
      ++los_checks_attempted;
      // visited_map_.data.at(gridToMapIndex(*current)) = 10;
      // map_pub_->publish(visited_map_);
      ++los_checks; // LOS metric counter
      if (lineOfSight(current, current->parent, char_map, size_x))
      {
        // visited_map_.data.at(gridToMapIndex(*current)) = 10;
        // map_pub_->publish(visited_map_);
        double current_g_cost = current->parent->g_cost + euclidean_distance(*current, *current->parent);
        if (current_g_cost < current->g_cost)
        {
          current->g_cost = current_g_cost;
          successful_parent_collapses++;
        }
      }
      else 
      {
        ++fallback_count;
        current->parent = current->grid_parent;
        current->g_cost = current->grid_parent->g_cost + euclidean_distance(*current, *current->grid_parent);
      }
      auto end_time = std::chrono::high_resolution_clock::now();
      los_check_time += std::chrono::duration<double, std::milli>(end_time - start_time).count();
    }

    if (current->x == goal_node->x && current->y == goal_node->y) {
      clearQueue();
      return current;
    }
    
    node_expansions++; // Node Expansion metric counter

    // UpdateVertex Expansion Phase
    for (const auto & d : dirs)
    {
      int nx = current->x + d.dx;
      int ny = current->y + d.dy;

      GridNode* nbr_node = get_node_from_pool(nx, ny);

      if (isGridOnMap(*nbr_node) && isMapCellFree(*nbr_node, char_map)) {
        double g_cost = current->g_cost + d.dist;
        double h_cost = euclidean_distance(*nbr_node, *goal_node);
        double f_cost = g_cost + h_cost;

        if ((nbr_node->g_cost + nbr_node->h_cost) > f_cost)
        {
          nbr_node->g_cost = g_cost;
          nbr_node->h_cost = h_cost;
          nbr_node->parent = current->parent;
          nbr_node->grid_parent = current;

          if(!nbr_node->is_in_queue){
            nbr_node->is_in_queue = true;
            open_queue_.push(nbr_node);
          }
        }
        // visited_map_.data.at(gridToMapIndex(*current)) = 10;
        // map_pub_->publish(visited_map_);
      }
    }

  }

  return nullptr;
}







GridNode* TestPlanner::runLazyThetaSkipLOS(
  GridNode* start_node,
  GridNode* goal_node,
  const int8_t* char_map,
  unsigned int size_x,
  double & los_check_time,
  size_t & los_checks,
  size_t & los_checks_attempted,
  size_t & node_expansions,
  size_t & fallback_count,
  size_t & successful_parent_collapses)
{
  // Reset counters for this run
  los_check_time = 0.0;
  los_checks = 0;
  los_checks_attempted = 0;
  node_expansions = 0;
  fallback_count = 0;
  successful_parent_collapses = 0; // Always 0 for Lazy Theta* (Grandparent collapsing is DRSP-specific)

  start_node->g_cost = 0.0;
  start_node->h_cost = euclidean_distance(*start_node, *goal_node);
  start_node->is_in_queue = true;
  start_node->grid_parent = start_node;
  start_node->parent = start_node;

  open_queue_.push(start_node);

  while (!open_queue_.empty())
  {
    GridNode* current = open_queue_.top();
    open_queue_.pop();

    current->is_in_queue = false;

    // Lazy Theta*: SetVertex Step (Verify line-of-sight to parent upon pop)
    if (current->parent && current->parent != current)
    {
      auto start_time = std::chrono::high_resolution_clock::now();
      ++los_checks_attempted;
      if(isNodeCloseToObstacle(*current) || isNodeDirectionChanged(current))
      {
        // visited_map_.data.at(gridToMapIndex(*current)) = 10;
        // map_pub_->publish(visited_map_);
        ++los_checks; // LOS metric counter
        if (lineOfSight(current, current->parent, char_map, size_x))
        {
          // visited_map_.data.at(gridToMapIndex(*current)) = 10;
          // map_pub_->publish(visited_map_);
          double current_g_cost = current->parent->g_cost + euclidean_distance(*current, *current->parent);
          if (current_g_cost < current->g_cost)
          {
            current->g_cost = current_g_cost;
            successful_parent_collapses++;
          }
        }
        else 
        {
          ++fallback_count;
          current->parent = current->grid_parent;
          current->g_cost = current->grid_parent->g_cost + euclidean_distance(*current, *current->grid_parent);
        }
      }
      auto end_time = std::chrono::high_resolution_clock::now();
      los_check_time += std::chrono::duration<double, std::milli>(end_time - start_time).count();
    }

    if (current->x == goal_node->x && current->y == goal_node->y) {
      clearQueue();
      return current;
    }
    
    node_expansions++; // Node Expansion metric counter

    // UpdateVertex Expansion Phase
    for (const auto & d : dirs)
    {
      int nx = current->x + d.dx;
      int ny = current->y + d.dy;

      GridNode* nbr_node = get_node_from_pool(nx, ny);

      if (isGridOnMap(*nbr_node) && isMapCellFree(*nbr_node, char_map)) {
        double g_cost = current->g_cost + d.dist;
        double h_cost = euclidean_distance(*nbr_node, *goal_node);
        double f_cost = g_cost + h_cost;

        if ((nbr_node->g_cost + nbr_node->h_cost) > f_cost)
        {
          nbr_node->g_cost = g_cost;
          nbr_node->h_cost = h_cost;
          nbr_node->parent = current->parent;
          nbr_node->grid_parent = current;

          if(!nbr_node->is_in_queue){
            nbr_node->is_in_queue = true;
            open_queue_.push(nbr_node);
          }
        }
        // visited_map_.data.at(gridToMapIndex(*current)) = 10;
        // map_pub_->publish(visited_map_);
      }
    }

  }

  return nullptr;
}





GridNode* TestPlanner::get_node_from_pool(int x, int y) {
  int index = gridToMapIndex(x, y);
  GridNode* node = &node_pool_[index];
  if (node_visited_id_[index] != run_id_) 
  {
    node->x = x;
    node->y = y;
    node->g_cost = std::numeric_limits<double>::infinity();
    node->h_cost = 0.0;
    node->parent = nullptr;
    node->grid_parent = nullptr;
    node->is_in_queue = false;

    node_visited_id_[index] = run_id_;
  }
  return node;
};





















bool TestPlanner::isCloseToObstacle(
    const int8_t* char_map,
    int cx,
    int cy,
    double clearance_m) const
{
  int sweep_dist_cells = static_cast<int>(std::ceil(clearance_m * map_meta_data_.inv_resolution));
  const int rad = sweep_dist_cells;

  if (!isMapCellFree(cx, cy, char_map))
    return true;

  for (int dx = -rad; dx <= rad; ++dx)
  {
    for (int dy = -rad; dy <= rad; ++dy)
    {
      if (!isGridOnMap(cx + dx, cy + dy))
        continue;

      if (!isMapCellFree(cx + dx, cy + dy, char_map))
        return true;
    }
  }

  return false;
}


GridNode TestPlanner::poseToGrid(const geometry_msgs::msg::Pose &pose) const
{
  int gx = static_cast<int>((pose.position.x - map_meta_data_.origin_x) * map_meta_data_.inv_resolution);
  int gy = static_cast<int>((pose.position.y - map_meta_data_.origin_y) * map_meta_data_.inv_resolution);

  return GridNode(gx, gy);
}

geometry_msgs::msg::Pose TestPlanner::gridToPose(const GridNode &grid) const
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = grid.x * map_meta_data_.resolution + map_meta_data_.origin_x;
  pose.position.y = grid.y * map_meta_data_.resolution + map_meta_data_.origin_y;
  pose.position.z = 0.0;

  return pose;
}

int TestPlanner::gridToMapIndex(const GridNode &grid_node) const
{
  return static_cast<int>(grid_node.y * map_meta_data_.size_x + grid_node.x);
}

int TestPlanner::gridToMapIndex(const int x, const int y) const
{
  return static_cast<int>(y * map_meta_data_.size_x + x);
}

bool TestPlanner::isGridOnMap(const GridNode &grid) const
{
  return (grid.x >= 0 && grid.x < map_meta_data_.size_x &&
          grid.y >= 0 && grid.y < map_meta_data_.size_y);
}

bool TestPlanner::isGridOnMap(const int x, const int y) const
{
  return (x >= 0 && x < map_meta_data_.size_x &&
          y >= 0 && y < map_meta_data_.size_y);
}

bool TestPlanner::isMapCellFree(const GridNode &grid, const int8_t* char_map) const
{
  return (char_map[gridToMapIndex(grid)] == 0);
}

bool TestPlanner::isMapCellFree(const int x, const int y, const int8_t* char_map) const
{
  return (char_map[gridToMapIndex(x, y)] == 0);
}

double TestPlanner::euclidean_distance(const GridNode &a, const GridNode &b) const{
  double dx = static_cast<double>(a.x - b.x);
  double dy = static_cast<double>(a.y - b.y);
  return std::sqrt(dx * dx + dy * dy);
}


bool TestPlanner::lineOfSight(
  GridNode *current,
  GridNode *previous,
  const int8_t* char_map,
  unsigned int size_x) const
{
  int x0 = current->x, y0 = current->y;
  int x1 = previous->x, y1 = previous->y;

  int dx = std::abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
  int dy = std::abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
  int cx = x0, cy = y0, e = dx - dy;

  int max_x = static_cast<int>(map_meta_data_.size_x);
  int max_y = static_cast<int>(map_meta_data_.size_y);

  auto isSafe = [&](int x, int y) -> bool {
    if (x < 0 || x >= max_x || y < 0 || y >= max_y) {
      return false;
    }
    int idx = y * static_cast<int>(size_x) + x;
    return char_map[idx] == 0;
  };

  while (cx != x1 || cy != y1) {
    if (!isSafe(cx, cy)) {
      return false;
    }

    int e2 = 2 * e;
    if (e2 > -dy && e2 <= dx) {
      if (!isSafe(cx + sx, cy) || !isSafe(cx, cy + sy)) {
        return false;
      }
      cx += sx;
      cy += sy;
      e += dx - dy;
    } else if (e2 > -dy) {
      cx += sx;
      e -= dy;
    } else {
      cy += sy;
      e += dx;
    }
  }

  // Final check for the destination node (x1, y1)
  return isSafe(x1, y1);
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
      map_meta_data_.resolution);

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
      const auto & parent = smoothed.poses[i - 1].pose.position;
      const auto & curr = smoothed.poses[i].pose.position;
      const auto & next = smoothed.poses[i + 1].pose.position;
      const auto & orig = orig_poses[i];

      // Calculate Gradient Descent displacement terms
      double rx = orig.x - curr.x;
      double ry = orig.y - curr.y;

      double sx = parent.x + next.x - (2.0 * curr.x);
      double sy = parent.y + next.y - (2.0 * curr.y);

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




























































// GridNode* TestPlanner::runLazyTheta(
//   GridNode* start_node,
//   GridNode* goal_node,
//   const int8_t* char_map,
//   unsigned int size_x,
//   double & los_check_time,
//   size_t & los_checks,
//   size_t & los_checks_attempted,
//   size_t & node_expansions,
//   size_t & fallback_count,
//   size_t & successful_parent_collapses)
// {
//   // Reset counters for this run
//   los_check_time = 0.0;
//   los_checks = 0;
//   los_checks_attempted = 0;
//   node_expansions = 0;
//   fallback_count = 0;
//   successful_parent_collapses = 0; // Always 0 for Lazy Theta* (Grandparent collapsing is DRSP-specific)

//   int sid = gridToMapIndex(*start_node);

//   start_node->g_cost = 0.0;
//   start_node->h_cost = euclidean_distance(*start_node, *goal_node);
//   start_node->parent = start_node;

//   open_queue_.push(start_node);
//   g_cost_cache_[sid] = 0.0;

//   while (!open_queue_.empty() /*&& rclcpp::ok()*/)
//   {

//     GridNode* current = open_queue_.top();
//     open_queue_.pop();

//     int cid = gridToMapIndex(*current);

//     if (visited_[cid]) {
//       continue;
//     }

//     if (current->g_cost > g_cost_cache_[cid]) {
//       continue;
//     }

//     // --------------------------------------------------
//     // LAZY VALIDATION / SETVERTEX PHASE
//     // --------------------------------------------------
//     if (current->parent && current->parent != current)
//     {
//       auto start_time = std::chrono::high_resolution_clock::now();

//       ++los_checks_attempted;
//       ++los_checks;
//       if (!lineOfSight(current, current->parent, char_map, size_x))
//       {
//         ++fallback_count;
//         double min_g = std::numeric_limits<double>::infinity();
//         GridNode* best_parent = nullptr;

//         for (const auto & d : dirs)
//         {
//           int nx = current->x + d.dx;
//           int ny = current->y + d.dy;
//           int nid = gridToMapIndex(nx, ny);

//           if (isGridOnMap(nx, ny) && isMapCellFree(nx, ny, char_map) && visited_[nid])
//           {
//             GridNode* nbr_node = get_node_from_pool2(nx, ny);
//             double g_val = g_cost_cache_[nid];
//             double cost_to_active = g_val + d.dist;
//             if (cost_to_active < min_g) {
//               min_g = cost_to_active;
//               best_parent = nbr_node;
//             }
//           }
//         }

//         if (best_parent) {
//           current->g_cost = min_g;
//           g_cost_cache_[cid] = min_g;
//           current->parent = best_parent;
//         } 
//         // else {
//         //   visited_[cid] = true;
//         //   continue; 
//         // }
//       }

//       auto end_time = std::chrono::high_resolution_clock::now();
//       los_check_time += std::chrono::duration<double, std::milli>(end_time - start_time).count();
//     }

//     if (current->x == goal_node->x && current->y == goal_node->y) {
//       clearQueue();
//       return current; // Path extraction target
//     }

//     visited_[cid] = true;

//     ++node_expansions;
//     // --------------------------------------------------
//     // EXPAND NEIGHBORS (OPTIMISTIC UPDATE)
//     // --------------------------------------------------
//     for (const auto & d : dirs)
//     {
//       int nx = current->x + d.dx;
//       int ny = current->y + d.dy;
//       int nid = gridToMapIndex(nx, ny);

//       if (isGridOnMap(nx, ny) && isMapCellFree(nx, ny, char_map) && !visited_[nid]) {
//         GridNode* optimistic_parent = current->parent ? current->parent : current;
//         GridNode* nbr_node = get_node_from_pool2(nx, ny);

//         double optimistic_parent_g = g_cost_cache_[gridToMapIndex(*optimistic_parent)];

//         double dist = euclidean_distance(*optimistic_parent, *nbr_node);
//         double new_cost = optimistic_parent_g + dist;

//         double current_nbr_cost = g_cost_cache_[nid];
//         if (current_nbr_cost < 0.0 || new_cost < current_nbr_cost)
//         {
//           g_cost_cache_[nid] = new_cost;

//           nbr_node->g_cost = new_cost;
//           nbr_node->h_cost = euclidean_distance(*nbr_node, *goal_node);
//           nbr_node->parent = optimistic_parent;

//           open_queue_.push(nbr_node);
//         }
//       }
//     }
//   }

//   return nullptr;
// }









// GridNode* TestPlanner::runLazyThetaSkipLOS(
//   GridNode* start_node,
//   GridNode* goal_node,
//   const int8_t* char_map,
//   unsigned int size_x,
//   double & los_check_time,
//   size_t & los_checks,
//   size_t & los_checks_attempted,
//   size_t & node_expansions,
//   size_t & fallback_count,
//   size_t & successful_parent_collapses)
// {
//   // Reset counters for this run
//   los_checks = 0;
//   los_checks_attempted = 0;
//   node_expansions = 0;
//   fallback_count = 0;
//   successful_parent_collapses = 0; // Always 0 for Lazy Theta* (Grandparent collapsing is DRSP-specific)

//   int sid = gridToMapIndex(*start_node);

//   start_node->g_cost = 0.0;
//   start_node->h_cost = euclidean_distance(*start_node, *goal_node);
//   start_node->parent = start_node;
//   start_node->grid_parent = start_node;

//   open_queue_.push(start_node);
//   g_cost_cache_[sid] = 0.0;

//   while (!open_queue_.empty() /*&& rclcpp::ok()*/)
//   {

//     GridNode* current = open_queue_.top();
//     open_queue_.pop();

//     int cid = gridToMapIndex(*current);

//     if (visited_[cid]) {
//       continue;
//     }

//     if (current->g_cost > g_cost_cache_[cid]) {
//       continue;
//     }

//     // --------------------------------------------------
//     // LAZY VALIDATION / SETVERTEX PHASE
//     // --------------------------------------------------
//     if (current->parent && current->parent != current)
//     {

//       auto start_time = std::chrono::high_resolution_clock::now();

//       ++los_checks_attempted;
//       if(isNodeCloseToObstacle(*current) || isNodeDirectionChanged(current)){
//         ++los_checks;
//         if (!lineOfSight(current, current->parent, char_map, size_x))
//         {
//           ++fallback_count;
//           double min_g = std::numeric_limits<double>::infinity();
//           GridNode* best_parent = nullptr;

//           for (const auto & d : dirs)
//           {
//             int nx = current->x + d.dx;
//             int ny = current->y + d.dy;
//             int nid = gridToMapIndex(nx, ny);

//             if (isGridOnMap(nx, ny) && isMapCellFree(nx, ny, char_map) && visited_[nid])
//             {
//               GridNode* nbr_node = get_node_from_pool2(nx, ny);
//               double g_val = g_cost_cache_[nid];
//               double cost_to_active = g_val + d.dist;
//               if (cost_to_active < min_g) {
//                 min_g = cost_to_active;
//                 best_parent = nbr_node;
//               }
//             }
//           }

//           if (best_parent) {
//             current->g_cost = min_g;
//             g_cost_cache_[cid] = min_g;
//             current->parent = best_parent;
//             current->grid_parent = best_parent;
//           } 
//           // else {
//           //   visited_[cid] = true;
//           //   continue; 
//           // }
//         }
//       }

//       auto end_time = std::chrono::high_resolution_clock::now();
//       los_check_time += std::chrono::duration<double, std::milli>(end_time - start_time).count();
//     }

//     if (current->x == goal_node->x && current->y == goal_node->y) {
//       clearQueue();
//       return current; // Path extraction target
//     }

//     visited_[cid] = true;

//     ++node_expansions;
//     // --------------------------------------------------
//     // EXPAND NEIGHBORS (OPTIMISTIC UPDATE)
//     // --------------------------------------------------
//     for (const auto & d : dirs)
//     {
//       int nx = current->x + d.dx;
//       int ny = current->y + d.dy;
//       int nid = gridToMapIndex(nx, ny);

//       if (isGridOnMap(nx, ny) && isMapCellFree(nx, ny, char_map) && !visited_[nid]) {
//         GridNode* optimistic_parent = current->parent ? current->parent : current;
//         GridNode* nbr_node = get_node_from_pool2(nx, ny);

//         double optimistic_parent_g = g_cost_cache_[gridToMapIndex(*optimistic_parent)];

//         double dist = euclidean_distance(*optimistic_parent, *nbr_node);
//         double new_cost = optimistic_parent_g + dist;

//         double current_nbr_cost = g_cost_cache_[nid];
//         if (current_nbr_cost < 0.0 || new_cost < current_nbr_cost)
//         {
//           g_cost_cache_[nid] = new_cost;

//           nbr_node->g_cost = new_cost;
//           nbr_node->h_cost = euclidean_distance(*nbr_node, *goal_node);
//           nbr_node->parent = optimistic_parent;
//           nbr_node->grid_parent = current;

//           open_queue_.push(nbr_node);
//         }
//       }
//     }
//   }

//   return nullptr;
// }








// GridNode* TestPlanner::get_node_from_pool2(int x, int y) {
//   int index = gridToMapIndex(x, y);
//   GridNode* node = &node2_pool_[index];
//   if (!node2_initialized_[index]) 
//   {
//     node->x = x;
//     node->y = y;
//     node->g_cost = std::numeric_limits<double>::infinity();
//     node->h_cost = 0.0;
//     node->parent = nullptr;
//     node->grid_parent = nullptr;
//     node->is_in_queue = false;

//     node2_initialized_[index] = true;
//   }
//   return node;
// };
































// GridNode* TestPlanner::runAStar(
//   GridNode* start_node,
//   GridNode* goal_node,
//   const int8_t* char_map,
//   unsigned int size_x,
//   double & los_check_time,
//   size_t & los_checks,
//   size_t & los_checks_attempted,
//   size_t & node_expansions,
//   size_t & fallback_count,
//   size_t & successful_parent_collapses,
//   bool smooth)
// {
//   // Reset counters for this run
//   los_check_time = 0.0;
//   los_checks = 0;
//   los_checks_attempted = 0;
//   node_expansions = 0;
//   fallback_count = 0;
//   successful_parent_collapses = 0; // Always 0 for Lazy Theta* (Grandparent collapsing is DRSP-specific)

//   start_node->g_cost = 0.0;
//   start_node->h_cost = euclidean_distance(*start_node, *goal_node);
//   start_node->is_in_queue = true;
//   start_node->parent = start_node;

//   open_queue_.push(*start_node);

//   while (!open_queue_.empty())
//   {
//     GridNode current_node = open_queue_.top();
//     open_queue_.pop();
//     update_node_pool(current_node);
//     GridNode* current = get_node_from_pool(current_node.x, current_node.y);

//     current->is_in_queue = false;

//     if (current->x == goal_node->x && current->y == goal_node->y) {
//       if(smooth){
//         clearQueue();
//         return greedyStringPullSmooth(
//           current,
//           char_map, 
//           size_x,
//           los_check_time,
//           los_checks,
//           los_checks_attempted,
//           fallback_count,
//           successful_parent_collapses
//         );
//       }
//       clearQueue();
//       return current;
//     }
    
//     node_expansions++; // Node Expansion metric counter

//     // UpdateVertex Expansion Phase
//     for (const auto & d : dirs)
//     {
//       int nx = current->x + d.dx;
//       int ny = current->y + d.dy;

//       GridNode* nbr_node = get_node_from_pool(nx, ny);

//       if (isGridOnMap(*nbr_node) && isMapCellFree(*nbr_node, char_map)) {
//         double g_cost = current->g_cost + d.dist;
//         double h_cost = euclidean_distance(*nbr_node, *goal_node);
//         double f_cost = g_cost + h_cost;

//         if ((nbr_node->g_cost + nbr_node->h_cost) > f_cost)
//         {
//           nbr_node->g_cost = g_cost;
//           nbr_node->h_cost = h_cost;
//           nbr_node->parent = current;

//           if(!nbr_node->is_in_queue){
//             nbr_node->is_in_queue = true;
//             open_queue_.push(*nbr_node);
//           }
//         }

//         // visited_map_.data.at(gridToMapIndex(*current)) = 10;
//         // map_pub_->publish(visited_map_);
//       }
//     }

//   }

//   return nullptr;
// }







// GridNode* TestPlanner::greedyStringPullSmooth(
//   GridNode* grid_node_path,
//   const int8_t* char_map,
//   unsigned int size_x,
//   double & los_check_time,
//   size_t & los_checks,
//   size_t & los_checks_attempted,
//   size_t & fallback_count,
//   size_t & successful_parent_collapses)
// {
//   if (!grid_node_path) {
//     return nullptr;
//   }

//   // 1. Unwind the linked list path from Goal -> Start into a vector
//   std::vector<GridNode*> poses;
//   GridNode* current = grid_node_path;
  
//   while (current != nullptr) {
//     poses.push_back(current);
    
//     if (current->parent == current || current->parent == nullptr) {
//       break;
//     }
//     current = current->parent;
//   }

//   std::reverse(poses.begin(), poses.end());

//   int n = static_cast<int>(poses.size());

//   if (n <= 2) {
//     return grid_node_path;
//   }

//   int i = 0;
//   int j = 2;

//   std::vector<GridNode*> smoothed_poses;
//   smoothed_poses.push_back(poses[0]);

//   while (true) {
//     if (!(j < n)) {
//       break;
//     }

//     auto start_time = std::chrono::high_resolution_clock::now();
//     ++los_checks_attempted;
//     if(isNodeCloseToObstacle(*poses[j]) || isNodeDirectionChanged2(poses[j])){
//       ++los_checks;
//       if (lineOfSight(poses[j], poses[i], char_map, size_x)) {
//         j = j;
//         ++successful_parent_collapses;
//       } else {
//         ++fallback_count;
//         i = j - 1;
        
//         poses[i]->parent = smoothed_poses.back();
//         smoothed_poses.push_back(poses[i]);
//       }
//     }

//     auto end_time = std::chrono::high_resolution_clock::now();
//     los_check_time += std::chrono::duration<double, std::milli>(end_time - start_time).count();

//     j += 1;
//   }

//   i = j - 1;
//   poses[i]->parent = smoothed_poses.back();
//   smoothed_poses.push_back(poses[i]);

//   GridNode* smoothed_grid_node_path = smoothed_poses.back();
//   return smoothed_grid_node_path;
// }




}  // namespace test_planner



int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<test_planner::TestPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#include "robot_navigation/test_planner.hpp"
#include <algorithm>

namespace test_planner
{

double round_to_3dp(double val) {
    return std::round(val * 1000.0) / 1000.0;
}


TestPlanner::TestPlanner() : Node("test_planner")
{
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
  node_initialized_.resize(10, false);
  g_cost_cache_.resize(10, -1.0);
  visited_.resize(10, false);

  RCLCPP_INFO_STREAM(logger_, "TestPlanner Node Has Started Successfully");
}

void TestPlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg){
  map_ = map_msg;
  map_meta_data_.update(map_);

  visited_map_.header.frame_id = map_->header.frame_id;
  visited_map_.info = map_->info;
  visited_map_.data = std::vector<int8_t>(map_meta_data_.size_x*map_meta_data_.size_y, -1);

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

  visited_map_.data = std::vector<int8_t>(current_map_size, -1);

  // Fallback memory check
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

  const int8_t* char_map = map_->data.data();

  // Start & Goal Node Setup
  GridNode raw_start = poseToGrid(start.pose);
  int start_idx = gridToMapIndex(raw_start);
  GridNode* start_node = get_node_from_pool(raw_start.x, raw_start.y, start_idx);

  GridNode raw_goal = poseToGrid(goal.pose);
  int goal_idx = gridToMapIndex(raw_goal);
  GridNode* goal_node = get_node_from_pool(raw_goal.x, raw_goal.y, goal_idx);

  // Execute Search

  GridNode* best_goal = runDRSPPlan(
    start_node, 
    goal_node, 
    char_map, 
    map_meta_data_.size_x
  );
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

    if (node->prev == node) {
      break;
    }
    node = node->prev;
  }

  std::reverse(path.poses.begin(), path.poses.end());
  return fillUpPath(path, goal, false);
}


GridNode* TestPlanner::runDRSPPlan(
  GridNode* start_node,
  GridNode* goal_node,
  const int8_t* char_map,
  unsigned int size_x)
{
  std::priority_queue<
    GridNode*,
    std::vector<GridNode*>,
    TestPlanner::CompareNode
  > nodes_to_explore;

  int start_idx = gridToMapIndex(*start_node);

  start_node->g_cost = 0.0;
  start_node->h_cost = euclidean_distance(*start_node, *goal_node);
  start_node->prev = start_node;
  // start_node->near_obstacle = true;

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

      if (lineOfSight(active, actual_grandparent, char_map, size_x))
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

          if (isGridOnMap(nbr_pos) && isMapCellFree(nbr_pos, char_map) && isFreeWithClearance(char_map, nx, ny) && visited_[nbr_idx])
          {
            GridNode* nbr_node = &node_pool_[nbr_idx];
            double g_val = g_cost_cache_[nbr_idx];
            double cost_to_active = g_val + d.dist;
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

      if (isGridOnMap(nbr_pos) && isMapCellFree(nbr_pos, char_map) && isFreeWithClearance(char_map, nx, ny) && !visited_[nbr_idx]) {
        GridNode* assumed_grandparent = active->prev ? active->prev : active;
        GridNode* neighbor_node = get_node_from_pool(nx, ny, nbr_idx);

        double new_g_cost = assumed_grandparent->g_cost + euclidean_distance(nbr_pos, *(assumed_grandparent));
        double nbr_g_cost = g_cost_cache_[nbr_idx];

        if (nbr_g_cost < 0.0 || new_g_cost < nbr_g_cost)
        {
          g_cost_cache_[nbr_idx] = new_g_cost;

          neighbor_node->g_cost = new_g_cost;
          neighbor_node->h_cost = euclidean_distance(*neighbor_node, *goal_node);
          neighbor_node->prev = active;

          nodes_to_explore.push(neighbor_node);
        }

        visited_map_.data.at(gridToMapIndex(*active)) = 10;
        map_pub_->publish(visited_map_); 
      }
    }
  }

  return nullptr;
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

bool TestPlanner::isGridOnMap(const GridNode &grid) const
{
  return (grid.x >= 0 && grid.x < map_meta_data_.size_x &&
          grid.y >= 0 && grid.y < map_meta_data_.size_y);
}

bool TestPlanner::isMapCellFree(const GridNode &grid, const int8_t* char_map) const
{
  return (char_map[gridToMapIndex(grid)] == 0);
}

double TestPlanner::euclidean_distance(const GridNode &a, const GridNode &b) const{
  double dx = static_cast<double>(a.x - b.x);
  double dy = static_cast<double>(a.y - b.y);
  return std::sqrt(dx * dx + dy * dy);
}

bool TestPlanner::isFreeWithClearance(
    const int8_t* char_map,
    int cx, int cy,
    double clearance) const
{
  int clearance_cells = clearance / map_meta_data_.resolution;
  int rad = std::ceil(clearance_cells);
  double clearance_sq = clearance_cells * clearance_cells;

  for (int dx = -rad; dx <= rad; ++dx) {
      for (int dy = -rad; dy <= rad; ++dy) {
          GridNode neighbor{cx + dx, cy + dy};

          if (!isGridOnMap(neighbor)) { continue; }

          if ((dx * dx + dy * dy <= clearance_sq) && !isMapCellFree(neighbor, char_map)) {
              return false;
          }
      }
  }

  return true;
}


bool TestPlanner::lineOfSight(
  GridNode *start,
  GridNode *end,
  const int8_t* char_map,
  unsigned int size_x) const
{

  int x0 = start->x; int y0 = start->y; 
  int x1 = end->x; int y1 = end->y;

  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  int stride_x = sx; 
  int stride_y = sy * static_cast<int>(size_x);

  int active_idx = y0 * size_x + x0;

  int max_x = static_cast<int>(map_meta_data_.size_x);
  int max_y = static_cast<int>(map_meta_data_.size_y);

  while (true)
  {
    // Safety Guard: Check map boundaries before reading char_map
    if (x0 < 0 || x0 >= max_x || y0 < 0 || y0 >= max_y) {
      return false;
    }

    if ((char_map[active_idx] != 0) || !isFreeWithClearance(char_map, x0, y0)) {
      return false;
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


GridNode* TestPlanner::get_node_from_pool(int x, int y, int index) {
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


}  // namespace test_planner



int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<test_planner::TestPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
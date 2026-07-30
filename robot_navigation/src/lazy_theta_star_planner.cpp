#include "robot_navigation/lazy_theta_star_planner.hpp"
#include <chrono>


namespace lazy_theta_star_planner
{

LazyThetaStarPlanner::LazyThetaStarPlanner() : Node("lazy_theta_star_planner")
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  rclcpp::QoS default_qos(10);

  rclcpp::QoS map_qos(10);
  map_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  map_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap",
    map_qos,
    std::bind(&LazyThetaStarPlanner::mapCallback, this, std::placeholders::_1)
  );

  goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose",
    default_qos,
    std::bind(&LazyThetaStarPlanner::goalPoseCallback, this, std::placeholders::_1)
  );

  path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "/theta_star/path",
    default_qos
  );

  map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "theta_star/visited_map",
    default_qos
  );

  RCLCPP_INFO_STREAM(get_logger(), "LazyThetaStarPlanner Node Has Started Successfully");

}

void LazyThetaStarPlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
 map_ = map;
 visited_map_.header.frame_id = map->header.frame_id;
 visited_map_.info = map_->info;
 visited_map_.data = std::vector<int8_t>(map_->info.width * map_->info.height, -1);

//  RCLCPP_INFO_STREAM(get_logger(), "Map Recieved");
}

void LazyThetaStarPlanner::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
 if(!map_){
  RCLCPP_ERROR(get_logger(), "No Map Received");
 }

 // reset the visited map
 visited_map_.data = std::vector<int8_t>(map_->info.width * map_->info.height, -1);

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
 geometry_msgs::msg::Pose robot_pose_in_map;
 robot_pose_in_map.position.x = robot_pose_in_map_tf.transform.translation.x;
 robot_pose_in_map.position.y = robot_pose_in_map_tf.transform.translation.y;
 robot_pose_in_map.position.z = robot_pose_in_map_tf.transform.translation.z;
 robot_pose_in_map.orientation = robot_pose_in_map_tf.transform.rotation;


 nav_msgs::msg::Path path = plan(robot_pose_in_map, pose->pose);

 if(!path.poses.empty()){
  RCLCPP_INFO_STREAM(get_logger(), "Shortest Path Found");
  path_pub_->publish(path);
 }
 else {
  RCLCPP_WARN_STREAM(get_logger(), "No Path Found To Goal");
 }

}

nav_msgs::msg::Path LazyThetaStarPlanner::plan(const geometry_msgs::msg::Pose &start, const geometry_msgs::msg::Pose &goal)
{
  auto start_time = std::chrono::steady_clock::now();

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

  int map_size = map_->info.width * map_->info.height;
  std::vector<bool> visited(map_size, false);

  auto start_node = std::make_shared<GridNode>(poseToGridNode(start));
  auto goal_node = std::make_shared<GridNode>(poseToGridNode(goal));

  start_node->h_cost = euclidean_distance(*start_node, *goal_node);
  nodes_to_explore.push(start_node);

  std::shared_ptr<GridNode> active_node = nullptr;

  while (!nodes_to_explore.empty() && rclcpp::ok()) {
    active_node = nodes_to_explore.top();
    nodes_to_explore.pop();
    
    int active_idx = gridNodeToMapIndex(*active_node);
    if (visited[active_idx]) {
      continue;
    }

    // LAZY THETA STAR -> try to shortcut active node to active.prev.prev (its grand parent)
    if(active_node->prev && active_node->prev->prev){
      auto grandparent = active_node->prev->prev;
      if (lineOfSight(*active_node, *grandparent)) {
          active_node->prev = grandparent;
          active_node->g_cost = grandparent->g_cost + euclidean_distance(*active_node, *grandparent);
      }
    }

    visited[active_idx] = true;

    if (*active_node == *goal_node) {
      break;
    }

    // -------- NEIGBHOR EXPANSION -------------
    for (const auto &dir : explore_directions) {
      // Uses your GridNode + std::pair operator!
      GridNode neighbor_pos = *active_node + dir.dir; 
      int neighbor_idx = gridNodeToMapIndex(neighbor_pos);

      if (!visited[neighbor_idx] && isGridNodeOnMap(neighbor_pos) && isMapCellFree(neighbor_pos)) {
        
        auto new_node = std::make_shared<GridNode>(neighbor_pos);

        // Always set initial candidate parent to active_node
        new_node->prev = active_node;

        // Optimistic cost: Assume we can shortcut through active_node's parent if available
        if (active_node->prev){
          new_node->g_cost = active_node->prev->g_cost 
                            + euclidean_distance(*new_node, *(active_node->prev)) 
                            + map_->data.at(neighbor_idx);
        } else {
          new_node->g_cost = active_node->g_cost 
                            + dir.t_cost
                            + map_->data.at(neighbor_idx);
        }

        new_node->h_cost = euclidean_distance(*new_node, *goal_node);
        nodes_to_explore.push(new_node);
      }
    }
  }

  auto end_time = std::chrono::steady_clock::now();

  std::chrono::duration<double> diff_sec = end_time - start_time;
  RCLCPP_INFO_STREAM(this->get_logger(), "planning_time = " << static_cast<int>(diff_sec.count()*1000) << " ms");

  // Reconstruction
  nav_msgs::msg::Path path;
  path.header.frame_id = map_->header.frame_id;
  path.header.stamp = now();

  auto curr_node = active_node;
  while (curr_node && rclcpp::ok()) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = map_->header.frame_id;
    pose_stamped.header.stamp = path.header.stamp;
    pose_stamped.pose = gridNodeToPose(*curr_node);
    
    path.poses.push_back(pose_stamped);
    curr_node = curr_node->prev; // Traces shared_ptr back safely
  }

  std::reverse(path.poses.begin(), path.poses.end());
  return path;
}

GridNode LazyThetaStarPlanner::poseToGridNode(const geometry_msgs::msg::Pose &pose)
{
  int grid_x =static_cast<int>((pose.position.x - map_->info.origin.position.x) / map_->info.resolution);
  int grid_y =static_cast<int>((pose.position.y - map_->info.origin.position.y) / map_->info.resolution);
  return GridNode(grid_x, grid_y);
}

geometry_msgs::msg::Pose LazyThetaStarPlanner::gridNodeToPose(const GridNode &grid_node)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = grid_node.x * map_->info.resolution + map_->info.origin.position.x;
  pose.position.y = grid_node.y * map_->info.resolution + map_->info.origin.position.y;
  return pose;
}

bool LazyThetaStarPlanner::isGridNodeOnMap(const GridNode &grid_node)
{
  return (grid_node.x >=0 && grid_node.x < static_cast<int>(map_->info.width)) && 
    (grid_node.y >=0 && grid_node.y < static_cast<int>(map_->info.height));
}

bool LazyThetaStarPlanner::isMapCellFree(const GridNode &grid_node)
{
  return (map_->data.at(gridNodeToMapIndex(grid_node)) >= 0) && (map_->data.at(gridNodeToMapIndex(grid_node)) < 99);
}

int LazyThetaStarPlanner::gridNodeToMapIndex(const GridNode &grid_node)
{
  return static_cast<int>(grid_node.y * map_->info.width + grid_node.x);
}

double LazyThetaStarPlanner::euclidean_distance(const GridNode &a, const GridNode &b){
  return std::hypot(a.x - b.x, a.y - b.y);
}

double LazyThetaStarPlanner::octile_distance(const GridNode &a, const GridNode &b){
  int dx = std::abs(a.x - b.x);
  int dy = std::abs(a.y - b.y);
  return (dx + dy) - 0.58578644 * std::min(dx, dy);
}

bool LazyThetaStarPlanner::lineOfSight(const GridNode &start, const GridNode &end)
{
  int x0 = start.x; int y0 = start.y;
  int x1 = end.x; int y1 = end.y;

  // Retrieve width directly from the OccupancyGrid metadata
  const unsigned int size_x = map_->info.width;
  const auto &data = map_->data; // Reference to std::vector<int8_t>

  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  // Memory offsets (strides) for moving 1 cell in X or Y
  int stride_x = sx; 
  int stride_y = sy * static_cast<int>(size_x);

  // Initial flat 1D array index
  int current_idx = y0 * size_x + x0;

  while (true)
  {
    if (data.at(current_idx) != 0)
      return false;

    if (x0 == x1 && y0 == y1)
      break;

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


}




int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lazy_theta_star_planner::LazyThetaStarPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
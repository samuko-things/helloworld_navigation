#include "robot_navigation/a_star_smooth_planner.hpp"


namespace robot_navigation
{

AStarSmoothPlanner::AStarSmoothPlanner() : Node("a_star_smooth_planner")
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
    std::bind(&AStarSmoothPlanner::mapCallback, this, std::placeholders::_1)
  );

  goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose",
    default_qos,
    std::bind(&AStarSmoothPlanner::goalPoseCallback, this, std::placeholders::_1)
  );

  path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "/a_star/path",
    default_qos
  );

  map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "a_star/visited_map",
    default_qos
  );

  RCLCPP_INFO_STREAM(get_logger(), "AStarSmoothPlanner Has Started Successfully");

}

void AStarSmoothPlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
 map_ = map;
 visited_map_.header.frame_id = map->header.frame_id;
 visited_map_.info = map_->info;
 visited_map_.data = std::vector<int8_t>(map_->info.width * map_->info.height, -1);

 RCLCPP_INFO_STREAM(get_logger(), "Map Recieved");
}

void AStarSmoothPlanner::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
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

nav_msgs::msg::Path AStarSmoothPlanner::plan(const geometry_msgs::msg::Pose &start, const geometry_msgs::msg::Pose &goal)
{
  std::vector<DirNode> explore_directions = {
    //(x_dir, y_dir, t_cost)
      DirNode({-1, 0}, 1), 
      DirNode({1, 0}, 1), 
      DirNode({0, 1}, 1), 
      DirNode({0, -1}, 1),
      DirNode({-1, 1}, 1.4142), 
      DirNode({1, -1}, 1.4142), 
      DirNode({1, 1}, 1.4142), 
      DirNode({-1, -1}, 1.4142),
  };
  
  // min-heap config. put the lowest at the top
  std::priority_queue<GridNode, std::vector<GridNode>, std::greater<GridNode>> nodes_to_explore;
  std::vector<GridNode> visited_nodes;

  GridNode start_node = poseToGridNode(start);
  GridNode goal_node = poseToGridNode(goal);

  start_node.h_cost = euclidean_distance(start_node, goal_node);

  nodes_to_explore.push(start_node);
  GridNode active_node;

  while (!nodes_to_explore.empty() && rclcpp::ok()){
    active_node = nodes_to_explore.top();
    nodes_to_explore.pop();

    if (active_node == poseToGridNode(goal)){
      break;
    }

    for (const auto &dir : explore_directions) {
      GridNode new_node = active_node + dir.dir;
      if (
        std::find(visited_nodes.begin(), visited_nodes.end(), new_node) == visited_nodes.end()
        && isGridNodeOnMap(new_node)
        && isMapCellFree(new_node)
        // && !isGridNodeCloseToObstacle(new_node)
      ) {
        new_node.g_cost = active_node.g_cost + dir.t_cost + map_->data.at(gridNodeToMapIndex(new_node));
        new_node.h_cost = euclidean_distance(new_node, goal_node);
        new_node.prev = std::make_shared<GridNode>(active_node);
        nodes_to_explore.push(new_node);
        visited_nodes.push_back(new_node);
      }
    }

    visited_map_.data.at(gridNodeToMapIndex(active_node)) = 10;
    map_pub_->publish(visited_map_); 

  }

  nav_msgs::msg::Path path;
  path.header.frame_id = map_->header.frame_id;
  while(active_node.prev && rclcpp::ok()) {
    // RCLCPP_INFO_STREAM(get_logger(), "\nnode (" << active_node.x << "," << active_node.y << ")");
    geometry_msgs::msg::Pose last_pose = gridNodeToPose(active_node);
    geometry_msgs::msg::PoseStamped last_pose_stamped;
    last_pose_stamped.header.frame_id = map_->header.frame_id;
    last_pose_stamped.pose = last_pose;
    path.poses.push_back(last_pose_stamped);
    active_node = *active_node.prev;
  }
  std::reverse(path.poses.begin(), path.poses.end());
  // return path;
  nav_msgs::msg::Path smooth_path;
  smooth_path = greedyStringPullSmoothIter(path, 5);
  // smooth_path = smoothAndDensifyPath(smooth_path);
  return smooth_path;
}

GridNode AStarSmoothPlanner::poseToGridNode(const geometry_msgs::msg::Pose &pose)
{
  int grid_x =static_cast<int>((pose.position.x - map_->info.origin.position.x) / map_->info.resolution);
  int grid_y =static_cast<int>((pose.position.y - map_->info.origin.position.y) / map_->info.resolution);
  return GridNode(grid_x, grid_y);
}

geometry_msgs::msg::Pose AStarSmoothPlanner::gridNodeToPose(const GridNode &grid_node)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = grid_node.x * map_->info.resolution + map_->info.origin.position.x;
  pose.position.y = grid_node.y * map_->info.resolution + map_->info.origin.position.y;
  return pose;
}

bool AStarSmoothPlanner::isGridNodeOnMap(const GridNode &grid_node)
{
  return (grid_node.x >=0 && grid_node.x < static_cast<int>(map_->info.width)) && 
    (grid_node.y >=0 && grid_node.y < static_cast<int>(map_->info.height));
}

bool AStarSmoothPlanner::isMapCellFree(const GridNode &grid_node)
{
  return (map_->data.at(gridNodeToMapIndex(grid_node)) >= 0) && (map_->data.at(gridNodeToMapIndex(grid_node)) < 99);
}

int AStarSmoothPlanner::gridNodeToMapIndex(const GridNode &grid_node)
{
  return static_cast<int>(grid_node.y * map_->info.width + grid_node.x);
}

// bool AStarSmoothPlanner::isGridNodeCloseToObstacle(const GridNode &grid_node)
// {
//   return false;
// }

double AStarSmoothPlanner::euclidean_distance(const GridNode &a, const GridNode &b){
  return std::hypot(a.x - b.x, a.y - b.y);
}

double AStarSmoothPlanner::octile_distance(const GridNode &a, const GridNode &b){
  int dx = std::abs(a.x - b.x);
  int dy = std::abs(a.y - b.y);
  return (dx + dy) - 0.58578644 * std::min(dx, dy);
}

bool AStarSmoothPlanner::lineOfSight(GridNode start, GridNode end, int8_t threshold)
{
  if (!map_) return false;

  const int size_x = static_cast<int>(map_->info.width);
  const int size_y = static_cast<int>(map_->info.height);

  int x0 = start.x; int y0=start.y; int x1=end.x; int y1=end.y;

  if (x0 < 0 || x0 >= size_x || y0 < 0 || y0 >= size_y ||
      x1 < 0 || x1 >= size_x || y1 < 0 || y1 >= size_y)
  {
    return false;
  }

  const int8_t* grid_data = map_->data.data();

  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  int stride_x = sx;
  int stride_y = sy * size_x;

  int current_idx = y0 * size_x + x0;

  while (true)
  {
    int8_t cell_cost = grid_data[current_idx];

    if (cell_cost != threshold) {
      return false;
    }

    if (x0 == x1 && y0 == y1)
      break;

    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x0 += sx;
      current_idx += stride_x;
    }
    if (e2 < dx) {
      err += dx;
      y0 += sy;
      current_idx += stride_y;
    }
  }

  return true;
}

nav_msgs::msg::Path AStarSmoothPlanner::greedyStringPullSmooth(const nav_msgs::msg::Path& npath) {
  // poses = npath.poses.copy()
  std::vector<geometry_msgs::msg::PoseStamped> poses = npath.poses;
  
  nav_msgs::msg::Path smoothed_path;
  // smoothed_path.header = npath.header
  smoothed_path.header = npath.header;
  
  // if len(poses) <= 2: return npath
  if (poses.size() <= 2) {
      return npath;
  }

  int i = 0;
  int j = 2;
  // n = len(poses)
  int n = static_cast<int>(poses.size());

  // while j < n:
  while (j < n) {
      // self.line_of_sight(self.pose_to_grid_node(poses[i].pose), self.pose_to_grid_node(poses[j].pose))
      // Note: poses[i] in ROS 2 Path is a PoseStamped, so it already contains a .pose field
      if (lineOfSight(poseToGridNode(poses[i].pose), poseToGridNode(poses[j].pose))) {
          // poses.pop(j-1)
          poses.erase(poses.begin() + (j - 1));
          // n = len(poses)
          n = static_cast<int>(poses.size());
      } else {
          // i = j-1
          i = j - 1;
          // j = j+1
          j = j + 1;
      }
  }

  // smoothed_path.poses = poses.copy()
  smoothed_path.poses = poses;
  return smoothed_path;

}

nav_msgs::msg::Path AStarSmoothPlanner::greedyStringPullSmoothIter(const nav_msgs::msg::Path& npath, int iter) {
    // path = Path()
    nav_msgs::msg::Path path;
    
    // path.header = npath.header
    path.header = npath.header;
    
    // path.poses = npath.poses.copy()
    path.poses = npath.poses;
    
    // for _ in range(iter):
    for (int i = 0; i < iter; ++i) {
        // path = self.greedy_string_pull_smooth(path)
        path = greedyStringPullSmooth(path);
    }

    // return path
    return path;
}

double AStarSmoothPlanner::distance2D(const Point2D& p1, const Point2D& p2) {
  return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

Point2D AStarSmoothPlanner::computeBezierPoint(const Point2D& p0, const Point2D& p1, const Point2D& p2, double t) {
  double one_minus_t = 1.0 - t;
  return p0 * (one_minus_t * one_minus_t) + p1 * (2.0 * one_minus_t * t) + p2 * (t * t);
}

Point2D AStarSmoothPlanner::calculateSafeAnchor(const Point2D& p_curr, const Point2D& p_neighbor, double d) {
  Point2D vector = p_neighbor - p_curr;
  double length = std::hypot(vector.x, vector.y);

  if (length == 0.0) {
    return p_curr;
  }

  Point2D unit_direction = vector * (1.0 / length);

  if (length >= 2.0 * d) {
    return p_curr + (unit_direction * d);
  } else {
    return p_curr + (vector * 0.5);
  }
}

void AStarSmoothPlanner::densifyStraightLineSegment(
  const Point2D& start_pt, const Point2D& end_pt, double resolution,
  std::vector<Point2D>& out_points)
{
  double dx = end_pt.x - start_pt.x;
  double dy = end_pt.y - start_pt.y;
  double dist = std::hypot(dx, dy);

  if (dist <= resolution) {
    out_points.push_back(end_pt);
    return;
  }

  int steps = std::max(1, static_cast<int>(dist / resolution));
  out_points.reserve(out_points.size() + steps);

  for (int i = 1; i <= steps; ++i) {
    double ratio = static_cast<double>(i) / static_cast<double>(steps);
    out_points.push_back({start_pt.x + dx * ratio, start_pt.y + dy * ratio});
  }
}

geometry_msgs::msg::Quaternion AStarSmoothPlanner::yawToQuaternion(double yaw) {
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

nav_msgs::msg::Path AStarSmoothPlanner::smoothAndDensifyPath(
  const nav_msgs::msg::Path& string_pulled_path,
  double d,
  int num_points_per_corner,
  double resolution)
{
  const auto& poses = string_pulled_path.poses;
  if (poses.size() < 3) {
    return string_pulled_path;
  }

  nav_msgs::msg::Path smoothed_path;
  smoothed_path.header = string_pulled_path.header;
  double z_height = poses[0].pose.position.z;

  std::vector<Point2D> raw_points;
  raw_points.reserve(poses.size() * num_points_per_corner);

  // 1. Pre-calculate safe curve anchor pairs for interior waypoints
  std::vector<BezierAnchor> anchors(poses.size());
  for (size_t i = 1; i < poses.size() - 1; ++i) {
    Point2D p_prev{poses[i - 1].pose.position.x, poses[i - 1].pose.position.y};
    Point2D p_curr{poses[i].pose.position.x, poses[i].pose.position.y};
    Point2D p_next{poses[i + 1].pose.position.x, poses[i + 1].pose.position.y};

    anchors[i] = {
      calculateSafeAnchor(p_curr, p_prev, d),
      p_curr,
      calculateSafeAnchor(p_curr, p_next, d)
    };
  }

  // 2. Build continuous sequential line coordinates
  Point2D start_pt{poses[0].pose.position.x, poses[0].pose.position.y};
  raw_points.push_back(start_pt);

  for (size_t i = 1; i < poses.size() - 1; ++i) {
    const auto& p0 = anchors[i].p0;
    const auto& p1 = anchors[i].p1;
    const auto& p2 = anchors[i].p2;

    // Densify straight line segment up to p0
    if (distance2D(raw_points.back(), p0) > 1e-4) {
      densifyStraightLineSegment(raw_points.back(), p0, resolution, raw_points);
    }

    // Draw high-density Bezier curve points
    for (int step = 1; step <= num_points_per_corner; ++step) {
      double t = static_cast<double>(step) / static_cast<double>(num_points_per_corner);
      Point2D bezier_pt = computeBezierPoint(p0, p1, p2, t);

      if (distance2D(raw_points.back(), bezier_pt) > 1e-4) {
        raw_points.push_back(bezier_pt);
      }
    }
  }

  // Densify final segment to goal
  Point2D p_goal{poses.back().pose.position.x, poses.back().pose.position.y};
  if (distance2D(raw_points.back(), p_goal) > 1e-4) {
    densifyStraightLineSegment(raw_points.back(), p_goal, resolution, raw_points);
  }

  // 3. Dynamic Orientation Extraction & Packing Loop
  smoothed_path.poses.reserve(raw_points.size());

  for (size_t i = 0; i < raw_points.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = smoothed_path.header;
    pose.pose.position.x = raw_points[i].x;
    pose.pose.position.y = raw_points[i].y;
    pose.pose.position.z = z_height;

    if (i < raw_points.size() - 1) {
      double dx = raw_points[i + 1].x - raw_points[i].x;
      double dy = raw_points[i + 1].y - raw_points[i].y;
      double yaw = std::atan2(dy, dx);
      pose.pose.orientation = yawToQuaternion(yaw);
    } else {
      // Retain goal's exact target orientation
      pose.pose.orientation = poses.back().pose.orientation;
    }

    smoothed_path.poses.push_back(pose);
  }

  return smoothed_path;
}


}




int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_navigation::AStarSmoothPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#include "robot_navigation/string_bezier_smoother.hpp"


namespace string_bezier_smoother
{

StringBezierSmoother::StringBezierSmoother() : Node("string_bezier_smoother"),
string_smooth_iterations(5), points_per_bezier_curve(20), bezier_anchor_distance(0.3)
{
  declare_parameter<int>("string_smooth_iterations", string_smooth_iterations);
  declare_parameter<int>("points_per_bezier_curve", points_per_bezier_curve);
  declare_parameter<double>("bezier_anchor_distance", bezier_anchor_distance);
  string_smooth_iterations = get_parameter("string_smooth_iterations").as_int();
  points_per_bezier_curve = get_parameter("points_per_bezier_curve").as_int();
  bezier_anchor_distance = get_parameter("bezier_anchor_distance").as_double();

  rclcpp::QoS default_qos(10);

  rclcpp::QoS map_qos(10);
  map_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  map_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/global_costmap/costmap",
    map_qos,
    std::bind(&StringBezierSmoother::mapCallback, this, std::placeholders::_1)
  );

  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    "/plan",
    default_qos,
    std::bind(&StringBezierSmoother::pathCallback, this, std::placeholders::_1)
  );

  path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "/plan_smoothed",
    default_qos
  );


  RCLCPP_INFO_STREAM(get_logger(), "StringBezierSmoother Node Has Started Successfully");

}

void StringBezierSmoother::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
 map_ = map;
 RCLCPP_INFO_STREAM(get_logger(), "Map Recieved");
}

void StringBezierSmoother::pathCallback(const nav_msgs::msg::Path::SharedPtr path)
{
  nav_msgs::msg::Path raw_path = *path;
  nav_msgs::msg::Path smooth_path = smooth(raw_path);
  path_pub_->publish(smooth_path);
  RCLCPP_INFO_STREAM(get_logger(), "Smmoth Path Published");
}

nav_msgs::msg::Path StringBezierSmoother::smooth(const nav_msgs::msg::Path &npath)
{
  nav_msgs::msg::Path path = npath;
  path = greedyStringPullSmoothIter(path, string_smooth_iterations);
  // path = greedyStringPullSmooth(path);
  path = smoothAndDensifyPath(
    path,
    bezier_anchor_distance,
    points_per_bezier_curve
  );
  return path;
}

GridNode StringBezierSmoother::poseToGridNode(const geometry_msgs::msg::Pose &pose)
{
  int grid_x =static_cast<int>((pose.position.x - map_->info.origin.position.x) / map_->info.resolution);
  int grid_y =static_cast<int>((pose.position.y - map_->info.origin.position.y) / map_->info.resolution);
  return GridNode(grid_x, grid_y);
}

geometry_msgs::msg::Pose StringBezierSmoother::gridNodeToPose(const GridNode &grid_node)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = grid_node.x * map_->info.resolution + map_->info.origin.position.x;
  pose.position.y = grid_node.y * map_->info.resolution + map_->info.origin.position.y;
  return pose;
}

bool StringBezierSmoother::isGridNodeOnMap(const GridNode &node)
{
  return (
    (0 <= node.x && node.x < static_cast<int>(map_->info.width)) && 
    (0 <= node.y && node.y < static_cast<int>(map_->info.height))
  );
}

bool StringBezierSmoother::isMapCellFree(const GridNode &node)
{
  return (map_->data.at(gridNodeToMapIndex(node)) >= 0) && (map_->data.at(gridNodeToMapIndex(node)) < 99);
}

int StringBezierSmoother::gridNodeToMapIndex(const GridNode &node)
{
  return static_cast<int>(node.y * map_->info.width + node.x);
}

std::vector<GridNode> StringBezierSmoother::bresenhamLine(const GridNode &start, const GridNode &end)
{
  int x0 = start.x; int y0 = start.y;
  int x1 = end.x; int y1 = end.y;

  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);

  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;

  int err = dx - dy;

  std::vector<GridNode> line = {};

  while(true) {
    line.push_back(GridNode(x0, y0));

    if ((x0==x1) && (y0==y1)) break;

    int e2 = 2 * err;
    if(e2 > -dy){
      err -= dy;
      x0 += sx;
    }
    if(e2 < dx){
      err += dx;
      y0 += sy;
    }
  }

  return line;
}

bool StringBezierSmoother::lineCrossesObstacle(const std::vector<GridNode> &line){
  for (const GridNode &node : line) {
    if(!isGridNodeOnMap(node)) return true;
    if(!(map_->data.at(gridNodeToMapIndex(node)) == 0)) return true;
  }
  return false;
}

bool StringBezierSmoother::lineOfSight(const GridNode &start, const GridNode &end){
  if (start == end) return true;
  std::vector<GridNode> line = bresenhamLine(start, end);
  return !(lineCrossesObstacle(line));
}

nav_msgs::msg::Path StringBezierSmoother::greedyStringPullSmooth(const nav_msgs::msg::Path& npath) {

  std::vector<geometry_msgs::msg::PoseStamped> poses = npath.poses;
  std::vector<geometry_msgs::msg::PoseStamped> new_poses = npath.poses;
  new_poses.clear();
  
  nav_msgs::msg::Path smoothed_path;
  smoothed_path.header = npath.header;
  
  if (poses.size() <= 2) {
      return npath;
  }

  int i = 0;
  int j = 2;
  int n = static_cast<int>(poses.size());

  new_poses.push_back(poses[i]);

  while (true) {
    if (!(j<n)){
      break;
    }
    else if (lineOfSight(poseToGridNode(poses[i].pose), poseToGridNode(poses[j].pose))) {
      j=j;  
    } else {
        i = j - 1;
        new_poses.push_back(poses[i]);
    }
    j += 1;
  }
  i = j-1;
  new_poses.push_back(poses[i]);

  smoothed_path.poses = new_poses;
  return smoothed_path;

}

nav_msgs::msg::Path StringBezierSmoother::greedyStringPullSmoothIter(const nav_msgs::msg::Path& npath, int iter) {
    nav_msgs::msg::Path path;
    path.header = npath.header;
    path.poses = npath.poses;

    for (int i = 0; i < iter; ++i) {
        path = greedyStringPullSmooth(path);
    }

    return path;
}

double StringBezierSmoother::distance2D(const Point2D& p1, const Point2D& p2) {
  return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

Point2D StringBezierSmoother::computeBezierPoint(const Point2D& p0, const Point2D& p1, const Point2D& p2, double t) {
  double one_minus_t = 1.0 - t;
  return p0 * (one_minus_t * one_minus_t) + p1 * (2.0 * one_minus_t * t) + p2 * (t * t);
}

std::tuple<Point2D, Point2D, double> StringBezierSmoother::calculateSafeAnchor(const Point2D& p_prev, const Point2D& p_curr, const Point2D& p_next, double d) {
  Point2D vect_prev = p_prev - p_curr;
  double len_prev = std::hypot(vect_prev.x, vect_prev.y);
  Point2D u_vect_prev = vect_prev * (1.0 / len_prev);

  Point2D vect_next = p_next - p_curr;
  double len_next = std::hypot(vect_next.x, vect_next.y);
  Point2D u_vect_next = vect_next * (1.0 / len_next);

  if((len_prev < 2*d || len_next < 2*d) && (len_prev < len_next)){
    return {p_curr+(u_vect_prev*(0.5*len_prev)), p_curr+(u_vect_next*(0.5*len_prev)), (0.5*len_prev)};
  }
  else if((len_prev < 2*d || len_next < 2*d) && (len_prev > len_next)) {
    return {p_curr+(u_vect_prev*(0.5*len_next)), p_curr+(u_vect_next*(0.5*len_next)), (0.5*len_next)};
  }
  else if((len_prev < 2*d || len_next < 2*d) && (len_prev == len_next)) {
    return {p_curr+(u_vect_prev*(0.5*len_prev)), p_curr+(u_vect_next*(0.5*len_next)), (0.5*len_prev)};
  }
  else {
    return {p_curr+(u_vect_prev*d), p_curr+(u_vect_next*d), d};
  }
}

void StringBezierSmoother::densifyStraightLineSegment(
  const Point2D& start_pt, const Point2D& end_pt,
  std::vector<Point2D>& out_points)
{
  double resolution = static_cast<double>(map_->info.resolution);
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

geometry_msgs::msg::Quaternion StringBezierSmoother::yawToQuaternion(double yaw) {
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

nav_msgs::msg::Path StringBezierSmoother::smoothAndDensifyPath(
  const nav_msgs::msg::Path& string_pulled_path,
  double d,
  int num_points_per_corner)
{
  const auto& poses = string_pulled_path.poses;
  if (poses.empty()) {
    return string_pulled_path;
  }

  nav_msgs::msg::Path smoothed_path;
  smoothed_path.header = string_pulled_path.header;
  double z_height = poses[0].pose.position.z;

  std::vector<Point2D> raw_points;
  raw_points.reserve(poses.size() * std::max(num_points_per_corner, 10));

  Point2D start_pt{poses[0].pose.position.x, poses[0].pose.position.y};
  raw_points.push_back(start_pt);

  // If we have intermediate corner waypoints (3 or more poses), apply Bezier cornering
  if (poses.size() >= 3) {
    std::vector<BezierAnchor> anchors(poses.size());
    for (size_t i = 1; i < poses.size() - 1; ++i) {
      Point2D p_prev{poses[i - 1].pose.position.x, poses[i - 1].pose.position.y};
      Point2D p_curr{poses[i].pose.position.x, poses[i].pose.position.y};
      Point2D p_next{poses[i + 1].pose.position.x, poses[i + 1].pose.position.y};

      auto [a_prev, a_next, a_dist] = calculateSafeAnchor(p_prev, p_curr, p_next, d);
      anchors[i] = {a_prev, p_curr, a_next, a_dist};
    }

    for (size_t i = 1; i < poses.size() - 1; ++i) {
      const auto& p0 = anchors[i].p0;
      const auto& p1 = anchors[i].p1;
      const auto& p2 = anchors[i].p2;
      const auto& a_dist = anchors[i].a_dist;

      int num_of_points = static_cast<int>((a_dist / d) * num_points_per_corner);

      if (distance2D(raw_points.back(), p0) > 1e-4) {
        densifyStraightLineSegment(raw_points.back(), p0, raw_points);
      }

      for (int step = 1; step <= num_of_points; ++step) {
        double t = static_cast<double>(step) / static_cast<double>(num_of_points);
        Point2D bezier_pt = computeBezierPoint(p0, p1, p2, t);

        if (distance2D(raw_points.back(), bezier_pt) > 1e-4) {
          raw_points.push_back(bezier_pt);
        }
      }
    }
  }

  // Final Segment Densification (Runs for ALL paths with >= 2 points)
  if (poses.size() >= 2) {
    Point2D p_goal{poses.back().pose.position.x, poses.back().pose.position.y};
    if (distance2D(raw_points.back(), p_goal) > 1e-4) {
      densifyStraightLineSegment(raw_points.back(), p_goal, raw_points);
    }
  }

  // Dynamic Orientation Extraction & Packing
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
      
      // Prevent zero-length heading calculations
      if (std::hypot(dx, dy) > 1e-4) {
        double yaw = std::atan2(dy, dx);
        pose.pose.orientation = yawToQuaternion(yaw);
      } else if (!smoothed_path.poses.empty()) {
        pose.pose.orientation = smoothed_path.poses.back().pose.orientation;
      } else {
        pose.pose.orientation = poses.front().pose.orientation;
      }
    } else {
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
  auto node = std::make_shared<string_bezier_smoother::StringBezierSmoother>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
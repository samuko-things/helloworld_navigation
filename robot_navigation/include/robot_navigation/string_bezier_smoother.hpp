#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "rmw/qos_profiles.h"

#include <vector>
#include <cmath>
#include <algorithm>

namespace string_bezier_smoother
{

struct GridNode
{
  int x;
  int y;

  GridNode(int x_in, int y_in) : x(x_in), y(y_in)
  {}

  bool operator==(const GridNode &other) const 
  {
    return (this->x == other.x) && (this->y == other.y);
  }

  GridNode operator+(const GridNode &other) const
  {
    GridNode result(this->x+other.x, this->y+other.y);
    return result;
  }
};

struct Point2D {
  double x{0.0};
  double y{0.0};

  Point2D operator+(const Point2D& other) const { return {x + other.x, y + other.y}; }
  Point2D operator-(const Point2D& other) const { return {x - other.x, y - other.y}; }
  Point2D operator*(double scalar) const { return {x * scalar, y * scalar}; }
};

struct BezierAnchor {
  Point2D p0;
  Point2D p1;
  Point2D p2;
};

class StringBezierSmoother : public rclcpp::Node
{
public:
  StringBezierSmoother();

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;

  int string_smooth_iterations = 20;
  int points_per_bezier_curve = 15;
  double bezier_anchor_distance = 0.3;

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);

  void pathCallback(const nav_msgs::msg::Path::SharedPtr path);

  nav_msgs::msg::Path smooth(const nav_msgs::msg::Path &npath);

  GridNode poseToGridNode(const geometry_msgs::msg::Pose &pose);

  geometry_msgs::msg::Pose gridNodeToPose(const GridNode &grid_node);

  int gridNodeToMapIndex(const GridNode &grid_node);

  bool isGridNodeOnMap(const GridNode &grid_node);

  bool isMapCellFree(const GridNode &grid_node);

  std::vector<GridNode> bresenhamLine(const GridNode &start, const GridNode &end);

  bool lineCrossesObstacle(const std::vector<GridNode> &line);

  bool lineOfSight(const GridNode &start, const GridNode &end);

  nav_msgs::msg::Path greedyStringPullSmooth(const nav_msgs::msg::Path& path);

  nav_msgs::msg::Path greedyStringPullSmoothIter(const nav_msgs::msg::Path& path, int iter=5);

  double distance2D(const Point2D& p1, const Point2D& p2);

  Point2D computeBezierPoint(const Point2D& p0, const Point2D& p1, const Point2D& p2, double t);

  Point2D calculateSafeAnchor(const Point2D& p_curr, const Point2D& p_neighbor, double d);

  void densifyStraightLineSegment(const Point2D& start_pt, const Point2D& end_pt, std::vector<Point2D>& out_points);

  geometry_msgs::msg::Quaternion yawToQuaternion(double yaw);

  nav_msgs::msg::Path smoothAndDensifyPath(
    const nav_msgs::msg::Path& string_pulled_path,
    double d = 0.3,
    int num_points_per_corner = 15);

};

}
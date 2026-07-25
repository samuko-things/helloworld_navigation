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
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>

namespace theta_star_planner
{

struct GridNode
{
  int x;
  int y;
  double g_cost;
  double h_cost;
  std::shared_ptr<GridNode> prev;

  GridNode(int x_in, int y_in) : x(x_in), y(y_in), g_cost(0), h_cost(0)
  {}

  GridNode() : GridNode(0, 0)
  {}

  bool operator>(const GridNode &other) const 
  {
    return (this->g_cost + this->h_cost) > (other.g_cost + other.h_cost);
  }

  bool operator==(const GridNode &other) const 
  {
    return (this->x == other.x) && (this->y == other.y);
  }

  GridNode operator+(std::pair<int, int> const &other) const
  {
    GridNode result(this->x+other.first, this->y+other.second);
    return result;
  }

  GridNode operator+(const GridNode &other) const
  {
    GridNode result(this->x+other.x, this->y+other.y);
    return result;
  }
};

struct DirNode
{
  std::pair<int, int> dir;
  int t_cost;

  DirNode(std::pair<int, int> dir, int t_cost) : dir(dir), t_cost(t_cost)
  {}
};


class ThetaStarPlanner : public rclcpp::Node
{
public:
  ThetaStarPlanner();

private:
  bool use_lazy_ = true;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  nav_msgs::msg::OccupancyGrid visited_map_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::vector<std::tuple<int, int>> obs_dir_;

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose);

  GridNode poseToGridNode(const geometry_msgs::msg::Pose &pose);
  geometry_msgs::msg::Pose gridNodeToPose(const GridNode &grid_node);
  int gridNodeToMapIndex(const GridNode &grid_node);

  bool isGridNodeOnMap(const GridNode &grid_node);
  bool isMapCellFree(const GridNode &grid_node);

  double euclidean_distance(const GridNode &a, const GridNode &b);

  double octile_distance(const GridNode &a, const GridNode &b);

  bool lineOfSight(const GridNode &start, const GridNode &end);

  nav_msgs::msg::Path basic_plan(const geometry_msgs::msg::Pose &start, const geometry_msgs::msg::Pose &goal);
  nav_msgs::msg::Path lazy_plan(const geometry_msgs::msg::Pose &start, const geometry_msgs::msg::Pose &goal);

};

}
#ifndef MY_TEST_PLANNER_HPP_
#define MY_TEST_PLANNER_HPP_

#include <vector>
#include <queue>
#include <memory>
#include <string>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2/utils.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "rmw/qos_profiles.h"


namespace test_planner
{

struct GridNode
{
  int x{0};
  int y{0};

  double g_cost{0.0};
  double h_cost{0.0};
  double f_cost{0.0};

  bool is_in_queue{false};

  GridNode *prev{nullptr};
  GridNode *parent{nullptr};

  GridNode(int _x = 0, int _y = 0) : x(_x), y(_y) {}
};

struct Dir
{ int dx; 
  int dy; 
  double dist; 
};

struct MapMetaData {
  double resolution{0.0};
  double inv_resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  int size_x{0};
  int size_y{0};

  // Helper to update all fields atomically from a costmap pointer
  void update(const nav_msgs::msg::OccupancyGrid::SharedPtr map_) {
    if (!map_) return;
    resolution = map_->info.resolution;
    inv_resolution = (resolution > 0.0) ? (1.0 / resolution) : 0.0;
    origin_x = map_->info.origin.position.x;
    origin_y = map_->info.origin.position.y;
    size_x = static_cast<int>(map_->info.width);
    size_y = static_cast<int>(map_->info.height);
  }
};


class TestPlanner : public rclcpp::Node
{
public:
  TestPlanner();

  struct CompareNode
  {
    bool operator()(
      const GridNode* a,
      const GridNode* b) const
    {
      // Keeps the evaluation simple and fast for priority sorting tree shifts
      // return (a->g_cost + a->h_cost) > (b->g_cost + b->h_cost);
      return (a->f_cost) > (b->f_cost);
    }
  };

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  nav_msgs::msg::OccupancyGrid visited_map_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::vector<std::tuple<int, int>> obs_dir_;

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_pose);

  nav_msgs::msg::Path plan(
    const geometry_msgs::msg::PoseStamped &start_pose,
    const geometry_msgs::msg::PoseStamped &goal_pose);

  GridNode* runLazyThetaStarPlan(
    GridNode* start_node,
    GridNode* goal_node,
    const int8_t* char_map,
    unsigned int size_x,
    size_t & los_checks,
    size_t & node_expansions,
    size_t & fallback_count,
    size_t & successful_parent_collapses);

  GridNode* runLazyThetaStarPlanTest(
    GridNode* start_node,
    GridNode* goal_node,
    const int8_t* char_map,
    unsigned int size_x,
    size_t & los_checks,
    size_t & node_expansions,
    size_t & fallback_count,
    size_t & successful_parent_collapses);

  GridNode poseToGrid(const geometry_msgs::msg::Pose &pose) const;

  geometry_msgs::msg::Pose gridToPose(const GridNode &grid) const;

  int gridToMapIndex(const GridNode &grid) const;

  int gridToMapIndex(const int x, const int y) const;

  bool isGridOnMap(const GridNode &grid) const;

  bool isMapCellFree(const GridNode &grid, const int8_t* char_map) const;

  bool isMapCellFree(const int x, const int y, const int8_t* char_map) const;

  double euclidean_distance(const GridNode &a, const GridNode &b) const;

  bool isCloseToObstacle(
    const int8_t* char_map,
    int cx, int cy,
    int sweep_dist_cells) const;

  bool isNodeCloseToObstacle(
    const int8_t* char_map,
    GridNode &node) const;

  bool isEdgeRisky(
    const int8_t* char_map,
    GridNode* current,
    GridNode* parent) const;

  bool shouldCheckLOS(
    const int8_t* char_map,
    GridNode *current,
    GridNode *parent
  ) const;

  bool lineOfSight(
    GridNode *start,
    GridNode *end,
    const int8_t* char_map,
    unsigned int size_x) const;

  std::vector<Dir> generateDirections(int grid_radius);
  std::vector<Dir> generateDirectionRayCasts(int grid_radius);
  std::vector<Dir> generateSquareRing(int grid_radius);

  std::vector<geometry_msgs::msg::PoseStamped>
  addStraightLinePoses(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & end,
    double resolution) const;

  nav_msgs::msg::Path densifyPath(
    const nav_msgs::msg::Path & path, 
    const geometry_msgs::msg::PoseStamped & goal) const;

  nav_msgs::msg::Path smoothPath(
    const nav_msgs::msg::Path & path,
    double w_data=0.2,
    double w_smooth=0.4,
    int max_iterations=1000,
    double tolerance=1e-5) const;

  nav_msgs::msg::Path fillUpPath(
    const nav_msgs::msg::Path & path, 
    const geometry_msgs::msg::PoseStamped & goal,
    bool smooth=true) const;

  GridNode* get_node_from_pool(int x, int y, int index);

  MapMetaData map_meta_data_;

  std::vector<GridNode> node_pool_;
  std::vector<bool> node_initialized_; 
  std::vector<int> node_visited_id_;
  int run_id_ = 0;
  std::vector<double> g_cost_cache_;

  int los_shortcut_cost_limit_;
  double cost_travel_multiplier_;

  int planner_id_ = 1;

  Dir dirs[8] = {
    {-1,  0, 1.0},
    { 1,  0, 1.0},
    { 0, -1, 1.0},
    { 0,  1, 1.0},
    {-1, -1, 1.4142},
    {-1,  1, 1.4142},
    { 1, -1, 1.4142},
    { 1,  1, 1.4142}
  };

  std::vector<Dir> dirs_;

  rclcpp::Logger logger_{rclcpp::get_logger("TestPlanner")};
};

}  // namespace test_planner

#endif  // MY_TEST_PLANNER_PLUGIN_HPP_
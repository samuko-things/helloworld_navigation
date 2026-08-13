#ifndef TEST_PLANNER_PLUGIN_HPP_
#define TEST_PLANNER_PLUGIN_HPP_

#include <vector>
#include <queue>
#include <memory>
#include <string>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <functional>

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_utils.hpp"

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2/utils.h"

namespace test_planner_plugin
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

  DirNode(std::pair<int, int> dir_, int t_cost_) : dir(dir_), t_cost(t_cost_)
  {}
};


struct CostmapMeta {
  double resolution{0.0};
  double inv_resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  int size_x{0};
  int size_y{0};

  // Helper to update all fields atomically from a costmap pointer
  void update(const nav2_costmap_2d::Costmap2D* costmap) {
    if (!costmap) return;
    resolution = costmap->getResolution();
    inv_resolution = (resolution > 0.0) ? (1.0 / resolution) : 0.0;
    origin_x = costmap->getOriginX();
    origin_y = costmap->getOriginY();
    size_x = static_cast<int>(costmap->getSizeInCellsX());
    size_y = static_cast<int>(costmap->getSizeInCellsY());
  }
};


class TestPlanner : public nav2_core::GlobalPlanner
{
public:
  TestPlanner() = default;
  ~TestPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

protected:

  std::shared_ptr<GridNode> runTestPlan(
    std::shared_ptr<GridNode> start_node,
    std::shared_ptr<GridNode> goal_node,
    const std::function<bool()>& cancel_checker,
    const unsigned char* char_map,
    unsigned int size_x);

  std::shared_ptr<GridNode> runThetaStarPlan(
    std::shared_ptr<GridNode> start_node,
    std::shared_ptr<GridNode> goal_node,
    const std::function<bool()>& cancel_checker,
    const unsigned char* char_map,
    unsigned int size_x);

  std::shared_ptr<GridNode> runAStarPlan(
    std::shared_ptr<GridNode> start_node,
    std::shared_ptr<GridNode> goal_node,
    const std::function<bool()>& cancel_checker,
    const unsigned char* char_map,
    unsigned int size_x,
    bool smooth=true);

  std::shared_ptr<GridNode> greedyStringPullSmooth(
    std::shared_ptr<GridNode> grid_node_path,
    const std::function<bool()>& cancel_checker,
    const unsigned char* char_map,
    unsigned int size_x);

  
  //---------LAZY THETA STAR---------------------

  std::shared_ptr<GridNode> runLazyThetaStarPlan(
    std::shared_ptr<GridNode> start_node,
    std::shared_ptr<GridNode> goal_node,
    const std::function<bool()>& cancel_checker,
    const unsigned char* char_map,
    unsigned int size_x);

  //---------------------------------------------


  GridNode poseToGrid(const geometry_msgs::msg::Pose &pose);
  geometry_msgs::msg::Pose gridToPose(const GridNode &grid);
  int gridToMapIndex(const GridNode &grid);
  double getGridCost(const GridNode &grid, const unsigned char* char_map);
  bool isGridOnMap(const GridNode &grid);
  bool isMapCellFree(const GridNode &grid, const unsigned char* char_map);

  double euclidean_distance(const GridNode &a, const GridNode &b);

  bool lineOfSight(
    const GridNode &start, 
    const GridNode &end,
    const unsigned char* char_map,
    unsigned int size_x,
    bool relax=false) const;

  std::vector<geometry_msgs::msg::PoseStamped>
  addStraightLinePoses(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & end,
    double resolution) const;

  nav_msgs::msg::Path fillUpPath(
    const nav_msgs::msg::Path & path, 
    const geometry_msgs::msg::PoseStamped & goal) const;

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  CostmapMeta costmap_meta_;

  int los_shortcut_cost_limit_;

  double cost_travel_multiplier_;

  std::string planner_name_{"test"};

  rclcpp::Logger logger_{rclcpp::get_logger("TestPlanner")};
};

}  // namespace test_planner_plugin

#endif  // TEST_PLANNER_PLUGIN_HPP_
#ifndef TEST_SMOOTHER_PLUGIN_HPP_
#define TEST_SMOOTHER_PLUGIN_HPP_

#include <vector>
#include <queue>
#include <memory>
#include <string>
#include <cmath>
#include <chrono>

#include "nav2_core/smoother.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace test_smoother_plugin
{

struct GridNode
{
  int x;
  int y;

  GridNode(int x_in, int y_in) : x(x_in), y(y_in)
  {}
};


struct CostmapMeta {
  double resolution{0.0};
  double inv_resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  int size_x{0};
  int size_y{0};

  void update(const nav2_costmap_2d::Costmap2D *costmap) {
    if (!costmap) return;
    resolution = costmap->getResolution();
    inv_resolution = (resolution > 0.0) ? (1.0 / resolution) : 0.0;
    origin_x = costmap->getOriginX();
    origin_y = costmap->getOriginY();
    size_x = static_cast<int>(costmap->getSizeInCellsX());
    size_y = static_cast<int>(costmap->getSizeInCellsY());
  }
};


class TestSmoother : public nav2_core::Smoother
{
public:
  TestSmoother() = default;
  ~TestSmoother() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub) override;

  void cleanup() override { costmap_sub_.reset(); }
  void activate() override {}
  void deactivate() override {}

  bool smooth(nav_msgs::msg::Path & path, const rclcpp::Duration & max_time) override;

protected:

  GridNode poseToGrid(const geometry_msgs::msg::Pose &pose);

  bool isGridOnMap(const GridNode &grid);

  int gridToMapIndex(const GridNode &grid_node);

  unsigned char getGridCost(const GridNode &grid, const unsigned char* char_map);

  nav_msgs::msg::Path greedyStringPullSmooth(
    const nav_msgs::msg::Path& npath,
    const unsigned char* char_map,
    unsigned int size_x);

  bool lineOfSight(
    const GridNode &start, 
    const GridNode &end,
    const unsigned char* char_map,
    unsigned int size_x);

  std::vector<geometry_msgs::msg::PoseStamped>
  addStraightLinePoses(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & end,
    double resolution);

  nav_msgs::msg::Path fillUpPath(const nav_msgs::msg::Path & path);

  nav_msgs::msg::Path laplacianSmooth(
    const nav_msgs::msg::Path & path,
    const unsigned char* char_map);

  void updateOrientations(nav_msgs::msg::Path & path);

  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;

  CostmapMeta costmap_meta_;

  int cost_limit_;
  int iterations_;

  double w_data_;
  double w_smooth_;
  double tolerance_;
  int max_its_;

  rclcpp::Logger logger_{rclcpp::get_logger("TestSmoother")};
};

}  // namespace test_smoother_plugin

#endif  // TEST_SMMOTHER_PLUGIN_HPP_
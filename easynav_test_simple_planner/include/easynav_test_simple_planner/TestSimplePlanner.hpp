// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \file
/// \brief Declaration of the TestSimplePlanner class implementing A* path planning.

#ifndef EASYNAV_PLANNER__TEST_SIMPLEPLANNER_HPP_
#define EASYNAV_PLANNER__TEST_SIMPLEPLANNER_HPP_

#include <vector>

#include "nav_msgs/msg/path.hpp"

#include "easynav_core/PlannerMethodBase.hpp"
#include "easynav_simple_common/SimpleMap.hpp"
#include "easynav_common/types/NavState.hpp"

namespace easynav
{

struct GridNode
{
  int x{0};
  int y{0};

  double g_cost{0.0};
  double h_cost{0.0};

  bool near_obstacle = false;

  GridNode *prev{nullptr};

  GridNode(int _x = 0, int _y = 0) : x(_x), y(_y) {}
};


struct MapMetaData {
  double resolution{0.0};
  double inv_resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  int size_x{0};
  int size_y{0};

  // Helper to update all fields atomically from a costmap pointer
  void update(const SimpleMap &map) {
    resolution = map.resolution();
    inv_resolution = (resolution > 0.0) ? (1.0 / resolution) : 0.0;
    origin_x = map.origin_x();
    origin_y = map.origin_y();
    size_x = static_cast<int>(map.width());
    size_y = static_cast<int>(map.height());
  }
};

/// \brief A planner implementing the A* algorithm on a SimpleMap grid.
class TestSimplePlanner : public PlannerMethodBase
{
public:
  /**
   * @brief Default constructor.
   *
   * Initializes the internal variables and parameters of the planner.
   */
  explicit TestSimplePlanner();

  /**
   * @brief Initializes the planner.
   *
   * Configures publishers, retrieves parameters, and prepares the planner
   * for path generation using the available map data.
   *
   * @throws std::runtime_error if initialization fails.
   */
  virtual void on_initialize() override;

  /**
   * @brief Updates the planner by computing a new path.
   *
   * Uses the current navigation state (including the robot's position and goal)
   * to generate a path based on the A* algorithm.
   *
   * @param nav_state The current navigation state (contains odometry and goal information).
   */
  void update(NavState & nav_state) override;

  struct CompareNode
  {
    bool operator()(
      const GridNode* a,
      const GridNode* b) const
    {
      // Keeps the evaluation simple and fast for priority sorting tree shifts
      return (a->g_cost + a->h_cost) > (b->g_cost + b->h_cost);
    }
  };

protected:
  double robot_radius_;        ///< Radius of the robot used for collision checking.
  double clearance_distance_;  ///< Minimum clearance distance from obstacles in meters.

  nav_msgs::msg::Path current_path_;  ///< The last computed path.

  /// Publisher for the computed navigation path.
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  GridNode* get_node_from_pool(int x, int y, int index);

  MapMetaData map_meta_;

  std::vector<GridNode> node_pool_;
  std::vector<bool> node_initialized_; 
  std::vector<double> g_cost_cache_;
  std::vector<bool> visited_;
  double clearance_cells_ = 0.2;

  /**
   * @brief Runs the DRSP (Dynamic Recursive String Pulling) planning algorithm to compute a path.
   *
   * @param map The occupancy map used for path planning.
   * @param start The starting pose in world coordinates.
   * @param goal The target pose in world coordinates.
   * @param resolution The cell resolution of the map (in meters).
   * @return A sequence of poses representing the planned path.
   */
  std::vector<geometry_msgs::msg::Pose> plan_path(
    const SimpleMap & map,
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal);

  /**
   * @brief Checks whether a map cell is free, considering a clearance area.
   *
   * This function verifies if a cell and its surrounding cells (within the
   * specified clearance radius) are free of obstacles.
   *
   * @param map The occupancy map to query.
   * @param cx The x-coordinate of the cell.
   * @param cy The y-coordinate of the cell.
   * @param clearance_cells The clearance radius expressed in number of cells.
   * @return true if the cell and its clearance area are free, false otherwise.
   */
  bool isFreeWithClearance(
    const SimpleMap & map,
    int cx, int cy,
    double clearance_cells);

  /* --------- NEW FUNCTIONS ------------- */
  GridNode* runDRSPPlan(
    GridNode* start_node,
    GridNode* goal_node,
    const SimpleMap & map);

  GridNode poseToGrid(const geometry_msgs::msg::Pose &pose);

  geometry_msgs::msg::Pose gridToPose(const GridNode &grid);

  int gridToMapIndex(const GridNode &grid);

  bool isGridOnMap(const GridNode &grid);

  bool isMapCellFree(const GridNode &grid, const SimpleMap & map);

  double euclidean_distance(const GridNode &a, const GridNode &b);

  bool lineOfSight(
    int x0, int y0, 
    int x1, int y1,
    const SimpleMap & map);

  std::vector<geometry_msgs::msg::Pose> addStraightLinePoses(
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & end,
    double resolution);

  std::vector<geometry_msgs::msg::Pose> densifyPath(
    const std::vector<geometry_msgs::msg::Pose> & poses, 
    const geometry_msgs::msg::Pose & goal);

  /* ------------------------------- */

};

}  // namespace easynav

#endif  // EASYNAV_PLANNER__TEST_SIMPLEPLANNER_HPP_

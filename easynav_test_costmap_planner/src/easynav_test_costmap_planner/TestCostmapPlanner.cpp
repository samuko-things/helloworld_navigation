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
/// \brief Implementation of the TestCostmapPlanner class using A* on Costmap2D.

#include <queue>
#include <unordered_map>
#include <cmath>
#include <tuple>

#include "easynav_test_costmap_planner/TestCostmapPlanner.hpp"
#include "easynav_common/RTTFBuffer.hpp"

#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "easynav_costmap_common/costmap_2d.hpp"
#include "easynav_costmap_common/cost_values.hpp"

namespace easynav
{

static double compute_path_length(const nav_msgs::msg::Path & path)
{
  double total_length = 0.0;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & p1 = path.poses[i - 1].pose.position;
    const auto & p2 = path.poses[i].pose.position;
    total_length += std::hypot(p2.x - p1.x, p2.y - p1.y);
  }
  return total_length;
}

// Simple path smoother: moving average over a sliding window in XY.
// Keeps endpoints unchanged to preserve exact start and goal.
static void smooth_path(std::vector<geometry_msgs::msg::Pose> & poses, int window_size = 5)
{
  if (poses.size() < 3 || window_size <= 1) {
    return;
  }

  // Ensure window_size is odd and at least 3
  if (window_size < 3) {
    window_size = 3;
  }
  if (window_size % 2 == 0) {
    window_size += 1;
  }

  const int half = window_size / 2;
  const size_t n = poses.size();
  std::vector<geometry_msgs::msg::Pose> original = poses;

  // Leave first and last pose untouched
  for (size_t i = 1; i + 1 < n; ++i) {
    double sum_x = 0.0;
    double sum_y = 0.0;
    int count = 0;

    const int begin = static_cast<int>(std::max<size_t>(0,
        i > static_cast<size_t>(half) ? i - half : 0));
    const int end = static_cast<int>(std::min<size_t>(n - 1, i + half));

    for (int j = begin; j <= end; ++j) {
      sum_x += original[j].position.x;
      sum_y += original[j].position.y;
      ++count;
    }

    if (count > 0) {
      poses[i].position.x = sum_x / static_cast<double>(count);
      poses[i].position.y = sum_y / static_cast<double>(count);
    }
  }
}

TestCostmapPlanner::TestCostmapPlanner()
{
  NavState::register_printer<nav_msgs::msg::Path>(
    [](const nav_msgs::msg::Path & path) {
      std::ostringstream ret;
      ret << "{ " << rclcpp::Time(path.header.stamp).seconds() << " } Path with " <<
        path.poses.size() << " poses and length "
          << compute_path_length(path) << " m.";
      return ret.str();
    });
}

void TestCostmapPlanner::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();
  node->declare_parameter<int>(plugin_name + ".los_shortcut_cost_limit", 5);
  node->declare_parameter<double>(plugin_name + ".cost_travel_multiplier", 3.0);
  node->declare_parameter<bool>(plugin_name + ".smooth_path", true);
  node->declare_parameter<bool>(plugin_name + ".continuous_replan", true);

  node->get_parameter(plugin_name + ".los_shortcut_cost_limit", los_shortcut_cost_limit_);
  node->get_parameter(plugin_name + ".cost_travel_multiplier", cost_travel_multiplier_);
  node->get_parameter(plugin_name + ".smooth_path", smooth_path_);
  node->get_parameter(plugin_name + ".continuous_replan", continuous_replan_);

  path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/path", 10);
}

void TestCostmapPlanner::update(NavState & nav_state)
{
  if (!nav_state.has("goals") || !nav_state.has("robot_pose") || !nav_state.has("map")) {
    return;
  }

  const auto & goals = nav_state.get<nav_msgs::msg::Goals>("goals");
  if (goals.goals.empty()) {
    nav_state.set("path", current_path_);
    return;
  }

  const auto & map = nav_state.get<Costmap2D>("map");
  const auto & robot_pose = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");
  const auto & goal = goals.goals.front().pose;
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();


  costmap_meta_.update(map);
  unsigned int map_size = costmap_meta_.size_x * costmap_meta_.size_y;

  // Fallback memory check in case the costmap dynamically resizes during runtime
  if (node_pool_.size() != map_size) {
    node_pool_.resize(map_size);
    node_initialized_.resize(map_size);
    g_cost_cache_.resize(map_size);
    visited_.resize(map_size);
  }

  // --- FAST FLAT MEMORY RESETS ---
  std::fill(node_initialized_.begin(), node_initialized_.end(), false);
  std::fill(visited_.begin(), visited_.end(), false);
  std::fill(g_cost_cache_.begin(), g_cost_cache_.end(), -1.0);


  rclcpp::Time latest_stamp = nav_state.get<rclcpp::Time>("map_time");
  if (rclcpp::Time(robot_pose.header.stamp, latest_stamp.get_clock_type()) > latest_stamp) {
    latest_stamp = rclcpp::Time(robot_pose.header.stamp, latest_stamp.get_clock_type());
  }
  if (rclcpp::Time(goals.goals.front().header.stamp,
      latest_stamp.get_clock_type()) > latest_stamp)
  {
    latest_stamp = rclcpp::Time(goals.goals.front().header.stamp, latest_stamp.get_clock_type());
  }
  current_path_.header.stamp = latest_stamp;

  if (goals.header.frame_id != tf_info.map_frame) {
    RCLCPP_WARN(get_node()->get_logger(), "Goals frame is not 'map': %s",
        goals.header.frame_id.c_str());
    return;
  }

  unsigned int gx, gy;
  if (!map.worldToMap(goal.position.x, goal.position.y, gx, gy)) {
    RCLCPP_WARN(get_node()->get_logger(), "Goal (%.2f, %.2f) is outside the map", goal.position.x,
        goal.position.y);
    return;
  }

  auto goals_ts = rclcpp::Time(goals.header.stamp);
  if (!continuous_replan_ &&
    goals_ts < rclcpp::Time(current_path_.header.stamp) &&
    goals.goals.front().pose == current_goal_)
  {
    return;
  }

  current_goal_ = goal;

  // Lightweight skip: if inputs unchanged, avoid recomputation for a short window
  static int last_sx = -1;
  static int last_sy = -1;
  static geometry_msgs::msg::Pose last_goal_pose;
  static rclcpp::Time last_plan_time;

  unsigned int sx_chk, sy_chk;
  if (map.worldToMap(robot_pose.pose.pose.position.x, robot_pose.pose.pose.position.y, sx_chk,
      sy_chk))
  {
    const bool same_start_cell = (static_cast<int>(sx_chk) == last_sx) &&
      (static_cast<int>(sy_chk) == last_sy);
    const bool same_goal_pose = (
      std::fabs(goal.position.x - last_goal_pose.position.x) < 1e-6 &&
      std::fabs(goal.position.y - last_goal_pose.position.y) < 1e-6 &&
      goal.orientation.x == last_goal_pose.orientation.x &&
      goal.orientation.y == last_goal_pose.orientation.y &&
      goal.orientation.z == last_goal_pose.orientation.z &&
      goal.orientation.w == last_goal_pose.orientation.w);

    // Initialize last_plan_time on first use to current node time
    if (last_plan_time.nanoseconds() == 0) {
      last_plan_time = get_node()->now();
    }
    const double since_last = (get_node()->now() - last_plan_time).seconds();
    // Only allow skipping when continuous_replan_ is disabled (event-based planning)
    if (!continuous_replan_ && same_start_cell && same_goal_pose && since_last < 0.05) {
      // Skip re-planning when nothing relevant changed recently
      nav_state.set("path", current_path_);
      return;
    }
  }

  auto poses = plan_path(map, robot_pose.pose.pose, goal);
  if (!poses.empty()) {
    // Apply a light smoothing to the raw grid path
    if (smooth_path_){
      smooth_path(poses);
    }

    current_path_.poses.clear();
    current_path_.header.stamp = get_node()->now();
    current_path_.header.frame_id = goals.header.frame_id;
    for (const auto & pose : poses) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = goals.header.frame_id;
      pose_stamped.header.stamp = current_path_.header.stamp;
      pose_stamped.pose = pose;
      current_path_.poses.push_back(pose_stamped);
    }
    // Always publish a newly computed path
    if (path_pub_->get_subscription_count() > 0) {
      path_pub_->publish(current_path_);
    }
    // Update last inputs snapshot
    unsigned int sx, sy;
    if (map.worldToMap(robot_pose.pose.pose.position.x, robot_pose.pose.pose.position.y, sx, sy)) {
      last_sx = static_cast<int>(sx);
      last_sy = static_cast<int>(sy);
    }
    last_goal_pose = goal;
    last_plan_time = get_node()->now();
  }
  nav_state.set("path", current_path_);
}

std::vector<geometry_msgs::msg::Pose> TestCostmapPlanner::plan_path(
  const Costmap2D & map,
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal)
{
  unsigned int sx, sy, gx, gy;
  if (!map.worldToMap(start.position.x, start.position.y, sx, sy)) {return {};}
  if (!map.worldToMap(goal.position.x, goal.position.y, gx, gy)) {return {};}

  // Start & Goal Node Setup
  GridNode raw_start(static_cast<int>(sx), static_cast<int>(sy));
  int start_idx = grid_to_map_index(raw_start);
  GridNode* start_node = get_node_from_pool(raw_start.x, raw_start.y, start_idx);

  GridNode raw_goal(static_cast<int>(gx), static_cast<int>(gy));
  int goal_idx = grid_to_map_index(raw_goal);
  GridNode* goal_node = get_node_from_pool(raw_goal.x, raw_goal.y, goal_idx);

  const unsigned char* char_map = map.getCharMap();

  GridNode* best_goal = run_test_planner(
    start_node, 
    goal_node, 
    char_map, 
    costmap_meta_.size_x
  );

  // Path Reconstruction
  std::vector<geometry_msgs::msg::Pose> path;

  if (!best_goal) {
    return path;
  }

  GridNode* node = best_goal;
  while (node)
  {
    geometry_msgs::msg::Pose pose;
    pose = grid_to_pose(*node);
    path.push_back(pose);

    if (node->parent == node) {
      break;
    }
    node = node->parent;
  }

  std::reverse(path.begin(), path.end());
  return densify_path(path, goal);
}












/* --------------------- NEW FUNCTIONS ------------------------------ */

GridNode* TestCostmapPlanner::run_test_planner(
  GridNode* start_node,
  GridNode* goal_node,
  const unsigned char* char_map,
  unsigned int size_x)
{
  std::priority_queue<
    GridNode*,
    std::vector<GridNode*>,
    CompareGridNode
  > open;

  int start_idx = grid_to_map_index(*start_node);

  start_node->g_cost = 0.0;
  start_node->h_cost = euclidean_distance(*start_node, *goal_node);
  start_node->parent = start_node;

  open.push(start_node);
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

  while (!open.empty())
  {
    GridNode* current = open.top();
    open.pop();

    int active_idx = grid_to_map_index(*current);

    if (visited_[active_idx]) {
      continue;
    }

    if (current->g_cost > g_cost_cache_[active_idx]) {
      continue;
    }

    if (current->parent && current->parent->parent)
    {
      auto actual_grandparent = current->parent->parent;
      if (line_of_sight(current->x, current->y, actual_grandparent->x, actual_grandparent->y, char_map, size_x))
      {
        current->parent = actual_grandparent;
      }
      else
      {
        double min_g = std::numeric_limits<double>::infinity();
        GridNode* best_fallback_parent = nullptr;

        for (const auto & d : dirs)
        {
          int nx = current->x + d.dx;
          int ny = current->y + d.dy;
          GridNode nbr_pos(nx, ny);
          int nbr_idx = grid_to_map_index(nbr_pos);

          if (is_grid_on_map(nbr_pos) && is_map_cell_free(nbr_pos, char_map) && visited_[nbr_idx])
          {
            GridNode* nbr_node = &node_pool_[nbr_idx];
            double g_val = g_cost_cache_[nbr_idx];
            double cost_to_active = g_val + (d.dist * get_grid_cost(*current, char_map));
            if (cost_to_active < min_g) {
              min_g = cost_to_active;
              best_fallback_parent = nbr_node;
            }
          }
        }

        if (best_fallback_parent) {
          current->g_cost = min_g;
          g_cost_cache_[active_idx] = min_g;
          current->parent = best_fallback_parent;
        }
      }
    }

    if (current->x == goal_node->x && current->y == goal_node->y) {
      return current;
    }

    visited_[active_idx] = true;

    for (const auto & d : dirs)
    {
      int nx = current->x + d.dx;
      int ny = current->y + d.dy;

      GridNode nbr_pos(nx, ny);
      int nbr_idx = grid_to_map_index(nbr_pos);

      if (is_grid_on_map(nbr_pos) && is_map_cell_free(nbr_pos, char_map) && !visited_[nbr_idx]) {
        GridNode* assumed_grandparent = current->parent ? current->parent : current;
        GridNode* neighbor_node = get_node_from_pool(nx, ny, nbr_idx);

        double new_g_cost = assumed_grandparent->g_cost + (euclidean_distance(nbr_pos, *(assumed_grandparent)) * get_grid_cost(nbr_pos, char_map));
        double nbr_g_cost = g_cost_cache_[nbr_idx];

        if (nbr_g_cost < 0.0 || new_g_cost < nbr_g_cost)
        {
          g_cost_cache_[nbr_idx] = new_g_cost;

          neighbor_node->g_cost = new_g_cost;
          neighbor_node->h_cost = euclidean_distance(*neighbor_node, *goal_node);
          neighbor_node->parent = current;

          open.push(neighbor_node);
        }
      }
    }
  }

  return nullptr;
}


GridNode TestCostmapPlanner::pose_to_grid(const geometry_msgs::msg::Pose &pose)
{
  int gx = static_cast<int>((pose.position.x - costmap_meta_.origin_x) * costmap_meta_.inv_resolution);
  int gy = static_cast<int>((pose.position.y - costmap_meta_.origin_y) * costmap_meta_.inv_resolution);

  return GridNode(gx, gy);
}

geometry_msgs::msg::Pose TestCostmapPlanner::grid_to_pose(const GridNode &grid)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = grid.x * costmap_meta_.resolution + costmap_meta_.origin_x;
  pose.position.y = grid.y * costmap_meta_.resolution + costmap_meta_.origin_y;
  pose.position.z = 0.0;

  return pose;
}

int TestCostmapPlanner::grid_to_map_index(const GridNode &grid_node)
{
  return static_cast<int>(grid_node.y * costmap_meta_.size_x + grid_node.x);
}

bool TestCostmapPlanner::is_grid_on_map(const GridNode &grid)
{
  return (grid.x >= 0 && grid.x < costmap_meta_.size_x &&
          grid.y >= 0 && grid.y < costmap_meta_.size_y);
}

double TestCostmapPlanner::get_grid_cost(const GridNode &grid, const unsigned char* char_map)
{
  return  1.0+(cost_travel_multiplier_ * std::clamp(static_cast<double>(char_map[grid_to_map_index(grid)]) / 252.0, 0.0, 1.0));
}

bool TestCostmapPlanner::is_map_cell_free(const GridNode &grid, const unsigned char* char_map)
{
  return /*(char_map[grid_to_map_index(grid)] >= 0) &&*/ (char_map[grid_to_map_index(grid)] < static_cast<unsigned char>(los_shortcut_cost_limit_+120));
}

double TestCostmapPlanner::euclidean_distance(const GridNode &a, const GridNode &b){
  double dx = static_cast<double>(a.x - b.x);
  double dy = static_cast<double>(a.y - b.y);
  return std::sqrt(dx * dx + dy * dy);
}


bool TestCostmapPlanner::line_of_sight(
  int x0, int y0, 
  int x1, int y1,
  const unsigned char* char_map,
  unsigned int size_x
) const
{
  int dx = std::abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
  int dy = std::abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  int max_x = static_cast<int>(costmap_meta_.size_x);
  int max_y = static_cast<int>(costmap_meta_.size_y);

  const auto threshold = static_cast<unsigned char>(los_shortcut_cost_limit_);

  // Helper lambda to encapsulate boundary and cost checks
  auto isSafe = [&](int x, int y) -> bool {
    if (x < 0 || x >= max_x || y < 0 || y >= max_y) {
      return false;
    }
    int idx = y * static_cast<int>(size_x) + x;
    return char_map[idx] <= threshold;
  };

  while (true) {
    if (!isSafe(x0, y0)) {
      return false;
    }

    if (x0 == x1 && y0 == y1) {
      break;
    }

    int e2 = 2 * err;
    if (e2 > -dy) { 
      err -= dy; 
      x0 += sx; 
    }
    if (e2 < dx) { 
      err += dx; 
      y0 += sy; 
    }
  }

  return true;
}


GridNode* TestCostmapPlanner::get_node_from_pool(int x, int y, int index) {
  GridNode* node = &node_pool_[index];
  if (!node_initialized_[index]) {
    node->x = x;
    node->y = y;
    node->g_cost = std::numeric_limits<double>::max();
    node->h_cost = 0.0;
    node->parent = nullptr;
    node_initialized_[index] = true;
  }
  return node;
}


std::vector<geometry_msgs::msg::Pose>
TestCostmapPlanner::add_straight_line_poses(
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & end,
  double resolution)
{
  std::vector<geometry_msgs::msg::Pose> out;

  double dx = end.position.x - start.position.x;
  double dy = end.position.y - start.position.y;
  double dist = std::hypot(dx, dy);

  if (dist == 0.0) {
    return out;
  }

  int steps = std::max(1, static_cast<int>(dist / resolution));

  for (int i = 1; i <= steps; ++i)
  {
    geometry_msgs::msg::Pose p = start;
    p.position.x = start.position.x + dx * (static_cast<double>(i) / steps);
    p.position.y = start.position.y + dy * (static_cast<double>(i) / steps);
    out.push_back(p);
  }

  return out;
}



std::vector<geometry_msgs::msg::Pose>
TestCostmapPlanner::densify_path(
  const std::vector<geometry_msgs::msg::Pose> & poses, 
  const geometry_msgs::msg::Pose & goal)
{
  if (poses.empty()) {
    return {};
  }

  // 1. Interpolate and densify positions
  std::vector<geometry_msgs::msg::Pose> dense_poses;
  dense_poses.push_back(poses.front());

  for (size_t i = 1; i < poses.size(); ++i)
  {
    auto seg = add_straight_line_poses(
      poses[i - 1],
      poses[i],
      costmap_meta_.resolution);

    dense_poses.insert(dense_poses.end(), seg.begin(), seg.end());
  }

  // 2. Calculate yaw orientations for intermediate waypoints
  for (size_t i = 0; i < dense_poses.size() - 1; ++i)
  {
    double dx = dense_poses[i + 1].position.x - dense_poses[i].position.x;
    double dy = dense_poses[i + 1].position.y - dense_poses[i].position.y;
    double yaw = std::atan2(dy, dx);

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    dense_poses[i].orientation.x = q.x();
    dense_poses[i].orientation.y = q.y();
    dense_poses[i].orientation.z = q.z();
    dense_poses[i].orientation.w = q.w();
  }

  // 3. Force the absolute last waypoint to match the exact goal orientation
  dense_poses.back().orientation = goal.orientation;

  return dense_poses;
}

/* --------------------- NEW FUNCTIONS ------------------------------ */

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::TestCostmapPlanner, easynav::PlannerMethodBase)

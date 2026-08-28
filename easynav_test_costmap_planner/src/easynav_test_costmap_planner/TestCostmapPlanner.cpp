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

struct GridNode
{
  int x, y;
  double cost;
  double priority;
  bool operator>(const GridNode & other) const
  {
    return priority > other.priority;
  }
};

static double heuristic(int x1, int y1, int x2, int y2)
{
  return std::hypot(x2 - x1, y2 - y1);
}

static std::vector<std::pair<int, int>> neighbors8 = {
  {-1, -1}, {-1, 0}, {-1, 1},
  {0, -1}, {0, 1},
  {1, -1}, {1, 0}, {1, 1}
};

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
  node->declare_parameter<double>(plugin_name + ".cost_factor", 2.0);
  node->declare_parameter<double>(plugin_name + ".inflation_penalty", 5.0);
  node->declare_parameter<double>(plugin_name + ".heuristic_scale", 1.0);
  node->declare_parameter<bool>(plugin_name + ".continuous_replan", true);

  node->get_parameter(plugin_name + ".cost_factor", cost_factor_);
  node->get_parameter(plugin_name + ".inflation_penalty", inflation_penalty_);
  node->get_parameter(plugin_name + ".heuristic_scale", heuristic_scale_);
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

  auto poses = a_star_path(map, robot_pose.pose.pose, goal);
  if (!poses.empty()) {
    // Apply a light smoothing to the raw grid path
    smooth_path(poses);

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

std::vector<geometry_msgs::msg::Pose> TestCostmapPlanner::a_star_path(
  const Costmap2D & map,
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal)
{
  unsigned int sx, sy, gx, gy;
  if (!map.worldToMap(start.position.x, start.position.y, sx, sy)) {return {};}
  if (!map.worldToMap(goal.position.x, goal.position.y, gx, gy)) {return {};}

  int width = map.getSizeInCellsX();
  // Precompute constants used inside the neighbor loop
  // const double axial_cost = 1.0;
  // const double diagonal_cost = std::sqrt(2.0);
  map_resolution = map.getResolution();

  auto idx = [&](int x, int y) {return y * width + x;};

  std::priority_queue<GridNode, std::vector<GridNode>, std::greater<>> open;

  const int height = map.getSizeInCellsY();
  const int total_cells = width * height;
  std::vector<int> gparent_x(total_cells, -1);
  std::vector<int> gparent_y(total_cells, -1);
  std::vector<int> parent_x(total_cells, -1);
  std::vector<int> parent_y(total_cells, -1);
  std::vector<double> cost_so_far(total_cells, std::numeric_limits<double>::infinity());
  std::vector<bool> visited(total_cells, false);

  const double initial_h = heuristic(static_cast<int>(sx), static_cast<int>(sy),
      static_cast<int>(gx), static_cast<int>(gy)) * heuristic_scale_;
  open.push(GridNode{static_cast<int>(sx), static_cast<int>(sy), 0.0, initial_h});
  cost_so_far[idx(sx, sy)] = 0.0;

  while (!open.empty()) {
    auto current = open.top();
    open.pop();
    int cid = idx(current.x, current.y);

    if (visited[cid]) {
      continue;
    }

    if (current.cost > cost_so_far[cid]) {
      continue;
    }

    if((parent_x[cid] != -1 && parent_y[cid] != -1) && (gparent_x[cid] != -1 && gparent_y[cid] != -1)) {
      if(lineOfSight(current.x, current.y, gparent_x[cid], gparent_y[cid], map)) {
        parent_x[cid] = gparent_x[cid];
        parent_y[cid] = gparent_y[cid];
      }
    }

    if (current.x == static_cast<int>(gx) && current.y == static_cast<int>(gy)) {break;}

    visited[cid] = true;

    for (auto [dx, dy] : neighbors8) {
      int nx = current.x + dx;
      int ny = current.y + dy;
      int nid = idx(nx, ny);

      if (!map.inBounds(nx, ny)) {continue;}

      uint8_t cell_cost = map.getCost(nx, ny);
      // Reject cells that would cause collision (>= INSCRIBED_INFLATED_OBSTACLE = 253)
      if (cell_cost >= INSCRIBED_INFLATED_OBSTACLE) {continue;}

      if (!visited[cid]) {continue;}

      int assumed_gparent_x = (parent_x[cid] != -1) ? parent_x[cid] : current.x;
      int assumed_gparent_y = (parent_y[cid] != -1) ? parent_y[cid] : current.y;
      int gid = idx(assumed_gparent_x, assumed_gparent_y);

      // Calculate traversal cost: cost_factor_ acts as a direct multiplier on cell cost
      double traversal_cost = 1.0 + cost_factor_ * static_cast<double>(cell_cost);

      double new_cost = cost_so_far[gid] + traversal_cost * heuristic(assumed_gparent_x, assumed_gparent_y, nx, ny);

      if (new_cost < cost_so_far[nid]) {
        cost_so_far[nid] = new_cost;
        const double h = heuristic(nx, ny, static_cast<int>(gx), static_cast<int>(gy)) *
          heuristic_scale_;
        open.push(GridNode{nx, ny, new_cost, new_cost + h});
        parent_x[nid] = current.x;
        parent_y[nid] = current.y;
      }
    }
  }

  std::vector<geometry_msgs::msg::Pose> path;
  int cx = static_cast<int>(gx), cy = static_cast<int>(gy);
  while (parent_x[idx(cx, cy)] != -1) {
    double wx, wy;
    map.mapToWorld(cx, cy, wx, wy);
    geometry_msgs::msg::Pose pose;
    pose.position.x = wx;
    pose.position.y = wy;
    pose.orientation = goal.orientation;
    path.push_back(pose);
    int px = parent_x[idx(cx, cy)];
    int py = parent_y[idx(cx, cy)];
    cx = px;
    cy = py;
  }
  std::reverse(path.begin(), path.end());

  if (path.empty()) {path.push_back(goal);}

  return densifyPath(path, goal);
}


bool 
TestCostmapPlanner::lineOfSight(
  int x0, int y0, 
  int x1, int y1,
  const Costmap2D & map)
{
  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  int max_x = static_cast<int>(map.getSizeInCellsX());
  int max_y = static_cast<int>(map.getSizeInCellsY());

  while (true)
  {
    // 1. Boundary check
    if (x0 < 0 || x0 >= max_x || y0 < 0 || y0 >= max_y) {
      return false;
    }

    // 2. Cost threshold check (Adjust 254 / LETHAL_OBSTACLE to preferred threshold)
    if (map.getCost(x0, y0) >= 10) {
      return false;
    }

    // Target reached
    if (x0 == x1 && y0 == y1) {
      break;
    }

    int e2 = 2 * err;

    // Prevent diagonal corner-cutting by handling steps strictly
    if (e2 > -dy && e2 < dx) {
      // Handles strictly diagonal step: check intermediate orthogonal steps if needed
      err -= dy;
      x0 += sx;
      err += dx;
      y0 += sy;
    } else {
      if (e2 > -dy) { 
        err -= dy; 
        x0 += sx; 
      }
      if (e2 < dx)  { 
        err += dx; 
        y0 += sy; 
      }
    }
  }

  return true;
}


std::vector<geometry_msgs::msg::Pose>
TestCostmapPlanner::addStraightLinePoses(
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
TestCostmapPlanner::densifyPath(
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
    auto seg = addStraightLinePoses(
      poses[i - 1],
      poses[i],
      map_resolution);

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


}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::TestCostmapPlanner, easynav::PlannerMethodBase)

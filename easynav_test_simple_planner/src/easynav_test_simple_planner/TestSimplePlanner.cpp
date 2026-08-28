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

#include <queue>
#include <unordered_map>
#include <cmath>

#include "easynav_test_simple_planner/TestSimplePlanner.hpp"
#include "easynav_common/RTTFBuffer.hpp"

#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/odometry.hpp"

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

double heuristic(int x1, int y1, int x2, int y2)
{
  return hypot(x2 - x1, y2 - y1);
}

std::vector<std::pair<int, int>> neighbors8 = {
  {-1, -1}, {-1, 0}, {-1, 1},
  {0, -1}, {0, 1},
  {1, -1}, {1, 0}, {1, 1}
};

double compute_path_length(const nav_msgs::msg::Path & path)
{
  double total_length = 0.0;

  if (path.poses.size() < 2) {
    return 0.0;
  }

  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & p1 = path.poses[i - 1].pose.position;
    const auto & p2 = path.poses[i].pose.position;

    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    total_length += std::sqrt(dx * dx + dy * dy);
  }

  return total_length;
}


TestSimplePlanner::TestSimplePlanner()
{
  NavState::register_printer<nav_msgs::msg::Path>(
    [](const nav_msgs::msg::Path & path) {
      std::ostringstream ret;

      ret << "{ " << rclcpp::Time(path.header.stamp).seconds() << " } Path with " <<
        path.poses.size() << " poses and length " <<
        compute_path_length(path) << " m.";

      return ret.str();
    });
}

void
TestSimplePlanner::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  node->declare_parameter<double>(plugin_name + ".robot_radius", 0.3);
  node->declare_parameter<double>(plugin_name + ".clearance_distance", 0.2);
  node->get_parameter<double>(plugin_name + ".robot_radius", robot_radius_);
  node->get_parameter<double>(plugin_name + ".clearance_distance", clearance_distance_);

  path_pub_ = get_node()->create_publisher<nav_msgs::msg::Path>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/path", 10);
}

void
TestSimplePlanner::update(NavState & nav_state)
{
  current_path_.poses.clear();

  if (!nav_state.has("goals")) {return;}
  if (!nav_state.has("robot_pose")) {return;}

  const auto & goals = nav_state.get<nav_msgs::msg::Goals>("goals");

  if (goals.goals.empty()) {
    nav_state.set("path", current_path_);
    return;
  }

  if (!nav_state.has("map")) {
    RCLCPP_WARN(get_node()->get_logger(), "TestSimplePlanner::update map map not found");
    return;
  }

  SimpleMap map_typed;
  if (nav_state.has("map")) {
    map_typed = nav_state.get<SimpleMap>("map");
  } else {
    RCLCPP_WARN(get_node()->get_logger(), "There is yet no a map");
    return;
  }

  const auto & robot_pose = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");
  const auto & goal = goals.goals.front().pose;
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();

  const auto clock_type = get_node()->get_clock()->get_clock_type();
  rclcpp::Time latest_stamp(robot_pose.header.stamp, clock_type);
  if (rclcpp::Time(goals.goals.front().header.stamp,
      latest_stamp.get_clock_type()) > latest_stamp)
  {
    latest_stamp = rclcpp::Time(goals.goals.front().header.stamp, latest_stamp.get_clock_type());
  }

  auto downsampled_map = map_typed.downsample(0.1);

  if (goals.header.frame_id != tf_info.map_frame) {
    RCLCPP_WARN(get_node()->get_logger(),
      "TestSimplePlanner::update goals frame is not map (%s)", goals.header.frame_id.c_str());
    return;
  }

  if (!downsampled_map->check_bounds_metric(goal.position.x, goal.position.y)) {
    RCLCPP_WARN(get_node()->get_logger(),
      "TestSimplePlanner::update goal (%lf, %lf) outside the map", goal.position.x, goal.position.y);
    return;
  }

  auto poses = test_planner_path(
    *downsampled_map,
    robot_pose.pose.pose,
    goal,
    downsampled_map->resolution());

  if (!poses.empty()) {
    current_path_.header.stamp = latest_stamp;
    current_path_.header.frame_id = goals.header.frame_id;

    for (const auto & pose : poses) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = goals.header.frame_id;
      pose_stamped.header.stamp = latest_stamp;
      pose_stamped.pose = pose;
      current_path_.poses.push_back(pose_stamped);
    }

    if (path_pub_->get_subscription_count() > 0) {
      path_pub_->publish(current_path_);
    }
  }

  nav_state.set("path", current_path_);
}


bool
TestSimplePlanner::isFreeWithClearance(
  const SimpleMap & map,
  int cx, int cy,
  double clearance_cells)
{
  int width = map.width();
  int height = map.height();
  int rad = std::ceil(clearance_cells);

  for (int dx = -rad; dx <= rad; ++dx) {
    for (int dy = -rad; dy <= rad; ++dy) {
      int nx = cx + dx;
      int ny = cy + dy;
      if (nx < 0 || ny < 0 || nx >= width || ny >= height) {continue;}
      if (std::hypot(dx, dy) <= clearance_cells && map.at(nx, ny)) {
        return false;
      }
    }
  }
  return true;
}

bool 
TestSimplePlanner::lineOfSight(
  int x0, int y0, 
  int x1, int y1,
  const SimpleMap & map,
  double min_clearance)
{
  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  int max_x = map.width();
  int max_y = map.height();

  while (true)
  {
    // Check map boundaries before reading cell values
    if (x0 < 0 || x0 >= max_x || y0 < 0 || y0 >= max_y) {
      return false;
    }

    // Evaluate cell occupancy and minimum obstacle clearance
    if ((map.at(x0, y0) != 0) || !isFreeWithClearance(map, x0, y0, min_clearance)) {
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
    if (e2 < dx)  { 
      err += dx; 
      y0 += sy; 
    }
  }

  return true;
}

std::vector<geometry_msgs::msg::Pose>
TestSimplePlanner::test_planner_path(
  const SimpleMap & map,
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal,
  double resolution)
{
  RCLCPP_DEBUG(get_node()->get_logger(), "Running DRSP Planner ============");
  RCLCPP_DEBUG(get_node()->get_logger(), "Path from (%lf m, %lf m) ->  (%lf m, %lf m)",
    start.position.x, start.position.y,
    goal.position.x, goal.position.y);

  int width = map.width();
  int height = map.height();

  auto [sx, sy] = map.metric_to_cell(start.position.x, start.position.y);
  auto [gx, gy] = map.metric_to_cell(goal.position.x, goal.position.y);

  RCLCPP_DEBUG(get_node()->get_logger(), "Path from (%d, %d) ->  (%d, %d)",
    sx, sy, gx, gy);

  std::priority_queue<GridNode, std::vector<GridNode>, std::greater<GridNode>> open;
  std::unordered_map<int, std::pair<int, int>> grandparent;
  std::unordered_map<int, std::pair<int, int>> parent;
  std::unordered_map<int, double> cost_so_far;
  std::unordered_map<int, bool> visited;

  auto idx = [&](int x, int y) {return y * width + x;};

  open.push({sx, sy, 0.0, heuristic(sx, sy, gx, gy)});
  cost_so_far[idx(sx, sy)] = 0.0;

  double min_clearance = (robot_radius_ + clearance_distance_) / resolution;

  while (!open.empty()) {
    auto current = open.top();
    open.pop();

    int cid = idx(current.x, current.y);

    // if (visited.find(cid) != visited.end()) {continue;}

    // if (cost_so_far.find(cid) != cost_so_far.end() && current.cost < cost_so_far[cid]) {continue;}

    if (parent.find(cid) != parent.end() && grandparent.find(cid) != grandparent.end()) {

      if (lineOfSight(current.x, current.y, grandparent[cid].first, grandparent[cid].second, map, min_clearance)) {
        parent[cid].first = grandparent[cid].first;
        parent[cid].second = grandparent[cid].second;
      }

      // else {
      //   double min_cost = std::numeric_limits<double>::infinity();
      //   std::pair<int, int> best_parent = {-1, -1};

      //   for (auto [dx, dy] : neighbors8) {
      //     int nx = current.x + dx;
      //     int ny = current.y + dy;
      //     int nid = idx(nx, ny);

      //     if (nx < 0 || ny < 0 || nx >= width || ny >= height) {continue;}
      //     if (!isFreeWithClearance(map, nx, ny, min_clearance)) {continue;}
      //     if (visited.find(cid) == visited.end()) {continue;}

      //     double cost_to_current = cost_so_far[nid] + hypot(dx, dy);
      //     if(cost_to_current < min_cost) {
      //       min_cost = cost_to_current;
      //       best_parent.first = nx;
      //       best_parent.second = ny;
      //     }
      //   }

      //   if (best_parent.first != -1 && best_parent.second != -1) {
      //     current.cost = min_cost;
      //     cost_so_far[cid] = min_cost;
      //     parent[cid].first = best_parent.first;
      //     parent[cid].second = best_parent.second;
      //   }
      // }
    }

    if (current.x == gx && current.y == gy) {break;}

    for (auto [dx, dy] : neighbors8) {
      int nx = current.x + dx;
      int ny = current.y + dy;
      int nid = idx(nx, ny);

      if (nx < 0 || ny < 0 || nx >= width || ny >= height) {continue;}
      if (!isFreeWithClearance(map, nx, ny, min_clearance)) {continue;}
      // if (visited.find(cid) != visited.end()) {continue;}

      std::pair<int, int> assumed_grandparent = (parent.find(cid) != parent.end()) 
                                                ? parent.at(cid) 
                                                : std::make_pair(current.x, current.y);
      int gid = idx(assumed_grandparent.first, assumed_grandparent.second);

      double new_cost = cost_so_far[gid] + heuristic(assumed_grandparent.first, assumed_grandparent.second, nx, ny);

      if (cost_so_far.find(nid) == cost_so_far.end() || new_cost < cost_so_far[nid]) {
        cost_so_far[nid] = new_cost;
        double priority = new_cost + heuristic(nx, ny, gx, gy);
        open.push({nx, ny, new_cost, priority});
        parent[nid] = {current.x, current.y};
      }
    }
  }

  std::vector<geometry_msgs::msg::Pose> path;
  int cx = gx, cy = gy;
  while (parent.find(idx(cx, cy) ) != parent.end()) {
    geometry_msgs::msg::Pose pose;

    auto [px, py] = map.cell_to_metric(cx, cy);
    pose.position.x = px;
    pose.position.y = py;
    pose.orientation = goal.orientation;

    path.push_back(pose);

    RCLCPP_DEBUG(get_node()->get_logger(), "\t(%d, %d) = (%lf m, %lf m)",
      cx, cy, px, py);

    std::tie(cx, cy) = parent[idx(cx, cy)];
  }
  std::reverse(path.begin(), path.end());

  if (path.empty()) {
    path.push_back(goal);
  }

  return path;
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::TestSimplePlanner, easynav::PlannerMethodBase)

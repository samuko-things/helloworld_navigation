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
  auto downsampled_map = map_typed.downsample(0.1);


  map_meta_.update(*downsampled_map);
  unsigned int map_size = map_meta_.size_x * map_meta_.size_y;

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

  clearance_cells_ = (robot_radius_ + clearance_distance_)*map_meta_.inv_resolution;


  const auto clock_type = get_node()->get_clock()->get_clock_type();
  rclcpp::Time latest_stamp(robot_pose.header.stamp, clock_type);
  if (rclcpp::Time(goals.goals.front().header.stamp,
      latest_stamp.get_clock_type()) > latest_stamp)
  {
    latest_stamp = rclcpp::Time(goals.goals.front().header.stamp, latest_stamp.get_clock_type());
  }

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

  auto poses = plan_path(*downsampled_map, robot_pose.pose.pose, goal);
  if (!poses.empty()) {
    // Apply a light smoothing to the raw grid path
    smooth_path(poses);

    current_path_.poses.clear();
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

std::vector<geometry_msgs::msg::Pose>
TestSimplePlanner::plan_path(
  const SimpleMap & map,
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal)
{
  RCLCPP_DEBUG(get_node()->get_logger(), "Running DRSP Planner ============");
  RCLCPP_DEBUG(get_node()->get_logger(), "Path from (%lf m, %lf m) ->  (%lf m, %lf m)",
    start.position.x, start.position.y,
    goal.position.x, goal.position.y);

  auto [sx, sy] = map.metric_to_cell(start.position.x, start.position.y);
  auto [gx, gy] = map.metric_to_cell(goal.position.x, goal.position.y);

  RCLCPP_DEBUG(get_node()->get_logger(), "Path from (%d, %d) ->  (%d, %d)",
    sx, sy, gx, gy);

  // Start & Goal Node Setup
  GridNode raw_start(sx, sy);
  int start_idx = gridToMapIndex(raw_start);
  GridNode* start_node = get_node_from_pool(raw_start.x, raw_start.y, start_idx);

  GridNode raw_goal(gx, gy);
  int goal_idx = gridToMapIndex(raw_goal);
  GridNode* goal_node = get_node_from_pool(raw_goal.x, raw_goal.y, goal_idx);

  GridNode* best_goal = runDRSPPlan(
    start_node, 
    goal_node, 
    map
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
    pose = gridToPose(*node);
    path.push_back(pose);

    if (node->prev == node) {
      break;
    }
    node = node->prev;
  }

  std::reverse(path.begin(), path.end());
  return densifyPath(path, goal);
}





/* --------------------- NEW FUNCTIONS ------------------------------ */

GridNode* TestSimplePlanner::runDRSPPlan(
  GridNode* start_node,
  GridNode* goal_node,
  const SimpleMap & map)
{
  std::priority_queue<
    GridNode*,
    std::vector<GridNode*>,
    TestSimplePlanner::CompareNode
  > nodes_to_explore;

  int start_idx = gridToMapIndex(*start_node);

  start_node->g_cost = 0.0;
  start_node->h_cost = euclidean_distance(*start_node, *goal_node);
  start_node->prev = start_node;
  // start_node->near_obstacle = true;

  nodes_to_explore.push(start_node);
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

  while (!nodes_to_explore.empty() /*&& rclcpp::ok()*/)
  {
    GridNode* active = nodes_to_explore.top();
    nodes_to_explore.pop();

    int active_idx = gridToMapIndex(*active);

    if (visited_[active_idx]) {
      continue;
    }

    if (active->g_cost > g_cost_cache_[active_idx]) {
      continue;
    }

    if (active->prev && active->prev->prev)
    {
      auto actual_grandparent = active->prev->prev;

      if (lineOfSight(active->x, active->y, actual_grandparent->x, actual_grandparent->y, map))
      {
        active->prev = actual_grandparent;
      }

      else
      {
        double min_g = std::numeric_limits<double>::infinity();
        GridNode* best_parent = nullptr;

        for (const auto & d : dirs)
        {
          int nx = active->x + d.dx;
          int ny = active->y + d.dy;
          GridNode nbr_pos(nx, ny);
          int nbr_idx = gridToMapIndex(nbr_pos);

          if (isGridOnMap(nbr_pos) && isMapCellFree(nbr_pos, map) && isFreeWithClearance(map, nx, ny, clearance_cells_) && visited_[nbr_idx])
          {
            GridNode* nbr_node = &node_pool_[nbr_idx];
            double g_val = g_cost_cache_[nbr_idx];
            double cost_to_active = g_val + d.dist;
            if (cost_to_active < min_g) {
              min_g = cost_to_active;
              best_parent = nbr_node;
            }
          }
        }

        if (best_parent) {
          active->g_cost = min_g;
          g_cost_cache_[active_idx] = min_g;
          active->prev = best_parent;
        }
      }
    }

    if (active->x == goal_node->x && active->y == goal_node->y) {
      return active;
    }

    visited_[active_idx] = true;

    for (const auto & d : dirs)
    {
      int nx = active->x + d.dx;
      int ny = active->y + d.dy;

      GridNode nbr_pos(nx, ny);
      int nbr_idx = gridToMapIndex(nbr_pos);

      if (isGridOnMap(nbr_pos) && isMapCellFree(nbr_pos, map) && isFreeWithClearance(map, nx, ny, clearance_cells_) && !visited_[nbr_idx]) {
        GridNode* assumed_grandparent = active->prev ? active->prev : active;
        GridNode* neighbor_node = get_node_from_pool(nx, ny, nbr_idx);

        double new_g_cost = assumed_grandparent->g_cost + euclidean_distance(nbr_pos, *(assumed_grandparent));
        double nbr_g_cost = g_cost_cache_[nbr_idx];

        if (nbr_g_cost < 0.0 || new_g_cost < nbr_g_cost)
        {
          g_cost_cache_[nbr_idx] = new_g_cost;

          neighbor_node->g_cost = new_g_cost;
          neighbor_node->h_cost = euclidean_distance(*neighbor_node, *goal_node);
          neighbor_node->prev = active;

          nodes_to_explore.push(neighbor_node);
        }

      }
    }
  }

  return nullptr;
}

GridNode TestSimplePlanner::poseToGrid(const geometry_msgs::msg::Pose &pose)
{
  int gx = static_cast<int>((pose.position.x - map_meta_.origin_x) * map_meta_.inv_resolution);
  int gy = static_cast<int>((pose.position.y - map_meta_.origin_y) * map_meta_.inv_resolution);

  return GridNode(gx, gy);
}

geometry_msgs::msg::Pose TestSimplePlanner::gridToPose(const GridNode &grid)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = grid.x * map_meta_.resolution + map_meta_.origin_x;
  pose.position.y = grid.y * map_meta_.resolution + map_meta_.origin_y;
  pose.position.z = 0.0;

  return pose;
}

int TestSimplePlanner::gridToMapIndex(const GridNode &grid_node)
{
  return static_cast<int>(grid_node.y * map_meta_.size_x + grid_node.x);
}

bool TestSimplePlanner::isGridOnMap(const GridNode &grid)
{
  return (grid.x >= 0 && grid.x < map_meta_.size_x &&
          grid.y >= 0 && grid.y < map_meta_.size_y);
}

bool TestSimplePlanner::isMapCellFree(const GridNode &grid, const SimpleMap & map)
{
  return (map.at(grid.x, grid.y) == 0);
}

double TestSimplePlanner::euclidean_distance(const GridNode &a, const GridNode &b)
{
  double dx = static_cast<double>(a.x - b.x);
  double dy = static_cast<double>(a.y - b.y);
  return std::sqrt(dx * dx + dy * dy);
}


bool TestSimplePlanner::lineOfSight(
  int x0, int y0, 
  int x1, int y1,
  const SimpleMap & map)
{
  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  int max_x = static_cast<int>(map_meta_.size_x);
  int max_y = static_cast<int>(map_meta_.size_y);

  while (true)
  {
    // Safety Guard: Check map boundaries
    if (x0 < 0 || x0 >= max_x || y0 < 0 || y0 >= max_y) {
      return false;
    }

    if ((map.at(x0, y0) != 0) || !isFreeWithClearance(map, x0, y0, clearance_cells_)) {
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


GridNode* TestSimplePlanner::get_node_from_pool(int x, int y, int index) {
  GridNode* node = &node_pool_[index];
  if (!node_initialized_[index]) {
    node->x = x;
    node->y = y;
    node->g_cost = std::numeric_limits<double>::max();
    node->h_cost = 0.0;
    node->prev = nullptr;
    node_initialized_[index] = true;
  }
  return node;
}


std::vector<geometry_msgs::msg::Pose>
TestSimplePlanner::addStraightLinePoses(
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
TestSimplePlanner::densifyPath(
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
      map_meta_.resolution);

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
PLUGINLIB_EXPORT_CLASS(easynav::TestSimplePlanner, easynav::PlannerMethodBase)

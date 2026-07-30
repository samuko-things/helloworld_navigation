#include "test_planner_plugin/test_planner_plugin.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <algorithm>

namespace test_planner_plugin
{


void TestPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock lifecycle node");
  }

  logger_ = node->get_logger();
  costmap_ros_ = costmap_ros;

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".cose_limit", rclcpp::ParameterValue(20.0));
  node->declare_parameter(name + ".cost_limit", 20.0);

  node->get_parameter(name + ".cost_limit", cost_limit_);

  RCLCPP_INFO_STREAM(logger_, "Configured Test Planner Plugin with cost_limit=" << cost_limit_);
}





void TestPlanner::activate()
{
  if (!costmap_ros_) {
    return;
  }

  auto costmap = costmap_ros_->getCostmap();
  costmap_meta_.update(costmap);

  RCLCPP_INFO_STREAM(logger_, "Plugin Activated Successfully");
}




void TestPlanner::deactivate() {}




void TestPlanner::cleanup()
{
  costmap_ros_.reset();

  RCLCPP_INFO_STREAM(logger_, "Plugin Cleaned Up Successfully");
}





nav_msgs::msg::Path TestPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  if (!costmap_ros_) {
    return nav_msgs::msg::Path();
  }

  auto costmap = costmap_ros_->getCostmap();

  // Atomically refresh metadata struct
  costmap_meta_.update(costmap);
  const unsigned char* char_map = costmap->getCharMap();

  auto start_node = std::make_shared<GridNode>(poseToGrid(start.pose));
  auto goal_node = std::make_shared<GridNode>(poseToGrid(goal.pose));

  // Execute Search

  // GridNode* best_goal = runLazyThetaStar(
  //   start_node, 
  //   goal_node, 
  //   cancel_checker, 
  //   char_map, 
  //   costmap_meta_.size_x
  // );

  auto best_goal = runTestPlan(
    start_node, 
    goal_node, 
    cancel_checker, 
    char_map, 
    costmap_meta_.size_x
  );


  // Reconstruction
  nav_msgs::msg::Path path;
  path.header.frame_id = costmap_ros_->getGlobalFrameID();

  if (!best_goal) {
    return path;
  }

  auto curr_node = best_goal;
  while (curr_node) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = path.header.frame_id;
    pose_stamped.header.stamp = path.header.stamp;
    pose_stamped.pose = gridToPose(*curr_node);
    
    path.poses.push_back(pose_stamped);
    curr_node = curr_node->prev; // Traces shared_ptr back safely
  }

  std::reverse(path.poses.begin(), path.poses.end());
  return path;

  // std::reverse(path.poses.begin(), path.poses.end());
  // return fillUpPath(path, goal);
}








std::shared_ptr<GridNode> TestPlanner::runTestPlan(
  std::shared_ptr<GridNode> start_node,
  std::shared_ptr<GridNode> goal_node,
  const std::function<bool()>& cancel_checker,
  const unsigned char* char_map,
  unsigned int size_x)
{
  std::vector<DirNode> explore_directions = {
      DirNode({-1, 0}, 1.0), 
      DirNode({1, 0}, 1.0), 
      DirNode({0, 1}, 1.0), 
      DirNode({0, -1}, 1.0),
      DirNode({-1, 1}, 1.4142), 
      DirNode({1, -1}, 1.4142), 
      DirNode({1, 1}, 1.4142), 
      DirNode({-1, -1}, 1.4142),
  };

  // Comparator comparing dereferenced shared pointers using your > operator
  auto comp = [](const std::shared_ptr<GridNode>& a, const std::shared_ptr<GridNode>& b) {
    return *a > *b;
  };

  std::priority_queue<
    std::shared_ptr<GridNode>, 
    std::vector<std::shared_ptr<GridNode>>, 
    decltype(comp)
  > nodes_to_explore(comp);

  int map_size = costmap_meta_.size_x * costmap_meta_.size_y;
  std::vector<bool> visited(map_size, false);

  start_node->h_cost = euclidean_distance(*start_node, *goal_node);
  nodes_to_explore.push(start_node);

  std::shared_ptr<GridNode> active_node = nullptr;

  while (!nodes_to_explore.empty() /*&& rclcpp::ok()*/) {
    if (cancel_checker && cancel_checker()) {
      return nullptr;
    }

    active_node = nodes_to_explore.top();
    nodes_to_explore.pop();
    
    int active_idx = gridToMapIndex(*active_node);
    if (visited[active_idx]) {
      continue;
    }
    visited[active_idx] = true;

    if (*active_node == *goal_node) {
      return active_node;
    }

    for (const auto &dir : explore_directions) {
      GridNode neighbor_pos = *active_node + dir.dir; 
      int neighbor_idx = gridToMapIndex(neighbor_pos);

      if (!visited[neighbor_idx] && isGridOnMap(neighbor_pos) && isMapCellFree(neighbor_pos, char_map)) {
        
        auto new_node = std::make_shared<GridNode>(neighbor_pos);

        // Check Theta* Line of Sight
        if (active_node->prev && lineOfSight(*new_node, *(active_node->prev), char_map, size_x)) {
          // Point directly to grandparent (preserves real memory pointer)
          new_node->prev = active_node->prev;
          new_node->g_cost = active_node->prev->g_cost 
                           + euclidean_distance(*new_node, *(active_node->prev)) 
                           + char_map[neighbor_idx];
        } else {
          // Standard step to parent
          new_node->prev = active_node;
          new_node->g_cost = active_node->g_cost 
                           + dir.t_cost 
                           + char_map[neighbor_idx];
        }

        new_node->h_cost = euclidean_distance(*new_node, *goal_node);
        nodes_to_explore.push(new_node);
      }
    }
  }

  return nullptr;
}









GridNode TestPlanner::poseToGrid(const geometry_msgs::msg::Pose &pose)
{
  int gx = static_cast<int>((pose.position.x - costmap_meta_.origin_x) * costmap_meta_.inv_resolution);
  int gy = static_cast<int>((pose.position.y - costmap_meta_.origin_y) * costmap_meta_.inv_resolution);

  return GridNode(gx, gy);
}

geometry_msgs::msg::Pose TestPlanner::gridToPose(const GridNode &grid)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = grid.x * costmap_meta_.resolution + costmap_meta_.origin_x;
  pose.position.y = grid.y * costmap_meta_.resolution + costmap_meta_.origin_y;
  pose.position.z = 0.0;

  return pose;
}

int TestPlanner::gridToMapIndex(const GridNode &grid_node)
{
  return static_cast<int>(grid_node.y * costmap_meta_.size_x + grid_node.x);
}

bool TestPlanner::isGridOnMap(const GridNode &grid)
{
  return (grid.x >= 0 && grid.x < costmap_meta_.size_x &&
          grid.y >= 0 && grid.y < costmap_meta_.size_y);
}

bool TestPlanner::isMapCellFree(const GridNode &grid, const unsigned char* char_map)
{
  return /*(char_map[gridToMapIndex(grid)] >= 0) &&*/ (char_map[gridToMapIndex(grid)] < 99);
}


double TestPlanner::euclidean_distance(const GridNode &a, const GridNode &b){
  return std::hypot(a.x - b.x, a.y - b.y);
}


bool TestPlanner::lineOfSight(
  const GridNode &start, 
  const GridNode &end,
  const unsigned char* char_map,
  unsigned int size_x) const
{
  int x0 = start.x; int y0 = start.y;
  int x1 = end.x; int y1 = end.y;

  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  // Calculate pre-computed memory offsets (strides)
  int stride_x = sx; 
  int stride_y = sy * static_cast<int>(size_x);

  // Compute the starting flat memory index
  int current_idx = y0 * size_x + x0;

  while (true)
  {
    // Direct, ultra-fast array lookup bypassing getCost() overhead
    if (char_map[current_idx] > cost_limit_)
      return false;

    if (x0 == x1 && y0 == y1)
      break;

    int e2 = 2 * err;
    if (e2 > -dy) { 
      err -= dy; 
      x0 += sx; 
      current_idx += stride_x; // Direct index stride addition
    }
    if (e2 < dx)  { 
      err += dx; 
      y0 += sy; 
      current_idx += stride_y; // Direct index stride addition
    }
  }

  return true;
}


std::vector<geometry_msgs::msg::PoseStamped>
TestPlanner::addStraightLinePoses(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & end,
  double resolution) const
{
  std::vector<geometry_msgs::msg::PoseStamped> out;

  double dx = end.pose.position.x - start.pose.position.x;
  double dy = end.pose.position.y - start.pose.position.y;
  double dist = std::hypot(dx, dy);

  if (dist == 0)
    return {};

  int steps = std::max(1, static_cast<int>(dist / resolution));

  for (int i = 1; i <= steps; i++)
  {
    geometry_msgs::msg::PoseStamped ps = start;
    ps.pose.position.x = start.pose.position.x + dx * (i / (double)steps);
    ps.pose.position.y = start.pose.position.y + dy * (i / (double)steps);
    out.push_back(ps);
  }

  return out;
}



nav_msgs::msg::Path
TestPlanner::fillUpPath(
  const nav_msgs::msg::Path & path, 
  const geometry_msgs::msg::PoseStamped & goal) const
{
  nav_msgs::msg::Path filled;
  filled.header = path.header;

  if (path.poses.empty())
    return filled;

  // 1. Interpolate and densify the positions
  filled.poses.push_back(path.poses.front());
  for (size_t i = 1; i < path.poses.size(); i++)
  {
    auto seg = addStraightLinePoses(
      path.poses[i - 1],
      path.poses[i],
      costmap_ros_->getCostmap()->getResolution());

    filled.poses.insert(filled.poses.end(), seg.begin(), seg.end());
  }

  // 2. Calculate Yaw Orientations for intermediate waypoints
  for (size_t i = 0; i < filled.poses.size() - 1; i++)
  {
    double dx = filled.poses[i+1].pose.position.x - filled.poses[i].pose.position.x;
    double dy = filled.poses[i+1].pose.position.y - filled.poses[i].pose.position.y;
    double yaw = std::atan2(dy, dx);

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    filled.poses[i].pose.orientation.x = q.x();
    filled.poses[i].pose.orientation.y = q.y();
    filled.poses[i].pose.orientation.z = q.z();
    filled.poses[i].pose.orientation.w = q.w();
  }

  // 3. Force the absolute last waypoint to match the exact goal orientation
  filled.poses.back().pose.orientation = goal.pose.orientation;

  return filled;
}

}  // namespace theta_star_smooth_planner_plugin

PLUGINLIB_EXPORT_CLASS(
  test_planner_plugin::TestPlanner,
  nav2_core::GlobalPlanner)
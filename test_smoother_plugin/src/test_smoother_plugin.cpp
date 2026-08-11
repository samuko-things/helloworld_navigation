#include "test_smoother_plugin/test_smoother_plugin.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <algorithm>

namespace test_smoother_plugin
{

using namespace std::chrono;


void TestSmoother::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> /*footprint_sub*/)
{
  costmap_sub_ = costmap_sub;
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock LifecycleNode in TestSmoother::configure");
  }
  logger_ = node->get_logger();

  // Parameters
  if (!node->has_parameter(name + ".iterations")) {
    node->declare_parameter(name + ".iterations", 1);
  }
  if (!node->has_parameter(name + ".cost_limit")) {
    node->declare_parameter(name + ".cost_limit", 20);
  }

  // Parameters for Laplacian Relaxation
  if (!node->has_parameter(name + ".w_data")) {
    node->declare_parameter(name + ".w_data", 0.2);
  }
  if (!node->has_parameter(name + ".w_smooth")) {
    node->declare_parameter(name + ".w_smooth", 0.3);
  }
  if (!node->has_parameter(name + ".tolerance")) {
    node->declare_parameter(name + ".tolerance", 1e-4);
  }
  if (!node->has_parameter(name + ".max_its")) {
    node->declare_parameter(name + ".max_its", 200);
  }


  node->get_parameter(name + ".iterations", iterations_);
  node->get_parameter(name + ".cost_limit", cost_limit_);

  node->get_parameter(name + ".w_data", w_data_);
  node->get_parameter(name + ".w_smooth", w_smooth_);
  node->get_parameter(name + ".tolerance", tolerance_);
  node->get_parameter(name + ".max_its", max_its_);

  cost_limit_ = std::clamp(cost_limit_, 0, 255);
}




bool TestSmoother::smooth(nav_msgs::msg::Path & path, const rclcpp::Duration & /*max_time*/)
{
  if (!costmap_sub_) {
    RCLCPP_ERROR(logger_, "Costmap subscriber not configured");
    return false;
  }

  auto costmap = costmap_sub_->getCostmap();
  if (!costmap) {
    RCLCPP_ERROR(logger_, "Costmap not available");
    return false;
  }

  // Lock the costmap mutex to ensure thread-safe access to raw map memory during string pulling
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  costmap_meta_.update(costmap.get());
  const unsigned char* char_map = costmap->getCharMap();

  // string pull the path
  path = greedyStringPullSmooth(path, char_map, costmap_meta_.size_x);

  // fillup path
  path = fillUpPath(path);

  // Laplacian relaxation (rounds off corners)
  path = laplacianSmooth(path, char_map);

  // Orientation recalculation (syncs yaw headings to rounded curve)
  updateOrientations(path);
  
  return true;
}



bool TestSmoother::isGridOnMap(const GridNode &grid)
{
  return (grid.x >= 0 && grid.x < costmap_meta_.size_x &&
          grid.y >= 0 && grid.y < costmap_meta_.size_y);
}

int TestSmoother::gridToMapIndex(const GridNode &grid_node)
{
  return static_cast<int>(grid_node.y * costmap_meta_.size_x + grid_node.x);
}

unsigned char TestSmoother::getGridCost(const GridNode &grid, const unsigned char* char_map)
{
  return char_map[gridToMapIndex(grid)];
}

GridNode TestSmoother::poseToGrid(const geometry_msgs::msg::Pose &pose)
{
  int gx = static_cast<int>((pose.position.x - costmap_meta_.origin_x) * costmap_meta_.inv_resolution);
  int gy = static_cast<int>((pose.position.y - costmap_meta_.origin_y) * costmap_meta_.inv_resolution);

  gx = std::clamp(gx, 0, std::max(0, costmap_meta_.size_x - 1));
  gy = std::clamp(gy, 0, std::max(0, costmap_meta_.size_y - 1));

  return GridNode(gx, gy);
}




nav_msgs::msg::Path TestSmoother::greedyStringPullSmooth(
    const nav_msgs::msg::Path& npath,
    const unsigned char* char_map,
    unsigned int size_x) 
{

  std::vector<geometry_msgs::msg::PoseStamped> poses = npath.poses;
  std::vector<geometry_msgs::msg::PoseStamped> new_poses = npath.poses;
  new_poses.clear();
  
  nav_msgs::msg::Path smoothed_path;
  smoothed_path.header = npath.header;
  
  if (poses.size() <= 2) {
      return npath;
  }

  int i = 0;
  int j = 2;
  int n = static_cast<int>(poses.size());

  new_poses.push_back(poses[i]);

  while (true) {
    if (!(j<n)){
      break;
    }
    else if (!lineOfSight(poseToGrid(poses[i].pose), poseToGrid(poses[j].pose), char_map, size_x)) {
        i = j - 1;
        new_poses.push_back(poses[i]);
    }
    j += 1;
  }
  i = j-1;
  new_poses.push_back(poses[i]);

  smoothed_path.poses = new_poses;
  return smoothed_path;
}








bool TestSmoother::lineOfSight(
  const GridNode &start, 
  const GridNode &end,
  const unsigned char* char_map,
  unsigned int size_x)
{
  if (!isGridOnMap(start) || !isGridOnMap(end)) {
    return false;
  }

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
  int current_idx = y0 * static_cast<int>(size_x) + x0;

  while (true)
  {
    // Boundary check safety guard
    if (!isGridOnMap(GridNode(x0, y0))) {
      return false;
    }

    // Direct, ultra-fast array lookup bypassing getCost() overhead
    if (char_map[current_idx] > static_cast<unsigned char>(cost_limit_))
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
TestSmoother::addStraightLinePoses(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & end,
  double resolution)
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



nav_msgs::msg::Path TestSmoother::fillUpPath(const nav_msgs::msg::Path & path)
{
  nav_msgs::msg::Path filled;
  filled.header = path.header;

  if (path.poses.empty()) {
    return filled;
  }

  // 1. Interpolate and densify positions
  filled.poses.push_back(path.poses.front());
  for (size_t i = 1; i < path.poses.size(); i++) {
    auto seg = addStraightLinePoses(
      path.poses[i - 1],
      path.poses[i],
      costmap_meta_.resolution);

    filled.poses.insert(filled.poses.end(), seg.begin(), seg.end());
  }

  // 2. Compute straight-line heading orientation (yaw) for intermediate waypoints
  for (size_t i = 0; i < filled.poses.size() - 1; i++) {
    double dx = filled.poses[i + 1].pose.position.x - filled.poses[i].pose.position.x;
    double dy = filled.poses[i + 1].pose.position.y - filled.poses[i].pose.position.y;
    double yaw = std::atan2(dy, dx);

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    filled.poses[i].pose.orientation = tf2::toMsg(q);
  }

  // Preserve the exact final goal orientation requested by the planner
  filled.poses.back().pose.orientation = path.poses.back().pose.orientation;

  return filled;
}



nav_msgs::msg::Path TestSmoother::laplacianSmooth(
  const nav_msgs::msg::Path & path,
  const unsigned char* char_map)
{
  if (path.poses.size() <= 2) {
    return path;
  }

  nav_msgs::msg::Path smoothed = path;
  const nav_msgs::msg::Path original = path;
  const size_t path_size = path.poses.size();

  double change = tolerance_;
  int its = 0;

  while (change >= tolerance_ && its < max_its_) {
    change = 0.0;
    its++;

    nav_msgs::msg::Path current_pass = smoothed;

    // Preserve start (index 0) and goal (index path_size - 1) anchor points
    for (size_t i = 1; i < path_size - 1; ++i) {
      double x_orig = original.poses[i].pose.position.x;
      double y_orig = original.poses[i].pose.position.y;

      double x_i = current_pass.poses[i].pose.position.x;
      double y_i = current_pass.poses[i].pose.position.y;

      double x_prev = current_pass.poses[i - 1].pose.position.x;
      double y_prev = current_pass.poses[i - 1].pose.position.y;

      double x_next = current_pass.poses[i + 1].pose.position.x;
      double y_next = current_pass.poses[i + 1].pose.position.y;

      // Mass-spring relaxation step
      double x_new = x_i + w_data_ * (x_orig - x_i) + w_smooth_ * (x_next + x_prev - 2.0 * x_i);
      double y_new = y_i + w_data_ * (y_orig - y_i) + w_smooth_ * (y_next + y_prev - 2.0 * y_i);

      // Verify safety against costmap for the NEW coordinate
      geometry_msgs::msg::Pose test_pose = current_pass.poses[i].pose;
      test_pose.position.x = x_new;
      test_pose.position.y = y_new;

      GridNode grid = poseToGrid(test_pose);
      if (isGridOnMap(grid)) {
        unsigned char cost = getGridCost(grid, char_map);

        // Abort relaxation if point enters an obstacle/infeasible cell
        if (cost > static_cast<unsigned char>(cost_limit_) && cost != nav2_costmap_2d::NO_INFORMATION) {
          RCLCPP_DEBUG(logger_, "Smoothing pass reached collision bound at iteration %d. Reverting.", its);
          return smoothed; // Return last safe trajectory state
        }
      }

      change += std::hypot(x_new - x_i, y_new - y_i);

      smoothed.poses[i].pose.position.x = x_new;
      smoothed.poses[i].pose.position.y = y_new;
    }
  }

  return smoothed;
}

void TestSmoother::updateOrientations(nav_msgs::msg::Path & path)
{
  if (path.poses.size() <= 1) {
    return;
  }

  // Re-estimate heading quaternions (yaw) along the rounded points
  for (size_t i = 0; i < path.poses.size() - 1; ++i) {
    double dx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
    double dy = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
    double yaw = std::atan2(dy, dx);

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    path.poses[i].pose.orientation = tf2::toMsg(q);
  }
}




}  // namespace test_smoother_plugin

PLUGINLIB_EXPORT_CLASS(
  test_smoother_plugin::TestSmoother,
  nav2_core::Smoother)
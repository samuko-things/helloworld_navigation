#include "test_smoother_plugin/test_smoother_plugin.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <cmath>
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

  // Existing Parameters
  if (!node->has_parameter(name + ".iterations")) {
    node->declare_parameter(name + ".iterations", 1);
  }
  if (!node->has_parameter(name + ".cost_limit")) {
    node->declare_parameter(name + ".cost_limit", 20);
  }

  if (!node->has_parameter(name + ".chaikin_iterations")) {
    node->declare_parameter(name + ".chaikin_iterations", 2);
  }
  if (!node->has_parameter(name + ".target_spacing")) {
    node->declare_parameter(name + ".target_spacing", 0.05);
  }
  if (!node->has_parameter(name + ".min_segment_dist")) {
    node->declare_parameter(name + ".min_segment_dist", 0.10);
  }

  // --- NEW: Quadratic Bézier Parameters ---
  // if (!node->has_parameter(name + ".max_radius")) {
  //   node->declare_parameter(name + ".max_radius", 0.5);
  // }
  // if (!node->has_parameter(name + ".curve_resolution")) {
  //   node->declare_parameter(name + ".curve_resolution", 8);
  // }

  // Get parameter values
  node->get_parameter(name + ".cost_limit", cost_limit_);

  node->get_parameter(name + ".chaikin_iterations", chaikin_iterations_);
  node->get_parameter(name + ".target_spacing", target_spacing_);
  node->get_parameter(name + ".min_segment_dist", min_segment_dist_);

  // node->get_parameter(name + ".max_radius", max_radius_);
  // node->get_parameter(name + ".curve_resolution", curve_resolution_);

  cost_limit_ = std::clamp(cost_limit_, 0, 255);

  // max_radius_ = std::max(0.0, max_radius_);
  // curve_resolution_ = std::max(1, curve_resolution_);
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

  // TBD :add cuvrves to the string pulled path
  path = smoothStringPulledPath(path);
  
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







nav_msgs::msg::Path TestSmoother::smoothStringPulledPath(const nav_msgs::msg::Path & input_path)
{
  const auto & raw_poses = input_path.poses;
  const size_t raw_n = raw_poses.size();

  nav_msgs::msg::Path smooth_path;
  smooth_path.header = input_path.header;

  if (raw_poses.empty()) {
    return smooth_path;
  }

  // Fallback to costmap resolution if target_spacing parameter is <= 0
  const double step_spacing = (target_spacing_ > 0.0) ? target_spacing_ : costmap_meta_.resolution;

  // =========================================================================
  // STAGE 1: Chaikin Corner Cutting with Configurable Iterations & Min Distance
  // =========================================================================
  std::vector<geometry_msgs::msg::PoseStamped> chaikin_poses = raw_poses;

  // Only run Chaikin if interior corners exist (>= 3 points)
  if (raw_n >= 3) {
    for (int pass = 0; pass < chaikin_iterations_; ++pass) {
      std::vector<geometry_msgs::msg::PoseStamped> next_poses;
      next_poses.push_back(chaikin_poses.front()); // Preserve start pose

      for (size_t i = 0; i < chaikin_poses.size() - 1; ++i) {
        const auto & poseA = chaikin_poses[i];
        const auto & poseB = chaikin_poses[i + 1];

        double dx = poseB.pose.position.x - poseA.pose.position.x;
        double dy = poseB.pose.position.y - poseA.pose.position.y;
        double seg_len = std::hypot(dx, dy);

        // MINIMUM DISTANCE CHECK:
        // Skip cutting if segment is shorter than min_segment_dist_
        if (seg_len <= min_segment_dist_) {
          next_poses.push_back(poseB);
          continue;
        }

        // Q = 0.75 * A + 0.25 * B
        geometry_msgs::msg::PoseStamped poseQ = poseA;
        poseQ.pose.position.x = 0.75 * poseA.pose.position.x + 0.25 * poseB.pose.position.x;
        poseQ.pose.position.y = 0.75 * poseA.pose.position.y + 0.25 * poseB.pose.position.y;

        // R = 0.25 * A + 0.75 * B
        geometry_msgs::msg::PoseStamped poseR = poseA;
        poseR.pose.position.x = 0.25 * poseA.pose.position.x + 0.75 * poseB.pose.position.x;
        poseR.pose.position.y = 0.25 * poseA.pose.position.y + 0.75 * poseB.pose.position.y;

        next_poses.push_back(poseQ);
        next_poses.push_back(poseR);
      }

      next_poses.push_back(chaikin_poses.back()); // Preserve goal pose
      chaikin_poses = std::move(next_poses);
    }
  }

  // =========================================================================
  // STAGE 2: Fixed Arc-Length Resampling (Using target_spacing_)
  // =========================================================================
  std::vector<geometry_msgs::msg::PoseStamped> uniform_poses;

  if (chaikin_poses.size() < 2) {
    smooth_path.poses = chaikin_poses;
    return smooth_path;
  }

  // 1. Calculate cumulative path distance
  std::vector<double> cum_dist(chaikin_poses.size(), 0.0);
  for (size_t i = 0; i < chaikin_poses.size() - 1; ++i) {
    double dx = chaikin_poses[i + 1].pose.position.x - chaikin_poses[i].pose.position.x;
    double dy = chaikin_poses[i + 1].pose.position.y - chaikin_poses[i].pose.position.y;
    cum_dist[i + 1] = cum_dist[i] + std::hypot(dx, dy);
  }

  double total_length = cum_dist.back();

  if (total_length < 1e-4) {
    smooth_path.poses = {chaikin_poses.front()};
    return smooth_path;
  }

  // 2. Interpolate waypoints strictly every step_spacing
  uniform_poses.push_back(chaikin_poses.front()); // Preserve exact start pose

  double current_target_dist = step_spacing;
  size_t idx = 0;

  while (current_target_dist < total_length) {
    while (idx < chaikin_poses.size() - 2 && cum_dist[idx + 1] < current_target_dist) {
      idx++;
    }

    const auto & pA = chaikin_poses[idx];
    const auto & pB = chaikin_poses[idx + 1];

    double seg_dist_A = cum_dist[idx];
    double seg_dist_B = cum_dist[idx + 1];
    double seg_len = seg_dist_B - seg_dist_A;

    double t = (seg_len > 1e-6) ? ((current_target_dist - seg_dist_A) / seg_len) : 0.0;

    geometry_msgs::msg::PoseStamped interp_pose = pA;
    interp_pose.pose.position.x = pA.pose.position.x + t * (pB.pose.position.x - pA.pose.position.x);
    interp_pose.pose.position.y = pA.pose.position.y + t * (pB.pose.position.y - pA.pose.position.y);
    interp_pose.pose.position.z = pA.pose.position.z;

    uniform_poses.push_back(interp_pose);

    current_target_dist += step_spacing;
  }

  // Always retain the exact destination pose
  uniform_poses.push_back(raw_poses.back());

  // =========================================================================
  // STAGE 3: Calculate Orientation Yaw Quaternions
  // =========================================================================
  for (size_t k = 0; k < uniform_poses.size() - 1; ++k) {
    double dx = uniform_poses[k + 1].pose.position.x - uniform_poses[k].pose.position.x;
    double dy = uniform_poses[k + 1].pose.position.y - uniform_poses[k].pose.position.y;
    double yaw = 0.0;

    if (std::hypot(dx, dy) > 1e-4) {
      yaw = std::atan2(dy, dx);
    } else if (k > 0) {
      yaw = tf2::getYaw(uniform_poses[k - 1].pose.orientation);
    }

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    uniform_poses[k].pose.orientation = tf2::toMsg(q);
  }

  // Preserve final goal orientation requested by planner
  uniform_poses.back().pose.orientation = raw_poses.back().pose.orientation;

  smooth_path.poses = std::move(uniform_poses);
  return smooth_path;
}










// nav_msgs::msg::Path TestSmoother::smoothStringPulledPath(const nav_msgs::msg::Path & input_path)
// {
//   const auto & raw_poses = input_path.poses;
//   const size_t raw_n = raw_poses.size();

//   nav_msgs::msg::Path smooth_path;
//   smooth_path.header = input_path.header;

//   if (raw_poses.empty()) {
//     return smooth_path;
//   }

//   // --------------------------------------------------------------------------
//   // Step 0: Pre-filter micro-waypoints caused by costmap grid rasterization
//   // --------------------------------------------------------------------------
//   std::vector<geometry_msgs::msg::PoseStamped> sparse_poses;
//   sparse_poses.push_back(raw_poses.front());

//   // Minimum distance threshold between waypoints (3x grid resolution)
//   const double min_waypoint_dist = 3.0 * costmap_meta_.resolution;

//   for (size_t i = 1; i < raw_n - 1; ++i) {
//     double dist = std::hypot(
//       raw_poses[i].pose.position.x - sparse_poses.back().pose.position.x,
//       raw_poses[i].pose.position.y - sparse_poses.back().pose.position.y);

//     if (dist >= min_waypoint_dist) {
//       sparse_poses.push_back(raw_poses[i]);
//     }
//   }

//   // Always retain the exact final goal pose
//   if (raw_n > 1) {
//     sparse_poses.push_back(raw_poses.back());
//   }

//   const size_t n = sparse_poses.size();

//   // --------------------------------------------------------------------------
//   // Case 1: Less than 3 points - No interior corners to smooth
//   // --------------------------------------------------------------------------
//   if (n < 3) {
//     smooth_path.poses.push_back(sparse_poses.front());
//     for (size_t i = 1; i < n; ++i) {
//       auto seg = addStraightLinePoses(
//         sparse_poses[i - 1],
//         sparse_poses[i],
//         costmap_meta_.resolution);

//       smooth_path.poses.insert(smooth_path.poses.end(), seg.begin(), seg.end());
//     }

//     // Compute straight-line heading orientation (yaw)
//     for (size_t i = 0; i < smooth_path.poses.size() - 1; ++i) {
//       double dx = smooth_path.poses[i + 1].pose.position.x - smooth_path.poses[i].pose.position.x;
//       double dy = smooth_path.poses[i + 1].pose.position.y - smooth_path.poses[i].pose.position.y;
//       double yaw = std::atan2(dy, dx);

//       tf2::Quaternion q;
//       q.setRPY(0.0, 0.0, yaw);
//       smooth_path.poses[i].pose.orientation = tf2::toMsg(q);
//     }

//     smooth_path.poses.back().pose.orientation = sparse_poses.back().pose.orientation;
//     return smooth_path;
//   }

//   // --------------------------------------------------------------------------
//   // Case 2: >= 3 points - Densified Straight Lines + Smooth Bézier Curves
//   // --------------------------------------------------------------------------
//   std::vector<geometry_msgs::msg::PoseStamped> dense_poses;

//   // Track endpoint of the previous segment (starts at original Start pose)
//   geometry_msgs::msg::PoseStamped prev_curve_end = sparse_poses[0];
//   dense_poses.push_back(prev_curve_end);

//   // Process each interior corner
//   for (size_t i = 1; i < n - 1; ++i) {
//     const auto & posA = sparse_poses[i - 1].pose.position;
//     const auto & posB = sparse_poses[i].pose.position;      // Corner vertex
//     const auto & posC = sparse_poses[i + 1].pose.position;

//     // 1. Calculate incoming (AB) and outgoing (BC) vectors
//     double ab_x = posB.x - posA.x;
//     double ab_y = posB.y - posA.y;
//     double bc_x = posC.x - posB.x;
//     double bc_y = posC.y - posB.y;

//     double len_AB = std::hypot(ab_x, ab_y);
//     double len_BC = std::hypot(bc_x, bc_y);

//     if (len_AB < 1e-4 || len_BC < 1e-4) {
//       continue;
//     }

//     // Normalized directions
//     double dir_ab_x = ab_x / len_AB;
//     double dir_ab_y = ab_y / len_AB;
//     double dir_bc_x = bc_x / len_BC;
//     double dir_bc_y = bc_y / len_BC;

//     // Skip shallow/near-collinear corners (angle < ~11 degrees)
//     double dot_product = dir_ab_x * dir_bc_x + dir_ab_y * dir_bc_y;
//     if (dot_product > 0.98) {
//       continue;
//     }

//     // 2. Dynamic Safety Clamping (d cannot exceed 45% of either adjacent segment)
//     double max_safe_d = std::min(len_AB * 0.45, len_BC * 0.45);
//     double d = std::min(max_radius_, max_safe_d);

//     // 3. Compute Bézier Control Points
//     double p0_x = posB.x - (dir_ab_x * d);
//     double p0_y = posB.y - (dir_ab_y * d);

//     double p1_x = posB.x;
//     double p1_y = posB.y;

//     double p2_x = posB.x + (dir_bc_x * d);
//     double p2_y = posB.y + (dir_bc_y * d);

//     // Build P0 Pose (Start of Bézier Curve)
//     geometry_msgs::msg::PoseStamped p0_pose;
//     p0_pose.header = input_path.header;
//     p0_pose.pose.position.x = p0_x;
//     p0_pose.pose.position.y = p0_y;
//     p0_pose.pose.position.z = posB.z;

//     // 4. Densify straight line from end of previous curve to P0
//     auto straight_seg = addStraightLinePoses(
//       prev_curve_end,
//       p0_pose,
//       costmap_meta_.resolution);
    
//     dense_poses.insert(dense_poses.end(), straight_seg.begin(), straight_seg.end());

//     // 5. Sample the Quadratic Bézier Curve (P0 -> P2)
//     for (int step = 1; step <= curve_resolution_; ++step) {
//       double t = static_cast<double>(step) / static_cast<double>(curve_resolution_);
//       double one_minus_t = 1.0 - t;

//       double curve_x = (one_minus_t * one_minus_t * p0_x) +
//                        (2.0 * one_minus_t * t * p1_x) +
//                        (t * t * p2_x);

//       double curve_y = (one_minus_t * one_minus_t * p0_y) +
//                        (2.0 * one_minus_t * t * p1_y) +
//                        (t * t * p2_y);

//       geometry_msgs::msg::PoseStamped pose;
//       pose.header = input_path.header;
//       pose.pose.position.x = curve_x;
//       pose.pose.position.y = curve_y;
//       pose.pose.position.z = posB.z;

//       dense_poses.push_back(pose);
//     }

//     prev_curve_end = dense_poses.back();
//   }

//   // 6. Densify final straight line from last curve's end (P2) to Goal
//   auto final_straight_seg = addStraightLinePoses(
//     prev_curve_end,
//     sparse_poses.back(),
//     costmap_meta_.resolution);

//   dense_poses.insert(dense_poses.end(), final_straight_seg.begin(), final_straight_seg.end());

//   // 7. Compute Orientations (Yaw quaternions)
//   for (size_t k = 0; k < dense_poses.size() - 1; ++k) {
//     double dx = dense_poses[k + 1].pose.position.x - dense_poses[k].pose.position.x;
//     double dy = dense_poses[k + 1].pose.position.y - dense_poses[k].pose.position.y;
//     double yaw = 0.0;

//     if (std::hypot(dx, dy) > 1e-4) {
//       yaw = std::atan2(dy, dx);
//     } else if (k > 0) {
//       yaw = tf2::getYaw(dense_poses[k - 1].pose.orientation);
//     }

//     tf2::Quaternion q;
//     q.setRPY(0.0, 0.0, yaw);
//     dense_poses[k].pose.orientation = tf2::toMsg(q);
//   }

//   // Preserve the exact final goal orientation requested by the planner
//   dense_poses.back().pose.orientation = sparse_poses.back().pose.orientation;

//   smooth_path.poses = std::move(dense_poses);
//   return smooth_path;
// }



}  // namespace test_smoother_plugin

PLUGINLIB_EXPORT_CLASS(
  test_smoother_plugin::TestSmoother,
  nav2_core::Smoother)
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose, Quaternion

from math import hypot, cos, sin, atan2
import numpy as np


class GridNode:
  def __init__(self, x, y):
    self.x = x
    self.y = y


class StringPullBezierSmoother(Node):
  def __init__(self):
    super().__init__("string_pull_bezier_smoother")

    default_qos = QoSProfile(depth=10)

    map_qos = QoSProfile(depth=10)
    map_qos.reliability = ReliabilityPolicy.RELIABLE
    map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

    # read incomming map
    self.map_subscriber = self.create_subscription(
      OccupancyGrid,
      "/global_costmap/costmap",
      self.map_callback,
      map_qos
    )

    # read incoming raw path
    self.path_subscriber = self.create_subscription(
      Path,
      "/plan",
      self.path_callback,
      default_qos
    )

    # send computed path
    self.path_publisher = self.create_publisher(
      Path,
      "/plan_smooth",
      default_qos
    )

    self.map_ = None

    self.get_logger().info("StringBezierSmoother Node Has Started Successfully")


  def map_callback(self, map_msg: OccupancyGrid):
    self.map_ = map_msg

    self.get_logger().info("Map received successfully")

  
  def path_callback(self, path_msg: Path):
    raw_path: Path = path_msg
    smooth_path: Path = self.smooth(raw_path)
    self.path_publisher.publish(smooth_path)

    self.get_logger().info("Smooth Path Published")


  def smooth(self, npath: Path) -> Path:
    path = Path()
    path.header = npath.header
    path.poses = npath.poses.copy()

    path = self.greedy_string_pull_smooth_iter(path, 20)
    path = self.smooth_and_densify_path(path)
    
    return path


  def pose_to_grid_node(self, pose: Pose) -> GridNode:
    grid_x = int((pose.position.x - self.map_.info.origin.position.x) / self.map_.info.resolution)
    grid_y = int((pose.position.y - self.map_.info.origin.position.y) / self.map_.info.resolution)
    return GridNode(grid_x, grid_y)
  
  def grid_node_to_pose(self, node: GridNode) -> Pose:
    pose = Pose()
    pose.position.x = (node.x * self.map_.info.resolution) + self.map_.info.origin.position.x
    pose.position.y = (node.y * self.map_.info.resolution) + self.map_.info.origin.position.y
    return pose
  
  def is_grid_node_on_map(self, node: GridNode) -> bool:
    return (0 <= node.x < self.map_.info.width) and (0 <= node.y < self.map_.info.height)
  
  def is_map_cell_free(self, node: GridNode) -> bool:
    return (self.map_.data[self.grid_node_to_map_data_index(node)] >= 0) and (self.map_.data[self.grid_node_to_map_data_index(node)] < 99)
    
  def grid_node_to_map_data_index(self, node: GridNode) -> int:
    """ convert graph node into the corresponding index of the ros2 occupancy grid vector data index (array)"""
    return node.y * self.map_.info.width + node.x

  def bresenham_line(self, start: GridNode, end: GridNode):
    x0, y0 = start.x, start.y
    x1, y1 = end.x, end.y

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)

    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1

    err = dx - dy
    line = []

    while True:
      line.append((x0, y0))

      if x0 == x1 and y0 == y1:
          break

      e2 = 2 * err
      if e2 > -dy:
          err -= dy
          x0 += sx
      if e2 < dx:
          err += dx
          y0 += sy

    return line
      

  def line_crosses_obstacle(self, line) -> bool:
    for x, y in line:
        grid_node = GridNode(x, y) 
        if not self.is_grid_node_on_map(grid_node):
            return True
        # if (not self.is_map_cell_free(grid_node)) or (not self.is_not_close_to_obstacle(grid_node)):
        if not (self.map_.data[self.grid_node_to_map_data_index(grid_node)] == 0):
            return True
    return False

  def line_of_sight(self, start: GridNode, end: GridNode) -> bool:
      if start == end:
          return True
      line = self.bresenham_line(start, end)
      return not self.line_crosses_obstacle(line)
  

  def greedy_string_pull_smooth(self, npath: Path) -> Path:
      poses = npath.poses.copy()
      smoothed_path = Path()
      smoothed_path.header = npath.header
      
      if len(poses) <= 2:
          return npath
  
      i = 0
      j = 2
      n = len(poses)

      while j < n:
        if self.line_of_sight(self.pose_to_grid_node(poses[i].pose),self.pose_to_grid_node(poses[j].pose)):
            poses.pop(j-1)
            n = len(poses)
        else:
            i = j-1
            j = j+1

      smoothed_path.poses = poses.copy()
      return smoothed_path

  def greedy_string_pull_smooth_iter(self, npath: Path, iter: int) -> Path:
      path = Path()
      path.header = npath.header
      path.poses = npath.poses.copy()
      
      for _ in range(iter):
          path = self.greedy_string_pull_smooth(path)

      return path
  

  def compute_bezier_point(self, p0, p1, p2, t):
      """Calculates a 2D point along a Quadratic Bezier curve at time t."""
      return (1 - t)**2 * p0 + 2 * (1 - t) * t * p1 + t**2 * p2


  def calculate_safe_anchor(self, p_curr, p_neighbor, d):
      """
      Calculates the safe anchor point (P0 or P2) along the segment 
      connecting the current corner node (P1) to its neighbor.
      """
      vector = p_neighbor - p_curr
      length = np.linalg.norm(vector)

      if length == 0.0:
          return p_curr

      unit_direction = vector / length

      if length >= 2 * d:
          return p_curr + (d * unit_direction)
      # elif d <= length < 2 * d:
      #     return p_curr + (0.5 * vector)
      else:
          # return p_neighbor
          return p_curr + (0.5 * vector)
  

  def densify_straight_line_segment(self, start_pt: np.ndarray, end_pt: np.ndarray, resolution: float) -> list[np.ndarray]:
      """
      Fills in intermediate 2D array points along a straight path segment
      based on a strict maximum resolution distance metric.
      """
      dx = end_pt[0] - start_pt[0]
      dy = end_pt[1] - start_pt[1]
      dist = hypot(dx, dy)

      if dist <= resolution:
          return [end_pt]

      steps = max(1, int(dist / resolution))
      segment_points = []
      
      for i in range(1, steps + 1):
          ratio = i / float(steps)
          interp_pt = np.array([
              start_pt[0] + dx * ratio,
              start_pt[1] + dy * ratio
          ])
          segment_points.append(interp_pt)
          
      return segment_points
  
  def yaw_to_quaternion(self, yaw: float) -> Quaternion:
    """Converts a yaw angle (radians) to a geometry_msgs/Quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = sin(yaw / 2.0)
    q.w = cos(yaw / 2.0)
    return q


  def smooth_and_densify_path(self, string_pulled_path: Path, d: float = 0.25, num_points_per_corner: int = 15, resolution: float = 0.05) -> Path:
      """
      Rounds the corners of a sparse ROS 2 Path using Quadratic Bezier curves 
      with dynamically bounded safety margins for P0 and P2.
      
      :param string_pulled_path: The input sparse nav_msgs/Path.
      :param d: The desired corner-cutting distance constraint from P1.
      :param num_points_per_corner: Number of interpolation points per curve.
      :param resolution: straight line densification dist segment.
      """
      poses = string_pulled_path.poses
      if len(poses) < 3:
          return string_pulled_path

      smoothed_path = Path()
      smoothed_path.header = string_pulled_path.header
      z_height = poses[0].pose.position.z
      
      raw_points = []
      
      # 1. Pre-calculate safe curve anchor pairs for every interior waypoint
      anchors = {}
      for i in range(1, len(poses) - 1):
          p_prev = np.array([poses[i-1].pose.position.x, poses[i-1].pose.position.y])
          p_curr = np.array([poses[i].pose.position.x, poses[i].pose.position.y])
          p_next = np.array([poses[i+1].pose.position.x, poses[i+1].pose.position.y])
          
          anchors[i] = {
              'p0': self.calculate_safe_anchor(p_curr, p_prev, d),
              'p1': p_curr,
              'p2': self.calculate_safe_anchor(p_curr, p_next, d)
          }

      # 2. Build the continuous sequential line map coordinates
      start_pt = np.array([poses[0].pose.position.x, poses[0].pose.position.y])
      raw_points.append(start_pt)
      
      for i in range(1, len(poses) - 1):
          p0 = anchors[i]['p0']
          p1 = anchors[i]['p1']
          p2 = anchors[i]['p2']
          
          # Densify straight line segments up to the curve entry point (p0)
          if np.linalg.norm(raw_points[-1] - p0) > 1e-4:
              straight_pts = self.densify_straight_line_segment(raw_points[-1], p0, resolution)
              raw_points.extend(straight_pts)
              
          # Draw the curve high-density points
          for t in np.linspace(1.0 / num_points_per_corner, 1.0, num_points_per_corner):
              bezier_pt = self.compute_bezier_point(p0, p1, p2, t)
              if np.linalg.norm(raw_points[-1] - bezier_pt) > 1e-4:
                  raw_points.append(bezier_pt)

      # Densify the final straight line segment to the goal coordinate position
      p_goal = np.array([poses[-1].pose.position.x, poses[-1].pose.position.y])
      if np.linalg.norm(raw_points[-1] - p_goal) > 1e-4:
          final_straight_pts = self.densify_straight_line_segment(raw_points[-1], p_goal, resolution)
          raw_points.extend(final_straight_pts)

      # 3. Dynamic Orientation Extraction & Packing Loop
      for i in range(len(raw_points)):
          pose = PoseStamped()
          pose.header = smoothed_path.header
          pose.pose.position.x = float(raw_points[i][0])
          pose.pose.position.y = float(raw_points[i][1])
          pose.pose.position.z = z_height
          
          if i < len(raw_points) - 1:
              # Look-ahead heading calculation
              dx = raw_points[i+1][0] - raw_points[i][0]
              dy = raw_points[i+1][1] - raw_points[i][1]
              
              yaw = atan2(dy, dx)
              pose.pose.orientation = self.yaw_to_quaternion(yaw)
          else:
              # Force the final point to match your commanded goal orientation exactly
              pose.pose.orientation = poses[-1].pose.orientation

          smoothed_path.poses.append(pose)

      return smoothed_path




def main():
  rclpy.init()
  node = StringPullBezierSmoother()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()


if __name__=="__main__":
  main()
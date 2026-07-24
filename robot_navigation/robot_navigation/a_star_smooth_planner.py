#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from tf2_ros import Buffer, TransformListener, LookupException
from tf_transformations import quaternion_from_euler

from queue import PriorityQueue
from math import hypot, cos, sin, radians, atan2
import numpy as np


class GridNode:
  def __init__(self, x, y, cost=0, heuristic=0, prev=None):
    self.x = x
    self.y = y
    self.cost = cost
    self.heuristic = heuristic
    self.prev = prev

  def __lt__(self, other): # needed for priority queue ordering
    return (self.cost + self.heuristic) < (other.cost + other.heuristic)
  
  def __eq__(self, other):
    return self.x == other.x and self.y == other.y
  
  def __hash__(self): #unique identifier
    return hash((self.x, self.y))
  
  def __add__(self, other):
    return GridNode(
      x=self.x + other.x,
      y=self.y + other.y
    )


class AStarSmoothPlanner(Node):
  def __init__(self):
    super().__init__("a_star_smooth_planner")

    self.declare_parameter("obstacle_clearance_radius", 0.3)
    self.obstacle_clearance_radius = self.get_parameter("obstacle_clearance_radius").value

    self.declare_parameter("obstacle_clearance_kernel_size", 8)
    self.obstacle_clearance_kernel_size = self.get_parameter("obstacle_clearance_kernel_size").value

    if self.obstacle_clearance_kernel_size != 8 or self.obstacle_clearance_kernel_size != 16:
       self.obstacle_clearance_kernel_size = 8

    self.declare_parameter("smoother_corner_cutting_dist", 0.3)
    self.smoother_corner_cutting_dist = self.get_parameter("smoother_corner_cutting_dist").value

    self.declare_parameter("smoother_points_per_curve", 15)
    self.smoother_points_per_curve = self.get_parameter("smoother_points_per_curve").value

    self.declare_parameter("smoother_line_densification_dist", 0.05)
    self.smoother_line_densification_dist = self.get_parameter("smoother_line_densification_dist").value

    default_qos = QoSProfile(depth=10)

    map_qos = QoSProfile(depth=10)
    map_qos.reliability = ReliabilityPolicy.RELIABLE
    map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

    # read incomming map
    self.map_subscriber = self.create_subscription(
      OccupancyGrid,
      "/costmap",
      self.map_callback,
      map_qos
    )

    # read incomming goal pose
    self.goal_pose_subscriber = self.create_subscription(
      PoseStamped,
      "/goal_pose",
      self.goal_pose_callback,
      default_qos
    )

    # send computed path
    self.path_publisher = self.create_publisher(
      Path,
      "/a_star/path",
      default_qos
    )

    # send computed path
    self.smooth_path_publisher = self.create_publisher(
      Path,
      "/theta_star/path",
      default_qos
    )

    # send new map for visualization
    self.map_publisher = self.create_publisher(
      OccupancyGrid,
      "/a_star/visited_map",
      default_qos
    )

    self.map_ = None
    self.visited_map_ = OccupancyGrid()

    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)

    self.obs_dir = None

    self.get_logger().info("AStarSmoothPlanner Node Has Started Successfully")


  def map_callback(self, map_msg: OccupancyGrid):
    self.map_ = map_msg

    if self.smoother_line_densification_dist < self.map_.info.resolution:
       self.smoother_line_densification_dist = self.map_.info.resolution

    # initialize the visited map as the incoming map. Also as a unknown space (-1)
    self.visited_map_.header.frame_id = map_msg.header.frame_id
    self.visited_map_.info = map_msg.info
    self.visited_map_.data = [-1] * (map_msg.info.width * map_msg.info.height)

    self.rr_grid = self.obstacle_clearance_radius / self.map_.info.resolution

    if self.obstacle_clearance_kernel_size == 8:
      self.obs_dir = [
        ( int(self.rr_grid * cos(radians(0))), int(self.rr_grid * sin(radians(0))) ),

        ( int(self.rr_grid * cos(radians(45))), int(self.rr_grid * sin(radians(45))) ),
        ( int(self.rr_grid * cos(radians(90))), int(self.rr_grid * sin(radians(90))) ),
        ( int(self.rr_grid * cos(radians(135))), int(self.rr_grid * sin(radians(135))) ),

        ( int(self.rr_grid * cos(radians(180))), int(self.rr_grid * sin(radians(180))) ),

        ( int(self.rr_grid * cos(radians(-135))), int(self.rr_grid * sin(radians(-135))) ),
        ( int(self.rr_grid * cos(radians(-90))), int(self.rr_grid * sin(radians(-90))) ),
        ( int(self.rr_grid * cos(radians(-45))), int(self.rr_grid * sin(radians(-45))) ),
      ]

    elif self.obstacle_clearance_kernel_size == 16:
      self.obs_dir = [
        ( int(self.rr_grid * cos(radians(0))), int(self.rr_grid * sin(radians(0))) ),

        ( int(self.rr_grid * cos(radians(22.5))), int(self.rr_grid * sin(radians(22.5))) ),
        ( int(self.rr_grid * cos(radians(45))), int(self.rr_grid * sin(radians(45))) ),
        ( int(self.rr_grid * cos(radians(67.5))), int(self.rr_grid * sin(radians(67.5))) ),
        ( int(self.rr_grid * cos(radians(90))), int(self.rr_grid * sin(radians(90))) ),
        ( int(self.rr_grid * cos(radians(112.5))), int(self.rr_grid * sin(radians(112.5))) ),
        ( int(self.rr_grid * cos(radians(135))), int(self.rr_grid * sin(radians(135))) ),
        ( int(self.rr_grid * cos(radians(157.5))), int(self.rr_grid * sin(radians(157.5))) ),

        ( int(self.rr_grid * cos(radians(180))), int(self.rr_grid * sin(radians(180))) ),

        ( int(self.rr_grid * cos(radians(-157.5))), int(self.rr_grid * sin(radians(-157.5))) ),
        ( int(self.rr_grid * cos(radians(-135))), int(self.rr_grid * sin(radians(-135))) ),
        ( int(self.rr_grid * cos(radians(-112.5))), int(self.rr_grid * sin(radians(-112.5))) ),
        ( int(self.rr_grid * cos(radians(-90))), int(self.rr_grid * sin(radians(-90))) ),
        ( int(self.rr_grid * cos(radians(-67.5))), int(self.rr_grid * sin(radians(-67.5))) ),
        ( int(self.rr_grid * cos(radians(-45))), int(self.rr_grid * sin(radians(-45))) ),
        ( int(self.rr_grid * cos(radians(-22.5))), int(self.rr_grid * sin(radians(-22.5))) ),
      ]

    self.get_logger().info("Map received successfully")

  def goal_pose_callback(self, pose_msg: PoseStamped):
    if self.map_ is None:
      self.get_logger().error("No Map Received")
      return
    
    # reset the visited map
    self.visited_map_.data = [-1] * (self.map_.info.width * self.map_.info.height)

    # get current position of the robot in the map (or relative to the map) [MAP -> ROBOT]
    # i.e transform data FROM the Target Frame (map) TO the Source Frame (base_link)
    try:
      map_to_robot_base_tf = self.tf_buffer.lookup_transform(
        target_frame=self.map_.header.frame_id,
        source_frame= "base_link",
        time=rclpy.time.Time()
      )
    except LookupException:
      self.get_logger().error("could not transform from map to base_link")
      return
    
    # convert transform data to pose msg
    map_to_robot_base_pose = Pose()
    map_to_robot_base_pose.position.x = map_to_robot_base_tf.transform.translation.x
    map_to_robot_base_pose.position.y = map_to_robot_base_tf.transform.translation.y
    map_to_robot_base_pose.orientation = map_to_robot_base_tf.transform.rotation

    path, p = self.plan(
      start_pose=map_to_robot_base_pose,
      goal_pose=pose_msg.pose
    )

    if p.poses:
      self.get_logger().info("shortest path found")
      self.path_publisher.publish(path)
      self.smooth_path_publisher.publish(p)
    else:
      self.get_logger().warn("No path found to the goal")


  def plan(self, start_pose: Pose, goal_pose: Pose) -> Path:
    explore_direction = [
      #(x_dir, y_dir, cost)
      (-1, 0, 1), 
      (1, 0, 1), 
      (0, 1, 1), 
      (0, -1, 1),
      (-1, 1, 1.4142), 
      (1, -1, 1.4142), 
      (1, 1, 1.4142), 
      (-1, -1, 1.4142),
    ]
    pending_nodes = PriorityQueue()
    visited_nodes = set()
    start_node: GridNode = self.pose_to_grid_node(start_pose)
    goal_node: GridNode = self.pose_to_grid_node(goal_pose)

    start_node.heuristic = self.euclidean_distance(start_node, goal_node)
    pending_nodes.put(start_node)

    while not pending_nodes.empty() and rclpy.ok():
      active_node: GridNode = pending_nodes.get()

      if active_node == goal_node:
        break

      for dir_x, dir_y, dir_cost in explore_direction:
        new_node: GridNode = active_node + GridNode(dir_x, dir_y)
        if (
            new_node not in visited_nodes
            and self.is_grid_node_on_map(new_node)
            and self.is_map_cell_free(new_node) 
            # and self.is_not_close_to_obstacle(new_node)
          ):
          new_node.cost = active_node.cost + dir_cost + self.map_.data[self.grid_node_to_map_data_index(new_node)]
          new_node.heuristic = self.euclidean_distance(new_node, goal_node)
          new_node.prev = active_node
          pending_nodes.put(new_node)
          visited_nodes.add(new_node)

      self.visited_map_.data[self.grid_node_to_map_data_index(active_node)] = -106 # nice orange color
      self.map_publisher.publish(self.visited_map_)

    path = Path()
    path.header.frame_id = self.map_.header.frame_id

    #construct node from last(goal) to first(start).
    while active_node and active_node.prev and rclpy.ok():
      last_pose: Pose = self.grid_node_to_pose(active_node)
      last_pose_stamped = PoseStamped()
      last_pose_stamped.header.frame_id = self.map_.header.frame_id
      last_pose_stamped.pose = last_pose
      path.poses.append(last_pose_stamped)
      active_node = active_node.prev

    # resverse the poses construction from first(start) to last(goal)

    # path.poses.reverse()
    # return path

    path.poses.reverse()
    smooth_path = self.greedy_string_pull_smooth_iter(path, 10)
    smooth_path_ = self.smooth_and_densify_path(
       string_pulled_path=smooth_path,
       d=self.smoother_corner_cutting_dist,
       num_points_per_corner=self.smoother_points_per_curve,
       resolution=self.smoother_line_densification_dist
    )
    return smooth_path_, path






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
  
  def is_not_close_to_obstacle(self, node: GridNode) -> bool:
    for dir_x, dir_y in self.obs_dir:
      check_node: GridNode = node + GridNode(dir_x, dir_y)
      if (not self.is_grid_node_on_map(check_node)) or (not self.is_map_cell_free(check_node)):
        return False
    return True

  def grid_node_to_map_data_index(self, node: GridNode) -> int:
    """ convert graph node into the corresponding index of the ros2 occupancy grid vector data index (array)"""
    return node.y * self.map_.info.width + node.x
  
  def manhattan_distance(self, node: GridNode, goal_node: GridNode):
    dx = node.x - goal_node.x
    dy = node.y - goal_node.y
    return abs(dx) + abs(dy)
  
  def euclidean_distance(self, a: GridNode, b: GridNode):
    return hypot(a.x - b.x, a.y - b.y)
  
  def octile_distance(self, node: GridNode, goal_node: GridNode) -> float:
    dx = abs(node.x - goal_node.x)
    dy = abs(node.y - goal_node.y)
    return (dx + dy) - 0.58578644 * min(dx, dy)
  

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
  

  # def greedy_string_pull_smooth(self, npath: Path) -> Path:
  #   smoothed_path = Path()
  #   smoothed_path.header = npath.header
    
  #   if len(npath.poses) <= 2:
  #       return npath

  #   smoothed_path.poses.append(npath.poses[0])
    
  #   i = 0
  #   while i < len(npath.poses) - 1:
  #     furthest_j = i + 1
      
  #     for j in range(i + 2, len(npath.poses)):
  #       start_node = self.pose_to_grid_node(npath.poses[i].pose)
  #       end_node = self.pose_to_grid_node(npath.poses[j].pose)
        
  #       if self.line_of_sight(start_node, end_node):
  #         furthest_j = j
  #       else:
  #         break

  #     smoothed_path.poses.append(npath.poses[furthest_j])
  #     i = furthest_j

  #   return smoothed_path

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
    [x, y, z, w] = quaternion_from_euler(0.0, 0.0, yaw)
    q.x = x
    q.y = y
    q.z = z
    q.w = w
    return q


  def smooth_and_densify_path(self, string_pulled_path: Path, d: float = 0.2, num_points_per_corner: int = 20, resolution: float = 0.05) -> Path:
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
  node = AStarSmoothPlanner()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()


if __name__=="__main__":
  main()
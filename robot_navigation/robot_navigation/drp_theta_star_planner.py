#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose
from tf2_ros import Buffer, TransformListener, LookupException

from queue import PriorityQueue, Queue
from math import hypot
import time


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


class DRPThetaStarPlanner(Node):
  def __init__(self):
    super().__init__("drp_theta_star_planner")

    default_qos = QoSProfile(depth=10)

    map_qos = QoSProfile(depth=10)
    map_qos.reliability = ReliabilityPolicy.RELIABLE
    map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

    # read incomming map
    self.map_subscriber = self.create_subscription(
      OccupancyGrid,
      # "/costmap",
      "/map",
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
      "/theta_star/path",
      default_qos
    )

    # send new map for visualization
    self.map_publisher = self.create_publisher(
      OccupancyGrid,
      "/theta_star/visited_map",
      default_qos
    )

    self.map_ = None
    self.visited_map_ = OccupancyGrid()

    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)

    self.obs_dir = None

    self.get_logger().info("DRPThetaStarPlanner Node Has Started Successfully")


  def map_callback(self, map_msg: OccupancyGrid):
    self.map_ = map_msg

    # initialize the visited map as the incoming map. Also as a unknown space (-1)
    self.visited_map_.header.frame_id = map_msg.header.frame_id
    self.visited_map_.info = map_msg.info
    self.visited_map_.data = [-1] * (map_msg.info.width * map_msg.info.height)

    # self.get_logger().info("Map received successfully")

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

    path = self.plan(
      start_pose=map_to_robot_base_pose,
      goal_pose=pose_msg.pose
    )

    if path.poses:
      self.get_logger().info("shortest path found")
      self.path_publisher.publish(path)
    else:
      self.get_logger().warn("No path found to the goal")




#-----------------------------------------------------------------------------------

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
  
      nodes_to_explore = PriorityQueue()
      nodes_already_explored = set()

      start_node: GridNode = self.pose_to_grid_node(start_pose)
      goal_node: GridNode = self.pose_to_grid_node(goal_pose)

      start_node.cost = 0
      start_node.heuristic = self.euclidean_distance(start_node, goal_node)
      nodes_to_explore.put(start_node)

      active_node = None

      start_time = time.time_ns()

      while not nodes_to_explore.empty() and rclpy.ok():
        active_node = nodes_to_explore.get()

        if active_node in nodes_already_explored:
          continue

        if active_node.prev and active_node.prev.prev:
          grandparent = active_node.prev.prev
          if self.line_of_sight(active_node, grandparent):
            active_node.prev = grandparent
            active_node.cost = grandparent.cost + self.euclidean_distance(active_node, grandparent) # + self.map_.data[self.grid_node_to_map_data_index(new_node)]
            # nodes_to_explore = PriorityQueue()

        nodes_already_explored.add(active_node)

        if active_node == goal_node:
          break

        # --- NEIGHBOR EXPANSION ---
        for dir_x, dir_y, dir_cost in explore_direction:
          new_node: GridNode = active_node + GridNode(dir_x, dir_y)
          
          if (
              new_node not in nodes_already_explored
              and self.is_grid_node_on_map(new_node)
              and self.is_map_cell_free(new_node) 
            ):
            
            # Always set initial candidate parent to active_node
            new_node.prev = active_node
            
            # Optimistic cost: Assume we can shortcut through active_node's parent if available
            if active_node.prev:
              new_node.cost = active_node.prev.cost + self.euclidean_distance(new_node, active_node.prev) # + self.map_.data[self.grid_node_to_map_data_index(new_node)]
            else:
              new_node.cost = active_node.cost + dir_cost # + self.map_.data[self.grid_node_to_map_data_index(new_node)]

            # new_node.cost = active_node.cost + dir_cost # + self.map_.data[self.grid_node_to_map_data_index(new_node)]
            new_node.heuristic = self.euclidean_distance(new_node, goal_node)
            nodes_to_explore.put(new_node)

        self.visited_map_.data[self.grid_node_to_map_data_index(active_node)] = 10
        self.map_publisher.publish(self.visited_map_)

      dt = int((time.time_ns() - start_time)/1000000)
      self.get_logger().info(f"planning_time = {dt} ms")

      path = Path()
      path.header.frame_id = self.map_.header.frame_id

      while active_node and rclpy.ok():
        last_pose: Pose = self.grid_node_to_pose(active_node)
        last_pose_stamped = PoseStamped()
        last_pose_stamped.header.frame_id = self.map_.header.frame_id
        last_pose_stamped.pose = last_pose
        path.poses.append(last_pose_stamped)
        active_node = active_node.prev

      path.poses.reverse()
      return path

#----------------------------------------------------------------------------------------------





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
    # return (self.map_.data[self.grid_node_to_map_data_index(node)] >= 0) and (self.map_.data[self.grid_node_to_map_data_index(node)] < 99)
    return self.map_.data[self.grid_node_to_map_data_index(node)] == 0

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


  def line_of_sight(self, start: GridNode, end: GridNode) -> bool:
    """
    Checks line of sight between start and end GridNodes across an OccupancyGrid map.
    
    :param start: GridNode with .x and .y attributes
    :param end: GridNode with .x and .y attributes
    :param map_msg: nav_msgs.msg.OccupancyGrid message
    :return: True if clear line of sight, False if blocked by an obstacle/unknown cell
    """
    x0, y0 = start.x, start.y
    x1, y1 = end.x, end.y

    size_x = self.map_.info.width
    data = self.map_.data  # Flat tuple/list of int8 values

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    # Memory offsets (strides) for 1D array indexing
    stride_x = sx
    stride_y = sy * size_x

    current_idx = y0 * size_x + x0

    while True:
        # ROS OccupancyGrid: > 0 (occupied/cost) or -1 (unknown)
        cell_value = data[current_idx]
        if cell_value > 0 or cell_value == -1:
            return False

        if x0 == x1 and y0 == y1:
            break

        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
            current_idx += stride_x

        if e2 < dx:
            err += dx
            y0 += sy
            current_idx += stride_y

    return True




def main():
  rclpy.init()
  node = DRPThetaStarPlanner()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()


if __name__=="__main__":
  main()
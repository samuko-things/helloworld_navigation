#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose
from tf2_ros import Buffer, TransformListener, LookupException

from queue import PriorityQueue
from math import hypot, cos, sin, radians


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


class ThetaStarPlanner(Node):
  def __init__(self):
    super().__init__("theta_star_planner")

    default_qos = QoSProfile(depth=10)

    map_qos = QoSProfile(depth=10)
    map_qos.reliability = ReliabilityPolicy.RELIABLE
    map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

    # read incomming map
    self.map_subscriber = self.create_subscription(
      OccupancyGrid,
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

    self.robot_radius: float = 0.25

    self.obs_dir = None

    self.get_logger().info("ThetaStar Planner Node Has Started Successfully")


  def map_callback(self, map_msg: OccupancyGrid):
    self.map_ = map_msg

    # initialize the visited map as the incoming map. Also as a unknown space (-1)
    self.visited_map_.header.frame_id = map_msg.header.frame_id
    self.visited_map_.info = map_msg.info
    self.visited_map_.data = [-1] * (map_msg.info.width * map_msg.info.height)

    self.rr_grid = self.robot_radius / self.map_.info.resolution

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

    # self.obs_dir = [
    #   ( int(self.rr_grid * cos(radians(0))), int(self.rr_grid * sin(radians(0))) ),

    #   ( int(self.rr_grid * cos(radians(22.5))), int(self.rr_grid * sin(radians(22.5))) ),
    #   ( int(self.rr_grid * cos(radians(45))), int(self.rr_grid * sin(radians(45))) ),
    #   ( int(self.rr_grid * cos(radians(67.5))), int(self.rr_grid * sin(radians(67.5))) ),
    #   ( int(self.rr_grid * cos(radians(90))), int(self.rr_grid * sin(radians(90))) ),
    #   ( int(self.rr_grid * cos(radians(112.5))), int(self.rr_grid * sin(radians(112.5))) ),
    #   ( int(self.rr_grid * cos(radians(135))), int(self.rr_grid * sin(radians(135))) ),
    #   ( int(self.rr_grid * cos(radians(157.5))), int(self.rr_grid * sin(radians(157.5))) ),

    #   ( int(self.rr_grid * cos(radians(180))), int(self.rr_grid * sin(radians(180))) ),

    #   ( int(self.rr_grid * cos(radians(-157.5))), int(self.rr_grid * sin(radians(-157.5))) ),
    #   ( int(self.rr_grid * cos(radians(-135))), int(self.rr_grid * sin(radians(-135))) ),
    #   ( int(self.rr_grid * cos(radians(-112.5))), int(self.rr_grid * sin(radians(-112.5))) ),
    #   ( int(self.rr_grid * cos(radians(-90))), int(self.rr_grid * sin(radians(-90))) ),
    #   ( int(self.rr_grid * cos(radians(-67.5))), int(self.rr_grid * sin(radians(-67.5))) ),
    #   ( int(self.rr_grid * cos(radians(-45))), int(self.rr_grid * sin(radians(-45))) ),
    #   ( int(self.rr_grid * cos(radians(-22.5))), int(self.rr_grid * sin(radians(-22.5))) ),
    # ]

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

    path = self.plan(
      start_pose=map_to_robot_base_pose,
      goal_pose=pose_msg.pose
    )

    if path.poses:
      self.get_logger().info("shortest path found")
      self.path_publisher.publish(path)
    else:
      self.get_logger().warn("No path found to the goal")




  

  def plan(self, start_pose: Pose, goal_pose: Pose) -> Path:
    explore_direction = [
        (-1,  0, 1.0),
        ( 1,  0, 1.0),
        ( 0,  1, 1.0),
        ( 0, -1, 1.0),
        (-1,  1, 1.4142),
        ( 1, -1, 1.4142),
        ( 1,  1, 1.4142),
        (-1, -1, 1.4142),
    ]

    pending_nodes = PriorityQueue()
    closed_set = set()
    
    # Store explicit node instances mapped to their coordinates to maintain structural history
    # This prevents cross-contamination of .prev references
    node_registry = {}
    g_score = {}
    
    sequence_counter = 0  

    start_node: GridNode = self.pose_to_grid_node(start_pose)
    goal_node: GridNode = self.pose_to_grid_node(goal_pose)

    start_node.cost = 0.0
    start_node.heuristic = self.octile_distance(start_node, goal_node)
    start_node.prev = start_node

    # Register our initial node coordinates
    node_registry[(start_node.x, start_node.y)] = start_node
    g_score[(start_node.x, start_node.y)] = 0.0
    
    pending_nodes.put((start_node.cost + start_node.heuristic, sequence_counter, start_node))

    final_active_node = None

    while not pending_nodes.empty() and rclpy.ok():
        _, _, active_node = pending_nodes.get()
        active_coord = (active_node.x, active_node.y)

        if active_coord in closed_set:
            continue

        # --- LAZY THETA* VERTEX RESOLUTION ---
        if active_coord != (start_node.x, start_node.y):
            if not self.line_of_sight(active_node.prev, active_node):
                min_cost = float("inf")
                best_parent = None
                
                for dir_x, dir_y, dir_cost in explore_direction:
                    neighbor_coord = (active_node.x - dir_x, active_node.y - dir_y)
                    if neighbor_coord in closed_set:
                        potential_cost = g_score.get(neighbor_coord, float("inf")) + dir_cost
                        if potential_cost < min_cost:
                            min_cost = potential_cost
                            best_parent = node_registry[neighbor_coord]
                
                if best_parent is not None:
                    g_score[active_coord] = min_cost
                    active_node.cost = min_cost
                    active_node.prev = best_parent
                else:
                    continue

        closed_set.add(active_coord)

        if active_node.x == goal_node.x and active_node.y == goal_node.y:
            final_active_node = active_node
            break

        # --- NEIGHBOR EXPANSION ---
        for dir_x, dir_y, dir_cost in explore_direction:
            nx, ny = active_node.x + dir_x, active_node.y + dir_y
            neighbor_coord = (nx, ny)

            if neighbor_coord in closed_set:
                continue

            # Create an independent structural object for this loop iteration
            test_node = GridNode(nx, ny)

            if (
                self.is_grid_node_on_map(test_node)
                and self.is_map_cell_free(test_node)
                and self.is_not_close_to_obstacle(test_node)
            ):
                parent = active_node.prev
                parent_coord = (parent.x, parent.y)
                tentative_cost = g_score[parent_coord] + self.euclidean_distance(parent, test_node)

                if tentative_cost < g_score.get(neighbor_coord, float("inf")):
                    # Unique structural instance preservation
                    g_score[neighbor_coord] = tentative_cost
                    test_node.cost = tentative_cost
                    test_node.heuristic = self.octile_distance(test_node, goal_node)
                    test_node.prev = parent
                    
                    node_registry[neighbor_coord] = test_node

                    f_score = tentative_cost + test_node.heuristic
                    sequence_counter += 1
                    pending_nodes.put((f_score, sequence_counter, test_node))

        # Update map visualization
        self.visited_map_.data[self.grid_node_to_map_data_index(active_node)] = -106
        self.map_publisher.publish(self.visited_map_)

    # --- PATH RECONSTRUCTION ---
    path = Path()
    path.header.frame_id = self.map_.header.frame_id

    trace_node = final_active_node
    visited_trace = set() # Standard fallback protection against cyclical pointer loops

    while trace_node and rclpy.ok():
        trace_coord = (trace_node.x, trace_node.y)
        if trace_coord in visited_trace:
            break
        visited_trace.add(trace_coord)

        last_pose: Pose = self.grid_node_to_pose(trace_node)
        last_pose_stamped = PoseStamped()
        last_pose_stamped.header.frame_id = self.map_.header.frame_id
        last_pose_stamped.pose = last_pose
        path.poses.append(last_pose_stamped)
        
        if trace_node.x == start_node.x and trace_node.y == start_node.y:
            break
            
        trace_node = trace_node.prev

    path.poses.reverse()
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
    return self.map_.data[self.grid_node_to_map_data_index(node)] == 0
  
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
        # Crucial fix: Make sure the evaluated line segment point falls inside map bounds
        if not self.is_grid_node_on_map(grid_node):
            return True
        if (not self.is_map_cell_free(grid_node)) or (not self.is_not_close_to_obstacle(grid_node)):
            return True
    return False

  def line_of_sight(self, start: GridNode, end: GridNode) -> bool:
      # Handle matching positions to avoid redundant overhead evaluations
      if start == end:
          return True
      line = self.bresenham_line(start, end)
      return not self.line_crosses_obstacle(line)


def main():
  rclpy.init()
  node = ThetaStarPlanner()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()


if __name__=="__main__":
  main()
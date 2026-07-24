#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Path
from geometry_msgs.msg import TwistStamped, PoseStamped
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_matrix, concatenate_matrices, quaternion_from_matrix, translation_from_matrix, inverse_matrix

from math import hypot


class PurePursuitController(Node):
  def __init__(self):
    super().__init__("pure_pursuit_controller")

    self.declare_parameter("look_ahead_distance", 0.3)
    self.look_ahead_distance = self.get_parameter("look_ahead_distance").value

    self.declare_parameter("max_linear_velocity", 0.1)
    self.max_linear_velocity = self.get_parameter("max_linear_velocity").value

    self.declare_parameter("max_angular_velocity", 0.6)
    self.max_angular_velocity = self.get_parameter("max_angular_velocity").value

    default_qos = QoSProfile(depth=10)

    self.path_subcriber = self.create_subscription(
      Path,
      '/a_star/path',
      self.path_callback,
      default_qos
    )
    
    self.cmd_vel_publisher = self.create_publisher(
      TwistStamped,
      '/cmd_vel_nav',
      default_qos
    )

    self.carrot_publisher = self.create_publisher(
      PoseStamped,
      "/pp/carrot",
      default_qos
    )

    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)

    self.timer = self.create_timer(
      0.5, # 10Hz
      self.control_loop
    )

    self.global_plan = None

    self.get_logger().info("PurePursuitController Node Has Started Successfully")


  def path_callback(self, path_msg: Path):
    self.global_plan = path_msg


  def control_loop(self):
    if not self.global_plan or not self.global_plan.poses:
      return
    
    try:
      robot_tf_pose_in_odom = self.tf_buffer.lookup_transform(
        "odom", "base_link", rclpy.time.Time()
      )
    except Exception as e:
      self.get_logger().warn(f"Cound not transform: {e}")
      return
    
    if not self.transform_plan_from_map_to_odom(robot_tf_pose_in_odom.header.frame_id):
      self.get_logger().info(f"Unable to transform Plan to odom fram")
      return

    robot_pose = PoseStamped()
    robot_pose.header.frame_id = robot_tf_pose_in_odom.header.frame_id
    robot_pose.pose.position.x = robot_tf_pose_in_odom.transform.translation.x
    robot_pose.pose.position.y = robot_tf_pose_in_odom.transform.translation.y
    robot_pose.pose.orientation = robot_tf_pose_in_odom.transform.rotation

    # the carrot pose is in the robot's baselink frame.
    carrot_pose: PoseStamped = self.get_carrot_pose(robot_pose)

    dx = carrot_pose.pose.position.x - robot_pose.pose.position.x
    dy = carrot_pose.pose.position.y - robot_pose.pose.position.y
    dist = hypot(dx, dy)

    if dist <= 0.1:
      self.get_logger().info("Goal Reached")
      self.global_plan.poses.clear()
      return

    self.carrot_publisher.publish(carrot_pose)

    robot_pose_tf_matrix = quaternion_matrix([
      robot_pose.pose.orientation.x,
      robot_pose.pose.orientation.y,
      robot_pose.pose.orientation.z,
      robot_pose.pose.orientation.w
    ])
    robot_pose_tf_matrix[0][3] = robot_pose.pose.position.x
    robot_pose_tf_matrix[1][3] = robot_pose.pose.position.y
    robot_pose_tf_matrix[2][3] = robot_pose.pose.position.z

    carrot_pose_tf_matrix = quaternion_matrix([
      carrot_pose.pose.orientation.x,
      carrot_pose.pose.orientation.y,
      carrot_pose.pose.orientation.z,
      carrot_pose.pose.orientation.w
    ])
    carrot_pose_tf_matrix[0][3] = carrot_pose.pose.position.x
    carrot_pose_tf_matrix[1][3] = carrot_pose.pose.position.y
    carrot_pose_tf_matrix[2][3] = carrot_pose.pose.position.z

    # L expressed in robor's (base_link) frame
    # T_OC = T_OR * T_RC
    # T_RC = inv(T_OR) * T_OC
    carrot_pose_robot_tf_matrix = concatenate_matrices(inverse_matrix(robot_pose_tf_matrix), carrot_pose_tf_matrix)
    carrot_pose_robot = PoseStamped()
    carrot_pose_robot.pose.position.x = carrot_pose_robot_tf_matrix[0][3]
    carrot_pose_robot.pose.position.y = carrot_pose_robot_tf_matrix[1][3]
    carrot_pose_robot.pose.position.z = carrot_pose_robot_tf_matrix[2][3]
    q = quaternion_from_matrix(carrot_pose_robot_tf_matrix)
    carrot_pose_robot.pose.orientation.x = q[0]
    carrot_pose_robot.pose.orientation.y = q[1]
    carrot_pose_robot.pose.orientation.z = q[2]
    carrot_pose_robot.pose.orientation.w = q[3]

    curvature = self.get_curvature(carrot_pose_robot)
    cmd_vel = TwistStamped()
    cmd_vel.twist.linear.x = self.max_linear_velocity
    cmd_vel.twist.angular.z = curvature*self.max_angular_velocity
    cmd_vel.header.frame_id = carrot_pose_robot.header.frame_id
    # cmd_vel.header.stamp = rclpy.time.Time()

    self.cmd_vel_publisher.publish(cmd_vel)





  def transform_plan_from_map_to_odom(self, frame_id):
    if self.global_plan.header.frame_id == frame_id:
      return True

    try:
      map_tf_pose_in_odom = self.tf_buffer.lookup_transform(
        "odom", self.global_plan.header.frame_id, rclpy.time.Time()
      )
    except Exception as e:
      self.get_logger().error(f"Could not transform Global plan to {frame_id} frame")

    map_tf_matrix_in_odom = quaternion_matrix([
      map_tf_pose_in_odom.transform.rotation.x,
      map_tf_pose_in_odom.transform.rotation.y,
      map_tf_pose_in_odom.transform.rotation.z,
      map_tf_pose_in_odom.transform.rotation.w,
    ])

    map_tf_matrix_in_odom[0][3] = map_tf_pose_in_odom.transform.translation.x
    map_tf_matrix_in_odom[1][3] = map_tf_pose_in_odom.transform.translation.y
    map_tf_matrix_in_odom[2][3] = map_tf_pose_in_odom.transform.translation.z

    for pose in self.global_plan.poses:
      # plan_tf_matrix_in_odom = map_tf_matrix_in_odom * plan_tf_in_map_frame
      plan_tf_matrix_in_map = quaternion_matrix([
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w
      ])
      plan_tf_matrix_in_map[0][3] = pose.pose.position.x
      plan_tf_matrix_in_map[1][3] = pose.pose.position.y
      plan_tf_matrix_in_map[1][3] = pose.pose.position.z

      plan_tf_matrix_in_odom = concatenate_matrices(map_tf_matrix_in_odom, plan_tf_matrix_in_map)

      #extract pose from transformation matrix
      [
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w
      ] = quaternion_from_matrix(plan_tf_matrix_in_odom)
      [
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z,
      ] = translation_from_matrix(plan_tf_matrix_in_odom)

    self.global_plan.header.frame_id = frame_id

    return True
  
  def get_carrot_pose(self, robot_pose: PoseStamped):
    carrot_pose = self.global_plan.poses[-1]
    for pose in reversed(self.global_plan.poses):
      dx = pose.pose.position.x - robot_pose.pose.position.x
      dy = pose.pose.position.y - robot_pose.pose.position.y
      dist = hypot(dx, dy)
      if dist > self.look_ahead_distance:
        carrot_pose.pose = pose.pose
      else:
        break
    return carrot_pose
  
  def get_curvature(self, carrot_pose: PoseStamped):
    carrot_dist = (carrot_pose.pose.position.x * carrot_pose.pose.position.x) * (carrot_pose.pose.position.y * carrot_pose.pose.position.y)

    if carrot_dist < 0.001:
      return 0.0
    else:
      return 2.0 * carrot_pose.pose.position.y / carrot_dist


def main():
  rclpy.init()
  node = PurePursuitController()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()


if __name__=="__main__":
  main()
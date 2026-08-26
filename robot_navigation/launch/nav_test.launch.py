import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
  DeclareLaunchArgument,
  IncludeLaunchDescription)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression, LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml, ReplaceString


def generate_launch_description():
  # Set the path to this package.
  robot_sim_pkg_path = get_package_share_directory('robot_simulation')
  robot_nav_pkg_path = get_package_share_directory('robot_navigation')

  # Set the path to the nav params file
  nav_params_file_name = 'nav2_bringup_params.yaml'
  nav_params_file = os.path.join(robot_nav_pkg_path, 'config', nav_params_file_name)

  # Set the path to the map file used by AMCL
  world_file_name = 'room_with_walls'
  # world_file_name = 'bookstore'
  # world_file_name = 'hospital'
 
  bt_nav_to_pose_xml = os.path.join(robot_nav_pkg_path, 'config', 'bt', 'navigate_to_pose_w_smoothing.xml')
  bt_nav_through_poses_xml = os.path.join(robot_nav_pkg_path, 'config', 'bt', 'navigate_through_pose_w_smoothing.xml')

  rewritten_nav_params_file = RewrittenYaml(
    source_file=nav_params_file,
    root_key='',
    param_rewrites={
        'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': bt_nav_to_pose_xml,
        'bt_navigator.ros__parameters.default_nav_through_poses_bt_xml': bt_nav_through_poses_xml,
    },
    convert_types=True,
  )

  #--------------------------------------------------------------------------

  # Launch configuration variables specific to simulation
  use_sim_time = LaunchConfiguration('use_sim_time')
  nav_params = LaunchConfiguration('nav_params')
  world_name = LaunchConfiguration('world_name')

  declare_use_sim_time_cmd = DeclareLaunchArgument(
      name='use_sim_time', 
      default_value='True',
      description='Flag to enable use_sim_time'
    )
  
  declare_nav_params_cmd = DeclareLaunchArgument(
      name='nav_params',
      default_value=nav_params_file,
      # default_value=rewritten_nav_params_file,
      description='file path to the parameter file'
    )
  
  declare_world_name_cmd = DeclareLaunchArgument(
      name='world_name',
      default_value=world_file_name,
      description='file path to the map needed for navigation'
    )



  #-----------------------------------------------------------------------------
  sim_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          [os.path.join(robot_sim_pkg_path,'launch','sim.launch.py')]
      ), 
      launch_arguments={
        'use_sim_time': 'True',
        'world_name': world_name,
        'use_rviz': 'False',
      }.items(),
  )

  #-----------------------------------------------------------------------------

  rviz_config_file = os.path.join(robot_nav_pkg_path,'config','amcl.rviz')

  # create needed nodes or launch files
  rviz_node = Node(
      package='rviz2',
      executable='rviz2',
      arguments=['-d', rviz_config_file],
      output='screen'
  )

  #-----------------------------------------------------------------------------

  amcl_localization_path = os.path.join(robot_nav_pkg_path, 'launch', 'amcl_localization.launch.py')
  
  amcl_localization_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(amcl_localization_path),
      launch_arguments={
              'use_sim_time': use_sim_time,
              'map_name': world_name,
      }.items()
    )

  #--------------------------------------------------------------------------------

  lifecycle_nodes = [
    'planner_server',
    'controller_server',
    'bt_navigator',
    'behavior_server',
    'smoother_server',
    'waypoint_follower',
  ]

  remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

  nav2_planner_server_node = Node(
    package='nav2_planner',
    executable='planner_server',
    name='planner_server',
    output='screen',
    parameters=[
      nav_params,
      {'use_sim_time': use_sim_time}
    ],
    remappings=remappings,
  )

  nav2_smoother_server_node = Node(
    package='nav2_smoother',
    executable='smoother_server',
    name='smoother_server',
    output='screen',
    parameters=[
      nav_params,
      {'use_sim_time': use_sim_time}
    ],
    remappings=remappings,
  )

  nav2_controller_server_node = Node(
    package='nav2_controller',
    executable='controller_server',
    name='controller_server',
    output='screen',
    parameters=[
      nav_params,
      {'use_sim_time': use_sim_time}
    ],
    remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
  )

  nav2_bt_navigator_node = Node(
    package='nav2_bt_navigator',
    executable='bt_navigator',
    name='bt_navigator',
    output='screen',
    parameters=[
      nav_params,
      {'use_sim_time': use_sim_time}
    ],
    remappings=remappings,
  )

  nav2_behavior_server_node = Node(
    package='nav2_behaviors',
    executable='behavior_server',
    name='behavior_server',
    output='screen',
    parameters=[
      nav_params,
      {'use_sim_time': use_sim_time}
    ],
    remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
  )

  nav2_waypoint_follower_node = Node(
    package='nav2_waypoint_follower',
    executable='waypoint_follower',
    name='waypoint_follower',
    output='screen',
    parameters=[
      nav_params,
      {'use_sim_time': use_sim_time}
    ],
    remappings=remappings,
  )

  nav2_lifecycle_manager_node = Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    output='screen',
    parameters=[{"autostart": True, "bond_timeout": 0.0}, {'node_names': lifecycle_nodes}],
  )

  #--------------------------------------------------------------------------------

  # Create the launch description
  ld = LaunchDescription()
 
  # add the necessary declared launch arguments to the launch description
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_nav_params_cmd)
  ld.add_action(declare_world_name_cmd)
 
  # Add the nodes to the launch description
  ld.add_action(sim_launch)
  ld.add_action(rviz_node)
  ld.add_action(amcl_localization_launch)
  ld.add_action(nav2_planner_server_node)
  ld.add_action(nav2_smoother_server_node)
  ld.add_action(nav2_controller_server_node)
  ld.add_action(nav2_bt_navigator_node)
  ld.add_action(nav2_behavior_server_node)
  ld.add_action(nav2_waypoint_follower_node)
  ld.add_action(nav2_lifecycle_manager_node)

  return ld

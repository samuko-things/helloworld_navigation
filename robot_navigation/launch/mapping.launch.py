import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
  DeclareLaunchArgument
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

 
def generate_launch_description():
  # Set the path to this package.
  robot_nav_pkg_path = get_package_share_directory('robot_navigation') 
 
  #--------------------------------------------------------------------------

  lifecycle_nodes = ["map_saver_server",
                     "slam_toolbox"]

  #--------------------------------------------------------------------------
 
  # Launch configuration variables specific to simulation
  use_sim_time = LaunchConfiguration('use_sim_time')
  param_name = LaunchConfiguration('param_name')
     
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use simulation (Gazebo) clock if true')
 
  declare_param_name_cmd = DeclareLaunchArgument(
    name='param_name',
    default_value='slam_toolbox_mapping',
    description='name of the slam mapping param file')
  
  #--------------------------------------------------------------------------

  param_path = PathJoinSubstitution([
          robot_nav_pkg_path,
          "config",
          PythonExpression(expression=["'", param_name, "'", " + '.yaml'"])
      ]
  )

  nav2_map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver_server",
        output="screen",
        parameters=[
            param_path,
            {"use_sim_time": use_sim_time},
        ],
    )

  slam_toolbox = Node(
      package="slam_toolbox",
      executable="sync_slam_toolbox_node",
      name="slam_toolbox",
      output="screen",
      parameters=[
          param_path,
          {"use_sim_time": use_sim_time},
      ],
  )

  nav2_lifecycle_manager = Node(
      package="nav2_lifecycle_manager",
      executable="lifecycle_manager",
      name="lifecycle_manager_slam",
      output="screen",
      parameters=[
          {"node_names": lifecycle_nodes},
          {"use_sim_time": use_sim_time},
          {"autostart": True, "bond_timeout": 0.0}
      ],
  )

  

  #--------------------------------------------------------------------------
  
  # Create the launch description
  ld = LaunchDescription()

  # add the necessary declared launch arguments to the launch description
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_param_name_cmd)

  # Add the nodes to the launch description
  ld.add_action(nav2_map_saver)
  ld.add_action(slam_toolbox)
  ld.add_action(nav2_lifecycle_manager)

 
  return ld
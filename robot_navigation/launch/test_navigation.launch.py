import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
  DeclareLaunchArgument,
  IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
  # Set the path to this package.
  robot_nav_pkg_path = get_package_share_directory('robot_navigation')
 
  #--------------------------------------------------------------------------

  # Launch configuration variables specific to simulation
  use_sim_time = LaunchConfiguration('use_sim_time')
  map_name = LaunchConfiguration('map_name')

  declare_use_sim_time_cmd = DeclareLaunchArgument(
      name='use_sim_time', 
      default_value='True',
      description='Flag to enable use_sim_time'
    )
  
  declare_map_name_cmd = DeclareLaunchArgument(
      name='map_name',
      default_value='bookstore',
      description='file path to the map needed for navigation'
    )

  #-----------------------------------------------------------------------------

  amcl_localization_path = os.path.join(robot_nav_pkg_path, 'launch', 'amcl_localization.launch.py')

  amcl_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(amcl_localization_path),
        launch_arguments={
                'use_sim_time': use_sim_time,
                'map_name': map_name,
        }.items()
    )

  #--------------------------------------------------------------------------------

  # test_planner = Node(
  #   package='robot_navigation',
  #   executable='a_star_smooth_planner.py',
  #   name='a_star_smooth_planner',
  #   output='screen',
  #   parameters=[
  #     {
  #       'obstacle_clearance_radius': 0.4,
  #       'obstacle_clearance_kernel_size': 8,
  #       'smoother_corner_cutting_dist': 0.2,
  #       'smoother_points_per_curve': 20,
  #       'smoother_line_densification_dist': 0.05
  #     }
  #   ],
  # )

  test_planner = Node(
      package='robot_navigation',
      executable='a_star_smooth_planner.py',
      name='a_star_smooth_planner',
      output='screen',
    )

  test_controller = Node(
    package='robot_navigation',
    executable='pure_pursuit_controller',
    name='pure_pursuit_controller',
    output='screen',
    parameters=[{'look_ahead_distance': 0.15,
                 'max_linear_velocity': 0.2,
                 'max_angular_velocity': 0.8,
                 'path_topic': '/a_star/path'
                 }],
    remappings=[('/cmd_vel', '/cmd_vel_nav')],
  )

  #--------------------------------------------------------------------------------

  # Create the launch description
  ld = LaunchDescription()
 
  # add the necessary declared launch arguments to the launch description
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_map_name_cmd)
 
  # Add the nodes to the launch description
  ld.add_action(amcl_localization_launch)
  ld.add_action(test_planner)
  ld.add_action(test_controller)

  return ld

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
      default_value='room_with_walls',
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

  test_planner = Node(
        package='robot_navigation',
        executable='test_planner',
        name='test_planner',
        output='screen',
        parameters=[
          {'planner_id': 1 }
        ],
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

  return ld
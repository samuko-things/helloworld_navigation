import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
  DeclareLaunchArgument,
  ExecuteProcess,
  IncludeLaunchDescription,
  SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

 
def generate_launch_description():
  # Set the path to this package.
  robot_nav_pkg_path = get_package_share_directory('robot_navigation') 
 
  #--------------------------------------------------------------------------

  lifecycle_nodes = [
    "map_server",
    "amcl",
    "costmap"
  ]

  #--------------------------------------------------------------------------
 
  # Launch configuration variables specific to simulation
  use_sim_time = LaunchConfiguration('use_sim_time')
  amcl_param_name = LaunchConfiguration('amcl_param_name')
  costmap_param_name = LaunchConfiguration('costmap_param_name')
  map_name = LaunchConfiguration('map_name')
     
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use simulation (Gazebo) clock if true')
 
  declare_amcl_param_name_cmd = DeclareLaunchArgument(
    name='amcl_param_name',
    default_value='amcl_localization',
    description='name of the amcl param file')
  
  declare_map_name_cmd = DeclareLaunchArgument(
    name='map_name',
    default_value='hospital',
    description='name of the map yaml file')
   
  declare_costmap_param_name_cmd = DeclareLaunchArgument(
    name='costmap_param_name',
    default_value='costmap',
    description='name of the costmap param file')
  
  #--------------------------------------------------------------------------

  amcl_param_path = PathJoinSubstitution([
          robot_nav_pkg_path,
          "config",
          PythonExpression(expression=["'", amcl_param_name, "'", " + '.yaml'"])
      ]
  )

  map_path = PathJoinSubstitution([
          robot_nav_pkg_path,
          "maps",
          PythonExpression(expression=["'", map_name, "'", " + '.yaml'"])
      ]
  )

  costmap_param_path = PathJoinSubstitution([
            robot_nav_pkg_path,
            "config",
            PythonExpression(expression=["'", costmap_param_name, "'", " + '.yaml'"])
        ]
    )

  nav2_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output="screen",
        parameters=[
            amcl_param_path,
            {"use_sim_time": use_sim_time},
            {'yaml_filename': map_path}
        ],
    )
  
  nav2_amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
                amcl_param_path,
                {"use_sim_time": use_sim_time}
            ],
    )

  nav2_costmap_2d = Node(
    package='nav2_costmap_2d',
    executable='nav2_costmap_2d',
    name='costmap',
    output='screen',
    parameters=[
            costmap_param_path,
            {"use_sim_time": use_sim_time}
        ],
  )

  

  nav2_lifecycle_manager = Node(
      package="nav2_lifecycle_manager",
      executable="lifecycle_manager",
      name="lifecycle_manager_amcl",
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
  ld.add_action(declare_amcl_param_name_cmd)
  ld.add_action(declare_map_name_cmd)
  ld.add_action(declare_costmap_param_name_cmd)

  # Add the nodes to the launch description
  ld.add_action(nav2_map_server)
  ld.add_action(nav2_amcl)
  ld.add_action(nav2_costmap_2d)
  ld.add_action(nav2_lifecycle_manager)

 
  return ld
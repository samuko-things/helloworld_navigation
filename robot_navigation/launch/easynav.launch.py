# Copyright (c) 2025 Intelligent Robotics Lab (URJC)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    robot_nav_pkg_path = get_package_share_directory('robot_navigation')

    simple_params_file = LaunchConfiguration('simple_params_file')
    costmap_params_file = LaunchConfiguration('costmap_params_file')
    use_costmap = LaunchConfiguration('use_costmap')
    # rviz_config = LaunchConfiguration('rviz_config')


    declare_simple_params_file_cmd = DeclareLaunchArgument(
                'simple_params_file',
                default_value=os.path.join(
                    robot_nav_pkg_path, 'config', 'easynav_simple_params.yaml'),
                description='Full path to the ROS2 parameters file for easynav',)
    
    declare_costmap_params_file_cmd = DeclareLaunchArgument(
            'costmap_params_file',
            default_value=os.path.join(
                robot_nav_pkg_path, 'config', 'easynav_costmap_params.yaml'),
            description='Full path to the ROS2 parameters file for easynav',)

    declare_use_costmap_argument = DeclareLaunchArgument(
            'use_costmap',
            default_value='false',
            description='use easynav costmap if true')
    

    # declare_rviz_config_cmd = DeclareLaunchArgument(
    #     'rviz_config',
    #     default_value=os.path.join(
    #         bringup_dir, 'rviz_config', 'easynav_simple.rviz'),
    #     description='Full path to the RViz2 configuration file',
    # )

    easynav_system_simple = Node(
        package='easynav_system',
        executable='system_main',
        parameters=[simple_params_file],
        output='screen',
        remappings=[
            ('/cmd_vel', '/cmd_vel_easynav'),
        ],
        condition=UnlessCondition(use_costmap)
    )

    easynav_system_costmap = Node(
        package='easynav_system',
        executable='system_main',
        parameters=[costmap_params_file],
        output='screen',
        remappings=[
            ('/cmd_vel', '/cmd_vel_easynav'),
        ],
        condition=IfCondition(use_costmap)
    )

    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        # parameters=[
        #     {"frame_id"}
        # ],
        output='screen',
        remappings=[
            ('/cmd_vel_in', '/cmd_vel_easynav'),
            ('/cmd_vel_out', '/cmd_vel_nav'),
        ],
    )

    # rviz_cmd = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=['-d', rviz_config],
    #     parameters=[{'use_sim_time': True}],
    #     output='screen',
    # )


    ld = LaunchDescription()

    ld.add_action(declare_simple_params_file_cmd)
    ld.add_action(declare_costmap_params_file_cmd)
    ld.add_action(declare_use_costmap_argument)
    # ld.add_action(declare_rviz_config_cmd)

    ld.add_action(easynav_system_simple)
    ld.add_action(easynav_system_costmap)
    ld.add_action(twist_stamper)
    # ld.add_action(rviz_cmd)

    return ld
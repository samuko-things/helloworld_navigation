import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                            RegisterEventHandler)
from launch.conditions import IfCondition, UnlessCondition
from launch.events import matches_action
from launch.substitutions import (AndSubstitution, LaunchConfiguration,
                                  NotSubstitution)
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch_ros.descriptions import ParameterFile

from launch_ros.actions import Node


def generate_launch_description():

    robot_nav_pkg_path = get_package_share_directory('robot_navigation')


    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')


    simple_params_file = LaunchConfiguration('simple_params_file')
    costmap_params_file = LaunchConfiguration('costmap_params_file')
    use_costmap = LaunchConfiguration('use_costmap')


    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the slamtoolbox. '
                    'Ignored when use_lifecycle_manager is true.')
    
    declare_use_lifecycle_manager = DeclareLaunchArgument(
        'use_lifecycle_manager', default_value='false',
        description='Enable bond connection during node activation')
    
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(robot_nav_pkg_path,
                                   'config', 'slam_toolbox_mapping.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')


    declare_simple_params_file_cmd = DeclareLaunchArgument(
            'simple_params_file',
            default_value=os.path.join(
                robot_nav_pkg_path, 'config', 'easynav_simple_mapping_params.yaml'),
            description='Full path to the ROS2 parameters file for easynav',)

    declare_costmap_params_file_cmd = DeclareLaunchArgument(
            'costmap_params_file',
            default_value=os.path.join(
                robot_nav_pkg_path, 'config', 'easynav_costmap_mapping_params.yaml'),
            description='Full path to the ROS2 parameters file for easynav',)

    declare_use_costmap_argument = DeclareLaunchArgument(
            'use_costmap',
            default_value='false',
            description='use easynav costmap if true')
    

     # Perform substitution `$find-pkg-share`
    slam_params_file_w_subst = ParameterFile(
        slam_params_file,
        allow_substs=True,
    )
    
    start_async_slam_toolbox_node = LifecycleNode(
        parameters=[
          slam_params_file_w_subst,
          {
            'use_lifecycle_manager': use_lifecycle_manager,
            'use_sim_time': use_sim_time
          }
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=''
    )

    configure_event = EmitEvent(
        event=ChangeState(
          lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
          transition_id=Transition.TRANSITION_CONFIGURE
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_async_slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )


    easynav_system_simple = Node(
        package='easynav_system',
        executable='system_main',
        parameters=[simple_params_file],
        output='screen',
        remappings=[
            ('/maps_manager_node/simple/incoming_map', '/map'),
        ],
        condition=UnlessCondition(use_costmap)
    )


    easynav_system_costmap = Node(
        package='easynav_system',
        executable='system_main',
        parameters=[costmap_params_file],
        output='screen',
        # remappings=[
        #     ('/maps_manager_node/costmap/incoming_map', '/map'),
        # ],
        condition=IfCondition(use_costmap)
    )
    

    ld = LaunchDescription()

    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_lifecycle_manager)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_simple_params_file_cmd)
    ld.add_action(declare_costmap_params_file_cmd)
    ld.add_action(declare_use_costmap_argument)

    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(configure_event)
    ld.add_action(activate_event)
    ld.add_action(easynav_system_simple)
    ld.add_action(easynav_system_costmap)

    return ld
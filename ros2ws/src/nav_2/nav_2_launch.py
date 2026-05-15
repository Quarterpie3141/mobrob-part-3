import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, NotEqualsSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # ── Launch arguments ──────────────────────────────────────────────── #
    mode         = LaunchConfiguration('mode',         default='hardware')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml     = LaunchConfiguration('map',          default='my_map.yaml')
    autostart    = LaunchConfiguration('autostart',    default='true')
    
        # ── Conditions ────────────────────────────────────────────────────── #
    is_slam          = EqualsSubstitution(mode, 'slam')
    is_nav           = EqualsSubstitution(mode, 'nav')
    is_slam_or_slam_nav = PythonExpression(["'", mode, "' in ['slam', 'slam_nav']"])
    is_nav_or_slam_nav  = PythonExpression(["'", mode, "' in ['nav', 'slam_nav']"])
    
    my_nav_pkg_dir = get_package_share_directory('nav_2')
    
    # Find your package path
    my_nav_pkg_dir = get_package_share_directory('nav_2')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(my_nav_pkg_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    # 3. Include the standard Nav2 navigation launch
    # This starts the controller_server, planner_server, recoveries, etc.
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false', # Setting to false for your physical robot
            'params_file': LaunchConfiguration('params_file'),
            'autostart': 'true'
        }.items()
    )
    # This node bypasses the collision_monitor by manually bridging the topics
    cmd_relay_node = Node(
        package='nav_2',        
        executable='cmd_vel_relay', 
        name='cmd_vel_relay',
        output='screen'
    )

     # ================================================================== #
    # SLAM MODE                                                           #
    # ================================================================== #

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        condition=IfCondition(is_slam_or_slam_nav),
        output='screen',
        parameters=[mapper_params, {'use_sim_time': false}],
    )

    # Lifecycle: configure then activate slam_toolbox once it starts.
    # TimerAction periods are relative to when the process starts.
    slam_lifecycle = RegisterEventHandler(
        condition=IfCondition(is_slam_or_slam_nav),
        event_handler=OnProcessStart(
            target_action=slam_toolbox,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[ExecuteProcess(
                        cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
                        output='screen',
                    )],
                ),
                TimerAction(
                    period=6.0,
                    actions=[ExecuteProcess(
                        cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
                        output='screen',
                    )],
                ),
            ],
        ),
    )

    return LaunchDescription([
        declare_params_file_cmd,
        navigation_launch,
        cmd_relay_node,
        slam_toolbox,
        slam_lifecycle
    ])
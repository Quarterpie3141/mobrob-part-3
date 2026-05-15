import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
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
            'autostart': 'true',
            'use_collision_monitor': 'false',
        }.items()
    )
    # This node bypasses the collision_monitor by manually bridging the topics
    cmd_relay_node = Node(
        package='nav_2',        
        executable='cmd_vel_relay', 
        name='cmd_vel_relay',
        output='screen'
    )

    return LaunchDescription([
        declare_params_file_cmd,
        navigation_launch,
        cmd_relay_node
    ])
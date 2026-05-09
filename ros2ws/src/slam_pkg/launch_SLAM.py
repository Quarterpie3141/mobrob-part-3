import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('slam_pkg')
    slam_config_path = os.path.join(pkg_dir, 'resource', 'mapping_params.yaml')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config_path]
    )

    configure_slam = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2',
                    'service',
                    'call',
                    '/slam_toolbox/change_state',
                    'lifecycle_msgs/srv/ChangeState',
                    "{transition: {label: 'configure'}}",
                ],
                output='screen',
            )
        ],
    )

    activate_slam = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2',
                    'service',
                    'call',
                    '/slam_toolbox/change_state',
                    'lifecycle_msgs/srv/ChangeState',
                    "{transition: {label: 'activate'}}",
                ],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        slam_node,
        configure_slam,
        activate_slam,
    ])
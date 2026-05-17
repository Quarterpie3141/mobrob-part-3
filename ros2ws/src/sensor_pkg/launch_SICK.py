from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():

    sick_lidar = ExecuteProcess(
        cmd=[
            'ros2',
            'launch',
            'sick_scan_xd',
            'sick_tim_7xx.launch.py',
            'hostname:=192.168.0.1',
            'frame_id:=laser_frame',
            'tf_base_frame_id:=base_link',
            'range_min:=0.2',
            'scantime:=0.3',  # Target 10Hz
            'skip:=1',        # Set to 1 if you want
        ],
        output='screen',
    )

    lidar_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x',
            '0.125',
            '--y',
            '0',
            '--z',
            '0.38',
            '--yaw',
            '0',
            '--pitch',
            '0',
            '--roll',
            '0',
            '--frame-id',
            'base_link',
            '--child-frame-id',
            'laser_frame',
        ],
        output='screen',
    )

    return LaunchDescription([
        sick_lidar,
        lidar_transform,
    ])

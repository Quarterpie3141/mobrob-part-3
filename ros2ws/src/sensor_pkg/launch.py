import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():


    ekf_config = os.path.join(
        get_package_share_directory('ekf_pkg'),
        'resource',
        'ekf_config.yaml',
    )




    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
    )


    pkg_imu_orientation = get_package_share_directory('imu_filter_madgwick')
    pkg_imu_node = get_package_share_directory('phidgets_spatial')

    phidget_imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_imu_node, 'launch', 'spatial-launch.py')
        )
        
    )

    magwick_imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_imu_orientation, 'launch', 'imu_filter.launch.py')
        ),
        launch_arguments={
            'publish_tf': 'false',
            'fixed_frame': 'imu',
        }.items()
    )

    heading_printer_node = Node,
        package='ekf_pkg',
        executable='heading_printer',
        name='heading_printer',
        output='screen',
    )
        

    gps_node = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        remappings=[
            ('/fix', '/gnss/fix'),
        ],
        parameters=[{
            'port': '/dev/ttyACM0',
            'baud_rate': 4800,
        }],
        output='screen',
    )

    sick_lidar = ExecuteProcess(
        cmd=[
            'ros2',
            'launch',
            'sick_scan_xd',
            'sick_tim_7xx.launch.py',
            'hostname:=192.168.0.1',
            'frame_id:=laser_frame',
            'tf_base_frame_id:=base_link',
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

    gps_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x',
            '0.125',
            '--y',
            '-0.13',
            '--z',
            '0.275',
            '--yaw',
            '0',
            '--pitch',
            '0',
            '--roll',
            '0',
            '--frame-id',
            'base_link',
            '--child-frame-id',
            'gps_link',
        ],
        output='screen',
    )

    imu_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0.21",
            "--y",
            "0.0",
            "--z",
            "0.21",
            "--yaw",
            "0.0",
            "--pitch",
            "0",
            "--roll",
            "0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "imu",
        ],
    )

    return LaunchDescription([
        ekf_node,
        phidget_imu,
        magwick_imu,
        heading_printer_node,
        gps_node,
        sick_lidar,
        lidar_transform,
        gps_transform,
        imu_transform,
    ])

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

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
    return LaunchDescription([
        gps_node,
        gps_transform,
    ])

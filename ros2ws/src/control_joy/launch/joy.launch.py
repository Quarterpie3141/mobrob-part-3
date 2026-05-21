from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[{"autorepeat_rate": 20.0}],
        output="both",
    )

    control_joy_node = Node(
        package="control_joy",
        executable="control_joy_node",
        name="control_joy_node",
        output="both",
    )

    return LaunchDescription([joy_node, control_joy_node])

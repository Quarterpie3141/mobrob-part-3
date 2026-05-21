#!/usr/bin/env python3

from enum import Enum
from geometry_msgs.msg import Twist
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Joy
from std_msgs.msg import String


class ControllerState(str, Enum):
    WAITING = 'waiting for transition to autonomous mode'
    DRIVING = 'driving to waypoint'
    TAKING_PICTURE = 'taking picture'
    STOPPED = 'stopped'


class ControlJoyNode(Node):
    def __init__(self) -> None:
        super().__init__("control_joy_node")

        self.declare_parameter("linear_axis", 1)
        self.declare_parameter("angular_axis", 0)
        self.declare_parameter("linear_scale", 0.5)
        self.declare_parameter("angular_scale", 0.5)
        self.declare_parameter("linear_deadzone", 0.08)
        self.declare_parameter("angular_deadzone", 0.08)

        self._linear_axis = int(self.get_parameter("linear_axis").value)
        self._angular_axis = int(self.get_parameter("angular_axis").value)
        self._linear_scale = float(self.get_parameter("linear_scale").value)
        self._angular_scale = float(self.get_parameter("angular_scale").value)
        self._linear_deadzone = float(self.get_parameter("linear_deadzone").value)
        self._angular_deadzone = float(self.get_parameter("angular_deadzone").value)

        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.slave_status_pub = self.create_publisher(String, "/slave/status", 10)
        
        self.create_subscription(Joy, "/joy", self._joy_callback, 10)
        self.create_subscription(String, "/master/status", self._master_status_callback, 10)

        self._was_active = False
        self._master_state = ControllerState.WAITING.value

    def _master_status_callback(self, msg: String) -> None:
        self._master_state = msg.data

    def _joy_callback(self, msg: Joy) -> None:
        if self._linear_axis >= len(msg.axes) or self._angular_axis >= len(msg.axes):
            self.get_logger().warn("Joystick axis index out of range", throttle_duration_sec=2.0)
            return

        if msg.buttons[1] == 1:
            self.slave_status_pub.publish(String(data='waiting'))
        elif msg.buttons[0] == 1:
            self.slave_status_pub.publish(String(data='transition'))
            
        else:
            is_active = (msg.buttons[9] == 1 and msg.buttons[10] == 1)

            if is_active:
                if self._master_state == ControllerState.DRIVING.value:
                    self.get_logger().warn("Master is driving. Manual override blocked.", throttle_duration_sec=2.0)
                    return

                linear_raw = float(msg.axes[self._linear_axis])
                angular_raw = float(msg.axes[self._angular_axis])

                cmd = Twist()
                cmd.linear.x = self._apply_deadzone(linear_raw, self._linear_deadzone) * self._linear_scale
                cmd.angular.z = self._apply_deadzone(angular_raw, self._angular_deadzone) * self._angular_scale
                self._cmd_pub.publish(cmd)
                
                self._was_active = True
            
            else:
                if self._was_active:
                    cmd = Twist()
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    self._cmd_pub.publish(cmd)
                    
                    self._was_active = False

    @staticmethod
    def _apply_deadzone(value: float, deadzone: float) -> float:
        if abs(value) < deadzone:
            return 0.0
        return value


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControlJoyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
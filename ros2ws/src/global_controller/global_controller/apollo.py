from enum import Enum
from typing import List, Optional, Tuple
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String


from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient


# Hardcoded waypoints (Lat, Lon, Alt)
HARDCODED_WAYPOINTS: List[Tuple[float, float, float]] = [
    (1.5, 2.0, 0.0),
    (4.0, -1.0, 90.0),
    (0.0, 0.0, 0.0)  # Return to origin
]

class ControllerState(str, Enum):
    WAITING = 'waiting for transition to autonomous mode'
    DRIVING = 'driving to waypoint'
    TAKING_PICTURE = 'taking picture'
    STOPPED = 'stopped'

class GlobalControllerNode(Node):
    def __init__(self) -> None:
        super().__init__('global_controller')

        self.declare_parameter('goal_publish_period_sec', 1.0)
        self.declare_parameter('status_log_period_sec', 2.0)

#NA2 stuff 
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_handle = None
        # Initialize waypoints list with hardcoded values
        self._waypoints = list(HARDCODED_WAYPOINTS)
        self._goal_publish_period = float(self.get_parameter('goal_publish_period_sec').value)
        self._status_log_period = float(self.get_parameter('status_log_period_sec').value)

        self._state = ControllerState.WAITING
        self._current_waypoint_index = 0
        self._last_goal_index: Optional[int] = None
        self._last_state_log_time = self.get_clock().now()
        self._start_position: Optional[Tuple[float, float, float]] = None
        self._start_waypoint_added = False

        # Publishers
        self._master_status_pub = self.create_publisher(String, '/master/status', 10)
        #self._goal_pub = self.create_publisher(String, '/gnss/goal', 10)
        
        # Subscriptions
        self.create_subscription(String, '/slave/status', self._handle_slave_status, 10)
        self.create_subscription(NavSatFix, '/gnss/fix', self._handle_gnss_fix, 10)
        
        # Timer
        self.create_timer(self._goal_publish_period, self._on_timer)

        if not self._waypoints:
            self.get_logger().warn('No hardcoded waypoints loaded.')
        else:
            self.get_logger().info(f'Loaded {len(self._waypoints)} hardcoded waypoint(s).')

        self._publish_state()
        self.get_logger().info(f'Global controller started in state: {self._state.value}')


    def _handle_slave_status(self, msg: String) -> None:
        command = msg.data.strip().lower()
        self.get_logger().info(f'Received /slave/status: {command}')

        if command == 'waiting':
            self._current_waypoint_index = 0
            self._last_goal_index = None
            self._transition_to(ControllerState.WAITING)
            return

        # Transition from WAITING to DRIVING
        if self._state == ControllerState.WAITING and command == 'transition':
            if self._has_waypoints_remaining():
                self._transition_to(ControllerState.DRIVING)

                x, y, phi = self._waypoints[self._current_waypoint_index]
                self._send_nav2_goal(x, y, phi)

        # Advance mission logic
        if self._state == ControllerState.DRIVING and command == 'finished':
            # self._advance_mission()
            self._transition_to(ControllerState.TAKING_PICTURE)
            return

        # Restart from STOPPED
        if self._state == ControllerState.STOPPED and command == 'transition':
            if self._has_waypoints_remaining():
                self._transition_to(ControllerState.DRIVING)
                x, y, phi = self._waypoints[self._current_waypoint_index]
                self._send_nav2_goal(x, y, phi) 
#NAV2
    def _send_nav2_goal(self, x, y, theta_degrees):
        """Sends an X, Y, and Phi (Heading) goal to the Nav2 Action Server."""
        if not self._nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Nav2 Action Server not available!')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Position
        goal_msg.pose.position.x = float(x)
        goal_msg.pose.position.y = float(y)
        goal_msg.pose.position.z = 0.0

        phi_rad = math.radians(theta_degrees)
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = math.sin(phi_rad / 2.0)
        goal_msg.pose.orientation.w = math.cos(phi_rad / 2.0)

        self.get_logger().info(f'Requesting Nav2: X={x}, Y={y}, Phi={theta_degrees}°')
        
        self._send_goal_future = self._nav_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by Nav2')
            return
        
        self._goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        """This replaces your 'slave status == finished' logic."""
        self.get_logger().info('Target Reached! Advancing mission...')
        self._advance_mission()


    def _on_timer(self) -> None:
        self._publish_state()
        self._log_status_heartbeat()

        # if self._state == ControllerState.DRIVING:
        #     self._publish_current_goal()

    def _transition_to(self, new_state: ControllerState) -> None:
        if self._state == new_state:
            return
        old_state = self._state
        self._state = new_state
        self.get_logger().info(f'State transition: {old_state.value} -> {new_state.value}')
        self._publish_state()

    def _publish_state(self) -> None:
        status_msg = String()
        status_msg.data = self._state.value
        self._master_status_pub.publish(status_msg)

    def _log_status_heartbeat(self) -> None:
        now = self.get_clock().now()
        elapsed_sec = (now - self._last_state_log_time).nanoseconds / 1e9
        if elapsed_sec < self._status_log_period:
            return

        total = len(self._waypoints)
        curr = self._current_waypoint_index
        self.get_logger().info(
            f'[HEARTBEAT] state={self._state.value} | '
            f'waypoint={curr + 1}/{total} | '
            f'remaining={max(0, total - curr)}'
        )
        self._last_state_log_time = now

    def _publish_current_goal(self, force: bool = False) -> None:
        if not self._has_waypoints_remaining():
            self._transition_to(ControllerState.STOPPED)
            return

        x, y, phi = self._waypoints[self._current_waypoint_index]

        if force or self._last_goal_index != self._current_waypoint_index:
            self.get_logger().info(f'[GOAL STATUS] Next target: X={x}, Y={y}')
        
        self._last_goal_index = self._current_waypoint_index
    def _advance_mission(self) -> None:
        """Increments the index and checks if there are more points (like Home) to visit."""
        self._current_waypoint_index += 1
        if self._has_waypoints_remaining():
            self.get_logger().info(f'Advancing to waypoint index {self._current_waypoint_index}')
            self._transition_to(ControllerState.DRIVING)


            x, y, phi = self._waypoints[self._current_waypoint_index]
            self._send_nav2_goal(x, y, phi)
        else:
            self.get_logger().info('Mission Complete!')
            self._transition_to(ControllerState.STOPPED)
        
    def _has_waypoints_remaining(self) -> bool:
        return self._current_waypoint_index < len(self._waypoints)

def main(args=None) -> None:
    rclpy.init(args=args)
    node = GlobalControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
from enum import Enum
from typing import List, Optional, Tuple
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

# Nav2 Additions
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose 
from action_msgs.msg import GoalStatus # Add this import at the top


HARDCODED_WAYPOINTS: List[Tuple[float, float, float]] = [
    (5.0, 0.0, 0.0), 
    (10.0, 1.0, 0.785), 
    (5.0, 0.0, 0.0), 
    (0.0, 0.0, 0.0), 
    (-5.0, 0.0, -0.785),
    (0.0, 0.0, 0.0)
]

class ControllerState(str, Enum):   
    WAITING = 'waiting for transition to autonomous mode'
    DRIVING = 'driving to waypoint'
    TAKING_PICTURE = 'taking picture'
    STOPPED = 'stopped'

def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """
    Converts a yaw angle (in radians) to a fully normalized unit quaternion (x, y, z, w).
    Guarantees compatibility with Nav2/tf2.
    """
    half_yaw = yaw * 0.5
    qz = math.sin(half_yaw)
    qw = math.cos(half_yaw)
    
    # Explicit normalization safety check
    magnitude = math.sqrt(qz**2 + qw**2)
    if magnitude > 0.0:
        qz /= magnitude
        qw /= magnitude
        
    return (0.0, 0.0, qz, qw)


class GlobalControllerNode(Node):
    def __init__(self) -> None:
        super().__init__('global_controller')

        self.declare_parameter('status_log_period_sec', 2.0)
        self._status_log_period = float(self.get_parameter('status_log_period_sec').value)

        #waypoints
        self._waypoints = list(HARDCODED_WAYPOINTS)
        self._state = ControllerState.WAITING
        self._current_waypoint_index = 0
        self._last_state_log_time = self.get_clock().now()
    
        self._goal_handle = None
        self._action_future = None

        # status publish
        self._master_status_pub = self.create_publisher(String, '/master/status', 10)
        
        self.create_subscription(String, '/slave/status', self._handle_slave_status, 10)
        
        # Nav2 Action Client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        #timer 
        self.create_timer(1.0, self._on_timer)

        self.get_logger().info(f'Loaded {len(self._waypoints)} Nav2 waypoints.')
        self._publish_state()
        self.get_logger().info(f'Global controller started in state: {self._state.value}')

    def _handle_slave_status(self, msg: String) -> None:
        command = msg.data.strip().lower()
        # logging this every time can flood the console, but good for now
        # self.get_logger().info(f'Received /slave/status: {command}')

        # FIX: Only allow the 'waiting' command to reset the mission if we aren't 
        # currently in the middle of an active Nav2 goal.
        if command == 'waiting':
            if self._state != ControllerState.DRIVING:
                self._cancel_current_nav_goal()
                self._current_waypoint_index = 0
                self._transition_to(ControllerState.WAITING)
            return

        if self._state == ControllerState.WAITING and command == 'transition':
            if self._has_waypoints_remaining():
                self._transition_to(ControllerState.DRIVING)
                self._send_nav2_goal()
            else:
                self.get_logger().warn('Cannot drive: No waypoints available.')
                self._transition_to(ControllerState.STOPPED)
            return

        if self._state == ControllerState.TAKING_PICTURE and command == 'picture_done':
            self._advance_mission()
            return

        # Restart from Stopped
        if self._state == ControllerState.STOPPED and command == 'transition':
            if self._has_waypoints_remaining():
                self._current_waypoint_index = 0 # Reset to start if coming from Stopped
                self._transition_to(ControllerState.DRIVING)
                self._send_nav2_goal()

    def _on_timer(self) -> None:
        self._publish_state()
        self._log_status_heartbeat()

    def _send_nav2_goal(self) -> None:
        """Constructs and sends a NavigateToPose Action request to Nav2."""
        if not self._has_waypoints_remaining():
            self._transition_to(ControllerState.STOPPED)
            return

        self.get_logger().info('Waiting for Nav2 "navigate_to_pose" action server...')
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Nav2 action server not available! Aborting goal send.')
            return

        x, y, phi = self._waypoints[self._current_waypoint_index]
        qx, qy, qz, qw = yaw_to_quaternion(phi)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(f'[NAV2 GOAL] Sending Target Index {self._current_waypoint_index}: X={x}, Y={y}, Phi={phi}')
        
        self._action_future = self._nav_client.send_goal_async(goal_msg)
        self._action_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
            self._goal_handle = future.result() # Store the handle so we can cancel it later
            if not self._goal_handle.accepted:
                self.get_logger().error('Goal rejected by Nav2')
                return
            
            self._result_future = self._goal_handle.get_result_async()
            self._result_future.add_done_callback(self._get_result_callback)
    def _get_result_callback(self, future):
            status = future.result().status
            
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Goal succeeded! Moving to next waypoint.')
                self._current_waypoint_index += 1
                if self._current_waypoint_index < len(self._waypoints):
                    self._send_nav2_goal()
                else:
                    self.get_logger().info('Mission Complete!')
                    self._transition_to(ControllerState.STOPPED)
            
            elif status == GoalStatus.STATUS_ABORTED: 
                self.get_logger().warn('Nav2 Aborted (Status 6). Likely a CPU/Timeout spike. Retrying...')
                # DO NOT transition to WAITING. 
                # Use a timer to retry so we don't spam the server instantly
                self.retry_timer = self.create_timer(2.0, self._handle_oneshot_retry)                
            else:
                # For other failures (Canceled, etc.), now we can halt
                self.get_logger().error(f'Goal failed with status code: {status}. Mission Halted.')
                self._transition_to(ControllerState.WAITING)
    def _handle_oneshot_retry(self):
        self.retry_timer.cancel()  # Kill it immediately so it only runs once
        self._retry_current_waypoint()
    def _retry_current_waypoint(self):
        """Helper to resend the goal without resetting the mission."""
        self.get_logger().info(f'Retrying waypoint {self._current_waypoint_index + 1}...')
        self._send_nav2_goal()

    def _cancel_current_nav_goal(self) -> None:
        if self._goal_handle is not None:
            self.get_logger().info('Canceling current Nav2 goal execution.')
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

    def _advance_mission(self) -> None:
        self._current_waypoint_index += 1

        if self._has_waypoints_remaining():
            self.get_logger().info(f'Advancing to next waypoint (Index {self._current_waypoint_index})')
            self._transition_to(ControllerState.DRIVING)
            self._send_nav2_goal()
        else:
            self.get_logger().info('Mission Complete. All waypoints reached.')
            self._transition_to(ControllerState.STOPPED)
        
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
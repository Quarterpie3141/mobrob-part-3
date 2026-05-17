# from enum import Enum
# from typing import List, Optional, Tuple
# import math

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix
# from std_msgs.msg import String

# # Hardcoded waypoints (Lat, Lon, Alt)
# HARDCODED_WAYPOINTS: List[Tuple[float, float, float]] = [
#     (-31.9805064, 115.8177856, 180.0),
#     (-31.9803605, 115.8177973, 90.0),
#     (-31.9801137, 115.8178302, 30.0)
# ]

# class ControllerState(str, Enum):
#     WAITING = 'waiting for transition to autonomous mode'
#     DRIVING = 'driving to waypoint'
#     TAKING_PICTURE = 'taking picture'
#     STOPPED = 'stopped'

# class GlobalControllerNode(Node):
#     def __init__(self) -> None:
#         super().__init__('global_controller')

#         self.declare_parameter('goal_publish_period_sec', 1.0)
#         self.declare_parameter('status_log_period_sec', 2.0)

#         # Initialize waypoints list with hardcoded values
#         self._waypoints = list(HARDCODED_WAYPOINTS)
#         self._goal_publish_period = float(self.get_parameter('goal_publish_period_sec').value)
#         self._status_log_period = float(self.get_parameter('status_log_period_sec').value)

#         self._state = ControllerState.WAITING
#         self._current_waypoint_index = 0
#         self._last_goal_index: Optional[int] = None
#         self._last_state_log_time = self.get_clock().now()
#         self._start_position: Optional[Tuple[float, float, float]] = None
#         self._start_waypoint_added = False

#         # Publishers
#         self._master_status_pub = self.create_publisher(String, '/master/status', 10)
#         self._goal_pub = self.create_publisher(String, '/gnss/goal', 10)
        
#         # Subscriptions
#         self.create_subscription(String, '/slave/status', self._handle_slave_status, 10)
#         self.create_subscription(NavSatFix, '/gnss/fix', self._handle_gnss_fix, 10)
        
#         # Timer
#         self.create_timer(self._goal_publish_period, self._on_timer)

#         if not self._waypoints:
#             self.get_logger().warn('No hardcoded waypoints loaded.')
#         else:
#             self.get_logger().info(f'Loaded {len(self._waypoints)} hardcoded waypoint(s).')

#         self._publish_state()
#         self.get_logger().info(f'Global controller started in state: {self._state.value}')

#     def _handle_gnss_fix(self, msg: NavSatFix) -> None:
#         """Captures the first valid GNSS fix as the 'Home' return point."""
#         if self._start_waypoint_added:
#             return

#         if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude):
#             return

#         self._start_position = (msg.latitude, msg.longitude, msg.altitude)
#         self._waypoints.append(self._start_position)
#         self._start_waypoint_added = True

#         self.get_logger().info(
#             f'[GPS] HOME CAPTURED: lat={msg.latitude}, lon={msg.longitude}. '
#             f'Total waypoints in mission: {len(self._waypoints)}'
#         )

#     def _handle_slave_status(self, msg: String) -> None:
#         command = msg.data.strip().lower()
#         self.get_logger().info(f'Received /slave/status: {command}')

#         if command == 'waiting':
#             self._current_waypoint_index = 0
#             self._last_goal_index = None
#             self._transition_to(ControllerState.WAITING)
#             return

#         # Transition from WAITING to DRIVING
#         if self._state == ControllerState.WAITING and command == 'transition':
#             if self._has_waypoints_remaining():
#                 self._transition_to(ControllerState.DRIVING)
#                 self._publish_current_goal(force=True)
#             else:
#                 self.get_logger().warn('Cannot drive: No waypoints available.')
#                 self._transition_to(ControllerState.STOPPED)
#             return

#         # Advance mission logic
#         if self._state == ControllerState.DRIVING and command == 'finished':
#             # self._advance_mission()
#             self._transition_to(ControllerState.TAKING_PICTURE)
#             return

#         # Restart from STOPPED
#         if self._state == ControllerState.STOPPED and command == 'transition':
#             if self._has_waypoints_remaining():
#                 self._transition_to(ControllerState.DRIVING)
#                 self._publish_current_goal(force=True)

#     def _on_timer(self) -> None:
#         self._publish_state()
#         self._log_status_heartbeat()

#         if self._state == ControllerState.DRIVING:
#             self._publish_current_goal()

#     def _transition_to(self, new_state: ControllerState) -> None:
#         if self._state == new_state:
#             return
#         old_state = self._state
#         self._state = new_state
#         self.get_logger().info(f'State transition: {old_state.value} -> {new_state.value}')
#         self._publish_state()

#     def _publish_state(self) -> None:
#         status_msg = String()
#         status_msg.data = self._state.value
#         self._master_status_pub.publish(status_msg)

#     def _log_status_heartbeat(self) -> None:
#         now = self.get_clock().now()
#         elapsed_sec = (now - self._last_state_log_time).nanoseconds / 1e9
#         if elapsed_sec < self._status_log_period:
#             return

#         total = len(self._waypoints)
#         curr = self._current_waypoint_index
#         self.get_logger().info(
#             f'[HEARTBEAT] state={self._state.value} | '
#             f'waypoint={curr + 1}/{total} | '
#             f'remaining={max(0, total - curr)}'
#         )
#         self._last_state_log_time = now

#     def _publish_current_goal(self, force: bool = False) -> None:
#         if not self._has_waypoints_remaining():
#             self._transition_to(ControllerState.STOPPED)
#             return

#         lat, lon, alt = self._waypoints[self._current_waypoint_index]
#         goal_msg = String()
#         goal_msg.data = f'{lat},{lon},{alt}'
#         self._goal_pub.publish(goal_msg)

#         if force or self._last_goal_index != self._current_waypoint_index:
#             self.get_logger().info(f'[GOAL] Targeted index {self._current_waypoint_index}: {lat}, {lon}')
        
#         self._last_goal_index = self._current_waypoint_index

#     def _advance_mission(self) -> None:
#         """Increments the index and checks if there are more points (like Home) to visit."""
#         self._current_waypoint_index += 1
#         self._last_goal_index = None

#         if self._has_waypoints_remaining():
#             self.get_logger().info(f'Advancing to next waypoint (Index {self._current_waypoint_index})')
#             self._transition_to(ControllerState.DRIVING)
#             self._publish_current_goal(force=True)
#         else:
#             self.get_logger().info('Mission Complete. All waypoints (including Home) reached.')
#             self._transition_to(ControllerState.STOPPED)
        
#     def _has_waypoints_remaining(self) -> bool:
#         return self._current_waypoint_index < len(self._waypoints)

# def main(args=None) -> None:
#     rclpy.init(args=args)
#     node = GlobalControllerNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
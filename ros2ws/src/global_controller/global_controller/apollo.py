from enum import Enum
import string
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
from nav_msgs.msg import OccupancyGrid


import cv2
import numpy as np
import math

HARDCODED_WAYPOINTS: List[Tuple[float, float, float]] = [
    (5.0, 0.0, 3.141), 
    (0.0, 0.0, 3.141), 
    (-5.0, 0.0, 0.0), 
    (0.0, 0.0, 3.141)
]

EXPLORE_WAYPOINTS: List[Tuple[float, float, float]] = [
    (0,0,0)
]

EXPLORE_WAYPOINTS_FLAG: List[bool] = [False] # parallel list to EXPLORE_WAYPOINTS to indicate if it's been sent to Nav2 yet

CLASSIFIED_WAYPOINTS: List[Tuple[float, float, float, str]] = [
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
        self._explore_way = list(EXPLORE_WAYPOINTS)
        self._explore_way_flag = list(EXPLORE_WAYPOINTS_FLAG)
        self._state = ControllerState.WAITING
        self._current_waypoint_index = 0
        self._current_explore_index = 1
        self._last_state_log_time = self.get_clock().now()
    
        self._goal_handle = None
        self._action_future = None

        # status publish
        self._gui_nav2_goal_pub = self.create_publisher(String, '/gui/nav2_goal', 10)
        self._master_status_pub = self.create_publisher(String, '/master/status', 10)
        self._status_log_pub = self.create_publisher(String, '/status_log', 10)
        
        self.create_subscription(String, '/slave/status', self._handle_slave_status, 10)

        #cost map sub
        self.create_subscription(OccupancyGrid, '/map', self._handle_costmap, 10)
        self.costmap_data = None

        #poi pub
        self.poi_pub = self.create_publisher(String, '/poi', 10)
        self.objects_isolated = False

        #classified pub
        self.classified_poi_pub = self.create_publisher(String, '/classified_poi', 10)

        #phase
        self.create_subscription(String, '/phase', self._handle_phase, 10)
        self.phase = None

        #detect letter signal publisher
        self.start_letter_detection_pub = self.create_publisher(String, '/check_label', 10)
        self.letter_detection_sub = self.create_subscription(String, '/camera/letter_detection', self.letter_detection_callback, 10)
        self.last_detected_letter = None

        # Nav2 Action Client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        #timer 
        self.create_timer(1.0, self._on_timer)

        self.get_logger().info(f'Loaded {len(self._waypoints)} Nav2 waypoints.')
        self._publish_state()
        self.get_logger().info(f'Global controller started in state: {self._state.value}')

    def letter_detection_callback(self, msg: String) -> None:
        self.last_detected_letter = msg.data
        if msg.data in ("nothing detected", "confidence too low", "no frame available"):
            self.start_letter_detection_pub.publish(String(data="classify"))
            return
        else:
            x = self._explore_way[self._current_explore_index-1][0]
            y = self._explore_way[self._current_explore_index-1 ][1]
            phi = self._explore_way[self._current_explore_index-1][2]
            CLASSIFIED_WAYPOINTS.append((x, y, phi, self.last_detected_letter))

            s = str([{'x': x, 'y': y, 'phi': phi, 'label': label} for x, y, phi, label in CLASSIFIED_WAYPOINTS])
            self.get_logger().info(f'Publishing classified poi to GUI: {s}')
            self.classified_poi_pub.publish(String(data=s))
            self._publish_status_log(
                "Classified " + self.last_detected_letter + " at:" + str(self._explore_way[self._current_explore_index-1])
            )


            self.get_logger().info('IMAGE CLASSIFICATION COMPLETE. Detected letter: ' + self.last_detected_letter)
            self.get_logger().info('IMAGE CLASSIFICATION COMPLETE. Detected letter: ' + self.last_detected_letter)
            self.get_logger().info('IMAGE CLASSIFICATION COMPLETE. Detected letter: ' + self.last_detected_letter)
            self.get_logger().info('IMAGE CLASSIFICATION COMPLETE. Detected letter: ' + self.last_detected_letter)
            self.get_logger().info('IMAGE CLASSIFICATION COMPLETE. Detected letter: ' + self.last_detected_letter)
            self.get_logger().info('IMAGE CLASSIFICATION COMPLETE. Detected letter: ' + self.last_detected_letter)
            self.last_detected_letter = None # reset last detected letter before next classification



            if self._current_waypoint_index < (len(self._waypoints)):
                self.get_logger().info(" EXPLORE POI:"+str(self._explore_way[self._current_explore_index]))
                self.get_logger().info(" EXPLORE POI:"+str(self._explore_way[self._current_explore_index]))
                self.get_logger().info(" EXPLORE POI:"+str(self._explore_way[self._current_explore_index]))
                self.get_logger().info(" EXPLORE POI:"+str(self._explore_way[self._current_explore_index]))
                self.get_logger().info(" EXPLORE POI:"+str(self._explore_way[self._current_explore_index]))
                self.get_logger().info(" EXPLORE POI:"+str(self._explore_way[self._current_explore_index]))
                self.get_logger().info(" EXPLORE POI:"+str(self._explore_way[self._current_explore_index]))
                self._current_explore_index += 1
                self._send_nav2_goal()

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
        self._publish_waypoints()

    def _publish_waypoints(self) -> None: # publishes classified waypoints every second
        # TO DO MOVE THIS TO BENS CLASSIFICATION SCRIPT OCE ITS READY
        
        pass

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
        self._gui_nav2_goal_pub.publish(String(data=f'{x},{y},{phi}'))
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
                    self.get_logger().info('Mapping Initial Complete!')
                    #self._transition_to(ControllerState.STOPPED)
                    #call stuff here
                    if self._current_explore_index ==1:
                        self.isolate_objects()

                    if self._current_explore_index < len(self._explore_way):
                        dist_btw_points = math.sqrt((self._explore_way[self._current_explore_index][0] - self._waypoints[-1][0])**2 + (self._explore_way[self._current_explore_index][1] - self._waypoints[-1][1])**2)
                        self.get_logger().info(f'Distance from last waypoint to next explore point: {dist_btw_points} meters')
                        if dist_btw_points > 4.0:
                            self.get_logger().warn('Next explore point is quite far from last waypoint. Consider adding intermediate waypoints for better navigation.')
                            interm_x = (self._waypoints[-1][0] + self._explore_way[self._current_explore_index][0]) / 2
                            interm_y = (self._waypoints[-1][1] + self._explore_way[self._current_explore_index][1]) / 2
                            delta_x = self._explore_way[self._current_explore_index][0] - self._waypoints[-1][0]
                            delta_y = self._explore_way[self._current_explore_index][1] - self._waypoints[-1][1]
                            interm_phi = math.atan2(delta_y, delta_x)
                            self._waypoints.append((interm_x, interm_y, interm_phi))
                            self._explore_way_flag.append(False) # intermediate point flag is false
                            self.get_logger().info(f'Added intermediate waypoint at X={interm_x}, Y={interm_y}, Phi={interm_phi} to bridge gap to explore point.')
                            self._publish_status_log(
                            "Going to intermediate waypoint:" + str(self._waypoints[-1])
                            )

                        self._waypoints.append(self._explore_way[self._current_explore_index])
                        self._explore_way_flag.append(True)
                        self._publish_status_log(
                            "SENDING EXPLORE POI:" + str(self._explore_way[self._current_explore_index])
                        )

                        if self._current_explore_index != 1 and self._explore_way_flag[self._current_explore_index] == True:
                            #cv detection drive stuff
                            #TAKE PHOTO HERE
                            #classified waypoints
                            self._publish_status_log(
                            "Starting Classification at:" + str(self._explore_way[self._current_explore_index])
                            )
                            self.start_letter_detection_pub.publish(String(data="classify"))
                        
                        if self._current_explore_index == 1:
                            # self._current_explore_index += 1
                            self._send_nav2_goal()
                            self._current_explore_index += 1


                        #self._current_waypoint_index += 1

                        
                           

            elif status == GoalStatus.STATUS_ABORTED: 
                self.get_logger().warn('Nav2 Aborted (Status 6). Likely a CPU/Timeout spike. Retrying...')
                # DO NOT transition to WAITING. 
                # Use a timer to retry so we don't spam the server instantly
                self.retry_timer = self.create_timer(2.0, self._handle_oneshot_retry)                
            else:
                # For other failures (Canceled, etc.), now we can halt
                self.get_logger().info(f'waiting for classification')

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

    def _publish_status_log(self, message: str) -> None:
        log_msg = String()
        log_msg.data = message
        self._status_log_pub.publish(log_msg)

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

    def _handle_costmap(self, msg: OccupancyGrid) -> None:
        self.costmap_data = msg
        
    def _handle_phase(self, msg: String) -> None:
        self.phase = msg.data
        # if self.phase == 'phase_2' and self.objects_isolated == False:
        #     self.objects_isolated = True
        #     self.isolate_objects()
        self.get_logger().info(f'Received phase update: {self.phase}')

        if self.phase == 'phase_2':
            self.get_logger().info('Phase 2 detected. Starting object isolation.')
            self.isolate_objects()

    def isolate_objects(self):
        self.get_logger().info('Isolating objects from costmap...')
        costmap = self.costmap_data
        object_positions = []
        #conver to binary image
        grid = np.array(costmap.data, dtype=np.int8).reshape(
        costmap.info.height, costmap.info.width
        )
        img = np.zeros_like(grid, dtype=np.uint8)
        img[grid == 100] = 255      # occupied  → white
        img[grid == 0]   = 0        # free      → black
        img[grid == -1]  = 0        # unknown   → black (or 127 if you want it visible)
        _, binary = cv2.threshold(img, 250, 255, cv2.THRESH_BINARY)

                
        kernel = np.ones((3, 3), np.uint8)
        # dilated = cv2.dilate(binary, kernel, iterations=1)
       
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary)

        #Apply minimum area
        min_area = 5
        max_area = 100
        for i in range(1, num_labels):
            if stats[i, cv2.CC_STAT_AREA] > min_area and stats[i, cv2.CC_STAT_AREA] < max_area:
                #find x,y coords
                res = costmap.info.resolution 
                area_m2 = stats[i, cv2.CC_STAT_AREA] * (res ** 2)
                w_m     = stats[i, cv2.CC_STAT_WIDTH]  * res
                h_m     = stats[i, cv2.CC_STAT_HEIGHT] * res

                cx_m = centroids[i][0] * res + costmap.info.origin.position.x
                cy_m = centroids[i][1] * res + costmap.info.origin.position.y

            
                if abs(cx_m) < 10 and abs(cy_m) < 7 and math.sqrt(cx_m**2 + cy_m**2) < 11.0: # sanity check to filter out bad detections near the robot
                    if len(object_positions) == 0:
                        object_positions.append((cx_m, cy_m, 0.0))
                    else: 
                        if math.sqrt((object_positions[-1][0] - cx_m)**2  +  (object_positions[-1][1] - cy_m)**2) > 0.80:
                            object_positions.append((cx_m, cy_m, 0.0))

        for i in object_positions:
            angle_calc = round(math.atan2(i[1], i[0]),2)
            new_x = round(i[0] - 1.0 * math.cos(angle_calc),2)
            new_y = round(i[1] - 1.0 * math.sin(angle_calc),2)
            self._explore_way.append((new_x, new_y, angle_calc))


        s = str([{'x': x, 'y': y, 'phi': phi} for x, y, phi in object_positions])
        self.get_logger().info(f'publishing isolated objects: {s}')
        self.poi_pub.publish(String(data=s))


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

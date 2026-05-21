from enum import Enum
import string
from typing import List, Optional, Tuple
import math
import time
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

# Nav2 Additions
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose 
from action_msgs.msg import GoalStatus # Add this import at the top
from nav_msgs.msg import OccupancyGrid
from vision_msgs.msg import Detection2DArray


import cv2
import numpy as np
import math
import json

HARDCODED_WAYPOINTS: List[Tuple[float, float, float]] = [
    (4.0, 0.0, 0.0), 
    (8.0, 0.0, 3.141), 
    (0.0, 0.0, 3.141),
    (-4.0, 0.0, 0.0), 
    (0.0, 0.0, 0.0)
]

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

        self._state = ControllerState.WAITING
        self._current_waypoint_index = 0

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
        self.phase = 'phase_1'

        #detect letter signal publisher
        self.start_letter_detection_pub = self.create_publisher(String, '/check_label', 10)
        self.letter_detection_sub = self.create_subscription(String, '/camera/letter_detection', self.letter_detection_callback, 10)
        self.last_detected_letter = None
        self.classifying_letter = False

        
        #object detection signal publisher
        self.start_object_detection_pub = self.create_publisher(String, '/check_object', 10)
        self.object_detection_sub = self.create_subscription(Detection2DArray, '/camera/detections', self.object_detection_callback, 10)

        #waypoint route driving
        self.start_waypoint_route_sub = self.create_subscription(String, '/waypoint_order', self._handle_start_waypoint_route, 10)


        # Nav2 Action Client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        #timer 
        self.create_timer(1.0, self._on_timer)

        self.get_logger().info(f'Loaded {len(self._waypoints)} Nav2 waypoints.')
        self._publish_state()
        self.get_logger().info(f'Global controller started in state: {self._state.value}')

    def object_detection_callback(self, msg: Detection2DArray) -> None:
        if len(msg.detections) > 0:
            self.get_logger().info(f'Object detection callback received {len(msg.detections)} detections.')
            self._publish_status_log(
                f"Classified object as {msg.detections[0].results[0].hypothesis.class_id}."
            )
            if msg.detections[0].results[0].hypothesis.class_id == 'red trashcan' or msg.detections[0].results[0].hypothesis.class_id == 'yellow trashcan':
                self.classifying_letter = True
                self.start_letter_detection_pub.publish(String(data="classify"))
                # When starting classification:
                self._letter_timeout_timer = self.create_timer(10.0, self._on_letter_timeout)

                self.get_logger().info('Trashcan detected: Starting letter classification')
                self._publish_status_log(f"Classified object as {msg.detections[0].results[0].hypothesis.class_id} Starting Letter Classification.")
            else:
                self.get_logger().info(f"Classified object as {msg.detections[0].results[0].hypothesis.class_id}. Moving to next explore point.")
                self._publish_status_log(f"Classified object as {msg.detections[0].results[0].hypothesis.class_id} Moving to next waypoint.")
                self._send_nav2_goal()
        
        else:
            self.get_logger().info('Object detection callback received no detections.')
            self._send_nav2_goal()

    def letter_detection_callback(self, msg: String) -> None:
        if not self.classifying_letter:
            return
        self.last_detected_letter = msg.data
        if msg.data in ("nothing detected", "confidence too low", "no frame available"):
            self.start_letter_detection_pub.publish(String(data="classify"))
            return
        else:
           
            self.classifying_letter = False
            #kill timeout timer
            if self._letter_timeout_timer:
                self._letter_timeout_timer.cancel()
                self._letter_timeout_timer = None

            x = self._waypoints[self._current_waypoint_index-1 ][0]
            y = self._waypoints[self._current_waypoint_index-1][1]
            phi =  self._waypoints[self._current_waypoint_index-1 ][2]
            CLASSIFIED_WAYPOINTS.append((x, y, phi, self.last_detected_letter))

            s = str([{'x': x, 'y': y, 'phi': phi, 'label': label} for x, y, phi, label in CLASSIFIED_WAYPOINTS])
            self.classified_poi_pub.publish(String(data=s))
            self._publish_status_log(
                "Classified " + self.last_detected_letter + " at:" + str(self._waypoints[self._current_waypoint_index-1])
            )
            self.get_logger().info('IMAGE CLASSIFICATION COMPLETE. Detected letter: ' + self.last_detected_letter)
            self.last_detected_letter = None # reset last detected letter before next classification

            if self._current_waypoint_index < (len(self._waypoints)):
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


    # Timeout handler for letter classification.
    def _on_letter_timeout(self):
        self.classifying_letter = False
        self._letter_timeout_timer.cancel()
        self._letter_timeout_timer = None
        self.get_logger().warn('Letter classification timed out, moving on.')
        self._send_nav2_goal()


    def _publish_waypoints(self) -> None: # publishes classified waypoints every second
        
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
                self.get_logger().info('[RESULT][PHASE_1] Entered phase_1 success branch.')
                self._current_waypoint_index += 1
                self.get_logger().info(
                    f'[RESULT][PHASE_1] Incremented waypoint index to {self._current_waypoint_index} (total={len(self._waypoints)}).'
                )
                if self._current_waypoint_index < len(HARDCODED_WAYPOINTS):
                    self.get_logger().info(
                        f'[RESULT][PHASE_1] More base waypoints remain. Sending next waypoint index {self._current_waypoint_index}.'
                    )
                    self._publish_status_log(
                            "Exploring enviroment and building map..."
                    )
                    self._send_nav2_goal()
                else:
                    self.get_logger().info('Mapping Initial Complete!')

                    if self._current_waypoint_index == len(HARDCODED_WAYPOINTS):
                        self._save_costmap_to_disk()
                        self.isolate_objects()

                        self._publish_status_log(
                            "Going to POI P" + str(self._current_waypoint_index - len(HARDCODED_WAYPOINTS))
                        )
                        self._send_nav2_goal()
                    elif self._current_waypoint_index > len(HARDCODED_WAYPOINTS) and (self._current_waypoint_index < len(self._waypoints)-1):

                        self._publish_status_log(
                        "Starting Classification at: P" + str(self._current_waypoint_index - len(HARDCODED_WAYPOINTS))
                        )
                        self.start_object_detection_pub.publish(String(data="classify"))
                    elif self._current_waypoint_index == len(self._waypoints)-1:
                        self._publish_status_log(
                            "Returning Home..."
                        )
                        self._send_nav2_goal()


            elif status == GoalStatus.STATUS_ABORTED: 
                self.get_logger().warn('NAV2 HISSY FIT TRANSFORM TREE/CPU/MAP/SOMETHING ELSE IDK WHY THEY PUT SO MANY THINGS IN STATUS 6')
                # DO NOT transition to WAITING. 
                # Use a timer to retry so we don't spam the server instantly
                self.retry_timer = self.create_timer(2.0, self._handle_oneshot_retry)                
            else:
                # For other failures (Canceled, etc.), now we can halt
                self.get_logger().info(
                    f'[RESULT] Non-success status={status}. Current phase={self.phase}. Waiting for classification or retry trigger.'
                )

    def _handle_start_waypoint_route(self, msg: String):
        pass

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
        self.get_logger().info(f'Received phase update: {self.phase}')


    def _save_costmap_to_disk(self) -> None:
        if self.costmap_data is None:
            self.get_logger().warn('No costmap data available to save.')
            return

        costmap = self.costmap_data
        save_dir = os.path.expanduser('~/costmap_exports')
        os.makedirs(save_dir, exist_ok=True)
        stamp = int(time.time())
        img_path  = os.path.join(save_dir, f'costmap_{stamp}.png')
        yaml_path = os.path.join(save_dir, f'costmap_{stamp}.yaml')

        # Build grayscale image in map_saver style:
        #   free (0) → 254 (white), occupied (100) → 0 (black), unknown (-1) → 205 (grey)
        grid = np.array(costmap.data, dtype=np.int8).reshape(
            costmap.info.height, costmap.info.width
        )
        img = np.full((costmap.info.height, costmap.info.width), 205, dtype=np.uint8)
        img[grid == 0]   = 254
        img[grid == 100] = 0
        # Flip vertically: ROS grid has y=0 at bottom, PNG has y=0 at top
        img = cv2.flip(img, 0)
        cv2.imwrite(img_path, img)

        ox  = costmap.info.origin.position.x
        oy  = costmap.info.origin.position.y
        res = costmap.info.resolution
        with open(yaml_path, 'w') as f:
            f.write(
                f"image: {img_path}\n"
                f"resolution: {res}\n"
                f"origin: [{ox}, {oy}, 0.0]\n"
                f"negate: 0\n"
                f"occupied_thresh: 0.65\n"
                f"free_thresh: 0.196\n"
            )

        self.get_logger().info(f'Costmap exported → {img_path}  |  {yaml_path}')

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



        _, binary_before = cv2.threshold(img, 250, 255, cv2.THRESH_BINARY)

         # Remove isolated single pixels before dilation
        num_labels_clean, labels_clean, stats_clean, _ = cv2.connectedComponentsWithStats(binary_before, connectivity=8)
        for i in range(1, num_labels_clean):
            if stats_clean[i, cv2.CC_STAT_AREA] == 1:
                binary_before[labels_clean == i] = 0

    
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_before, connectivity=8)

        object_positions = []
        min_area = 10
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

            
                if (cx_m < 10 and cx_m > -6) and (cy_m < 4 and cy_m > -8) and math.sqrt(cx_m**2 + cy_m**2) < 11.0:                    
                    if len(object_positions) == 0:
                        object_positions.append((cx_m, cy_m, 0.0))
                    else: 
                        if math.sqrt((object_positions[-1][0] - cx_m)**2  +  (object_positions[-1][1] - cy_m)**2) > 0.80:
                            object_positions.append((cx_m, cy_m, 0.0))

        for i in object_positions:
            angle_calc = round(math.atan2(i[1], i[0]),2)
            new_x = round(i[0] - 2.0 * math.cos(angle_calc),2)
            new_y = round(i[1] - 2.0 * math.sin(angle_calc),2)
            self._waypoints.append((new_x, new_y, angle_calc))
        self._waypoints.append((0.0, 0.0, 0.0)) # add home position as final waypoint after all objects isolated
        self._publish_status_log(
            "Objects Isolated Complete"
        )
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

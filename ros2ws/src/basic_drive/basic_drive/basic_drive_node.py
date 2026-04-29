# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, String
import math
from sensor_msgs.msg import LaserScan, Joy

lat_calc = 111320
long_calc = 111320 * math.cos(math.radians(-31.9805064))

def normalize_angle_degrees(angle_deg):
    while angle_deg <= -180.0:
        angle_deg += 360.0
    while angle_deg > 180.0:
        angle_deg -= 360.0
    return angle_deg

class Waypoint_drive(Node):

    def __init__(self):
        super().__init__('Waypoint_drive')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.slave_status_pub = self.create_publisher(String, '/slave/status', 10)
        self.slave_state_pub = self.create_publisher(String, '/slave/state', 10)

        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/gnss/fix',
            self.convert_utm,
            10
        )

        self.current_joy = None

        self.heading_sub = self.create_subscription(Float64, '/imu/comp', self.heading_callback, 10)
        self.goal_sub = self.create_subscription(String, '/gnss/goal', self.goal_callback, 10)
        self.master_status_sub = self.create_subscription(String, '/master/status', self.master_status_callback, 10)
        self.joy_sub = self.create_subscription(Joy, "/joy", self._joy_callback, 10)

        self.current_lat_utm = None
        self.current_lon_utm = None
        self.current_heading_deg = None

        self.goal_lat_utm = None
        self.goal_lon_utm = None
        self.master_is_driving = False

        self.waypoint_threshold = 5.0
        self.declare_parameter('state_publish_period_sec', 1.0)
        self.declare_parameter('state_log_period_sec', 2.0)
        self._state_publish_period = float(self.get_parameter('state_publish_period_sec').value)
        self._state_log_period = float(self.get_parameter('state_log_period_sec').value)
        self.create_timer(self._state_publish_period, self._publish_state_heartbeat)

        self._last_waiting_reason = None
        self._last_invalid_fix = None
        self._last_master_status = None
        self._goal_reached_reported = False
        self._robot_stopped = False
        self._drive_state = 'initializing'
        self._last_state_log_time = self.get_clock().now()
        self._last_turn_state = None
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        self.get_logger().info(
            '[INIT] node started: waiting for /master/status=driving to waypoint, '
            '/gnss/goal, valid GNSS fix and /imu/comp messages'
        )

    def _joy_callback(self, msg: Joy) -> None:
        self.current_joy = msg

    def _set_drive_state(self, state: str) -> None:
        self._drive_state = state

    def _publish_state_heartbeat(self) -> None:
        state_msg = String()
        state_msg.data = self._drive_state
        self.slave_state_pub.publish(state_msg)

        now = self.get_clock().now()
        elapsed_sec = (now - self._last_state_log_time).nanoseconds / 1e9
        if elapsed_sec >= self._state_log_period:
            self.get_logger().info(f'[STATE] slave_state={self._drive_state}')
            self._last_state_log_time = now

    #slop debugging: will remove once validated
    def convert_utm(self, msg):
        self.get_logger().debug(
            f'[/fix RX] lat={msg.latitude} lon={msg.longitude} status={msg.status.status}'
        )

        lat_ok = math.isfinite(msg.latitude)
        lon_ok = math.isfinite(msg.longitude)
        if not lat_ok or not lon_ok:
            invalid_reason = f'lat_finite={lat_ok} lon_finite={lon_ok}'
            if invalid_reason != self._last_invalid_fix:
                self.get_logger().warn(
                    f'[GPS] invalid fix ignored ({invalid_reason}); '
                    f'keeping last valid position and waiting for a real GNSS fix'
                )
                self._last_invalid_fix = invalid_reason
            return

        self._last_invalid_fix = None
        lat = msg.latitude * lat_calc
        lon = msg.longitude * long_calc
        self.current_lat_utm = lat
        self.current_lon_utm = lon
        self.get_logger().info(
            f'[GPS] valid fix accepted: lat_utm={lat:.2f} lon_utm={lon:.2f}'
        )

        self.drive()

    #slop debugging: will remove once validated
    def goal_callback(self, msg):
        raw_goal = msg.data.strip()

        parts = [part.strip() for part in raw_goal.split(',')]
        if len(parts) < 2:
            self.get_logger().warn(
                f'[GOAL] invalid goal format "{raw_goal}"; expected "lat,lon[,alt]"'
            )
            return

        try:
            latitude = float(parts[0])
            longitude = float(parts[1])
        except ValueError:
            self.get_logger().warn(
                f'[GOAL] invalid numeric goal "{raw_goal}"; expected "lat,lon[,alt]"'
            )
            return

        if not math.isfinite(latitude) or not math.isfinite(longitude):
            self.get_logger().warn(
                f'[GOAL] non-finite goal ignored: lat={latitude} lon={longitude}'
            )
            return

        self.goal_lat_utm = latitude * lat_calc
        self.goal_lon_utm = longitude * long_calc
        self._goal_reached_reported = False
        self.get_logger().info(f'[GOAL] accepted: lat={latitude}, lon={longitude}')
        self.drive()

    def master_status_callback(self, msg):
        status_text = msg.data.strip()
        status_normalized = status_text.lower()
        self.master_is_driving = (
            status_normalized == 'driving to waypoint' or status_normalized == 'driving'
        )

        if status_text != self._last_master_status:
            self.get_logger().info(f'[MASTER] status={status_text}')
            self._last_master_status = status_text

        if not self.master_is_driving:
            self.terminate()

        self.drive()
    def lidar_callback(self, msg):
        # Total number of laser points (e.g., 811 points for some SICK models)
        num_points = len(msg.ranges)
        
        # The middle index is ALWAYS 0 radians (straight ahead)
        # because SICK 270 scans from -135 to +135.
        mid_idx = num_points // 2
        
        # Define a window size (how many degrees to look at for a "zone")
        # To look at a 20-degree cone in front:
        # steps = degrees / (angle_increment_in_degrees)
        window = int(math.radians(10) / msg.angle_increment)

        # 1. FRONT ZONE (Straight ahead)
        front_indices = msg.ranges[mid_idx - window : mid_idx + window]
        dist_front = self.get_min_range(front_indices, msg.range_max)

        # 2. LEFT ZONE (+90 degrees)
        # 90 degrees is pi/2 radians. 
        left_mid_idx = mid_idx + int(math.radians(90) / msg.angle_increment)
        left_indices = msg.ranges[left_mid_idx - window : left_mid_idx + window]
        dist_left = self.get_min_range(left_indices, msg.range_max)

        # 3. RIGHT ZONE (-90 degrees)
        right_mid_idx = mid_idx + int(math.radians(-90) / msg.angle_increment)
        right_indices = msg.ranges[right_mid_idx - window : right_mid_idx + window]
        dist_right = self.get_min_range(right_indices, msg.range_max)

        self.get_logger().info(f"Front: {dist_front:.2f}m | Left: {dist_left:.2f}m | Right: {dist_right:.2f}m")

    def get_min_range(self, range_slice, max_val):
        # Filter out 0.0, inf, and nan values
        valid_values = [x for x in range_slice if x > 0.05 and math.isfinite(x)]
        if not valid_values:
            return max_val
        return min(valid_values)
    
    def heading_callback(self, msg):
        self.current_heading_deg = msg.data
        self.drive()
        
    def at_goal(self):
        lat_diff_m  = (self.goal_lat_utm - self.current_lat_utm)
        lon_diff_m  = (self.goal_lon_utm - self.current_lon_utm)
        dist = math.sqrt(lat_diff_m**2 + lon_diff_m**2)
        if dist < self.waypoint_threshold:
            return True
        else:
            return False

    def terminate(self):
        if self._robot_stopped:
            self._set_drive_state('stopped')
            return

        command = Twist()

        command.linear.x = 0.0
        command.linear.y = 0.0
        command.linear.z = 0.0

        command.angular.x = 0.0
        command.angular.y = 0.0
        command.angular.z = 0.0

        self.publisher_.publish(command)
        self.get_logger().info(f'Publishing Twist command: linear.x={command.linear.x}, angular.z={command.angular.z}')
        self._robot_stopped = True
        self._set_drive_state('stopped')

        return 

    def drive(self):
        missing_inputs = []
        if self.current_heading_deg is None:
            missing_inputs.append('heading(/imu/comp)')
        if self.current_lat_utm is None or self.current_lon_utm is None:
            missing_inputs.append('valid_gnss(/gnss/fix)')
        if self.goal_lat_utm is None or self.goal_lon_utm is None:
            missing_inputs.append('goal_waypoint')
        if not self.master_is_driving:
            missing_inputs.append('master_state(driving to waypoint)')

        if missing_inputs:
            waiting_reason = ', '.join(missing_inputs)
            if waiting_reason != self._last_waiting_reason:
                self.get_logger().warn(f'[DRIVE] waiting for: {waiting_reason}')
                self._last_waiting_reason = waiting_reason
            self._set_drive_state('waiting')
            if not self.master_is_driving:
                self.terminate()
            return

        self._last_waiting_reason = None
        self._robot_stopped = False
        
        if self.at_goal():
            self.get_logger().info('[DRIVE] goal reached')
            self._set_drive_state('goal_reached')
            self.terminate()
            if not self._goal_reached_reported:
                done_msg = String()
                done_msg.data = 'finished'
                self.slave_status_pub.publish(done_msg)
                self.get_logger().info('[SLAVE] published /slave/status=finished')
                self._goal_reached_reported = True
            return

        else:
            linear_vel = 1.0
            angular_vel = 0.0

            current_lat_utm = self.current_lat_utm
            current_lon_utm = self.current_lon_utm

            lat_diff_utm  = self.goal_lat_utm - current_lat_utm
            long_diff_utm = self.goal_lon_utm - current_lon_utm

            dist = math.sqrt(lat_diff_utm**2 + long_diff_utm**2)

            topic_heading_deg = self.current_heading_deg
            current_yaw_deg = normalize_angle_degrees(topic_heading_deg + 90.0)

            #calculate angle from current position
            angle_to_goal_rad = math.atan2(lat_diff_utm, long_diff_utm)
            angle_to_goal_deg = normalize_angle_degrees(angle_to_goal_rad * 180.0 / math.pi)

            #calculate angle error
            angle_error_deg = normalize_angle_degrees(angle_to_goal_deg - current_yaw_deg)

            self.get_logger().info(
                f'[DRIVE] dist={dist:.2f}m | '
                f'imu_comp={topic_heading_deg:.1f}deg | '
                f'yaw={current_yaw_deg:.1f}deg | '
                f'goal_bearing={angle_to_goal_deg:.1f}deg | '
                f'angle_error={angle_error_deg:.1f}deg'
            )

            if (abs(angle_error_deg) > 10.0):
                if angle_error_deg < 0.0:
                    linear_vel  = 0.0
                    angular_vel = -0.2
                    self._set_drive_state('turning_right')
                    if self._last_turn_state != 'turning_right':
                        self.get_logger().info('[DRIVE] state=TURNING_RIGHT')
                        self._last_turn_state = 'turning_right'
                else:
                    linear_vel  = 0.0
                    angular_vel = 0.2
                    self._set_drive_state('turning_left')
                    if self._last_turn_state != 'turning_left':
                        self.get_logger().info('[DRIVE] state=TURNING_LEFT')
                        self._last_turn_state = 'turning_left'
            else:
                linear_vel  = 1.0
                angular_vel = max(-1.0, min(1.0, 0.07*angle_error_deg))
                self._set_drive_state('driving_forward')
                if self._last_turn_state != 'driving_forward':
                    self.get_logger().info('[DRIVE] state=DRIVING_FORWARD')
                    self._last_turn_state = 'driving_forward'

            if self.current_joy.buttons[9] == 1 and self.current_joy.buttons[10] == 1:
                command = Twist()
                command.linear.x = linear_vel
                command.linear.y = 0.0
                command.linear.z = 0.0
                command.angular.x = 0.0
                command.angular.y = 0.0
                command.angular.z = angular_vel

                self.publisher_.publish(command)
                self._robot_stopped = False
                self.get_logger().info(f'[/cmd_vel TX] linear.x={command.linear.x:.2f} angular.z={command.angular.z:.2f}')
            else:
                command = Twist()
                command.linear.x = 0.0
                command.linear.y = 0.0
                command.linear.z = 0.0
                command.angular.x = 0.0
                command.angular.y = 0.0
                command.angular.z = 0.0

                self.publisher_.publish(command)
                self._robot_stopped = False
                self.get_logger().info(f'[/cmd_vel TX] linear.x={command.linear.x:.2f} angular.z={command.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    waypoint_drive = Waypoint_drive()
    rclpy.spin(waypoint_drive)
    waypoint_drive.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
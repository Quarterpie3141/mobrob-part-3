#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Bool, String
from flask import Flask, render_template
from nav_msgs.msg import OccupancyGrid
import numpy as np
from flask_socketio import SocketIO
import threading
import os
import math
import json
import base64
from cv_bridge import CvBridge
import cv2


from ament_index_python.packages import get_package_share_directory

pkg_share = get_package_share_directory('gui')

app = Flask(__name__,
            template_folder=os.path.join(pkg_share, 'templates'),
            static_folder=os.path.join(pkg_share, 'static'))
app.config['SECRET_KEY'] = 'tuna'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

def euler_from_quaternion(q):
    x, y, z, w = q
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class WebGuiNode(Node):
    def __init__(self):
        super().__init__('web_gui_node')
        self.bridge = CvBridge()

        # Publishers
        self.pause_pub = self.create_publisher(Bool, '/pause', 10)
        self.phase_pub = self.create_publisher(String, '/phase', 10)
        self.waypoint_order_pub = self.create_publisher(String, '/waypoint_order', 10)

        # gotta publish something to start the whole thing mabye start paused and un pause

        # Subscribers
        # goota subscribe to map and pose to show the robot on the map
        self.baselink_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/pose', self.baselink_callback, 10)
        
        self.poi_sub = self.create_subscription(
            String, '/poi', self.poi_callback, 10
        )

        self.classified_poi_sub = self.create_subscription(
            String, '/classified_poi', self.classified_poi_callback, 10
        )

        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.costmap_callback,
            10
        )

        self.nav2_goal_sub = self.create_subscription(
            String,
            '/gui/nav2_goal',
            self.nav2_goal_callback,
            10
        )

        self.status_log_sub = self.create_subscription(
            String,
            '/status_log',
            self.status_log_callback,
            10
        )

        #these are the images for letter and object det
        self.letter_detection_image = self.create_subscription(Image, 'classified_poi/image', self.letter_detection_image_callback, 10)
        
        self.detections_sub = self.create_subscription(
            Detection2DArray, '/camera/detections',
            self.detections_callback, 10)

        self.object_detection_image_sub = self.create_subscription(
            Image, '/camera/detections/image',
            self.object_detection_image_callback, 10)

        

        # State
        self.current_phase = 1
        self.is_paused = False
        self.waypoint_sequence = []
        self.costmap_counter = 0
        self.last_costmap_meta = None
        self.poi_images = {}  # label -> base64 jpeg string
        self.last_classified_labels = []  # track what we've seen
        self.object_pois = {}        # label -> {x, y, phi, label}
        self.object_images = {}      # label -> b64 jpeg
        self.last_object_labels = []
        self.current_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}

        # All available Greek-letter waypoints
        self.waypoints = [
            'alpha', 'beta', 'delta', 'eta', 'gamma',
            'lambda', 'mu', 'phi', 'rho', 'tau'
        ]

        self.get_logger().info('Web GUI Node started')
        self.log_to_web('ROS 2 Web GUI Node initialized', 'info')

    def letter_detection_image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ok, buf = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                return
            b64 = base64.b64encode(buf.tobytes()).decode('ascii')

            # associate with most recent image and labels
            if self.last_classified_labels:
                label = self.last_classified_labels[-1]
                self.poi_images[label] = b64
                socketio.emit('poi_image', {'label': label, 'image': b64})
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')

    def detections_callback(self, msg):
        if not msg.detections:
            return

        new_labels = []
        for det in msg.detections:
            if not det.results:
                continue
            label = det.results[0].hypothesis.class_id
            new_labels.append(label)

            # Only add if we haven't seen this label yet
            # (otherwise replace with latest pose - your call)
            if label not in self.object_pois:
                self.object_pois[label] = {
                    'x': self.current_pose['x'],
                    'y': self.current_pose['y'],
                    'phi': self.current_pose['theta'],
                    'label': label,
                }

        self.last_object_labels = new_labels

        socketio.emit('object_poi_update', {
            'object_poi': json.dumps(list(self.object_pois.values()))
        })
    
    def object_detection_image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ok, buf = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                return
            b64 = base64.b64encode(buf.tobytes()).decode('ascii')

            if self.last_object_labels:
                label = self.last_object_labels[-1]
                self.object_images[label] = b64
                socketio.emit('object_image', {'label': label, 'image': b64})
        except Exception as e:
            self.get_logger().error(f'Object image conversion failed: {e}')
    
    def baselink_callback(self, msg):

        orientation_list = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.current_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'theta': yaw,
        }

        socketio.emit('robot_pose', {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'theta': yaw
        })

    def costmap_callback(self, msg):
      # Throttling 
      self.costmap_counter += 1
      if self.costmap_counter % 3 != 0:
          return

      width = msg.info.width
      height = msg.info.height
      resolution = msg.info.resolution
      origin_x = msg.info.origin.position.x
      origin_y = msg.info.origin.position.y

      #  data is int8[], values: -1=unknown, 0=free, 100=occupied
      data = np.array(msg.data, dtype=np.int8).tobytes()

      socketio.emit('costmap', {
          'width': width,
          'height': height,
          'resolution': resolution,
          'origin_x': origin_x,
          'origin_y': origin_y,
          'data': data, 
      })

    def poi_callback(self, msg):
        socketio.emit('poi_update', {'poi': msg.data})

    def classified_poi_callback(self, msg):
        socketio.emit('classified_poi_update', {'classified_poi': msg.data})
        # Track labels so we can pair the next incoming image
        try:
            fixed = msg.data.replace("'", '"')
            pois = json.loads(fixed)
            self.last_classified_labels = [p['label'] for p in pois]
        except Exception as e:
            self.get_logger().error(f'Failed to parse classified POIs: {e}')

    def object_detection_image_callback(self, msg):
        # Optional: stream live detection feed separately
        pass

    def nav2_goal_callback(self, msg):
        # msg.data is a string of the format "x,y,phi"
        try:
            x_str, y_str, phi_str = msg.data.split(',')
            x = float(x_str)
            y = float(y_str)
            phi = float(phi_str)
            socketio.emit('nav2_goal', {'x': x, 'y': y, 'phi': phi})
        except Exception as e:
            self.get_logger().error(f'Failed to parse Nav2 goal: {e}')

    def status_log_callback(self, msg):
        # Supports plain text or JSON: {"message": "...", "level": "info|success|warning|error"}
        message = msg.data
        level = 'info'

        try:
            payload = json.loads(msg.data)
            if isinstance(payload, dict):
                message = str(payload.get('message', message))
                level = str(payload.get('level', level))
        except json.JSONDecodeError:
            pass

        self.log_to_web(message, level)

    def set_phase(self, phase):
        if phase not in (1, 2):
            self.log_to_web(f'Invalid phase: {phase}', 'error')
            return
        self.current_phase = phase
        msg = String()
        msg.data = f'phase_{phase}'
        self.phase_pub.publish(msg)
        self.log_to_web(f'Switched to Phase {phase}', 'info')

    def toggle_pause(self):
        self.is_paused = not self.is_paused
        msg = Bool()
        msg.data = self.is_paused
        self.pause_pub.publish(msg)
        state = 'PAUSED' if self.is_paused else 'RESUMED'
        level = 'warning' if self.is_paused else 'success'
        self.log_to_web(f'Robot {state}', level)
        return self.is_paused

    def send_waypoint_sequence(self, sequence, coordinates=None):
        invalid = [w for w in sequence if w not in self.waypoints]
        if invalid:
            self.log_to_web(f'Invalid waypoints: {invalid}', 'error')
            return False
        if not sequence:
            self.log_to_web('Waypoint sequence is empty', 'warning')
            return False
        if not coordinates or len(coordinates) != len(sequence):
            self.log_to_web('Missing or mismatched coordinates for waypoints', 'error')
            return False

        self.waypoint_sequence = sequence
        msg = String()
        # Format: '[[1.0, 2.0, 0.5], [3.0, 4.0, 1.2]]'
        msg.data = json.dumps(coordinates)
        self.waypoint_order_pub.publish(msg)
        self.log_to_web(
            f'Sent waypoint sequence ({len(sequence)}): {" → ".join(sequence)}',
            'success')
        self.get_logger().info(f'Published coordinates: {msg.data}')
        return True

    def clear_waypoints(self):
        self.waypoint_sequence = []
        msg = String()
        msg.data = json.dumps([])
        self.waypoint_order_pub.publish(msg)
        self.log_to_web('Waypoint sequence cleared', 'info')

    def log_to_web(self, message, level='info'):
        socketio.emit('status_log', {'message': message, 'level': level})
        self.get_logger().info(f'[{level}] {message}')


ros_node = None


@app.route('/')
def index():
    return render_template('index.html')


# ---------- SocketIO Events ----------
@socketio.on('connect')
def handle_connect():
    if ros_node:
        socketio.emit('state_sync', {
            'phase': ros_node.current_phase,
            'paused': ros_node.is_paused,
            'waypoints': ros_node.waypoints,
            'sequence': ros_node.waypoint_sequence,
        })
        for label, b64 in ros_node.poi_images.items():
            socketio.emit('poi_image', {'label': label, 'image': b64})
        for label, b64 in ros_node.object_images.items():
            socketio.emit('object_image', {'label': label, 'image': b64})
        if ros_node.object_pois:
            socketio.emit('object_poi_update', {
                'object_poi': json.dumps(list(ros_node.object_pois.values()))
            })
        ros_node.log_to_web('Web client connected', 'info')


@socketio.on('set_phase')
def handle_set_phase(data):
    if ros_node:
        ros_node.set_phase(int(data.get('phase', 1)))


@socketio.on('toggle_pause')
def handle_toggle_pause():
    if ros_node:
        paused = ros_node.toggle_pause()
        socketio.emit('pause_state', {'paused': paused})


@socketio.on('send_waypoints')
def handle_send_waypoints(data):
    if ros_node:
        sequence = data.get('sequence', [])
        coordinates = data.get('coordinates', [])
        ros_node.send_waypoint_sequence(sequence, coordinates)


@socketio.on('clear_waypoints')
def handle_clear_waypoints():
    if ros_node:
        ros_node.clear_waypoints()


def ros_spin(node):
    rclpy.spin(node)


def main():
    global ros_node
    rclpy.init()
    ros_node = WebGuiNode()

    ros_thread = threading.Thread(target=ros_spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    try:
        socketio.run(app, host='0.0.0.0', port=6767, debug=False,
                     allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from flask import Flask, render_template
from flask_socketio import SocketIO
import threading
import os
import json

from ament_index_python.packages import get_package_share_directory

pkg_share = get_package_share_directory('gui')

app = Flask(__name__,
            template_folder=os.path.join(pkg_share, 'templates'),
            static_folder=os.path.join(pkg_share, 'static'))
app.config['SECRET_KEY'] = 'ros2webgui'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')


class WebGuiNode(Node):
    def __init__(self):
        super().__init__('web_gui_node')

        # Publishers
        self.pause_pub = self.create_publisher(Bool, '/pause', 10)
        self.phase_pub = self.create_publisher(String, '/phase', 10)
        self.waypoint_order_pub = self.create_publisher(
            String, '/waypoint_order', 10)
        self.status_pub = self.create_publisher(String, '/web_gui_status', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # State
        self.current_phase = 1
        self.is_paused = False
        self.waypoint_sequence = []

        # All available Greek-letter waypoints
        self.waypoints = [
            'alpha', 'beta', 'delta', 'eta', 'gamma',
            'lambda', 'mu', 'phi', 'rho', 'tau'
        ]

        self.get_logger().info('Web GUI Node started')
        self.log_to_web('ROS 2 Web GUI Node initialized', 'info')

    # ---------- ROS Callbacks ----------
    def odom_callback(self, msg):
        socketio.emit('robot_pose', {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'theta': 0.0
        })

    # ---------- Phase / Pause Functions ----------
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

    # ---------- Waypoint Functions ----------
    def send_waypoint_sequence(self, sequence):
        # Validate
        invalid = [w for w in sequence if w not in self.waypoints]
        if invalid:
            self.log_to_web(f'Invalid waypoints: {invalid}', 'error')
            return False
        if not sequence:
            self.log_to_web('Waypoint sequence is empty', 'warning')
            return False

        self.waypoint_sequence = sequence
        msg = String()
        msg.data = json.dumps(sequence)
        self.waypoint_order_pub.publish(msg)
        self.log_to_web(
            f'Sent waypoint sequence ({len(sequence)}): {" → ".join(sequence)}',
            'success')
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


# ---------- Flask Routes ----------
@app.route('/')
def index():
    return render_template('index.html')


# ---------- SocketIO Events ----------
@socketio.on('connect')
def handle_connect():
    if ros_node:
        # Send the current state to the new client
        socketio.emit('state_sync', {
            'phase': ros_node.current_phase,
            'paused': ros_node.is_paused,
            'waypoints': ros_node.waypoints,
            'sequence': ros_node.waypoint_sequence,
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
        ros_node.send_waypoint_sequence(sequence)


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
        socketio.run(app, host='0.0.0.0', port=5000, debug=False,
                     allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
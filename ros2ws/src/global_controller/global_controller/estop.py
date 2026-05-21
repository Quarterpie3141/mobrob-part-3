"""
E-Stop Node
============
Monitors /scan for nearby obstacles. After N consecutive detections,
activates an e-stop and saves the last 5 seconds of buffered topic
data to a rosbag for post-incident review.

Deadman switch: expects a Bool(True) heartbeat on /deadman at a
regular rate. If no heartbeat is received within the timeout,
the e-stop is activated automatically.

Clearing: publish "clear estop" on /master/status
"""

import os
import math
import threading
from collections import deque
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

import rosbag2_py
from rosidl_runtime_py.utilities import get_message


class EstopNode(Node):
    def __init__(self):
        super().__init__('estop_node')

        # ── Parameters ──────────────────────────────────────────
        self.declare_parameter('distance_threshold', 1.0)
        self.declare_parameter('required_hits', 5)
        self.declare_parameter('buffer_duration', 5.0)
        self.declare_parameter('output_dir', '/opt/ros2ws/img')
        self.declare_parameter('topics', [
            '/scan',
            '/cmd_vel',
            '/odom',
            '/imu/data',
            '/tf',
            '/tf_static',
        ])

        # Deadman switch parameters
        self.declare_parameter('deadman_enabled', True)
        self.declare_parameter('deadman_topic', '/deadman')
        self.declare_parameter('deadman_timeout', 1.0)  # seconds without heartbeat

        self.distance_threshold = self.get_parameter('distance_threshold').value
        self.required_hits = self.get_parameter('required_hits').value
        self.buffer_duration = self.get_parameter('buffer_duration').value
        self.output_dir = self.get_parameter('output_dir').value
        self.topics_to_record = self.get_parameter('topics').value

        self.deadman_enabled = self.get_parameter('deadman_enabled').value
        self.deadman_topic = self.get_parameter('deadman_topic').value
        self.deadman_timeout = self.get_parameter('deadman_timeout').value

        os.makedirs(self.output_dir, exist_ok=True)

        # ── State ───────────────────────────────────────────────
        self.consecutive_hits = 0
        self.estop_active = False
        self.bag_recorded = False

        # E-stop source tracking for logging
        self._estop_source = ''

        # Deadman state
        self._last_deadman_time = self.get_clock().now()
        self._deadman_received = False  # no heartbeat yet at startup
        self._deadman_tripped = False

        # ── Circular buffer for snapshot ────────────────────────
        self._buffer = deque()
        self._buffer_lock = threading.Lock()

        # ── E-stop publisher + heartbeat ────────────────────────
        self.estop_pub = self.create_publisher(Bool, '/estop', 10)
        self.create_timer(0.1, self._publish_estop_state)

        # ── Scan subscriber (for obstacle detection) ────────────
        self.create_subscription(LaserScan, '/scan', self._scan_callback, 10)

        # ── Status subscriber (for clearing) ────────────────────
        self.create_subscription(String, '/master/status', self._status_callback, 10)

        # ── Deadman switch subscriber + watchdog ────────────────
        if self.deadman_enabled:
            self.create_subscription(
                Bool, self.deadman_topic, self._deadman_callback, 10
            )
            self.create_timer(0.1, self._deadman_watchdog)
            self.get_logger().info(
                f'Deadman switch enabled on {self.deadman_topic} '
                f'(timeout={self.deadman_timeout}s)'
            )

        # ── Dynamic topic subscriptions for buffering ───────────
        self._subs = []
        self._topic_type_map = {}
        self._pending_topics = list(self.topics_to_record)
        self.create_timer(1.0, self._resolve_topics)

        # ── Periodic buffer pruning ─────────────────────────────
        self.create_timer(0.5, self._prune_buffer)

        self.get_logger().info(
            f'E-Stop node started | threshold={self.distance_threshold}m '
            f'hits={self.required_hits} buffer={self.buffer_duration}s'
        )

    # ── E-stop heartbeat ────────────────────────────────────────
    def _publish_estop_state(self):
        msg = Bool()
        msg.data = self.estop_active
        self.estop_pub.publish(msg)

    # ── Activate e-stop (shared logic) ──────────────────────────
    def _activate_estop(self, source: str):
        """Activate e-stop from any trigger source."""
        if self.estop_active:
            return

        self.estop_active = True
        self._estop_source = source
        self.get_logger().warn(f'E-STOP ACTIVATED [{source}]')

        if not self.bag_recorded:
            self.bag_recorded = True
            threading.Thread(
                target=self._write_snapshot, daemon=True
            ).start()

    # ── Scan processing ─────────────────────────────────────────
    def _scan_callback(self, msg: LaserScan):
        obstacle_detected = any(
            r < self.distance_threshold
            for r in msg.ranges
            if math.isfinite(r)
        )

        if obstacle_detected:
            self.consecutive_hits += 1
            self.get_logger().info(
                f'Obstacle detected ({self.consecutive_hits}/{self.required_hits})'
            )
        else:
            self.consecutive_hits = 0

        if self.consecutive_hits >= self.required_hits:
            self._activate_estop('obstacle')

    # ── Deadman switch ──────────────────────────────────────────
    def _deadman_callback(self, msg: Bool):
        if msg.data:
            self._last_deadman_time = self.get_clock().now()
            self._deadman_received = True

            # If deadman was the source, allow it to auto-clear
            # when operator grabs the switch again
            if self._deadman_tripped and not self.estop_active:
                self._deadman_tripped = False

    def _deadman_watchdog(self):
        """Check if deadman heartbeat has timed out."""
        if not self._deadman_received:
            # Haven't received first heartbeat yet — don't trip at startup
            return

        elapsed = (
            self.get_clock().now() - self._last_deadman_time
        ).nanoseconds / 1e9

        if elapsed > self.deadman_timeout and not self._deadman_tripped:
            self._deadman_tripped = True
            self._activate_estop('deadman')

    # ── Clear e-stop ────────────────────────────────────────────
    def _status_callback(self, msg: String):
        if msg.data.lower().strip() == 'clear estop':
            self.estop_active = False
            self.bag_recorded = False
            self.consecutive_hits = 0
            self._deadman_tripped = False
            self._estop_source = ''

            # Reset deadman timer so it doesn't immediately re-trip
            self._last_deadman_time = self.get_clock().now()

            self.get_logger().info('E-STOP CLEARED')

    # ── Topic discovery & buffering ─────────────────────────────
    def _resolve_topics(self):
        if not self._pending_topics:
            return

        known = dict(self.get_topic_names_and_types())
        still_pending = []

        for topic in self._pending_topics:
            if topic in known and known[topic]:
                type_str = known[topic][0]
                self._subscribe_topic(topic, type_str)
            else:
                still_pending.append(topic)

        self._pending_topics = still_pending

    def _subscribe_topic(self, topic: str, type_str: str):
        try:
            msg_type = get_message(type_str)
        except Exception as e:
            self.get_logger().warn(f'Cannot load {type_str} for {topic}: {e}')
            return

        self._topic_type_map[topic] = type_str

        qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        def cb(msg, _t=topic, _ts=type_str):
            stamp_ns = self.get_clock().now().nanoseconds
            data = serialize_message(msg)
            with self._buffer_lock:
                self._buffer.append((_t, data, _ts, stamp_ns))

        sub = self.create_subscription(msg_type, topic, cb, qos)
        self._subs.append(sub)
        self.get_logger().info(f'Buffering {topic} [{type_str}]')

    def _prune_buffer(self):
        cutoff = self.get_clock().now().nanoseconds - int(self.buffer_duration * 1e9)
        with self._buffer_lock:
            while self._buffer and self._buffer[0][3] < cutoff:
                self._buffer.popleft()

    # ── Write snapshot bag (runs in background thread) ──────────
    def _write_snapshot(self):
        with self._buffer_lock:
            snapshot = list(self._buffer)

        if not snapshot:
            self.get_logger().warn('Buffer empty — no snapshot written')
            return

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        bag_dir = os.path.join(self.output_dir, f'estop_bag_{timestamp}')

        try:
            writer = rosbag2_py.SequentialWriter()
            writer.open(
                rosbag2_py.StorageOptions(uri=bag_dir, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions(
                    input_serialization_format='cdr',
                    output_serialization_format='cdr',
                ),
            )

            registered = set()
            for topic, data, type_str, stamp_ns in snapshot:
                if topic not in registered:
                    writer.create_topic(rosbag2_py.TopicMetadata(
                        name=topic,
                        type=type_str,
                        serialization_format='cdr',
                    ))
                    registered.add(topic)

            for topic, data, type_str, stamp_ns in snapshot:
                writer.write(topic, data, stamp_ns)

            del writer

            self.get_logger().info(
                f'Snapshot saved: {bag_dir} '
                f'({len(snapshot)} msgs, {len(registered)} topics)'
            )

        except Exception as e:
            self.get_logger().error(f'Failed to write snapshot: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = EstopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
from std_msgs.msg import Empty, String, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import os
import math
import time
import rclpy
from sensor_msgs.msg import Joy, NavSatFix
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import depthai as dai
import cv2
import ultralytics


class DepthAICameraNode(Node):
    def __init__(self):
        super().__init__('depthai_camera_node')

# ...existing code...
        self.publisher_ = self.create_publisher(Image, 'camera/raw_image', 10)
        self.detection_pub = self.create_publisher(Detection2DArray, "camera/detections", 10)
        self.image_pub = self.create_publisher(Image, "camera/detections/image", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.slave_status_pub = self.create_publisher(String, '/slave/status', 10)
        self.master_status_pub = self.create_publisher(String, '/master/status', 10)
        self.scan_complete_pub = self.create_publisher(Bool, '/scan_complete', 10)

        self.bridge = CvBridge()
        self.trigger_sub = self.create_subscription(Empty, 'camera/take_photo', self.trigger_callback, 10)
        self.master_status_sub = self.create_subscription(String, '/master/status', self.master_status_callback, 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/Odom', self.odom_callback, 10)

        self.master_status = None
        self.last_master_status = None

        self.latest_lat = None
        self.latest_lon = None
        self.imgdir = os.path.join(os.getcwd(), "img/")
        self.modeldir = os.path.join(os.getcwd(),"src","cv", "models")

        #START TESTING STUFF
        #THESE WILL NEED TWEEKING
        self.target_locked_frames = 0
        self.required_locked_frames = 5

        self.center_threshold_px = 60
        self.min_box_area = 25000

        self.picture_taken = False

        self.searching = False
        self.search_start_yaw = 0.0
        self.accumulated_rotation = 0.0
        self.previous_yaw = 0.0
        # yaw/odom state defaults (prevent AttributeError before odom msgs arrive)
        self.prev_yaw = 0.0
        self.total_yaw = 0.0
        self.yaw_initialised = False   # used in odom_callback (British spelling)
        self.yaw_initialized = False   # used elsewhere (American spelling)
        # robot pose / velocity defaults
        self.robot_yaw = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        self.robot_moving = False
        self.full_rotation_threshold = 2.0 * math.pi
# ...existing code...

        #END TESTING STUFF

        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/gnss/fix',
            self.gps_callback,
            10
        )



        # DepthAI v3 Pipeline Setup
        self.pipeline = dai.Pipeline()
        self.cam_rgb = self.pipeline.create(dai.node.Camera).build()

        # Request interleaved BGR for native OpenCV/cv_bridge compatibility
        self.preview = self.cam_rgb.requestOutput(
            (640, 480),
            type=dai.ImgFrame.Type.BGR888i,
            fps=10
        )

        # maxSize=1 and blocking=False ensures we only grab the latest frame
        self.q_rgb = self.preview.createOutputQueue(maxSize=1, blocking=False)

        stream_high_res = self.cam_rgb.requestFullResolutionOutput(useHighestResolution=True)
        self.script = self.pipeline.create(dai.node.Script)
        stream_high_res.link(self.script.inputs["in"])

        self.script.setScript("""
            while True:
                frame = node.inputs["in"].get()
                trigger = node.inputs["trigger"].tryGet()
                if trigger is not None:
                    node.io["highest_res"].send(frame)
        """)

        self.q_high_res = self.script.outputs["highest_res"].createOutputQueue()
        self.q_trigger_in = self.script.inputs["trigger"].createInputQueue()

        self.pipeline.start()
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info("Camera Node Started. Publish to /camera/take_photo to capture.")

        weights = os.path.join(self.modeldir, "part3v1.pt")
        self.model = ultralytics.YOLO(weights)
        self.label_map = self.model.names

    def gps_callback(self, msg):
        self.latest_lat = msg.latitude
        self.latest_lon = msg.longitude

    def joy_callback(self, msg: Joy):
        pass

    def master_status_callback(self, msg: String):
        self.last_master_status = getattr(self, 'master_status', None)
        self.master_status = msg.data

    def odom_callback(self, msg):

        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y


        q = msg.pose.pose.orientation # conversion quartonioms
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.omega = msg.twist.twist.angular.z

        self.robot_moving = abs(self.vx) > 0.01 or abs(self.omega) > 0.01

        if self.searching:

            if not self.yaw_initialised:
                self.prev_yaw = self.robot_yaw
                self.yaw_initialised = True

            delta = self.yaw_diff(self.robot_yaw, self.prev_yaw)

            if abs(delta) < 0.01:
                delta = 0.0

            self.total_yaw += abs(delta)
            self.prev_yaw = self.robot_yaw


    def yaw_diff(self, current, previous):
        diff = current - previous

        while diff > math.pi:
            diff -= 2.0 * math.pi
        while diff < -math.pi:
            diff += 2.0 * math.pi

        return diff

    def timer_callback(self):
        """
            Timer callback for autonomous object image capture.

            When the robot enters the 'taking picture' state, this callback:
                1. Continuously acquires RGB frames from the camera.
                2. Runs object detection inference on each frame.
                3. Filters detections using:
                    - minimum confidence threshold
                    - minimum bounding box size
                4. Selects the most relevant object candidate.
                5. Rotates the robot in place using Twist commands until the
                   object is centered in the image frame.
                6. Applies temporal stability checks across multiple frames to
                   avoid transient detections.
                7. Stops robot motion once a stable centered target is achieved.
                8. Captures and saves an image of the detected object.
                9. Publishes a completion flag to notify the global
                   controller that image acquisition is complete.

            This callback also publishes annotated detection images

            Assumptions:
                - The mission/controller node handles waypoint generation and navigation.
                - This node is responsible only for local visual alignment and image capture.
                - The robot is capable of differential-drive turning in place.

        """


        #protect against this locking out/fighting global controller if needed
        if self.master_status != 'taking picture':
            return
        self.get_logger().info(f"Status OK. total_yaw={self.total_yaw}, searching={self.searching}")

        #stop searching after 360 spin
        if self.total_yaw >= self.full_rotation_threshold:
            self.get_logger().warn("360 scan complete, no valid object found")

            stop = Twist()
            self.cmd_vel_pub.publish(stop)

            self.searching = False
            self.yaw_initialized = False
            self.total_yaw = 0.0

            fail_msg = Bool()
            fail_msg.data = False
            self.scan_complete_pub.publish(fail_msg)

            self.master_status = "idle" #publish
            #status = String[idle] make a string message and publish the status in that message and publish
            #comms to global contr.

            return

        in_rgb = self.q_rgb.tryGet()

        if in_rgb is None:
            self.get_logger().warn("No frame from camera")
            return

        if not self.searching:
            self.searching = True
            self.total_yaw = 0.0
            self.yaw_initialised = False

        frame = in_rgb.getCvFrame()
        frame_h, frame_w = frame.shape[:2]
        frame_center_x = frame_w / 2.0
        best_detection = None
        best_area = 0

        results = self.model.predict(frame, conf=0.6, verbose=False)

        for result in results:
            if result.boxes is None:
                continue

            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

                box_width = x2 - x1
                box_height = y2 - y1
                area = box_width * box_height

                if area < self.min_box_area:
                    continue

                center_x = (x1 + x2) / 2.0

                if area > best_area:
                    best_area = area

                    #get class index and convert to label
                    cls = int(box.cls[0].cpu().numpy())
                    classification = self.label_map.get(cls, str(cls))

                    #store class lookup
                    best_detection = {
                        'classification': classification,
                        'x1': x1,
                        'y1': y1,
                        'x2': x2,
                        'y2': y2,
                        'center_x': center_x,
                        'area': area
                    }

        cmd = Twist()

        if best_detection is None:

            self.target_locked_frames = 0

            cmd.angular.z = 0.25
            self.cmd_vel_pub.publish(cmd)

            return



        #target found, try to centre - this may be optimistic, may need to add scoring metric as a function of area and distance too camera
        target_error = best_detection['center_x'] - frame_center_x
        kP = 0.0025
        cmd.angular.z = -kP * target_error
        cmd.angular.z = max(min(cmd.angular.z, 0.3), -0.3)

        self.cmd_vel_pub.publish(cmd)

        centered = abs(target_error) < self.center_threshold_px

        #stay centered for a few frames to combat false postives
        if centered:
            self.target_locked_frames += 1
        else:
            self.target_locked_frames = 0

        x1 = best_detection['x1']
        y1 = best_detection['y1']
        x2 = best_detection['x2']
        y2 = best_detection['y2']

        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        #draw bounding box
        cv2.line(
            frame,
            (int(frame_center_x), 0),
            (int(frame_center_x), frame_h),
            (255, 0, 0),
            2
        )

        #label object
        detected_class = best_detection['classification']
        cv2.putText(
            frame,
            f"Locked: {detected_class}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2,
        )

        if self.target_locked_frames >= self.required_locked_frames:

            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)

            timestamp = int(time.time())

            filename = f"{self.imgdir}/{detected_class}_{timestamp}.jpg"
            cv2.imwrite(filename, frame)
            self.get_logger().info(f"Saved image: {filename}")

            #inform controller
            done_msg = Bool()
            done_msg.data = True

            self.scan_complete_pub.publish(done_msg)

            self.target_locked_frames = 0
            self.master_status = 'idle' # publish

        detection_img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        detection_img_msg.header.stamp = self.get_clock().now().to_msg()
        detection_img_msg.header.frame_id = "camera_link"

        self.image_pub.publish(detection_img_msg)


    def trigger_callback(self, msg):
        """Called when a message is received on /camera/take_photo"""
        self.get_logger().info("Triggering high-res capture...")
        self.q_trigger_in.send(dai.Buffer())

    def save_photo(self, frame):
        timestamp = self.get_clock().now().nanoseconds

        if self.latest_lat is not None and self.latest_lon is not None:
            filename = f"loc_{self.latest_lat:.6f}_{self.latest_lon:.6f}_{timestamp}.png"
        else:
            filename = f"no_fix_{timestamp}.png"
            self.get_logger().warn("Saving photo: no fix :(")
        cv2.imwrite(os.path.join(self.imgdir, filename), frame)
        self.get_logger().info(f"Saved high-res photo to: {os.path.join(self.imgdir, filename)}")


def main(args=None):
    rclpy.init(args=args)
    node = DepthAICameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

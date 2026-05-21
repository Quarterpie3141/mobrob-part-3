from std_msgs.msg import Empty, String
from geometry_msgs.msg import Twist
import os
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

        self.publisher_ = self.create_publisher(Image, 'camera/raw_image', 10)
        self.detection_pub = self.create_publisher(Detection2DArray, "camera/detections", 10)
        self.image_pub = self.create_publisher(Image, "camera/detections/image", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.slave_status_pub = self.create_publisher(String, '/slave/status', 10)


        self.bridge = CvBridge()
        self.trigger_sub = self.create_subscription(Empty, 'camera/take_photo', self.trigger_callback, 10)
        self.master_status_sub = self.create_subscription(String, '/master/status', self.master_status_callback, 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.master_status = None
        self.last_master_status = None

        self.latest_lat = None
        self.latest_lon = None
        self.imgdir = os.path.join(os.getcwd(), "img/")
        self.modeldir = os.path.join(os.getcwd(),"src","cv", "models")

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

        weights = os.path.join(self.modeldir, "objects.pt")
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

    def timer_callback(self):
        if self.master_status != 'taking picture':
            return

        # Spin in place while searching
        cmd = Twist()
        cmd.angular.z = 0.3
        self.cmd_vel_pub.publish(cmd)

        in_rgb = self.q_rgb.tryGet()
        if in_rgb is None:
            return

        frame = in_rgb.getCvFrame()

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = "camera_link"
        self.publisher_.publish(img_msg)

        detection_array = Detection2DArray()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = "camera_link"

        results = self.model.predict(frame, conf=0.5, verbose=False)

        for result in results:
            if result.boxes is None:
                continue

            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

                center_x = (x1 + x2) / 2.0
                center_y = (y1 + y2) / 2.0
                box_width = float(x2 - x1)
                box_height = float(y2 - y1)

                conf = float(box.conf[0].cpu().numpy())
                cls = int(box.cls[0].cpu().numpy())

                detection_msg = Detection2D()
                detection_msg.header = detection_array.header
                detection_msg.bbox.center.position.x = center_x
                detection_msg.bbox.center.position.y = center_y
                detection_msg.bbox.size_x = box_width
                detection_msg.bbox.size_y = box_height

                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = self.label_map.get(cls, str(cls))
                hypothesis.hypothesis.score = conf
                detection_msg.results.append(hypothesis)

                detection_array.detections.append(detection_msg)

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label_text = f"{hypothesis.hypothesis.class_id}: {int(conf * 100)}%"
                cv2.putText(
                    frame,
                    label_text,
                    (x1 + 5, y1 + 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    1,
                )

        self.detection_pub.publish(detection_array)

        detection_img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        detection_img_msg.header.stamp = self.get_clock().now().to_msg()
        detection_img_msg.header.frame_id = "camera_link"
        self.image_pub.publish(detection_img_msg)

        if detection_array.detections:
            # Object found — stop spinning, save photo, signal state machine
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)

            #self.save_photo(frame)

            done_msg = String()
            done_msg.data = 'finished'
            self.slave_status_pub.publish(done_msg)
            self.get_logger().info(
                f'[CV] Detected {len(detection_array.detections)} object(s). '
                'Stopping and publishing /slave/status=finished.'
            )
            self.master_status = None  # prevent re-triggering until next state update



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

# import struct
#
# import cv2
# import depthai as dai
# import numpy as np
# import rclpy
# from cv_bridge import CvBridge
# from rclpy.node import Node
# from sensor_msgs.msg import Image, PointCloud2, PointField
# from std_msgs.msg import Header
#
# FPS = 20
# RGB_WIDTH = 640
# RGB_HEIGHT = 400
# DEPTH_WIDTH = 640
# DEPTH_HEIGHT = 400
#
#
# class DepthAICameraNode(Node):
#     def __init__(self):
#         super().__init__("depthai_camera_node")
#
#         self.bridge = CvBridge()
#         self.rgb_pub = self.create_publisher(Image, "camera/rgb/image_raw", 10)
#         self.pc_pub = self.create_publisher(PointCloud2, "camera/depth/points", 10)
#
#         self.fx = None
#         self.fy = None
#         self.cx = None
#         self.cy = None
#
#         self.pipeline = dai.Pipeline()
#         self.pipeline.setXLinkChunkSize(0)
#
#         self._build_rgb_output()
#         self._build_depth_output()
#
#         # DepthAI v3 style: start the pipeline directly.
#         self.pipeline.start()
#
#         self._load_intrinsics()
#
#         self.timer = self.create_timer(1.0 / FPS, self.timer_callback)
#         self.get_logger().info("DepthAI ROS2 node started using DepthAI v3 pipeline API")
#
#     def _build_rgb_output(self):
#         # DepthAI v3 camera API, matching the style used in pyCam.py
#         cam = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
#         self.rgb_queue = cam.requestOutput(
#             size=(RGB_WIDTH, RGB_HEIGHT),
#             type=dai.ImgFrame.Type.BGR888p,
#             fps=FPS,
#         ).createOutputQueue()
#
#     def _build_depth_output(self):
#         # Left mono camera using v3 Camera API
#         mono_left = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
#         mono_left_out = mono_left.requestOutput(
#             size=(DEPTH_WIDTH, DEPTH_HEIGHT),
#             type=dai.ImgFrame.Type.NV12,
#             fps=FPS,
#         )
#
#         # Right mono camera using v3 Camera API
#         mono_right = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
#         mono_right_out = mono_right.requestOutput(
#             size=(DEPTH_WIDTH, DEPTH_HEIGHT),
#             type=dai.ImgFrame.Type.NV12,
#             fps=FPS,
#         )
#
#         stereo = self.pipeline.create(dai.node.StereoDepth)
#         stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.FAST_DENSITY)
#         stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
#         stereo.setOutputSize(DEPTH_WIDTH, DEPTH_HEIGHT)
#         stereo.setSubpixel(True)
#         stereo.setLeftRightCheck(True)
#
#         mono_left_out.link(stereo.left)
#         mono_right_out.link(stereo.right)
#
#         # DepthAI v3 — create output queue directly on the depth output
#         self.depth_queue = stereo.depth.createOutputQueue()
#
#
#     def _load_intrinsics(self):
#         # With the v3 API the default device is owned by the running pipeline.
#         calib = dai.Device.getDefaultDevice().readCalibration()
#         intrinsics = calib.getCameraIntrinsics(
#             dai.CameraBoardSocket.CAM_A,
#             RGB_WIDTH,
#             RGB_HEIGHT,
#         )
#         self.fx = float(intrinsics[0][0])
#         self.fy = float(intrinsics[1][1])
#         self.cx = float(intrinsics[0][2])
#         self.cy = float(intrinsics[1][2])
#
#     def timer_callback(self):
#         rgb_in = self.rgb_queue.tryGet()
#         if rgb_in is not None:
#             rgb_frame = rgb_in.getCvFrame()
#             cv2.imshow("DepthAI RGB", rgb_frame)
#             self.publish_rgb(rgb_frame)
#
#
#         depth_in = self.depth_queue.tryGet()
#         if depth_in is not None:
#             depth_frame = depth_in.getFrame()
#             pointcloud_msg = self.depth_to_pointcloud2(depth_frame)
#             self.pc_pub.publish(pointcloud_msg)
#
#         key = cv2.waitKey(1) & 0xFF
#         if key == ord("q") or key == 27:
#             self.get_logger().info("Shutdown requested from OpenCV window")
#             rclpy.shutdown()
#
#     def publish_rgb(self, frame):
#         msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.header.frame_id = "rgb_camera_frame"
#         self.rgb_pub.publish(msg)
#
#     def depth_to_pointcloud2(self, depth_frame):
#         depth_m = depth_frame.astype(np.float32) / 1000.0
#         height, width = depth_m.shape
#
#         u_coords, v_coords = np.meshgrid(np.arange(width), np.arange(height))
#         valid = depth_m > 0.0
#
#         z = depth_m[valid]
#         x = ((u_coords[valid] - self.cx) * z) / self.fx
#         y = ((v_coords[valid] - self.cy) * z) / self.fy
#         points = np.column_stack((x, y, z)).astype(np.float32)
#
#         header = Header()
#         header.stamp = self.get_clock().now().to_msg()
#         header.frame_id = "rgb_camera_frame"
#
#         fields = [
#             PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
#             PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
#             PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
#         ]
#
#         cloud_data = b"".join(struct.pack("fff", *point) for point in points)
#
#         msg = PointCloud2()
#         msg.header = header
#         msg.height = 1
#         msg.width = points.shape[0]
#         msg.fields = fields
#         msg.is_bigendian = False
#         msg.point_step = 12
#         msg.row_step = msg.point_step * msg.width
#         msg.is_dense = False
#         msg.data = cloud_data
#         return msg
#
#     def destroy_node(self):
#         cv2.destroyAllWindows()
#         cv2.waitKey(1)
#         super().destroy_node()
#
# def main(args=None):
#     rclpy.init(args=args)
#     node = None
#     try:
#         node = DepthAICameraNode()
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     except Exception as e:
#         print(f"Error starting node: {e}")
#     finally:
#         if node is not None:
#             node.destroy_node()
#         if rclpy.ok():
#             rclpy.shutdown()
#
#
# if __name__ == "__main__":
#     main()

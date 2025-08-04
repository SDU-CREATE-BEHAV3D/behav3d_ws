#!/usr/bin/env python3
import os
import time
import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import tf2_ros
import numpy as np

class ImageProcessorCapture(Node):
    def __init__(self):
        super().__init__('image_processor_capture')

        # Parameters
        default_dir = os.path.expanduser('~/captures')
        self.declare_parameter('output_dir', default_dir)
        self.declare_parameter('base_frame', 'ur10e_wrist_3_link')
        self.declare_parameter('ee_frame', 'femto__depth_optical_frame')

        # Root output directory
        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        os.makedirs(self.output_dir, exist_ok=True)

        # Sub‐directories
        self.rgb_dir   = os.path.join(self.output_dir, 'rgb')
        self.depth_dir = os.path.join(self.output_dir, 'depth')
        self.pose_dir  = os.path.join(self.output_dir, 'poses')
        for d in (self.rgb_dir, self.depth_dir, self.pose_dir):
            os.makedirs(d, exist_ok=True)

        # Set up CvBridge
        self.bridge = CvBridge()
        self.latest_color = None
        self.latest_depth = None
        self.latest_color_ns = None
        self.latest_depth_ns = None

        # Subscribers for color and depth
        self.create_subscription(Image, '/camera/color/image_raw', self.color_cb, 10)
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_cb, 10)

        # TF listener for robot pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Display window and mouse callback
        self.window_name = 'Image Proc Capture'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1280, 720)
        cv2.setMouseCallback(self.window_name, self.mouse_cb)

        self.get_logger().info(f'Captures will be saved to: {self.output_dir}')
        self.get_logger().info("Press 'c' to capture RGB-D and pose, 't' for timestamp sync test, click window to capture, 'q' to quit.")

    def color_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.latest_color = frame
        self.latest_color_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def depth_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        if frame.dtype == np.float32:
            frame = (frame * 1000).astype(np.uint16)
        self.latest_depth = frame
        self.latest_depth_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def capture_frame(self):
        # 1) ensure frames exist
        if self.latest_color is None or self.latest_depth is None:
            self.get_logger().warning('No RGB-D frames available to capture!')
            return

        # 2) timestamp + filenames
        ts = time.strftime('%Y%m%d-%H%M%S')
        rgb_fn   = f'{ts}_rgb.png'
        depth_fn = f'{ts}_depth.png'
        pose_fn  = f'{ts}_pose.json'

        rgb_path   = os.path.join(self.rgb_dir,   rgb_fn)
        depth_path = os.path.join(self.depth_dir, depth_fn)
        pose_path  = os.path.join(self.pose_dir,  pose_fn)

        # 3) save images
        cv2.imwrite(rgb_path,   self.latest_color)
        cv2.imwrite(depth_path, self.latest_depth)

        # 4) lookup transform
        base = self.get_parameter('base_frame').get_parameter_value().string_value
        ee   = self.get_parameter('ee_frame').get_parameter_value().string_value
        try:
            if not self.tf_buffer.can_transform(base, ee, rclpy.time.Time(),
                                                timeout=rclpy.duration.Duration(seconds=1.0)):
                self.get_logger().error(f'Transform {base}→{ee} not available')
                return
            trans = self.tf_buffer.lookup_transform(base, ee, rclpy.time.Time())
        except Exception as e:
            self.get_logger().error(f'TF lookup failed for {ee}: {e}')
            return

        # 5) build pose record
        pose = {
            'timestamp': ts,
            'rgb_file': rgb_fn,
            'depth_file': depth_fn,
            'translation': {
                'x': trans.transform.translation.x,
                'y': trans.transform.translation.y,
                'z': trans.transform.translation.z,
            },
            'quad': {
                'x': trans.transform.rotation.x,
                'y': trans.transform.rotation.y,
                'z': trans.transform.rotation.z,
                'w': trans.transform.rotation.w,
            },
        }

        # 6) save individual pose JSON
        with open(pose_path, 'w') as f:
            json.dump(pose, f, indent=2)

        # 7) append to master all_poses.json
        master_path = os.path.join(self.output_dir, 'all_poses.json')
        try:
            with open(master_path, 'r') as mf:
                all_poses = json.load(mf)
        except (FileNotFoundError, json.JSONDecodeError):
            all_poses = []

        all_poses.append(pose)
        tmp = master_path + '.tmp'
        with open(tmp, 'w') as mf:
            json.dump(all_poses, mf, indent=2)
        os.replace(tmp, master_path)

        # 8) log success
        self.get_logger().info(
            f"Saved:\n"
            f"  RGB  → {rgb_path}\n"
            f"  Depth→ {depth_path}\n"
            f"  Pose → {pose_path}\n"
            f"  Appended to {master_path}"
        )

    def timestamp_sync_test(self):
        if self.latest_color_ns is None or self.latest_depth_ns is None:
            self.get_logger().warning('No timestamps available for sync test!')
            return
        now_ns = self.get_clock().now().nanoseconds
        delta_color = (now_ns - self.latest_color_ns) / 1e6
        delta_depth = (now_ns - self.latest_depth_ns) / 1e6
        self.get_logger().info(f'Timestamp deltas — color: {delta_color:.2f} ms, depth: {delta_depth:.2f} ms')

    def mouse_cb(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.capture_frame()

    def spin_loop(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.latest_color is not None:
                cv2.imshow(self.window_name, self.latest_color)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('c'):
                self.capture_frame()
            elif key == ord('t'):
                self.timestamp_sync_test()
            elif key == ord('q'):
                break

        cv2.destroyAllWindows()
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessorCapture()
    node.spin_loop()

if __name__ == '__main__':
    main()

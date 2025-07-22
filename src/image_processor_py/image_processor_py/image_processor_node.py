#!/usr/bin/env python3
import os
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor_py')
        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_header_ns = None

        # Subscribe to the color image topic
        self.sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_cb,
            10
        )

        # Prepare display window and mouse callback
        window_name = 'Gray Proc'
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 640, 360)
        cv2.setMouseCallback(window_name, self.mouse_cb)

        # Directory for captures
        self.out_dir = os.path.expanduser('~/captures')
        os.makedirs(self.out_dir, exist_ok=True)
        self.get_logger().info(f'Captures will be saved to: {self.out_dir}')
        self.get_logger().info("Press 'c' to capture, 't' for timestamp sync test, or click window to capture; 'q' to quit.")
        self.window_name = window_name

    def image_cb(self, msg):
        # Convert ROS Image to OpenCV grayscale
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.latest_frame = gray
        # Store header stamp in nanoseconds
        self.latest_header_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def capture_frame(self):
        if self.latest_frame is None:
            self.get_logger().warning('No frame yet to capture!')
            return
        ts = time.strftime('%Y%m%d-%H%M%S')
        filename = os.path.join(self.out_dir, f'capture_{ts}.png')
        cv2.imwrite(filename, self.latest_frame)
        self.get_logger().info(f'Captured: {filename}')

    def timestamp_sync_test(self):
        if self.latest_header_ns is None:
            self.get_logger().warning('No header stamp available for sync test!')
            return
        now_ns = self.get_clock().now().nanoseconds
        delta_ns = now_ns - self.latest_header_ns
        delta_ms = delta_ns / 1_000_000.0
        self.get_logger().info(f'Timestamp sync delta: {delta_ms:.2f} ms')

    def mouse_cb(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.capture_frame()

    def spin_loop(self):
        # Main loop: ROS callbacks and OpenCV events
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.latest_frame is not None:
                cv2.imshow(self.window_name, self.latest_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('c'):
                self.capture_frame()
            elif key == ord('t'):
                self.timestamp_sync_test()
            elif key == ord('q'):
                break

        # Clean up
        cv2.destroyAllWindows()
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    node.spin_loop()

if __name__ == '__main__':
    main()

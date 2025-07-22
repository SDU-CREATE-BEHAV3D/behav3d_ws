#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DashboardProcessor(Node):
    MODES = ['grayscale', 'blur', 'canny', 'threshold']

    def __init__(self):
        super().__init__('dashboard_processor')
        self.bridge = CvBridge()

        # Last frames cache
        self.last_color = None
        self.last_depth = None
        self.last_ir = None

        # Subscribers
        qos = 10
        self.create_subscription(Image, '/camera/color/image_raw', self.cb_color, qos)
        self.create_subscription(Image, '/camera/depth/image_rect_raw', self.cb_depth, qos)
        self.create_subscription(Image, '/camera/ir/image_raw', self.cb_ir, qos)

        # Build dashboard window + trackbar
        cv2.namedWindow('Dashboard', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Dashboard', 1280, 720)
        cv2.createTrackbar(
            'Mode', 'Dashboard',
            0, len(self.MODES) - 1,
            lambda v: None  # we’ll poll the trackbar
        )

        # Timer to refresh dashboard
        self.create_timer(1/30.0, self.timer_callback)
        self.get_logger().info('DashboardProcessor up and running')

    def cb_color(self, msg):
        self.last_color = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def cb_depth(self, msg):
        # raw depth as 16UC1
        depth_raw = self.bridge.imgmsg_to_cv2(msg, msg.encoding).astype(np.uint16)
        # normalize full range of depth_raw to 0–255
        depth_norm = cv2.normalize(
            depth_raw, None,
            alpha=0, beta=255,
            norm_type=cv2.NORM_MINMAX,
            dtype=cv2.CV_8U
        )
        self.last_depth = depth_norm

    def cb_ir(self, msg):
        ir_raw = self.bridge.imgmsg_to_cv2(msg, msg.encoding).astype(np.uint16)
        # normalize IR to 0–255
        ir_norm = cv2.normalize(
            ir_raw, None,
            alpha=0, beta=255,
            norm_type=cv2.NORM_MINMAX,
            dtype=cv2.CV_8U
        )
        self.last_ir = ir_norm

    def process(self, img):
        mode_idx = cv2.getTrackbarPos('Mode', 'Dashboard')
        mode = self.MODES[mode_idx]
        if img is None:
            return None
        if mode == 'grayscale':
            return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if mode == 'blur':
            k = 2 * mode_idx + 1  # odd kernel sizes 1,3,5,7
            return cv2.GaussianBlur(img, (k, k), 0)
        if mode == 'canny':
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            return cv2.Canny(gray, 50, 150)
        if mode == 'threshold':
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            _, th = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY)
            return th
        return img

    def timer_callback(self):
        if self.last_color is None:
            return

        # apply processing to color
        proc = self.process(self.last_color)

        # helper to ensure BGR for stacking
        def to_bgr(x):
            if x is None:
                return np.zeros_like(self.last_color)
            if len(x.shape) == 2:
                return cv2.cvtColor(x, cv2.COLOR_GRAY2BGR)
            return x

        c = cv2.resize(to_bgr(self.last_color), (640, 360))
        d = cv2.resize(to_bgr(self.last_depth), (640, 360))
        i = cv2.resize(to_bgr(self.last_ir),      (640, 360))
        p = cv2.resize(to_bgr(proc),              (640, 360))

        top = np.hstack([c, d])
        bot = np.hstack([i, p])
        mosaic = np.vstack([top, bot])

        # overlay current mode name
        mode_idx = cv2.getTrackbarPos('Mode', 'Dashboard')
        mode = self.MODES[mode_idx]
        cv2.putText(mosaic, f'Mode: {mode}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow('Dashboard', mosaic)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DashboardProcessor()
    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

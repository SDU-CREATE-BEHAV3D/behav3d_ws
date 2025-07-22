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

        # Subscribe to color image topic
        self.sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_cb,
            10
        )

        # Prepare OpenCV window
        cv2.namedWindow('Gray Proc', cv2.WINDOW_NORMAL)

        # Directory to save captures
        self.out_dir = os.path.expanduser('~/captures')
        os.makedirs(self.out_dir, exist_ok=True)
        self.get_logger().info(f'Saving captures to: {self.out_dir}')

    def image_cb(self, msg):
        # Convert ROS Image to OpenCV frame
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.latest_frame = gray

        # Display the processed frame
        cv2.imshow('Gray Proc', gray)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('c'):
            self.capture_frame()
        elif key == ord('q'):
            # Quit: destroy windows and shut down ROS
            cv2.destroyAllWindows()
            rclpy.shutdown()

        # Log each processed frame size
        h, w = gray.shape
        self.get_logger().info(f'Processed frame {w}×{h}')

    def capture_frame(self):
        if self.latest_frame is None:
            self.get_logger().warning('No frame available to capture')
            return
        timestamp = time.strftime('%Y%m%d-%H%M%S')
        filename = os.path.join(self.out_dir, f'capture_{timestamp}.png')
        cv2.imwrite(filename, self.latest_frame)
        self.get_logger().info(f'Captured image: {filename}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

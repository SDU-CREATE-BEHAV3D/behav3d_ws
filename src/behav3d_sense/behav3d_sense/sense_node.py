#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class SenseNode(Node):
    def __init__(self):
        super().__init__('sense_node')
        self.get_logger().info('Hello world from behav3d_sense!')

def main(args=None):
    rclpy.init(args=args)
    node = SenseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("py_test")
        self.counter = 0
        self.get_logger().info("Hello" + str(self.counter))
        self.create_timer(1, self.print)

    def print(self):
        self.counter += 1
        self.get_logger().info("Hello" + str(self.counter))


def main (args = None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main() 

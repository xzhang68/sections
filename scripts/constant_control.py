#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Publisher(Node):
    def __init__(self) -> None:
        super().__init__("publisher_node")
        self.get_logger().info('Publisher has been created...')
        self.counter = 1
        self.pub = self.create_publisher(String, '/cmd_vel', 10)
        self.pub_timer = self.create_timer(0.2, self.publish)
        
    def publish(self) -> None:
        msg = String()
        msg.data = f"sending constant control… {self.counter}"
        self.pub.publish(msg)
        self.counter += 1

def main(args=None):
    rclpy.init()        
    node = Publisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
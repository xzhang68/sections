#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class ConstantControlTwist(Node):
    def __init__(self) -> None:
        super().__init__('constant_control_twist')
        self.get_logger().info('constant_control_twist node has been created...')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.2, self.control_callback)
        self.kill_sub = self.create_subscription(Bool, '/kill', self.kill_callback, 10)
        self.running = True

    def control_callback(self) -> None:
        if self.running:
            msg = Twist()
            msg.linear.x = 0.5
            msg.angular.z = 0.0
            self.cmd_vel_pub.publish(msg)
            self.get_logger().info('sending constant control...')
    
    def kill_callback(self, msg: Bool) -> None:
        if msg.data:
            self.get_logger().warn('Kill command received! Stopping the robot.')
            self.running = False
            self.timer.cancel()
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)

def main(args=None):
    rclpy.init()
    node = ConstantControlTwist()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    
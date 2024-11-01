#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node

from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_lib.math_utils import wrap_angle

from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState

class HeadingController(BaseHeadingController): 
    def __init__(self):
        super().__init__()
        self.declare_parameter('kp', 20.0)

    @property
    def kp(self) -> float:
        return self.get_parameter('kp').value

    def compute_control_with_goal(
        self, state: TurtleBotState, goal: TurtleBotState
    ) -> TurtleBotControl:
        error = wrap_angle(goal.theta - state.theta)

        omega = self.kp * error

        control_msg = TurtleBotControl()
        control_msg.omega = omega
        control_msg.v = 0.0

        self.get_logger().info(f"Heading error: {error:.4f}, Omega: {omega:.4f}")

        return control_msg

def main(args=None):
    rclpy.init(args=args)
    controller = HeadingController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
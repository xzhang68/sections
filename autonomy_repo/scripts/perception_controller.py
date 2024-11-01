#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from asl_tb3_lib.control import BaseController
from asl_tb3_msgs.msg import TurtleBotControl
from std_msgs.msg import Bool
from rclpy.duration import Duration

class PerceptionController(BaseController):
    def __init__(self):
        super().__init__('perception_controller')
        
        self.declare_parameter('active', True)
        self._active = self.get_parameter('active').get_parameter_value().bool_value
        
        self.stop_time = None
        self.cooldown_duration = 5.0
        self.last_stop_time = -self.cooldown_duration

        self.subscription = self.create_subscription(
            Bool,
            '/detector_bool',
            self.detector_callback,
            10
        )
        self.subscription
        
        self.get_logger().info("PerceptionController initialized and subscribed to /detector_bool.")

    @property
    def active(self) -> bool:
        return self._active

    def detector_callback(self, msg: Bool):
        current_time_sec = self.get_clock().now().nanoseconds / 1e9
        
        if msg.data and self.active and (current_time_sec - self.last_stop_time) >= self.cooldown_duration:
            self.get_logger().info("Stop sign detected. Initiating stop.")
            self._active = False
            self.stop_time = current_time_sec
            self.last_stop_time = current_time_sec

    def compute_control(self) -> TurtleBotControl:
        msg = TurtleBotControl()

        current_time_sec = self.get_clock().now().nanoseconds / 1e9

        if self.active:
            msg.v = 0.0
            msg.omega = 0.5
            self.get_logger().debug("Robot is active. Spinning with omega=0.5.")
        else:
            if self.stop_time is None:
                self.stop_time = current_time_sec
                self.get_logger().debug("Stopping the robot.")
            
            if (current_time_sec - self.stop_time) >= 5.0:
                self._active = True
                self.stop_time = None
                self.get_logger().info("Resuming spinning after stop.")
                msg.v = 0.0
                msg.omega = 0.5
            else:
                msg.v = 0.0
                msg.omega = 0.0
                self.get_logger().debug(f"Robot is stopped. Time elapsed: {current_time_sec - self.stop_time:.2f} seconds.")
        
        return msg

def main(args=None):
    rclpy.init(args=args)
    controller = PerceptionController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("PerceptionController shutting down.")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



# Task 1:

# #!/usr/bin/env python3

# import rclpy
# from asl_tb3_lib.control import BaseController
# from asl_tb3_msgs.msg import TurtleBotControl

# class PerceptionController(BaseController):
#     def __init__(self):
#         super().__init__('perception_controller')
#         self.declare_parameter('active', True)
#         self.stop_time = None
    
#     @property
#     def active(self) -> bool:
#         return self.get_parameter('active').get_parameter_value().bool_value
    
#     def compute_control(self) -> TurtleBotControl:
#         msg = TurtleBotControl()
#         current_time = self.get_clock().now().nanoseconds / 1e9

#         if self.active:
#             msg.v = 0.0
#             msg.omega = 0.5
#             self.stop_time = None
#         else:
#             if self.stop_time is None:
#                 self.stop_time = current_time
#             elif current_time - self.stop_time >= 5.0:
#                 self.set_parameters([rclpy.parameter.Parameter('active', rclpy.Parameter.Type.BOOL, True)])
#                 self.stop_time = None
#             else:
#                 msg.v = 0.0
#                 msg.omega = 0.0

#         return msg

# def main(args=None):
#     rclpy.init(args=args)
#     controller = PerceptionController()
#     rclpy.spin(controller)
#     controller.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


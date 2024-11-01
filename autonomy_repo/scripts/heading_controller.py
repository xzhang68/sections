# #!/usr/bin/env python3

# import numpy as np
# import rclpy
# from asl_tb3_lib.control import BaseHeadingController
# from asl_tb3_lib.math_utils import wrap_angle
# from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState

# class HeadingController(BaseHeadingController):
#     def __init__(self):
#         super().__init__()
#         self.kp = 10.0
        
#     def compute_control_with_goal(self, state: TurtleBotState, goal: TurtleBotState) -> TurtleBotControl:
#         heading_error = wrap_angle(goal.theta - state.theta)
#         print(f"Current Theta: {state.theta}, Goal Theta: {goal.theta}, Heading Error: {heading_error}")

#         omega = self.kp * heading_error
#         print(f"Computed Omega: {omega}")

#         control_msg = TurtleBotControl()
#         control_msg.omega = omega
#         return control_msg

# if __name__ == "__main__":
#     rclpy.init()
#     heading_controller = HeadingController()
#     rclpy.spin(heading_controller)
#     rclpy.shutdown()



#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState
import matplotlib.pyplot as plt

class HeadingController(BaseHeadingController, Node):
    def __init__(self):
        BaseHeadingController.__init__(self)
        Node.__init__(self, 'heading_controller')
        self.kp = 2.0

        # Subscriber to get the current state of the TurtleBot
        self.subscription = self.create_subscription(
            TurtleBotState,
            '/state',
            self.state_callback,
            10
        )

        # Publisher to send control commands
        self.publisher = self.create_publisher(TurtleBotControl, '/cmd_pose', 10)

        self.current_state = None
        self.goal_state = None
        self.trajectory = []

    def state_callback(self, msg: TurtleBotState):
        self.current_state = msg
        self.trajectory.append((msg.x, msg.y))
        if self.goal_state is not None:
            control_msg = self.compute_control_with_goal(self.current_state, self.goal_state)
            self.publisher.publish(control_msg)

    def compute_control_with_goal(self, state: TurtleBotState, goal: TurtleBotState) -> TurtleBotControl:
        # Calculate heading error
        heading_error = wrap_angle(goal.theta - state.theta)
        print(f"Current Theta: {state.theta}, Goal Theta: {goal.theta}, Heading Error: {heading_error}")

        # Proportional control
        omega = self.kp * heading_error
        print(f"Computed Omega: {omega}")

        control_msg = TurtleBotControl()
        control_msg.omega = omega
        return control_msg

    def plot_trajectory(self):
        # Plotting the trajectory of the TurtleBot
        trajectory = np.array(self.trajectory)
        plt.figure()
        plt.plot(trajectory[:, 0], trajectory[:, 1], 'g-', label='A* solution path')
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.title('Plot from Simple Environment')
        plt.legend()
        plt.grid()
        plt.savefig('trajectory_plot.png')
        plt.show()

if __name__ == "__main__":
    rclpy.init()
    heading_controller = HeadingController()
    try:
        rclpy.spin(heading_controller)
    except KeyboardInterrupt:
        # Plot trajectory when the node is stopped
        heading_controller.plot_trajectory()
    finally:
        rclpy.shutdown()
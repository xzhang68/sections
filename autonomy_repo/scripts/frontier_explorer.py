#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from asl_tb3_msgs.msg import TurtleBotState
from nav_msgs.msg import OccupancyGrid
import numpy as np
from asl_tb3_lib.grids import StochOccupancyGrid2D
from scipy.signal import convolve2d

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        self.declare_parameter('active', True)
        self._active = True
        self.stop_time = None
        self.cooldown_duration = 5.0
        self.last_stop_time = -self.cooldown_duration

        self.nav_success_sub = self.create_subscription(
            Bool,
            '/nav_success',
            self.nav_success_callback,
            10)

        self.state_sub = self.create_subscription(
            TurtleBotState,
            '/state',
            self.state_callback,
            10)

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)

        self.detector_sub = self.create_subscription(
            Bool,
            '/detector_bool',
            self.detector_callback,
            10
        )

        self.cmd_nav_pub = self.create_publisher(
            TurtleBotState,
            '/cmd_nav',
            10)

        self.nav_success = True
        self.current_state = None
        self.occupancy = None
        self.goal_sent = False

        self.timer = self.create_timer(0.3, self.timer_callback)

    def nav_success_callback(self, msg):
        if msg.data:
            self.get_logger().info('Navigation succeeded.')
            self.nav_success = True
            self.goal_sent = False
        else:
            self.get_logger().warn('Navigation failed. Replanning...')
            self.nav_success = True
            self.goal_sent = False

    def state_callback(self, msg):
        self.current_state = np.array([msg.x, msg.y])

    def map_callback(self, msg):
        self.occupancy = StochOccupancyGrid2D(
            resolution=msg.info.resolution,
            size_xy=np.array([msg.info.width, msg.info.height]),
            origin_xy=np.array([msg.info.origin.position.x, msg.info.origin.position.y]),
            window_size=7,
            probs=msg.data,
            thresh=0.5
        )

    def detector_callback(self, msg: Bool):
        current_time_sec = self.get_clock().now().nanoseconds / 1e9

        if msg.data and self._active and (current_time_sec - self.last_stop_time) >= self.cooldown_duration:
            self.get_logger().info("Stop sign detected. Initiating stop.")
            self._active = False
            self.stop_time = current_time_sec
            self.last_stop_time = current_time_sec
            self.goal_sent = False
            self.send_stop_command()

    def send_stop_command(self):
        if self.current_state is not None:
            stop_msg = TurtleBotState()
            stop_msg.x = float(self.current_state[0])
            stop_msg.y = float(self.current_state[1])

            self.cmd_nav_pub.publish(stop_msg)
            self.get_logger().info('Sent stop command to robot.')
        else:
            self.get_logger().warn('Current state not available. Cannot send stop command.')

    def timer_callback(self):
        if self.current_state is None or self.occupancy is None:
            return

        current_time_sec = self.get_clock().now().nanoseconds / 1e9

        if not self._active:
            if (current_time_sec - self.stop_time) >= 5.0:
                self._active = True
                self.get_logger().info("Resuming exploration after stop.")
                self.nav_success = True
            else:
                return
        else:
            if self.nav_success and not self.goal_sent:
                frontier_states = self.explore()

                if frontier_states.size == 0:
                    self.get_logger().info('Finished exploring.')
                    return

                distances = np.linalg.norm(frontier_states - self.current_state, axis=1)
                closest_idx = np.argmin(distances)
                next_goal = frontier_states[closest_idx]

                goal_msg = TurtleBotState()
                goal_msg.x = float(next_goal[0])
                goal_msg.y = float(next_goal[1])
                goal_msg.theta = 0.0

                self.cmd_nav_pub.publish(goal_msg)
                self.get_logger().info(f'Sent new goal: x={goal_msg.x}, y={goal_msg.y}')
                self.goal_sent = True
                self.nav_success = False

    def explore(self):
        # Access the occupancy grid probabilities
        probs = self.occupancy.probs

        # Create binary grids
        unknown_grid = (probs == -1).astype(int)
        known_occupied_grid = (probs >= 0.5).astype(int)
        known_unoccupied_grid = ((probs >= 0) & (probs < 0.5)).astype(int)

        # Define the convolution kernel
        window_size = 13  # Adjust based on your heuristic window size
        kernel = np.ones((window_size, window_size), dtype=int)
        total_cells = window_size * window_size

        # Perform convolutions
        unknown_counts = convolve2d(unknown_grid, kernel, mode='same', boundary='fill', fillvalue=0)
        occupied_counts = convolve2d(known_occupied_grid, kernel, mode='same', boundary='fill', fillvalue=0)
        free_counts = convolve2d(known_unoccupied_grid, kernel, mode='same', boundary='fill', fillvalue=0)

        # Calculate percentages
        unknown_percentage = unknown_counts / total_cells
        free_percentage = free_counts / total_cells

        # Apply heuristics
        valid_cells = np.logical_and.reduce((
            unknown_percentage >= 0.2,    # At least 20% unknown cells
            occupied_counts == 0,         # No occupied cells
            free_percentage >= 0.3        # At least 30% free cells
        ))

        # Get indices of valid frontier cells
        valid_indices = np.argwhere(valid_cells)

        # Convert grid indices to state coordinates
        frontier_states = self.occupancy.grid2state(valid_indices[:, [1, 0]])

        return frontier_states

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
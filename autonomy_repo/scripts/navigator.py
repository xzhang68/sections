#!/usr/bin/env python3

import typing as T 
import numpy as np
import rclpy
from rclpy.node import Node
from asl_tb3_lib.navigation import BaseNavigator
from asl_tb3_msgs.msg import TurtleBotState, TurtleBotControl
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_lib.grids import StochOccupancyGrid2D
from asl_tb3_lib.tf_utils import quaternion_to_yaw
from scipy.interpolate import splev, splrep

from navigation import TrajectoryPlan
from astar import AStar, DetOccupancyGrid2D

class Navigator(BaseNavigator):
    def __init__(self):
        super().__init__()
        self.kp = 1.0
        self.kpx = 1.0
        self.kdx = 1.0
        self.kpy = 1.0
        self.kdy = 1.0
        self.V_prev = 0.0
        self.om_prev = 0.0
        self.t_prev = 0.0
        self.V_PREV_THRES = 0.0001

    def compute_heading_control(
        self, 
        state: TurtleBotState, 
        goal: TurtleBotState
    ) -> TurtleBotControl:
        heading_error = wrap_angle(goal.theta - state.theta)
        omega = self.kp * heading_error

        control_msg = TurtleBotControl()
        control_msg.omega = omega
        return control_msg

    def compute_trajectory_tracking_control(
        self, 
        state: TurtleBotState, 
        plan: TrajectoryPlan, 
        t: float
    ) -> TurtleBotControl:
        x_d = splev(t, plan.path_x_spline, der=0)
        y_d = splev(t, plan.path_y_spline, der=0)
        xd_d = splev(t, plan.path_x_spline, der=1)
        yd_d = splev(t, plan.path_y_spline, der=1)
        xdd_d = splev(t, plan.path_x_spline, der=2)
        ydd_d = splev(t, plan.path_y_spline, der=2)

        x = state.x
        y = state.y
        th = state.theta
        dt = t - self.t_prev

        xd = self.V_prev * np.cos(th)
        yd = self.V_prev * np.sin(th)
        self.V_prev = max(self.V_prev, self.V_PREV_THRES)
        u = np.array([
            xdd_d + self.kpx*(x_d - x) + self.kdx*(xd_d - xd),
            ydd_d + self.kpy*(y_d - y) + self.kdy*(yd_d - yd)
        ])
        j_inv = np.array([
            [np.cos(th), np.sin(th)],
            [-np.sin(th)/self.V_prev, np.cos(th)/self.V_prev]
        ])
        ctrl = j_inv @ u
        V = self.V_prev + ctrl[0] * dt
        om = ctrl[1]

        self.V_prev = V
        self.om_prev = om
        self.t_prev = t

        control_msg = TurtleBotControl()
        control_msg.v = V
        control_msg.omega = om
        self.get_logger().info(f"Computed control: v={V:.2f}, omega={om:.2f}")
        return control_msg

    def compute_trajectory_plan(
        self,
        state: TurtleBotState,
        goal: TurtleBotState,
        occupancy: StochOccupancyGrid2D,
        resolution: float,
        horizon: float
    ) -> T.Optional[TrajectoryPlan]:
        astar = AStar([state.x-horizon, state.y-horizon], [state.x+horizon, state.y+horizon], 
                      [state.x, state.y], [goal.x, goal.y], occupancy, resolution)
        if not astar.solve() or len(astar.path) < 4:
            print("No path found!")
            return None
        
        self.reset()
        path = np.array(astar.path)

        v_desired = 0.15
        spline_alpha = 0.05

        timestamps = [0]
        for i in range(len(path) - 1):
            timestamps.append(timestamps[-1] + astar.distance(path[i], path[i + 1]) / v_desired)
        
        path_x_spline = splrep(timestamps, path[:, 0], s = spline_alpha)
        path_y_spline = splrep(timestamps, path[:, 1], s = spline_alpha)

        return TrajectoryPlan(
            path=path,
            path_x_spline=path_x_spline,
            path_y_spline=path_y_spline,
            duration=timestamps[-1],
        )

    def reset(self):
        self.V_prev = 0.0
        self.om_prev = 0.0
        self.t_prev = 0.0

def main(args=None):
    rclpy.init(args=args)
    navigator = Navigator()
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
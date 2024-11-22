import typing as T 
import numpy as np
from asl_tb3_msgs.msg import TurtleBotState
from scipy.interpolate import splev

class TrajectoryPlan:
    
    """ Data structure for holding a trajectory plan comming for A* planner and
        a trajectory smoother

    See https://docs.python.org/3.10/library/dataclasses.html for how to work
    with dataclasses. In short, __init__ function is implicitly created with
    arguments replaced by the following properties. For example, you can
    create a trajectory plan with

    ```
    my_plan = TrajectoryPlan(path=..., path_x_spline=..., path_y_spline=..., duration=...)
    ```
    """
    def __init__(self, path: np.ndarray, 
                 path_x_spline: T.Tuple[np.ndarray, np.ndarray, int], 
                 path_y_spline: T.Tuple[np.ndarray, np.ndarray, int], 
                 duration: float) -> None:
        

        # raw planned path from A*
        self.path = path

        # cubic spline fit of the x and y trajectory,
        # should be return values from scipy.interpolate.splrep
        #   see https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.splrep.html
        self.path_x_spline = path_x_spline
        self.path_y_spline = path_y_spline

        # time duration of the trajectory plan
        self.duration = duration

    def desired_state(self, t: float) -> TurtleBotState:
        """ Get state from the plan at specified time point

        Args:
            t (float): time in [seconds]

        Returns:
            TurtleBotState: desired state at t
        """
        x_d = splev(t, self.path_x_spline, der=1)
        y_d = splev(t, self.path_y_spline, der=1)

        return TurtleBotState(
            x=float(splev(t, self.path_x_spline, der=0)),
            y=float(splev(t, self.path_y_spline, der=0)),
            theta=float(np.arctan2(y_d, x_d)),
        )

        # t = min(t, self.duration)

        # x = splev(t, self.path_x_spline, der=0)
        # y = splev(t, self.path_y_spline, der=0)

        # xd = splev(t, self.path_x_spline, der=1)
        # yd = splev(t, self.path_y_spline, der=1)

        # theta = np.arctan2(yd, xd)

        # desired_state = TurtleBotState()
        # desired_state.x = float(x)
        # desired_state.y = float(y)
        # desired_state.theta = float(theta)

        # return desired_state

    def smoothed_path(self, dt: float = 0.1) -> np.ndarray:
        """ Get the full smoothed path sampled with fixed time steps

        Args:
            dt (float): sampling duration in [seconds]

        Returns:
            np.ndarray: smoothed trajectory sampled @ dt
        """
        ts = np.arange(0., self.duration, dt)
        path = np.zeros((ts.shape[0], 2))
        path[:, 0] = splev(ts, self.path_x_spline, der=0)
        path[:, 1] = splev(ts, self.path_y_spline, der=0)

        return path
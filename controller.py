import os

import numpy as np

from pid import PidCtrl
from utilities import (
    calculate_angular_error,
    calculate_linear_error,
)

M_PI = 3.1415926535

P = 0
PD = 1
PI = 2
PID = 3


class Controller:
    # Default gains of the controller for linear and angular motions
    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2):
        # TODO Part 5 and 6: Modify the below lines to test your PD, PI, and PID controller
        self.PID_linear = PidCtrl(P, klp, klv, kli, filename_="linear.csv")
        self.PID_angular = PidCtrl(P, kap, kav, kai, filename_="angular.csv")

    def vel_request(self, pose, goal, status):
        e_lin = calculate_linear_error(pose, goal)
        e_ang = calculate_angular_error(pose, goal)

        linear_vel = self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel = self.PID_angular.update([e_ang, pose[3]], status)

        # TODO Part 4: Add saturation limits for the robot linear and angular velocity
        # For TurtleBot4 if not in TURTLEBOT3_MODEL not defined in env
        # https://clearpathrobotics.com/turtlebot-4/
        # https://emanual.robotis.com/docs/en/platform/turtlebot3/features/
        linear_vel_max = 0.31
        angular_vel_max = 1.90
        if "TURTLEBOT3_MODEL" in os.environ:
            linear_vel_max = 0.22
            angular_vel_max = 2.84
        linear_vel = min(linear_vel, linear_vel_max)
        angular_vel = min(angular_vel, angular_vel_max)

        return linear_vel, angular_vel


class TrajectoryController(Controller):
    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2):
        super().__init__(klp, klv, kli, kap, kav, kai)

    def vel_request(self, pose, list_goals, status):
        goal = self.look_far_for(pose, list_goals)

        final_goal = list_goals[-1]

        e_lin = calculate_linear_error(pose, final_goal)
        e_ang = calculate_angular_error(pose, goal)

        linear_vel = self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel = self.PID_angular.update([e_ang, pose[3]], status)

        # TODO Part 5: Add saturation limits for the robot linear and angular velocity

        linear_vel_max = 0.31
        angular_vel_max = 1.90
        if "TURTLEBOT3_MODEL" in os.environ:
            linear_vel_max = 0.22
            angular_vel_max = 2.84
        linear_vel = min(linear_vel, linear_vel_max)
        angular_vel = min(angular_vel, angular_vel_max)

        return linear_vel, angular_vel

    def look_far_for(self, pose, list_goals):
        pose_array = np.array([pose[0], pose[1]])
        list_goals_array = np.array(list_goals)

        distance_squared = np.sum((list_goals_array - pose_array) ** 2, axis=1)
        closest_index = np.argmin(distance_squared)

        return list_goals[min(closest_index + 3, len(list_goals) - 1)]

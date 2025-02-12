# Type of planner
from math import exp
from typing import Literal

POINT_PLANNER = 0
TRAJECTORY_PLANNER = 1


class Planner:
    def __init__(self, type_):
        self.type = type_

    def plan(self, goal_point=None, trajectory=None):
        if goal_point is None:
            goal_point = [-1.0, -1.0]
        if self.type == POINT_PLANNER:
            return self.point_planner(goal_point)

        elif self.type == TRAJECTORY_PLANNER:
            return self.trajectory_planner(trajectory)

    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return x, y

    # TODO Part 6: Implement the trajectories here
    @classmethod
    def trajectory_planner(cls, trajectory: Literal["parabola", "sigmoid"]):
        # the return should be a list of trajectory points: [ [x1,y1], ..., [xn,yn]]
        points = []
        if trajectory == "parabola":
            # Find y for each x value from 0 to 1.5 in 0.1 increments. Note that the upper bound of 1.6 is not included when using range.
            for x in range(16):
                x *= 0.1
                points += [[x, x**2]]
        elif trajectory == "sigmoid":
            # Find y for each x value from 0 to 2.6 in 0.1 increments. Note that the upper bound of 2.6 is not included when using range.
            for x in range(26):
                x *= 0.1
                y = (2 / (1 + exp(-2 * x))) - 1
                points += [[x, y]]
        return points

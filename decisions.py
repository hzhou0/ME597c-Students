# Imports
import os
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy import init, spin, spin_once
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSPresetProfiles

from controller import Controller, TrajectoryController
from localization import Localization, rawSensor
from planner import TRAJECTORY_PLANNER, POINT_PLANNER, Planner
from utilities import (
    calculate_linear_error,
)


# You may add any other imports you may need/want to use below
# import ...


class DecisionMaker(Node):
    def __init__(
        self,
        publisher_msg,
        publishing_topic,
        qos_publisher,
        goal_point=None,
        trajectory=None,
        rate=10,
        motion_type=POINT_PLANNER,
    ):
        super().__init__("decision_maker")

        # TODO Part 4: Create a publisher for the topic responsible for robot's motion
        self.publisher = self.create_publisher(
            publisher_msg, publishing_topic, qos_publisher
        )

        publishing_period = 1 / rate

        # Instantiate the controller
        # TODO Part 5: Tune your parameters here

        if motion_type == POINT_PLANNER:
            self.controller = Controller(klp=0.2, klv=0.5, kap=1.0, kav=0.6)
            self.planner = Planner(POINT_PLANNER)

        elif motion_type == TRAJECTORY_PLANNER:
            self.controller = TrajectoryController(
                klp=0.2, klv=0.8, kli=0.4, kap=0.8, kav=0.6
            )
            self.planner = Planner(TRAJECTORY_PLANNER)

        else:
            print("Error! you don't have this planner", file=sys.stderr)

        # Instantiate the localization, use rawSensor for now
        self.localizer = Localization(rawSensor)

        # Instantiate the planner
        # NOTE: goalPoint is used only for the pointPlanner
        self.goal = self.planner.plan(goal_point, trajectory)

        self.create_timer(publishing_period, self.timerCallback)

    def timerCallback(self):
        # TODO Part 3: Run the localization node
        # Remember that this file is already running the decision_maker node.

        # Run the localization node.
        spin_once(self.localizer)

        if self.localizer.get_pose() is None:
            print("waiting for odom msgs ....")
            return

        vel_msg = Twist()

        # TODO Part 3: Check if you reached the goal
        if type(self.goal) == list:
            # Check if the last point in the trajectory has been reached with a chosen tolerance.
            goal = self.goal[-1]
        else:
            goal = self.goal
        lin_err = calculate_linear_error(self.localizer.get_pose(), goal)
        reached_goal = lin_err < 0.05

        if reached_goal:
            print("reached goal")
            self.publisher.publish(vel_msg)

            self.controller.PID_angular.logger.save_log()
            self.controller.PID_linear.logger.save_log()

            # TODO Part 3: exit the spin
            sys.exit()

        velocity, yaw_rate = self.controller.vel_request(
            self.localizer.get_pose(), self.goal, True
        )

        # TODO Part 4: Publish the velocity to move the robot
        # Publish the velocity command.
        vel_msg.linear.x = velocity
        vel_msg.angular.z = yaw_rate
        self.publisher.publish(vel_msg)


import argparse


def main(args=None):
    init()

    # TODO Part 3: You might need to change the QoS profile based on whether you're using the real robot or in simulation.
    # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3

    odom_qos = QoSProfile(reliability=2, durability=2, history=1, depth=10)
    if "TURTLEBOT3_MODEL" in os.environ:
        odom_qos = (
            QoSPresetProfiles.SYSTEM_DEFAULT.value
        )  # Use the default profile in simulation

    # TODO Part 4: instantiate the decision_maker with the proper parameters for moving the robot
    if args.motion.lower() == "point":
        dm = DecisionMaker(Twist, "/cmd_vel", odom_qos, motion_type=POINT_PLANNER)
    elif args.motion.lower() == "parabola":
        dm = DecisionMaker(
            Twist,
            "/cmd_vel",
            odom_qos,
            trajectory="parabola",
            motion_type=TRAJECTORY_PLANNER,
        )
    elif args.motion.lower() == "sigmoid":
        dm = DecisionMaker(
            Twist,
            "/cmd_vel",
            odom_qos,
            trajectory="sigmoid",
            motion_type=TRAJECTORY_PLANNER,
        )
    else:
        raise RuntimeError("invalid motion type")

    try:
        spin(dm)
    except SystemExit:
        print(f"reached there successfully {dm.localizer.pose}")


if __name__ == "__main__":
    argParser = argparse.ArgumentParser(description="point or trajectory")
    argParser.add_argument("--motion", type=str, default="point")
    args = argParser.parse_args()

    main(args)

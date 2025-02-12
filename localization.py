import os
import sys

from utilities import Logger, euler_from_quaternion
from rclpy.time import Time
from rclpy.node import Node

from rclpy.qos import QoSProfile, QoSPresetProfiles
from nav_msgs.msg import Odometry

from rclpy import init, spin

rawSensor = 0


class Localization(Node):
    def __init__(self, localization_type=rawSensor):
        super().__init__("localizer")

        # TODO Part 3: Define the QoS profile variable based on whether you are using the simulation (Turtlebot 3 Burger) or the real robot (Turtlebot 4)
        # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3

        # Create a QoS profile that is reliable, has transient local durability, stores the last 10 messages.
        odom_qos = QoSProfile(reliability=2, durability=2, history=1, depth=10)
        if "TURTLEBOT3_MODEL" in os.environ:
            odom_qos = (
                QoSPresetProfiles.SYSTEM_DEFAULT.value
            )  # Use the default profile in simulation

        self.loc_logger = Logger("robot_pose.csv", ["x", "y", "theta", "stamp"])
        self.pose = None

        if localization_type == rawSensor:
            # TODO Part 3: subscribe to the position sensor topic (Odometry)
            # Create the Odom subscription with the associated callback.
            self.create_subscription(
                Odometry, "/odom", self.odom_callback, qos_profile=odom_qos
            )
        else:
            print("This type doesn't exist", sys.stderr)

    def odom_callback(self, pose_msg):
        # TODO Part 3: Read x,y, theta, and record the stamp
        # Log the X and Y position coordinates from the pose.
        # Also get the orientation as a quaternion, and convert to Euler angles to log with the timestamp in nanoseconds.
        self.pose = [
            pose_msg.pose.pose.position.x,
            pose_msg.pose.pose.position.y,
            euler_from_quaternion(
                [
                    pose_msg.pose.pose.orientation.x,
                    pose_msg.pose.pose.orientation.y,
                    pose_msg.pose.pose.orientation.z,
                    pose_msg.pose.pose.orientation.w,
                ]
            ),
            pose_msg.header.stamp,
        ]

        # Log the data
        self.loc_logger.log_values(
            [
                self.pose[0],
                self.pose[1],
                self.pose[2],
                Time.from_msg(self.pose[3]).nanoseconds,
            ]
        )

    def get_pose(self):
        return self.pose


# TODO Part 3
# Here put a guard that makes the node run, ONLY when run as a main thread!
# This is to make sure this node functions right before using it in decision.py
if __name__ == "__main__":
    # Initialize ROS.
    init()
    # Create the localization node.
    localization_node = Localization()
    # Spin the localization node.
    spin(localization_node)

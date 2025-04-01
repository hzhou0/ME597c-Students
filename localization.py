import sys
import time

import message_filters
from nav_msgs.msg import Odometry
from rclpy import init, spin
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time

from utilities import *

RAW_SENSORS = 0
PARTICLE_FILTER = 1


odom_qos = QoSProfile(reliability=2, durability=2, history=1, depth=10)


class localization(Node):
    def __init__(self, type_, loggerName="robotPose.csv", loggerHeaders=None):
        super().__init__("localizer")

        if loggerHeaders is None:
            loggerHeaders = [
                "odom_x",
                "odom_y",
                "odom_th",
                "odom_vx",
                "odom_yawrate",
                "pf_x",
                "pf_y",
                "pf_th",
                "stamp",
            ]
        self.loc_logger = Logger(loggerName, loggerHeaders)
        self.pose = None

        if type_ == RAW_SENSORS:
            self.initRawSensors()

        elif type_ == PARTICLE_FILTER:
            self.initParticleFilter()
        else:
            print("We don't have this type for localization", sys.stderr)
            return

        self.timelast = time.time()

    def initRawSensors(self):
        self.create_subscription(
            odom, "/odom", self.odom_callback, qos_profile=odom_qos
        )

    def initParticleFilter(self):
        self.odom_pose_sub = message_filters.Subscriber(
            self, Odometry, "/odom", qos_profile=odom_qos
        )
        self.pf_pose_sub = message_filters.Subscriber(
            self, Odometry, "/pf_pose", qos_profile=odom_qos
        )
        time_syncher = message_filters.ApproximateTimeSynchronizer(
            [self.odom_pose_sub, self.pf_pose_sub], queue_size=10, slop=0.1
        )
        time_syncher.registerCallback(self.odom_and_pf_pose_callback)

    def odom_and_pf_pose_callback(self, odom_msg: Odometry, pf_msg: Odometry):
        # Use the pf_msg to update the pose of the robot [x, y, theta, stamp]
        self.pose = [
            pf_msg.pose.pose.position.x,
            pf_msg.pose.pose.position.y,
            euler_from_quaternion(pf_msg.pose.pose.orientation),
            pf_msg.header.stamp,
        ]

        # log the values from the odom and the particle filter based on the headers
        # odom values: x, y, theta, vx, yawrate
        odom_values_list = [
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            euler_from_quaternion(odom_msg.pose.pose.orientation),
            odom_msg.twist.twist.linear.x,
            odom_msg.twist.twist.angular.z,
        ]
        # pf values: x, y, theta
        pf_values_list = [
            pf_msg.pose.pose.position.x,
            pf_msg.pose.pose.position.y,
            euler_from_quaternion(pf_msg.pose.pose.orientation),
        ]

        stamp = Time.from_msg(odom_msg.header.stamp).nanoseconds
        # Put all the values in a list
        values_to_log = odom_values_list + pf_values_list + [stamp]
        self.loc_logger.log_values(values_to_log)

    def odom_callback(self, pose_msg):
        self.pose = [
            pose_msg.pose.pose.position.x,
            pose_msg.pose.pose.position.y,
            euler_from_quaternion(pose_msg.pose.pose.orientation),
            pose_msg.header.stamp,
        ]

        stamp = Time.from_msg(pose_msg.header.stamp).nanoseconds
        # Put all the values in a list
        values_to_log = [
            pose_msg.pose.pose.position.x,
            pose_msg.pose.pose.position.y,
            euler_from_quaternion(pose_msg.pose.pose.orientation),
            pose_msg.twist.twist.linear.x,
            pose_msg.twist.twist.angular.z,
            0,
            0,
            0,
            stamp,
        ]
        self.loc_logger.log_values(values_to_log)

    def getPose(self):
        return self.pose


if __name__ == "__main__":
    init()

    LOCALIZER = localization()

    spin(LOCALIZER)

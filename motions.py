# Imports
import rclpy

from rclpy.node import Node

from utilities import Logger, euler_from_quaternion
from rclpy.qos import QoSProfile

# Import relevant libraries for sending commands to the robot (Twist), and for dealing with sensors and wheel encoders (Imu, LaserScan, Odometry).
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry

from rclpy.time import Time

# Types of motion for this lab.
CIRCLE = 0
SPIRAL = 1
ACC_LINE = 2
motion_types = ["circle", "spiral", "line"]


class motion_executioner(Node):
    def __init__(self, motion_type=0):
        super().__init__("motion_types")

        self.type = motion_type

        self.radius_ = 0.0

        self.successful_init = False
        self.imu_initialized = False
        self.odom_initialized = False
        self.laser_initialized = False

        # Create a publisher to send Twist commands to /cmd_vel topic with a queue size of 10.
        self.vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        # Initialize loggers.
        self.imu_logger = Logger(
            "imu_content_" + str(motion_types[motion_type]) + ".csv",
            headers=["acc_x", "acc_y", "angular_z", "stamp"],
        )
        self.imu_initialized = True

        self.odom_logger = Logger(
            "odom_content_" + str(motion_types[motion_type]) + ".csv",
            headers=["x", "y", "th", "stamp"],
        )
        self.odom_initialized = True

        self.laser_logger = Logger(
            "laser_content_" + str(motion_types[motion_type]) + ".csv",
            headers=["ranges", "angle_increment", "stamp"],
        )
        self.laser_initialized = True

        # Create a QoS profile that is reliable, has transient local durability, stores the last 10 messages.
        qos = QoSProfile(reliability=2, durability=2, history=1, depth=10)

        # TODO Part 5: Create below the subscription to the topics corresponding to the respective sensors
        # Create the IMU subscription with the associated callback.
        self.create_subscription(Imu, "/imu", self.imu_callback, qos_profile=qos)

        # ENCODER subscription
        # Create the ENCODER subscription with the associated callback.
        self.create_subscription(Odometry, "/odom", self.odom_callback, qos_profile=qos)

        # Create the LaserScan subscription with the associated callback.
        self.create_subscription(
            LaserScan, "/scan", self.laser_callback, qos_profile=qos
        )

        self.timer_call_count = 0
        # Create a timer with a 100 ms period.
        self.create_timer(0.1, self.timer_callback)

    # Callback function for IMU to log data.
    def imu_callback(self, imu_msg: Imu):
        # Log the X and Y components of the linear acceleration, and the angular velocity, along with the timestamp in nanoseconds.
        self.imu_logger.log_values(
            [
                imu_msg.linear_acceleration.x,
                imu_msg.linear_acceleration.y,
                imu_msg.angular_velocity.z,
                Time.from_msg(imu_msg.header.stamp).nanoseconds,
            ]
        )

    # Callback function for Odometry to log data.
    def odom_callback(self, odom_msg: Odometry):
        # Log the X and Y position coordinates from the pose.
        # Also get the orientation as a quaternion, and convert to Euler angles to log with the timestamp in nanoseconds.
        self.odom_logger.log_values(
            [
                odom_msg.pose.pose.position.x,
                odom_msg.pose.pose.position.y,
                euler_from_quaternion(
                    [
                        odom_msg.pose.pose.orientation.x,
                        odom_msg.pose.pose.orientation.y,
                        odom_msg.pose.pose.orientation.z,
                        odom_msg.pose.pose.orientation.w,
                    ]
                ),
                Time.from_msg(odom_msg.header.stamp).nanoseconds,
            ]
        )

    # Callback function for LaserScan to log data.
    def laser_callback(self, laser_msg: LaserScan):
        # Log the range data and angular increment along with the timestamp in nanoseconds.
        self.laser_logger.log_values(
            [
                laser_msg.ranges.tolist(),
                laser_msg.angle_increment,
                Time.from_msg(laser_msg.header.stamp).nanoseconds,
            ]
        )

    def timer_callback(self):
        if self.odom_initialized and self.laser_initialized and self.imu_initialized:
            self.successful_init = True

        if not self.successful_init:
            return

        cmd_vel_msg = Twist()

        if self.type == CIRCLE:
            cmd_vel_msg = self.make_circular_twist()

        elif self.type == SPIRAL:
            cmd_vel_msg = self.make_spiral_twist()

        elif self.type == ACC_LINE:
            cmd_vel_msg = self.make_acc_line_twist()

        else:
            print("type not set successfully, 0: CIRCLE 1: SPIRAL and 2: ACCELERATED LINE")
            raise SystemExit

        self.vel_publisher.publish(cmd_vel_msg)

    # TODO Part 4: Motion functions: complete the functions to generate the proper messages corresponding to the desired motions of the robot

    # Function to generate the circular twist.
    def make_circular_twist(self):
        msg = Twist()
        # To move in a circle, set the robot to have a constant linear velocity in the x-direction, and a constant angular velocity.
        msg.linear.x = 1.0
        msg.angular.z = 2.0
        return msg

    # Function to generate the sprial twist.
    def make_spiral_twist(self):
        msg = Twist()
        # To make a spiral twist, set the robot to have an increasing linear velocity in the x-direction, and a constant angular velocity.
        msg.linear.x = self.timer_call_count * 0.002
        msg.angular.z = 2.0
        # Up to a limit of 500 times, increment the number of times this timer has been called to increase the linear velocity.
        if self.timer_call_count < 500:
            self.timer_call_count += 1
        return msg

    # Function to generate the straight line twist.
    def make_acc_line_twist(self):
        msg = Twist()
        # To go in a straight line, set to the robot to have a constant linear velocity in the x-direction.
        msg.linear.x = 1.0
        return msg


import argparse

if __name__ == "__main__":
    argParser = argparse.ArgumentParser(description="input the motion type")

    argParser.add_argument("--motion", type=str, default="circle")

    rclpy.init()

    args = argParser.parse_args()

    ME = None
    if args.motion.lower() == "circle":
        ME = motion_executioner(motion_type=CIRCLE)
    elif args.motion.lower() == "line":
        ME = motion_executioner(motion_type=ACC_LINE)
    elif args.motion.lower() == "spiral":
        ME = motion_executioner(motion_type=SPIRAL)
    else:
        print(f"we don't have {args.motion.lower()} motion type")

    try:
        rclpy.spin(ME)
    except KeyboardInterrupt:
        print("Exiting")

#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class SafetyNode(Node):
    """
    ROS2 node for emergency braking.

    Subscribes to laser scan data and odometry info to detect obstacles and calculate time to collision (TTC).
    If we calculate a TTC to some object that is under the specified time before breaking (TBB), the node will publish
    a command to stop the ego car to prevent collision.

    Subscribing to:
        scan (type sensor_msgs.msg.LaserScan)
        ego_racecar/odom (type nav_msgs.msg.Odometry)

    Publishing to:
        drive (type ackermann_msgs.msg.AckermannDriveStamped)
    """

    def __init__(self):
        """
        Called at initialization of the class.

        Calls init from parent class to initialize node with name 'safety_node', and initializes the velocity,
        minimum TTC before breaking, subscriptions, and publisher members.
        """
        super().__init__('safety_node')

        self.velocity = 0.0
        self.tbb = 0.9

        self.create_subscription(LaserScan, 'scan', self.scan_callback, 1)
        self.create_subscription(Odometry, 'ego_racecar/odom', self.odom_callback, 1)

        self.pub_drive = self.create_publisher(AckermannDriveStamped, 'drive', 1)

    def odom_callback(self, odom_msg):
        """
        Called every time a new message is received from ego_racecar/odom topic.

        Extracts velocity of car in forward (positive) and backward (negative) direction.

        From odom_msg:
            odom_msg.twist.twist.linear.x (float) (varies)
        """
        self.velocity = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        """
        Called every time a new message is received from scan topic.

        Calculates TTC between ego car and the nearby areas in the environment. When theta = 0 rad, then
        the corresponding area is directly in front of the car. LaserScan gives a total view of about 6pi/4 radians
        around the car (about 3pi/4 to one side and 3pi/4 to the other).

        First, the valid distances are extracted and the angles for each are calculated.
        Next, the range rates are calculated (the max range rate is always at np.cos(0), as that is directly in
        front of the car). Finally, the time to collision for each valid nearby area is calculated (ttcs), and
        if the smallest value in the ttcs array is less than self.tbb, then signal the ego car to stop.

        From scan_msg:
            scan_msg.ranges (array) (varies)
            scan_msg.range_min (float) = 0.0
            scan_msg.range_max (float) = 30.0
            scan_msg.angle_min (float) ~= -2.35 ~= -3pi/4
            scan_msg.angle_max (float) ~= 2.35 ~= 3pi/4
            scan_msg.angle_increment (float) ~= 0.00435

        range_rate_min: The minimum value any range_rate can be to calculate ttcs. Removes problems due to dividing
                        by numbers very close to zero.
        """
        ranges = np.array(scan_msg.ranges)
        range_min = scan_msg.range_min
        range_max = scan_msg.range_max
        angle_min = scan_msg.angle_min
        angle_inc = scan_msg.angle_increment
        range_rate_min = 1e-4

        ranges = np.where((ranges < range_min) | (ranges > range_max) | np.isnan(ranges) | np.isinf(ranges), 0, ranges)
        theta_idxs = np.arange(len(ranges))[ranges != 0]
        thetas = angle_min + angle_inc * theta_idxs
        range_rates = self.velocity * np.cos(thetas)
        ttcs = ranges[ranges != 0][range_rates > range_rate_min]/range_rates[range_rates > range_rate_min]

        if ttcs.size > 0 and ttcs.min() < self.tbb:
            ack = AckermannDriveStamped()
            ack.drive.speed = 0.0
            self.pub_drive.publish(ack)
            print(f'Emergency Brake Activated (due to TTC = {ttcs.min():.4f} secs)')


def main(args=None):
    """
    Called when script is ran.

    Initializes Ros2 client library, creates instance of SafetyNode, keeps the node alive,
    and when the node is no longer in use, destroys the node and shuts down the Ros2 client library.
    """
    rclpy.init(args=args)
    safety_node = SafetyNode()
    print(f'Safety Node Initialized')
    rclpy.spin(safety_node)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except rclpy.ROSInterruptException:
        pass

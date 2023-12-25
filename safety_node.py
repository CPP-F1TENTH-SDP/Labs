#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.Ack = AckermannDriveStamped()
        self.odm = Odometry()
        self.speed = 0
        self.TBC = 0.7 #Time Before Collison is 0.5 s
        # TODO: create ROS subscribers and publishers.

        self.sub_scan = self.create_subscription(LaserScan, 'scan', self.scan_callback, 1)
        self.sub_odom = self.create_subscription(Odometry, 'ego_racecar/odom', self.odom_callback, 1)
        self.pub_drive = self.create_publisher(AckermannDriveStamped, 'drive', 1)
        
    def odom_callback(self, odom_msg):
        # TODO: update current speed
        # only need x
        # y and z dont have values
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        # TTC = range / max(range_rate, 0)
        # range (Note: values < range_min or > range_max should be discarded)
        # Don't forget to deal with infs or nans in your arrays
        # range_rate = speed * cos(theta)
        #angle min=-2.35 max=2.35
        
        range_temp = np.array([], dtype=np.float32)
        range_temp = scan_msg.ranges
        ranges = np.array([], dtype=np.float32)
        theta = np.array([], dtype=np.float32)
        #error_ranges = np.array([], dtype=np.float32)
        #error_theta = np.array([], dtype=np.float32)
        range_rate = np.array([], dtype=np.float32)
        self.TTC = np.array([], dtype=np.float32)

        range_min = scan_msg.range_min
        range_max = scan_msg.range_max + 0.1
        range_nan = np.isnan(range_temp)
        range_inf = np.isinf(range_temp)
        
        for i in range(len(range_temp)): 
            if range_temp[i] < range_min or range_temp[i] > range_max or range_nan[i] == True or range_inf[i] == True:
                pass
            else:
                ranges = np.append(ranges, [range_temp[i]]) # m
                theta = np.append(theta, [scan_msg.angle_min + i * scan_msg.angle_increment]) # radians
        
        range_rate = np.dot(self.speed, np.cos(theta)) # m/s
        range_rate[range_rate < 0] = 0 # max(range_rate, 0)

        #self.TTC = np.divide(ranges, range_rate, out=np.zeros_like(ranges), where=range_rate!=0) # s
        with np.errstate(divide = 'ignore', invalid = 'ignore'): # this is a better way to divide when you need to divide by 0
            self.TTC = np.true_divide(ranges, range_rate)
            self.TTC[self.TTC == np.inf] = 0
            self.TTC = np.nan_to_num(self.TTC)
            
        self.TTC[self.TTC == 0] = 100 #this is to ignore all valuse of zero without removing them from the array
        
        # TODO: publish brake message and publish controller bool
        if np.min(self.TTC) <= self.TBC: 
            self.Ack.drive.speed = 0.0 
            self.pub_drive.publish(self.Ack)
            print("stop")

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    try:
        main()
    except rclpy.ROSInterruptException:
        pass
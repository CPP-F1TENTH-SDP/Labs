#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
#atexit module in Python provides a simple interface to register functions to be called when a program is closing down. In the provided code, the shutdown() function is 
#called to close the file before the node shuts down.This ensures that the file is properly closed and any buffered data is written to disk before the program exits.
#Also, without the atexit module, there's a risk that the file might not be closed properly if the program is terminated unexpectedly or if there are any exceptions that cause the program to exit abruptly.
import atexit
#tf2_ros is used to handle transformations, specifically for converting quaternions to Euler angles.
import tf2_ros
#The os.path.expanduser() function is used to expand the ~ character in a file path to the user's home directory.
#Also, unlike Unix shell, Python does not do any automatic path expansions, therefore this call is required.
#this also allows the code to be more platform-independent because it automatically resolves the home directory regardless of the operating system being used.
from os.path import expanduser
#An addition to the os.path module, just allows the creation of a time stamp to represent the year, month, etc.
from time import gmtime, strftime
#The numpy.linalg module provides a variety of functions for linear algebra operations. In particular, numpy.linalg.norm() computes the norm of a vector or matrix.
#The included "as LA" is just an abbreviation to simplify the module when calling it in the code as seen later on with LA.norm(np.array([....
from numpy import linalg as LA
#The TransformStamped module is necessary because the code involves working with coordinate frame transformations.
#TransformStamped messages are commonly used to represent transformations between different coordinate frames. These messages contain information such as translation (x, y, z) and rotation (quaternion) 
from geometry_msgs.msg import TransformStamped
#Allows the acquisition of odometry data to estimate a "robots" pose based on its motion. Is necesary for Localization and Mapping
from nav_msgs.msg import Odometry

home = expanduser('~')
file = open(strftime(home+'/rcws/logs/wp-%Y-%m-%d-%H-%M-%S', gmtime())+'.csv', 'w')

def save_waypoint(data):
    quaternion = np.array([
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w
    ])
    #Where we use tf2_ros, why we needed to import it in the first place.
    euler = tf2_ros.transformations.euler_from_quaternion(quaternion)
    speed = LA.norm(np.array([
        data.twist.twist.linear.x,
        data.twist.twist.linear.y,
        data.twist.twist.linear.z
    ]), 2)
    if data.twist.twist.linear.x > 0.:
        print(data.twist.twist.linear.x)

    file.write('%f, %f, %f, %f\n' % (
        data.pose.pose.position.x,
        data.pose.pose.position.y,
        euler[2],
        speed
    ))

def shutdown():
    file.close()
    print('Goodbye')

class WaypointsLogger(Node):
    def __init__(self):
        super().__init__('waypoints_logger')
        self.create_subscription(
            Odometry,
            'pf/pose/odom',
            self.save_waypoint_callback,
            10
        )

    def save_waypoint_callback(self, data):
        save_waypoint(data)

def main(args=None):
    rclpy.init(args=args)
    print('Saving waypoints...')
    waypoints_logger_node = WaypointsLogger()
    try:
        rclpy.spin(waypoints_logger_node)
    except KeyboardInterrupt:
        pass
    waypoints_logger_node.destroy_node()
    rclpy.shutdown()
    atexit.register(shutdown)

if __name__ == '__main__':
    main()

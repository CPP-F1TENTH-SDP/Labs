#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
import numpy as np
from math import sin, cos
import atexit
import tf2_ros
from geometry_msgs.msg import Quaternion
from time import gmtime, strftime
from numpy import linalg as LA
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert Euler angles to quaternion.

    Parameters:
        roll: float, angle in radians around the x-axis
        pitch: float, angle in radians around the y-axis
        yaw: float, angle in radians around the z-axis

    Returns:
        numpy array: quaternion [x, y, z, w]
    """
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)

    qw = cy * cp * cr + sy * sp * sr
    qx = cy * cp * sr - sy * sp * cr
    qy = sy * cp * sr + cy * sp * cr
    qz = sy * cp * cr - cy * sp * sr

    return np.array([qx, qy, qz, qw])

class WaypointsLogger(Node):

    def __init__(self):
        super().__init__('waypoints_logger')
        self.subscription = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.save_waypoint,
            10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        #change path if needed
        self.path = '/sim_ws/src/pure_pursuit'
        log_dir = strftime(self.path + '/rcws/logs/%Y-%m-%d-%H-%M-%S', gmtime())
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        file_path = strftime(log_dir + '/wp-%Y-%m-%d-%H-%M-%S.csv', gmtime())
        self.file = open(file_path, 'w')
        self.get_logger().info('Saving waypoints...')

    def save_waypoint(self, data):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'ego_racecar/base_link', rclpy.time.Time())
            quaternion = np.array([transform.transform.rotation.x,
                                   transform.transform.rotation.y,
                                   transform.transform.rotation.z,
                                   transform.transform.rotation.w])
            euler = quaternion_from_euler(quaternion[0], quaternion[1], quaternion[2])
            speed = LA.norm(np.array([data.twist.twist.linear.x,
                                      data.twist.twist.linear.y,
                                      data.twist.twist.linear.z]), 2)
            if data.twist.twist.linear.x > 0.:
                self.get_logger().info('Linear Speed: %f' % data.twist.twist.linear.x)

            self.file.write('%f, %f, %f, %f\n' % (transform.transform.translation.x,
                                                   transform.transform.translation.y,
                                                   euler[2],
                                                   speed))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error('Error in TF lookup: %s' % str(e))

    def shutdown(self):
        self.file.close()
        self.get_logger().info('Goodbye')
    

def main(args=None):
    rclpy.init(args=args)

    waypoints_logger = WaypointsLogger()

    atexit.register(waypoints_logger.shutdown)

    rclpy.spin(waypoints_logger)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

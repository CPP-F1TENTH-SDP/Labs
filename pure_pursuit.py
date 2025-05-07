#!/usr/bin/env python3

import rclpy
import numpy as np
import csv
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class PurePursuitNode(Node):
    
    def __init__(self):
        super().__init__('pure_pursuit')
        csv_path = '/sim_ws/src/nodes/poses.csv'
        #Subscribers and Publishers
        self.pub_drive = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.planned_path = self.create_publisher(Path, '/planned_path', 10)
        self.next_point = self.create_publisher(Marker, '/next_point', 10)
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)

        self.x_vals = np.array([])
        self.y_vals = np.array([])

        self.lookahead_dist = 1.2
        self.flag = False
        self.current_index = 0
        
        # Create initial Path
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

        try:
            with open(csv_path, 'r') as csv_file:
                reader = csv.reader(csv_file)
                next(reader, None) # skips the first line of x,y,z
                
                for row in reader:
                    if len(row) < 3:
                        continue
                    try:
                        x, y, z = map(float, row[:3])
                    except ValueError:
                        self.get_logger().warn(f"Skipping Row Due to Non-Numeric Row: {row}")
                        continue

                    # z position is recorded but not used in this algorithm due to the assumption of a 2-D plane
                    ps = PoseStamped()
                    ps.header.frame_id = self.path_msg.header.frame_id
                    ps.pose.position.x = x
                    ps.pose.position.y = y
                    ps.pose.position.z = z

                    self.x_vals = np.append(self.x_vals, x)
                    self.y_vals = np.append(self.y_vals, y)

                    ps.pose.orientation.w = 1.0
                    self.path_msg.poses.append(ps)


        except FileNotFoundError:
            self.get_logger().error(f'CSV file not found: {csv_path}')
            rclpy.shutdown()
            return
        
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        for ps in self.path_msg.poses:
            ps.header.stamp = self.get_clock().now().to_msg()
        self.planned_path.publish(self.path_msg)
        self.get_logger().info("Planned Path Published")


    def odom_callback(self, msg: Odometry):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # Calculations to find the angle/heading that the car is currently moving at
        # Formula can be found here: 
        # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#:~:text=A%20direct%20formula%20for%20the,disadvantages%20near%20zero%20and%20one.
        term1 = 2 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z + msg.pose.pose.orientation.x * msg.pose.pose.orientation.y)
        term2 = 1.0 - 2.0 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y + msg.pose.pose.orientation.z * msg.pose.pose.orientation.z)
        current_angle = np.arctan2(term1, term2)

        # One time check to find out where the car is on the path initially
        if self.flag is False:
            shortest_dist = 100.0 # Arbitrary Large Number
            
            # Finds the index of the closest point to the car
            for i in range(len(self.x_vals)):
                dist_sq = (self.x_vals[i] - current_x)**2 + (self.y_vals[i] - current_y)**2
                if dist_sq < shortest_dist:
                    shortest_dist = dist_sq
                    self.current_index = i
            self.flag = True

        # While there is a next point in range continue to move up the path
        while ((self.x_vals[self.current_index] - current_x)**2 + (self.y_vals[self.current_index] - current_y)**2)**0.5 < self.lookahead_dist:
            self.current_index += 1
        if self.current_index >= len(self.x_vals) - 1:
            self.current_index = 0

        # Distance between the car and the next point on the path
        distance = ((self.x_vals[self.current_index] - current_x)**2 + (self.y_vals[self.current_index] - current_y)**2)**0.5
        # Angle required to reach next point
        lookahead_angle = np.arctan2(self.y_vals[self.current_index] - current_y, self.x_vals[self.current_index] - current_x)
        # Find the y distance between the car and next point
        del_y = distance * np.sin(lookahead_angle - current_angle)
        # Equation given from F1Tenth Lab 6 Pure Pursuit 
        angle = 2.0 * del_y / (distance**2)

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type   = Marker.SPHERE
        marker.action = Marker.ADD

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.pose.position.x = float(self.x_vals[self.current_index])
        marker.pose.position.y = float(self.y_vals[self.current_index])
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.next_point.publish(marker)

        _drive = AckermannDriveStamped()
        _drive.drive.steering_angle = angle

        if np.abs(angle) > (1.0 / 6.0) * np.pi:
            _drive.drive.speed = 1.67
        elif np.abs(angle) > (1.0 / 18.0) * np.pi:
            _drive.drive.speed = 3.67
        else:
            _drive.drive.speed = 7.5

        self.pub_drive.publish(_drive)


        pass


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
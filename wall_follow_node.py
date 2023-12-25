import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers
        self.sub_scan = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 1)
        self.pub_drive = self.create_publisher(AckermannDriveStamped, drive_topic, 1)

        # TODO: set PID gains
        self.kp = 0.255
        self.ki = 1.115
        self.kd = -0.028
        
        # TODO: store history
        self.error = 0
        self.integral = 0
        self.prev_error = 0
        self.velocity = 0.0

        # TODO: store any necessary values you think you'll need
        self.dist = 0.8 # distance you want from wall in meters
        self.lookahead_dist = 0.3 # future dist in meters used for calculations
        self.A_angle  = 50 * np.pi/180# an angle that is 0-70 degree away from b
        self.B_angle  = 90 * np.pi/180# -90 degree from right, 90 degree for left

    def get_range(self, range_data, angle_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. 

        Args:
            range_data: single range array from the LiDAR
            angle_data: single angle array from the LiDAR
            angle: desiered angle

        Returns:
            range: range measurement in meters at the closest angle

        """
        # As the angle in angle_data are not perfect, we can only choose the angles closest to what we want
        diff_array = np.absolute(angle_data - angle) #calculates difference of every value in angle_data to angle
        np.absolute(diff_array) # Calculates the absolute value element-wise so that negatives are not the min
        index = diff_array.argmin() #picks index of lowest value


        return range_data[index] 

    def get_error(self, range_data, angle_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            angle_data: single angle array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """
        
        #TODO:implement        
        A_dist = self.get_range(range_data, angle_data, self.A_angle)
        B_dist = self.get_range(range_data, angle_data, self.B_angle)
        C_dist = self.get_range(range_data, angle_data, 30 * np.pi/180)
        D_dist = self.get_range(range_data, angle_data, -30 * np.pi/180)
        theta = self.B_angle - self.A_angle
        
        alpha = np.arctan((A_dist * np.cos(theta) - B_dist) / (A_dist * np.sin(theta))) #alpha = arctan((A_dist*cos(theta)-B_dist)/(A_dist*sin(theta)))
        current_dist = B_dist * np.cos(alpha) #current_dist = B_dist*cos(alpha)
        error = dist - current_dist # prefered_dist - current_dist
        # If we simply use the current distance to the wall, we might end up turning too late, and the car may crash. 
        # Therefore, we must look to the future and project the car ahead by a certain lookahead distance
        future_error = dist - current_dist + self.lookahead_dist * np.sin(alpha) #future_error = dist - current_dist+lookahead_dist*sin(alpha)
        self.prev_error = C_dist -D_dist
        self.error = future_error
        self.integral = -self.velocity * np.sin(alpha)
        print("error")
        print(error)
        print(self.integral)
        print(self.prev_error)
        #print(theta)
        #print(alpha)
        #print(np.cos(theta))
        return future_error

    def pid_control(self, error):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error

        Returns:
            None
        """
        angle = 0.0 # in degrees

        # pid controller
        angle = self.kp * error + self.ki * self.integral + self.kd * self.prev_error 
        print("angle")
        print(angle)
        # velocity based on angle
        if np.abs(angle) < 11 * np.pi/180:
            self.velocity = 1.5 # velocity should be 1.5 m/s for 0-10 degrees
        elif np.abs(angle) < 21 * np.pi/180:
            self.velocity = 1.0 # velocity should be 1.0 m/s for 10-20 degrees
        else:
            self.velocity = 0.5 # velocity should be 0.5 m/s otherwise
        #velocity = 0.0
        # filled in drive messages to publish
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = -angle
        drive_msg.drive.steering_angle_velocity = 0.0 #Zero means change the steering angle as quickly as possible
        drive_msg.drive.speed = self.velocity 
        self.pub_drive.publish(drive_msg)

    def scan_callback(self, scan_msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        # range (Note: values < range_min or > range_max should be discarded)
        # Don't forget to deal with infs or nans in your arrays
        # range_rate = speed * cos(theta)
        # angle min=-2.35 max=2.35 in radians
        # angle incraments by something
        range_temp = np.array([], dtype=np.float32)
        range_temp = scan_msg.ranges
        range_data = np.array([], dtype=np.float32) # meters
        theta = np.array([], dtype=np.float32) # degrees

        range_min = scan_msg.range_min
        range_max = scan_msg.range_max + 0.1
        range_nan = np.isnan(range_temp)
        range_inf = np.isinf(range_temp)
        
        for i in range(len(range_temp)): 
            if range_temp[i] < range_min or range_temp[i] > range_max or range_nan[i] == True or range_inf[i] == True:
                pass
            else:
                range_data = np.append(range_data, [range_temp[i]]) # m
                theta = np.append(theta, [(scan_msg.angle_min + i * scan_msg.angle_increment)]) 

        # TODO: replace with error calculated by get_error()
        error = self.get_error(range_data, theta, self.dist) 
        
        self.pid_control(error) # TODO: actuate the car with PID


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except rclpy.ROSInterruptException:
        pass
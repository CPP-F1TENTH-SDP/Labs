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

        """
        The purpose of the proportional, is to have a large immediate reaction on the output to bring the process value close to the set point. 
        As the error becomes less, the influence of the proportional value on the output becomes less.
        """

        self.kp = 0.255 #Proportional Gain

        """
        The Integral is calculated by multiplying the I-Gain, by the error, then multiplying this by the cycle time of the controller 
        (how often the controller performs the PID calculation) and continuously accumulating this value as the “total integral”.
        """

        self.ki = -0.028 #Integral gain

        """
        The derivative is calculated by multiplying the D-Gain by the ramp rate of the process value. 
        The purpose of the derivative is to “predict” where the process value is going, and bias the output in the opposite direction of the 
        proportional and integral, to hopefully prevent the controller from over-shooting the set point if the ramp rate is to fast.
        """

        self.kd = 0.55 #Derivative Gain
        
        # TODO: store history
        self.error = 0
        self.integral = 0
        self.prev_error = 0
        self.velocity = 0.0

        # TODO: store any necessary values you think you'll need
        self.dist = 1.0 # distance you want from wall in meters
        """
        We cannot only use a current distance from an object because depending on the speed of the car and the distance
        it may be too late to turn so we utilize a distance ahead of the car to perform calculations
        """
        self.lookahead_dist = 0.3 # future dist in meters used for calculations
        self.A_angle  = np.radians(30) #The a-angle is anything between 0 to 70 degrees away from b, this is largely a calibration value
        self.B_angle  = np.radians(90) #The b-angle is 90 degrees from both the left and right sides of the car, this is also a calibration value

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

    def get_error(self, range_data, angle_data, dist): # Data is obtained from scan_callback method; refer to lab 2 for how this information is gathered.
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
        A_dist = self.get_range(range_data, angle_data, self.A_angle) #Calculate the distance from the wall utilizing the A-angle (We have chosen 50 degrees from the b-angle)
        B_dist = self.get_range(range_data, angle_data, self.B_angle) #Calculate the distance from the wall utilizing the B-angle (perpendicular from the car)
        C_dist = self.get_range(range_data, angle_data, np.radians(30))
        D_dist = self.get_range(range_data, angle_data, np.radians(-30))

        theta = self.B_angle - self.A_angle #Angle between the radar scan between our "B and A-Angles"
        
        alpha = np.arctan((A_dist * np.cos(theta) - B_dist) / (A_dist * np.sin(theta))) #Alpha angle is the angle between the B-Angle (perpendicular from car) 
                                                                                        #and the ray from the wall to the car **PERPENDICULAR TO THE WALL**
        
        current_dist = B_dist * np.cos(alpha) #We calculate the distance of the car perpendicular to the wall
        error = dist - current_dist # e(t) [error term] is calculated using our preferred distance to the wall (0.8 meters defined in the constructor) and subtracting the current distance
        
        # If we simply use the current distance to the wall, we might end up turning too late, and the car may crash. 
        # Therefore, we must look to the future and project the car ahead by a certain lookahead distance

        future_dist = current_dist + self.lookahead_dist * np.sin(alpha) #In case simulation fails return "current_dist" to "error" value
        #future_error = current_dist + self.lookahead_dist * np.sin(alpha) #Use this value in case the above fails

        #self.integral = C_dist - D_dist
        self.integral = self.dist - current_dist #Delete this if anything janky happens

        #self.error = future_error #Uncomment if code fails
        self.prev_error = -self.velocity * np.sin(alpha) #This is the calculation of the derivative value which determines the speed at which we approach the desired line

        #print(theta)
        #print(alpha)
        #print(np.cos(theta))
        return error #return value to "future_error" is necessary
        #return future_error #Use this if above fails

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
        # velocity based on angle
        if np.abs(angle) < np.radians(11):
            self.velocity = 1.5 # velocity should be 1.5 m/s for 0-10 degrees
        elif np.abs(angle) < np.radians(21):
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
        ranges_temp = np.array(scan_msg.ranges)
        range_min = scan_msg.range_min
        range_max = scan_msg.range_max
        angle_min = scan_msg.angle_min
        angle_inc = scan_msg.angle_increment
        range_rate_min = 1e-4

        ranges_temp = np.where((ranges_temp < range_min) | (ranges_temp > range_max) | np.isnan(ranges_temp) | np.isinf(ranges_temp), -1, ranges_temp)
        ranges = ranges_temp[ranges_temp != -1]
        theta_idxs = np.arange(len(ranges_temp))[ranges_temp != -1]
        thetas = angle_min + angle_inc * theta_idxs

        # TODO: replace with error calculated by get_error()
        error = self.get_error(ranges, thetas, self.dist) 
        
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

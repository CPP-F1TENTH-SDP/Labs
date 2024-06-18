import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        self.trim_value = 0 #self.desiredGapLengthMinimum // 2
        # TODO: Subscribe to LIDAR
        self.sub_scan = self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 1)
        
        # TODO: Publish to drive
        self.pub_drive = self.create_publisher(AckermannDriveStamped, drive_topic, 1)

        self.rejection_dist = 3
        self.bubble_dist = 2
        self.angle = 0

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        convolution_len = 3
        new_ranges = np.convolve(ranges, np.ones(convolution_len) / convolution_len, mode = 'same')
        #Convolution of the LiDAR array with an array of "ones" to smoothen out the signal

        new_ranges[new_ranges > self.rejection_dist] = 0
        #Rejection of values greater than a distance of a pre-defined value

        return new_ranges
    
    def get_angle_index(self, angle_data):
        """
        Simple helper to return the corresponding range measurement at a given angle. 

        Args:
            angle_data: single angle array from the LiDAR
            angle: desireed angle

        Returns:
            index of desired angle

        """
        # As the angle in angle_data are not perfect, we can only choose the angles closest to what we want
        index = np.argmin(np.abs(angle_data - self.angle)) # find the index closest to the desired angle

        return index 

    def find_closest_point(self, proc_ranges):
        """Find the closest point"""
        closest_index = np.argmin(proc_ranges)
        print("Closest Index:", closest_index)
        return closest_index

    def closest_point_bubble(self, proc_ranges, closest_point):
        """Draw a safety bubble around the closest point to our car"""

        counter = 0

        # Extract the coordinates of the closest point
        closest_index = closest_point

        # Define the radius of the safety bubble
        safety_radius = self.bubble_dist

        # Iterate through the LiDAR processed ranges
        for i in range(len(proc_ranges)):
            # Check if the current point is within the safety bubble radius of the closest point
            if abs(i - closest_index) <= safety_radius:
                # Mark this point as within the safety bubble by setting its range to 0
                counter = counter + 1
                proc_ranges[i] = 0  # or you can use another value to mark it differently

        print("Number of points inside of bubble: ", counter)
        return proc_ranges


    def find_max_gap(self, free_space_ranges):
        """Return the start index & end index of the max gap in free_space_ranges"""
        max_gap_start = 0
        max_gap_end = 0
        max_gap_length = 0
        current_start = 0
        current_length = 0

        for i in range(len(free_space_ranges)):
            if free_space_ranges[i] > 0:
                if current_length == 0:
                    current_start = i
                current_length += 1
            else:
                if current_length > max_gap_length:
                    max_gap_length = current_length
                    max_gap_start = current_start
                    max_gap_end = i - 1
                current_length = 0

        # Check again at the end of the loop
        if current_length > max_gap_length:
            max_gap_length = current_length
            max_gap_start = current_start
            max_gap_end = len(free_space_ranges) - 1

        print("Best Gap: ")
        print(max_gap_start)
        print(" - ")
        print(max_gap_end)
        return [max_gap_start, max_gap_end]
    
    
    def find_best_point(self, start_i, end_i, ranges):
        """Find the best point index within the specified range [start_i, end_i] in the 'ranges' array"""
        if start_i > end_i or start_i < 0 or end_i >= len(ranges):
            # Handle invalid range parameters
            return None  # or raise ValueError("Invalid range specified")

        # Extract the subarray of interest
        sub_ranges = ranges[start_i:end_i + 1]

        # Find the index of the point with the maximum value in the subarray
        best_point_index = np.argmax(sub_ranges) + start_i

        return best_point_index

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        #Process data for errors
        ranges_temp = np.array(data.ranges)
        range_min = data.range_min
        range_max = data.range_max + 1
        angle_min = data.angle_min
        angle_inc = data.angle_increment

        ranges_temp = np.where((ranges_temp < range_min) | (ranges_temp > range_max) | np.isnan(ranges_temp) | np.isinf(ranges_temp), -1, ranges_temp)
        ranges = ranges_temp[ranges_temp != -1]
        theta_idxs = np.arange(len(ranges_temp))[ranges_temp != -1]
        thetas = angle_min + angle_inc * theta_idxs

        proc_ranges = self.preprocess_lidar(ranges)

        # TODO:
        #Find closest point to LiDAR
        closest_point = self.find_closest_point(proc_ranges)
        #Eliminate all points inside 'bubble' (set them to zero) 
        proc_ranges = self.closest_point_bubble(proc_ranges, closest_point)

        #Find max length gap 
        max_length_gap_index = self.find_max_gap(proc_ranges)
        print("Max Gap Index 0:", max_length_gap_index[0])
        print("Max Gap Index 1:", max_length_gap_index[1])
        #Find the best point in the gap 
        best_point_index = self.find_best_point(max_length_gap_index[0], max_length_gap_index[1], ranges)

        # velocity based on angle
        velocity = 0.0
        if np.abs(thetas[best_point_index]) < 11 * np.pi/180:
            velocity = 1.5 # velocity should be 1.5 m/s for 0-10 degrees
        elif np.abs(thetas[best_point_index]) < 21 * np.pi/180:
            velocity = 1.0 # velocity should be 1.0 m/s for 10-20 degrees
        else:
            velocity = 0.5 # velocity should be 0.5 m/s otherwise
        #velocity = 0.0
        # #Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = thetas[best_point_index]
        drive_msg.drive.steering_angle_velocity = 0.0 #Zero means change the steering angle as quickly as possible
        drive_msg.drive.speed = velocity
        self.pub_drive.publish(drive_msg)
        #print("Angle Theta: ", thetas[best_point_index + 50])
        # print("stuff")
        # print(thetas[best_point_index + index_2])

def main(args=None):
    rclpy.init(args=args)
    print("GapFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

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

        # TODO: Subscribe to LIDAR
        self.sub_scan = self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 1)
        
        # TODO: Publish to drive
        self.pub_drive = self.create_publisher(AckermannDriveStamped, drive_topic, 1)

        self.bubble_dist = 2.0

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        ranges[ranges >= self.bubble_dist] = self.bubble_dist
        ranges[ranges < self.bubble_dist] = 0

        return ranges
    
    def get_angle_index(self, angle_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. 

        Args:
            angle_data: single angle array from the LiDAR
            angle: desiered angle

        Returns:
            index of desired angle

        """
        # As the angle in angle_data are not perfect, we can only choose the angles closest to what we want
        diff_array = np.absolute(angle_data - angle) #calculates difference of every value in angle_data to angle
        np.absolute(diff_array) # Calculates the absolute value element-wise so that negatives are not the min
        index = diff_array.argmin() #picks index of lowest value

        return index 

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        current_length = 0
        desiredGapLengthValue = 3
        scan_middle = len(free_space_ranges) / 2


        # Create Python lists to store gap indexes and lengths
        gap_indexes = []
        gap_indexes_length = []


        gap = [0, 0]  # Initialize gap as a Python list

        for i in range(len(free_space_ranges)):
            if free_space_ranges[i] == self.bubble_dist:  # Check if the value is 2 (assuming 2s represent free space)
                if current_length == 0:  # If this is the start of a new continuous set of 2s
                    gap[0] = i

                gap[1] = i  # Update the end index of the current continuous set of 2s
                    
                current_length += 1
                
                if (i == len(free_space_ranges) - 1):
                    if (current_length > desiredGapLengthValue):
                        gap_indexes.append(gap[:])  # Append a copy of gap to gap_indexes list
                        gap_indexes_length.append(current_length)

            else:
                
                if current_length > 0:  # If there was a continuous set of 2s
                    if (current_length > desiredGapLengthValue):
                        gap_indexes.append(gap[:])  # Append a copy of gap to gap_indexes list
                        gap_indexes_length.append(current_length)

                current_length = 0  # Reset the current length counter

        # Convert lists to NumPy arrays if needed
        gap_indexes = np.array(gap_indexes)
        gap_indexes_length = np.array(gap_indexes_length)

        # Calculate the mean between start and end indices of each gap
        gap_mean = np.mean(gap_indexes, axis=1)

        gap_mean = np.abs(gap_mean - scan_middle)
        chosen_mean = np.argmin(gap_mean)

        return gap_indexes[chosen_mean]
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        #index = np.argmax(ranges[start_i:end_i]) #finds the index that contains max value
        index = int((end_i - start_i) / 2 + start_i) #returnes the middle of the start and end index
        return index

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        #Process data for errors
        ranges_temp = np.array(data.ranges)
        range_min = data.range_min
        range_max = data.range_max
        angle_min = data.angle_min
        angle_inc = data.angle_increment
        range_rate_min = 1e-4

        ranges_temp = np.where((ranges_temp < range_min) | (ranges_temp > range_max) | np.isnan(ranges_temp) | np.isinf(ranges_temp), -1, ranges_temp)
        ranges = ranges_temp[ranges_temp != -1]
        theta_idxs = np.arange(len(ranges_temp))[ranges_temp != -1]
        thetas = angle_min + angle_inc * theta_idxs

        proc_ranges = self.preprocess_lidar(ranges)

        # TODO:
        #Find closest point to LiDAR

        #Eliminate all points inside 'bubble' (set them to zero) 

        #Find max length gap 
        max_lenght_gap_index = self.find_max_gap(proc_ranges)
        #Find the best point in the gap 
        best_point_index = self.find_best_point(max_lenght_gap_index[0], max_lenght_gap_index[1], ranges)
        # velocity based on angle
        velocity = 0.0
        if np.abs(thetas[best_point_index]) < 11 * np.pi/180:
            velocity = 1.5 # velocity should be 1.5 m/s for 0-10 degrees
        elif np.abs(thetas[best_point_index]) < 21 * np.pi/180:
            velocity = 1.0 # velocity should be 1.0 m/s for 10-20 degrees
        else:
            velocity = 0.5 # velocity should be 0.5 m/s otherwise
        
        # #Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = thetas[best_point_index]
        drive_msg.drive.steering_angle_velocity = 0.0 #Zero means change the steering angle as quickly as possible
        drive_msg.drive.speed = velocity
        self.pub_drive.publish(drive_msg)

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

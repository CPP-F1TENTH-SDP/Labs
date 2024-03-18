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
        """
        Preprocess the LiDAR scan array. Expert implementation includes:
            1. Setting each value to the mean over some window
            2. Rejecting high values (eg. > 3m)
        """
        try:
            ranges[ranges >= self.bubble_dist] = self.bubble_dist
        except Exception as e:
            print(f"Error occurred during LiDAR preprocessing: {e}")

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

        # As the angle in angle_data are not perfect, we can only choose the thetas closest to what we want
        diff_array = np.absolute(angle_data - angle)        # Calculates difference of every value in angle_data to angle
        np.absolute(diff_array)         # Calculates the absolute value element-wise so that negatives are not the min
        index = diff_array.argmin()     # Picks index of lowest value

        return index

    def find_max_gap(self,free_space_ranges):
        # Convert the input list to a NumPy array
        free_space_ranges = np.array(free_space_ranges)
        
        # Value is equal to self.bubble_dist
        bubble_eq = np.where(free_space_ranges == self.bubble_dist)[0]
        
        # Calculate the gaps between consecutive bubble indices
        gap_lengths = np.diff(bubble_eq)
        
        # Find the index of the maximum gap
        max_gap_index = np.argmax(gap_lengths)
        
        # Compute the start and end indices of the max gap
        start_index = bubble_eq[max_gap_index]
        end_index = bubble_eq[max_gap_index + 1]
        
        print(max_gap_index)  # Print the max_gap_index
        return np.array([start_index, end_index])

    def find_best_point(self, start_i, end_i, ranges):
        """
        Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        #index = np.argmax(ranges[start_i:end_i])        # Finds the index that contains furthest value
        index = int((end_i - start_i) / 2 + start_i)    # Returns the middle of the start and end index
        return index

    def lidar_callback(self, data):
        # Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message

        # Process data for errors
        ranges_temp = np.array(data.ranges) # Generate a temporary array of range values from the scan messages
        range_min = data.range_min
        range_max = data.range_max
        angle_min = data.angle_min
        angle_inc = data.angle_increment

        ranges_temp = np.where((ranges_temp < range_min) | (ranges_temp > range_max) | np.isnan(ranges_temp) | np.isinf(ranges_temp), -1, ranges_temp) 
        ranges = ranges_temp[ranges_temp != -1]
        theta_idxs = np.arange(len(ranges_temp))[ranges_temp != -1]
        thetas = angle_min + angle_inc * theta_idxs

        proc_ranges = self.preprocess_lidar(ranges)

        # Limit lidar ranges
        index_1 = self.get_angle_index(thetas, np.radians(90))
        index_2 = self.get_angle_index(thetas, np.radians(-90))

        # Find closest point to LiDAR

        # Eliminate all points inside 'bubble' (set them to zero) 

        # Find max length gap 
        max_length_gap_index = self.find_max_gap(proc_ranges[index_2:index_1])

        # Find the best point in the gap 
        best_point_index = self.find_best_point(max_length_gap_index[0], max_length_gap_index[1], ranges)

        # Velocity based on angle
        velocity = 0.0
        if np.abs(thetas[best_point_index + index_2]) < np.radians(11):
            velocity = 1.5          # Velocity should be 1.5 m/s for 0-10 degrees
        elif np.abs(thetas[best_point_index + index_2]) < np.radians(21):
            velocity = 1.0          # Velocity should be 1.0 m/s for 10-20 degrees
        else:
            velocity = 0.5          # Velocity should be 0.5 m/s otherwise

        # Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = thetas[best_point_index + index_2]
        drive_msg.drive.steering_angle_velocity = 0.0           # Zero means change the steering angle as quickly as possible
        drive_msg.drive.speed = velocity
        self.pub_drive.publish(drive_msg)

        print("stuff")
        print(thetas[best_point_index + index_2])
        

def main(args=None):
    rclpy.init(args=args)
    print("GapFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

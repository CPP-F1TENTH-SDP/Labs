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
        max_gap_index = np.array([0,0])
        temp = np.array([0,0])
        for i in range(len(free_space_ranges)-1):
            j = i+1
            
            if free_space_ranges[0] == self.bubble_dist:
                pass
            elif free_space_ranges[i] != self.bubble_dist and free_space_ranges[j] == self.bubble_dist:
                temp[0] = j
            elif free_space_ranges[i] == self.bubble_dist and free_space_ranges[j] == self.bubble_dist:
                pass
            elif free_space_ranges[i] == self.bubble_dist and free_space_ranges[j] != self.bubble_dist:
                temp[1] = i
                if temp[1]-temp[0] > max_gap_index[1]-max_gap_index[0]:
                    max_gap_index = temp
            elif free_space_ranges[-1] == self.bubble_dist:
                temp[1] = j
                if temp[1]-temp[0] > max_gap_index[1]-max_gap_index[0]:
                    max_gap_index = temp
        print(max_gap_index)
        return max_gap_index
    
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
        range_temp = np.array([], dtype=np.float32)
        range_temp = data.ranges
        ranges = np.array([], dtype=np.float32) # meters
        angles = np.array([], dtype=np.float32) # radians

        range_min = data.range_min
        range_max = data.range_max + 0.1
        range_nan = np.isnan(range_temp)
        range_inf = np.isinf(range_temp)
        
        for i in range(len(range_temp)): 
            if range_temp[i] < range_min or range_temp[i] > range_max or range_nan[i] == True or range_inf[i] == True:
                pass
            else:
                ranges = np.append(ranges, [range_temp[i]]) # m
                angles = np.append(angles, [(data.angle_min + i * data.angle_increment)]) # radians 
        
        proc_ranges = self.preprocess_lidar(ranges)
        
        #limit lidar ranges 
        index_1 = self.get_angle_index(angles, 90 * np.pi/180)
        index_2 = self.get_angle_index(angles, -90 * np.pi/180)

        # TODO:
        #Find closest point to LiDAR

        #Eliminate all points inside 'bubble' (set them to zero) 

        #Find max length gap 
        max_lenght_gap_index = self.find_max_gap(proc_ranges[index_2:index_1])
        #Find the best point in the gap 
        best_point_index = self.find_best_point(max_lenght_gap_index[0], max_lenght_gap_index[1], ranges)
        # velocity based on angle
        velocity = 0.0
        if np.abs(angles[best_point_index + index_2]) < 11 * np.pi/180:
            velocity = 1.5 # velocity should be 1.5 m/s for 0-10 degrees
        elif np.abs(angles[best_point_index + index_2]) < 21 * np.pi/180:
            velocity = 1.0 # velocity should be 1.0 m/s for 10-20 degrees
        else:
            velocity = 0.5 # velocity should be 0.5 m/s otherwise
        
        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angles[best_point_index + index_2]
        drive_msg.drive.steering_angle_velocity = 0.0 #Zero means change the steering angle as quickly as possible
        drive_msg.drive.speed = velocity
        self.pub_drive.publish(drive_msg)

        print("stuff")
        print(angles[best_point_index + index_2])
        

def main(args=None):
    rclpy.init(args=args)
    print("GapFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

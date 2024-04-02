#out of date, better version in braking_test.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import pandas as pd

LIDAR_TOPIC_NAME = '/scan'
class LidarStopFunction(Node):

    def _init_lidar_subscription(self):
        self.laser_subscriber = self.create_subscription(LaserScan, LIDAR_TOPIC_NAME, self.lidar_callback, self.QUEUE_SIZE)

        # Lidar info                                                                                                                                                                
        self.lidar_properties_set = False
        self.default_viewing_angle = 360
        self.default_front_degree_angle = 0
        self.default_right_degree_angle = 90
        self.default_left_degree_angle = 270
        self.range_max = None
        self.range_min = None
        self.num_scans = None
        self.angle_increment = None
        self.angle_min_radians = None
        self.angle_max_radians = None
        self.scan_ranges = None
        self.formatted_scan_data = "Empty"
 # ROS parameters (setttng default values)                                                                                                                                   
        self.declare_parameters(
            namespace='',
            parameters=[
                ('viewing_angle', self.default_viewing_angle),
                ('front_degree_angle', self.default_front_degree_angle),
                ('right_degree_angle', self.default_right_degree_angle),
                ('left_degree_angle', self.default_left_degree_angle),
                ('live_laser_feed', 0)
            ])

    def lidar_callback(self, data):
        if not self.lidar_properties_set:
            self.range_max = data.range_max
            self.range_min = data.range_min
            self.num_scans = len(np.array(data.ranges))
            self.angle_increment = data.angle_increment
            self.angle_min_radians = data.angle_min
            self.angle_max_radians = data.angle_max
            self.lidar_properties_set = True

            scan_ranges = np.array(data.ranges)
            self.scan_ranges = np.nan_to_num(scan_ranges, neginf=self.range_max, posinf=self.range_max, nan=self.range_max)


    def stop_drive_command(self):
        # Stop the vehicle                                                                                                                                                          
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 0.0
        drive_msg.drive.steering_angle = 0.0
        self.drive_pub.publish(drive_msg)

#!/usr/bin/env python3
import time
import rclpy
from std_srvs.srv import Empty
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import numpy as np
import cv2
from global_localizer import kidnap_solver as ks
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import yaml

class LaserScanFilter(Node):

    def __init__(self):
        super().__init__('global_localizer_node')

        self.max_range = 8.0  # max range of lidar in meters
        self.resolution = 0.05  # 5 cm per pixel
        self.image_size = int((2 * self.max_range) / self.resolution)  # image width and height in pixels
        self.origin_offset = int(self.max_range / self.resolution)  # origin offset in pixels

        self.min_distance = None
        self.scan_image = None
        self.map_image = None
        self.config_data = None

        self.scan_topic = "/scan" # default scan topic
        self.map_file_path = ""

        self.max_iterations = 30
        self.stop_search_threshold_f1 = 50
        
        self.lidar_max_range = 8.0 
        self.map_resolution = 0.05 # m/px

        self.load_parameters()
        self.load_map_file()
        
        self.subscription = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )

        self.srv = self.create_service(Empty, 'global_localization_srv', self.global_localization_callback)
        self.get_logger().info('Global Localization Service is ready.')


    def scan_callback(self, msg: LaserScan):
        # Initialize a black image
        image = np.zeros((self.image_size, self.image_size), dtype=np.uint8)
        
        min_distance = math.inf
        # Convert lidar points to pixels and draw on the image
        for i, range_val in enumerate(msg.ranges):
            if 0 < range_val < self.max_range:  # Ignore invalid or out-of-range values
                angle = msg.angle_min + i * msg.angle_increment
                # Convert polar coordinates (range, angle) to Cartesian (x, y)
                x = range_val * np.cos(angle)
                y = range_val * np.sin(angle)
                
                # Convert from meters to pixels
                px = int((x / self.resolution) + self.origin_offset)
                py = int((y / self.resolution) + self.origin_offset)
                min_distance = min(min_distance, range_val)
                # Draw the point on the image as a white circle with radius 1
                cv2.circle(image, (px, py), radius=1, color=255, thickness=-1)

        #print(f"Min distance: {min_distance}")

        self.scan_image = image.copy()
        self.min_distance = min_distance

    def load_config_file(self):
        package_share_directory = get_package_share_directory('global_localizer')
        yaml_file_path = Path(package_share_directory) / 'config' / 'config.yaml'
        try:
            with open(yaml_file_path, 'r') as file:
                self.config_data = yaml.safe_load(file)
        except Exception as e:
            self.get_logger().error(f"Error reading YAML file: {e}")
            exit()

    def load_parameters(self):
        self.load_config_file()
        self.scan_topic = self.config_data['scan_topic']
        self.map_file_path = self.config_data['map_file_path']

        self.max_iterations = self.config_data['max_iterations']
        self.stop_search_threshold_f1 = self.config_data['stop_search_threshold_f1']

        self.lidar_max_range = self.config_data['lidar_max_range']
        self.map_resolution = self.config_data['map_resolution']

        self.get_logger().info(f"Scan topic: {self.scan_topic}")
        self.get_logger().info(f"Map file path: {self.map_file_path}")
        self.get_logger().info(f"Max iterations: {self.max_iterations}")
        self.get_logger().info(f"Stop search threshold: {self.stop_search_threshold_f1}")
        self.get_logger().info(f"Lidar max range: {self.lidar_max_range}")
        self.get_logger().info(f"Map resolution: {self.map_resolution}")

    def load_map_file(self):
        map_file_path = self.map_file_path
        self.get_logger().info(f"Map file path: {map_file_path}")

        if Path(map_file_path).exists():
            map_image = cv2.imread(map_file_path, cv2.IMREAD_GRAYSCALE)
            self.map_image = map_image
        else:
            self.get_logger().error(f"Map file not found: {map_file_path}")
            exit()

    def global_localization_callback(self, request, response):
        self.get_logger().info('Global Localization Service triggered.')
        if self.min_distance is not None:
            self.get_logger().info('Found last laser-scan data. Matching in progress....')
            
            # Solve the kidnap problem
            ks.solve_kidnap(self.scan_image, self.map_image, self.min_distance, map_resolution = self.map_resolution,\
                             max_iterations = self.max_iterations, stop_search_threshold = self.stop_search_threshold_f1\
                                , lidar_range = self.lidar_max_range)
        else:
            self.get_logger().error('No laser-scan data found. Please make-sure there are laser scans available, and retry.')
        return response

def main(args=None):
    print("\n\n** Starting the localization service!!**\n\n")
    rclpy.init(args=args)
    laser_scan_filter = LaserScanFilter()
    rclpy.spin(laser_scan_filter)
    laser_scan_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


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

class LaserScanFilter(Node):

    def __init__(self):
        super().__init__('global_localizer_node')

        self.max_range = 8.0  # max range of lidar in meters
        self.resolution = 0.05  # 5 cm per pixel
        self.image_size = int((2 * self.max_range) / self.resolution)  # image width and height in pixels
        self.origin_offset = int(self.max_range / self.resolution)  # origin offset in pixels

        self.min_distance = None
        self.scan_image = None

        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
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
        


    def global_localization_callback(self, request, response):
        self.get_logger().info('Global Localization Service triggered.')
        if self.min_distance is not None:
            self.get_logger().info('Found last laser-scan data. Matching in progress....')
            map_image = cv2.imread("/home/saad/pro_ws/src/global_localizer/map_image.png", cv2.IMREAD_GRAYSCALE)

            # Solve the kidnap problem
            ks.solve_kidnap(self.scan_image, map_image, self.min_distance)
        else:
            self.get_logger().error('No laser-scan data found. Please make-sure there are laser scans available, and retry.')
        return response

def main(args=None):
    print("\n\n** Starting the localization service!!**\n\n")

    timeout = 2 #secs
    for i in range(timeout, 0, -1):
        print(f"Starting in {i} seconds...")
        time.sleep(1)


    rclpy.init(args=args)
    laser_scan_filter = LaserScanFilter()
    rclpy.spin(laser_scan_filter)
    laser_scan_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


import cv2
import numpy as np
from global_localizer import scanner_simulator as s_sim

map_resolution = 0.05 #m/pixel
max_range = 8 #m

def get_scan_image(map_image_bw, position, map_resolution = map_resolution, max_range = max_range):
    
    canvas = cv2.cvtColor(map_image_bw.copy(), cv2.COLOR_GRAY2BGR)
    map_image_bw = map_image_bw
    scan_image = np.zeros((int(2 * max_range / map_resolution), int(2 * max_range / map_resolution)), dtype=np.uint8)
    scanner_points = s_sim.get_lidar_points(map_image_bw, position[0], position[1], add_noise=True)
    scanner_points = np.array(scanner_points)
    cv2.circle(canvas, (position[0], position[1]), 4, (255, 0, 0), -1)

    
    for point in scanner_points:
        x = int(point[0] + max_range / map_resolution)
        y = int(point[1] + max_range / map_resolution)
        cv2.circle(scan_image, (x, y), 1, (255, 255, 255), -1)
        cv2.circle(canvas, (point[0]+position[0], point[1]+position[1]), 1, (0, 0, 255), -1)


    return scan_image







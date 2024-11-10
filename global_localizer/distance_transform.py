import cv2
import numpy as np
import matplotlib.pyplot as plt

map_resolution = 0.05 #m/px
threshold = 0.15 #m
threshold_px = threshold / map_resolution


def get_distance_transform(map_image, min_distance, map_resolution = map_resolution, threshold_px = threshold_px):
    distance_px = min_distance / map_resolution

    _, binary_image = cv2.threshold(map_image, 150, 255, cv2.THRESH_BINARY)
    distance_transform = cv2.distanceTransform(binary_image, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)

    # Create a copy of distance transform
    distance_transform_copy = distance_transform.copy()

    # Convert distance transform copy to RGB
    distance_transform_rgb = cv2.cvtColor(distance_transform_copy, cv2.COLOR_GRAY2RGB)

    # Set pixels near distance_px to red
    red_color = (255, 0, 0)
    near_pixels = np.abs(distance_transform_copy - distance_px) < threshold_px
    distance_transform_rgb[near_pixels] = red_color

    return distance_transform_rgb




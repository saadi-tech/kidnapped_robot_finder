import numpy as np
import math

def bresenham_ray_cast(image, x0, y0, angle):
    x0, y0 = int(x0), int(y0)
    angle_rad = math.radians(angle)
    dx = math.cos(angle_rad)
    dy = math.sin(angle_rad)

    x, y = x0, y0
    while 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
        if image[int(y), int(x)] < 100:
            return int(x), int(y)
        x += dx
        y += dy
    return -1, -1

def get_lidar_points(bw_image, x0, y0, add_noise = True):
    lidar_range = 8 #m
    map_resolution = 0.05 #m/pixel
    mean = 0
    std_dev = 0.01 #m

    angle_resolution = 1.0 #Degrees.
    angle_range = np.arange(0, 360,angle_resolution) #array of all angles in degrees.
    lidar_points = []
    for i in range(angle_range.shape[0]):
        noise_x = np.random.normal(mean, std_dev / map_resolution)
        noise_y = np.random.normal(mean, std_dev)

        point = bresenham_ray_cast(bw_image, x0, y0, angle_range[i])

        distance = math.sqrt((point[0] - x0)**2 + (point[1] - y0)**2)
        if distance < lidar_range / map_resolution and point[0] != -1 and point[1] != -1:
            if add_noise:
                noisy_x = point[0] + noise_x
                noisy_y = point[1] + noise_y
                lidar_points.append([int(noisy_x - x0), int(noisy_y - y0)])
            else:
                point = [point[0] - x0, point[1] - y0]
                lidar_points.append(point)

    return lidar_points




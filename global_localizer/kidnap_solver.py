import cv2
import numpy as np
from global_localizer import distance_transform as dt
from global_localizer import pose_finder as pf
import matplotlib.pyplot as plt
from global_localizer import feature_matching as fm
import math
import time

def calculate_white_pixel_ratio(orig_image, target_image):
    # Ensure the images have the same dimensions
    if orig_image.shape != target_image.shape:
        raise ValueError("Input images must have the same dimensions for white pixel ratio calculation.")
    
    kernel = np.ones((3, 3), np.uint8)
    dilated_orig_image = cv2.dilate(orig_image, kernel, iterations=2)
    dilated_target_image = cv2.dilate(target_image, kernel, iterations=2)

    white_pixels_reference = np.count_nonzero(dilated_orig_image)
    masked_img = cv2.bitwise_and(dilated_orig_image, dilated_target_image)
    white_pixels_target = np.count_nonzero(masked_img)

    pixel_ratio = white_pixels_target / white_pixels_reference

    return pixel_ratio



def get_score_2(reference_img, tf_img):
    kernel = np.ones((3, 3), np.uint8)
    dilated_reference_img = cv2.dilate(reference_img, kernel, iterations=0)
    dilated_tf_img = cv2.dilate(tf_img, kernel, iterations=0)

    mask = cv2.bitwise_and(dilated_reference_img, dilated_tf_img)
    mask2 = cv2.bitwise_and(reference_img, cv2.bitwise_not(tf_img))


    ratio = np.count_nonzero(mask) / np.count_nonzero(mask2)
    percentage = ratio * 100
    return percentage

def get_f1_score(reference_img, tf_img):

    # Adding 0.01 to avoid Division by zero
    TP = np.count_nonzero(cv2.bitwise_and(reference_img, tf_img)) + 0.01
    FP = np.count_nonzero(cv2.bitwise_and(tf_img, cv2.bitwise_not(reference_img))) + 0.01
    FN = np.count_nonzero(cv2.bitwise_and(reference_img, cv2.bitwise_not(tf_img))) + 0.01

    if TP == 0:
        return 0
    
    precision = TP / (TP + FP)
    recall = TP / (TP + FN)


    f1_score = 2 * (precision * recall) / (precision + recall)

    return f1_score * 100


def solve_kidnap(orig_scan_img, map_image, min_distance, map_resolution = 0.05, max_iterations = 30, stop_search_threshold = 50, lidar_range = 8.0):
    orig_scan_img = cv2.flip(orig_scan_img, 1)

    robot_coord = [orig_scan_img.shape[1]//2, orig_scan_img.shape[0]//2]
    distance =  min_distance

    st_time = time.perf_counter()

    candidate_area = dt.get_distance_transform(map_image, distance, map_resolution = map_resolution) #Rgb image, which red area where the robot can lie only.
    # Generate random coordinates within the red regions of the candidate area
    red_pixels = np.where(candidate_area[:,:,0] == 255)  # Find the indices of red pixels
    num_red_pixels = len(red_pixels[0])  # Get the number of red pixels
    
    candidate_area_to_show = candidate_area.copy()

    random_coords = []
    for _ in range(max_iterations):

        if num_red_pixels > 0:
            index = np.random.randint(num_red_pixels)  # Generate a random index within the range of red pixels
            x = red_pixels[0][index]  # Get the x-coordinate of the randomly selected red pixel
            y = red_pixels[1][index]  # Get the y-coordinate of the randomly selected red pixel
            random_coords.append((x, y))
            cv2.circle(candidate_area_to_show, (y, x), 3, (0, 255, 0), -1) 


            cv2.circle(candidate_area, (y, x), 20, (0, 255, 0), -1) # Draw GREEN color, so next random selection should not occur from here
            red_pixels = np.where(candidate_area[:,:,0] == 255)  # Find the indices of red pixels
            num_red_pixels = len(red_pixels[0])  # Get the number of red pixels

    threshold_accuracy = stop_search_threshold # % Needs fine tuning further ---- !
    max_accuracy = 0
    best_coord = None
    best_scan = None
    best_tf_img = None
    best_overlay = None
    best_tf_robot = None
    best_scan_center = None
    best_theta_degrees = None

    all_candidates = []

    iters = 0
    # Use the random coordinates for further processing
    for coord in random_coords:
        iters += 1
        print("Iteration: ", iters,  "/" , len(random_coords))
        x, y = coord
        # Do something with the coordinates
        scan_image = pf.get_scan_image(map_image.copy(), [y, x], map_resolution = map_resolution, max_range = 8)


        tf_orig_scan, overlay_img, tf_robot_pose, theta_degrees = fm.do_matching(orig_scan_img, scan_image, robot_pose = robot_coord)
        if tf_orig_scan is None:
            continue

        percentage = get_f1_score(scan_image, tf_orig_scan)
        all_candidates.append([percentage, y, x])
        print("F1 Score: ", percentage)
        if percentage > max_accuracy:
            max_accuracy = percentage
            best_coord = coord
            best_scan = scan_image
            best_tf_img = tf_orig_scan
            best_overlay = overlay_img
            best_tf_robot = tf_robot_pose
            best_scan_center = [y, x]
            best_theta_degrees = theta_degrees
            if max_accuracy >= threshold_accuracy:
                break


    end_time = time.perf_counter()
    print("\n\n\n-------------------------\n\n\n")
    x, y = best_coord
    print("Highest F1 score (x100): ", max_accuracy)
    print("Time taken: ms", (end_time - st_time) * 1000)
    #cv2.circle(candidate_area, (y, x), 5, (0, 255, 0), -1)
    #print("Old robot coord: ", robot_coord)
    #print("TF robot: ", best_tf_robot)
    cv2.circle(best_overlay, (int(best_tf_robot[0]), int(best_tf_robot[1])), 4, (255, 0, 0), -1)

    sorted_candidates = sorted(all_candidates, key = lambda x:x[0])


    robot_on_map = np.array(best_scan_center) + (np.array(best_tf_robot) - np.array(robot_coord))
    vector_length = 20 #px


    #print("BEst scan center: ", best_scan_center)
    print("Robot on map: ", robot_on_map)
    print("Theta in degrees: ", best_theta_degrees)


    #Converting map position to meter coords in map frame of ROS.
    map_origin = np.array([-7.47, -5.27]) # Find the origin values from map .yaml file.
    map_resolution = 0.05                 # Find the resolution from map .yaml file.

    robot_in_map_pixels = robot_on_map.copy()
    robot_in_map_pixels[1] = map_image.shape[0] - robot_in_map_pixels[1] #flip y axis
    robot_in_map_meters = robot_in_map_pixels * map_resolution + map_origin
    robot_angle_in_map = math.radians(-best_theta_degrees)

    print("----------------------------")
    print("Robot in map meters: ", robot_in_map_meters)
    print("Robot theta (on map) in degrees: ", math.degrees(robot_angle_in_map))

    

    # Display the scan_image, tf_orig_scan, and candidate_area images in a single Plt window side by side
    fig, axs = plt.subplots(1, 6, figsize=(12, 4))

    # Plot the orig_scan_image
    axs[0].imshow(orig_scan_img, cmap='gray')
    axs[0].set_title('Laser Scan Image')
    axs[0].axis('off')

    # Plot the scan_image
    axs[1].imshow(best_scan, cmap='gray')
    axs[1].set_title('Best Match simulated-scan')
    axs[1].axis('off')

    # Plot the tf_orig_scan
    axs[2].imshow(best_tf_img, cmap='gray')
    axs[2].set_title('transformed laser-scan')
    axs[2].axis('off')

    # Plot the overlay
    axs[3].imshow(best_overlay, cmap='gray')
    axs[3].set_title('Real/Sim scan overlay')
    axs[3].axis('off')

    # Plot the candidate_area
    axs[4].imshow(candidate_area_to_show, cmap='gray')
    axs[4].set_title('Candidate Area(Red), samples(Green)')
    axs[4].axis('off')

    # Convert map_image to BGR
    map_image_bgr = cv2.cvtColor(map_image, cv2.COLOR_GRAY2BGR)

    # Draw a red circle filled on the coords robot_on_map
    cv2.circle(map_image_bgr, (int(robot_on_map[0]), int(robot_on_map[1])), 10, (0, 0, 255), -1)

    # Calculate the endpoint of the line
    end_x = int(robot_on_map[0] + vector_length * math.cos(math.radians(best_theta_degrees)))
    end_y = int(robot_on_map[1] + vector_length * math.sin(math.radians(best_theta_degrees)))

    # Draw the line on the map_image
    cv2.line(map_image_bgr, (int(robot_on_map[0]), int(robot_on_map[1])), (end_x, end_y), (0, 0, 255), 2)

    # Plot the map_image with the red circle
    axs[5].imshow(map_image_bgr, cmap='gray')
    axs[5].set_title('Robot Position Estimate')
    axs[5].axis('off')


    plt.tight_layout()
    plt.show()

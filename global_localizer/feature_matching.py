import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt

# Function to perform feature matching between two images
def do_matching(orig_scan_img, sim_scan_img, robot_pose = None):

    # Initiate ORB detector
    orb = cv.ORB_create()
    # find the keypoints and descriptors with ORB
    kp1, des1 = orb.detectAndCompute(orig_scan_img,None)
    kp2, des2 = orb.detectAndCompute(sim_scan_img,None)

    # create BFMatcher object
    bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)
    # Match descriptors.
    matches = bf.match(des1,des2)
    # Sort them in the order of their distance.
    matches = sorted(matches, key = lambda x:x.distance)

    # RANSAC to estimate transformation
    MIN_MATCH_COUNT = 10
    if len(matches) > MIN_MATCH_COUNT:
        src_pts = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

        #print("Matches found:", len(matches))
        M, inliers = cv.estimateAffine2D(src_pts, dst_pts, method=cv.RANSAC, ransacReprojThreshold=5.0)
        matchesMask = inliers.ravel().tolist()

        # Calculate rotation in degrees
        rotation_rad = np.arctan2(M[1, 0], M[0, 0])
        rotation_deg = np.degrees(rotation_rad)

        transformed_orig_scan = cv.warpAffine(orig_scan_img, M, (orig_scan_img.shape[1], orig_scan_img.shape[0]))
       
        # Draw the inlier matches
        #draw_params = dict(matchColor=(0, 255, 0), singlePointColor=None, matchesMask=matchesMask, flags=2)
        #img4 = cv.drawMatches(orig_scan_img, kp1, sim_scan_img, kp2, matches, None, **draw_params)
        #plt.imshow(img4), plt.show()


        sim_scan_img_bgr = cv.cvtColor(sim_scan_img, cv.COLOR_GRAY2BGR)
        # Step 2: Set all white pixels in sim_scan_img_bgr to green
        # Define green in BGR format
        green = (0, 255, 0)
        # Apply green to all white pixels
        sim_scan_img_bgr[sim_scan_img == 255] = green

        # Step 3: Overlay red pixels from tf_scan_image on sim_scan_img_bgr
        # Define red in BGR format
        red = (0, 0, 255)
        # Apply red to all white pixels in tf_scan_image
        sim_scan_img_bgr[transformed_orig_scan == 255] = red

        if robot_pose is not None:
            # Convert robot pose to homogeneous coordinates
            robot_pose_homogeneous = np.array([robot_pose[0], robot_pose[1]]).reshape(-1, 1, 2)
            
            # Apply transformation to robot pose
            transformed_robot_pose = cv.transform(robot_pose_homogeneous, M)
            
            # Convert back to non-homogeneous coordinates
            transformed_robot_pose = transformed_robot_pose.squeeze()
            
            # Print the transformed robot pose
            # print("Transformed Robot Pose:")
            # print("x:", transformed_robot_pose[0])
            # print("y:", transformed_robot_pose[1])

            return transformed_orig_scan, sim_scan_img_bgr, transformed_robot_pose, rotation_deg
        else:
            return transformed_orig_scan, sim_scan_img_bgr, None, rotation_deg

    else:
        print("Not enough matches are found - %d/%d" % (len(matches), MIN_MATCH_COUNT))
        return None, None, None, None
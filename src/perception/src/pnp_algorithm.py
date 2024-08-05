import numpy as np
import math
import cv2

def correspondence_2d_3d(keypoints_2d):

    keypoints_3d = {
        0: [-114, 0, 0],
        1: [-69, 157, 0],
        2: [-39, 265, 0],
        3: [0, 325, 0],
        4: [114, 0, 0],
        5: [69, 157, 0],
        6: [39, 265, 0],
    }
    
    # Dictionary to store correspondences between 2D and 3D keypoints
    correspondences = {}

    # Iterate over each detected 2D keypoint
    for idx in keypoints_2d.keys():
        correspondences[idx] = (keypoints_2d[idx], keypoints_3d[idx])
    
    return correspondences


def PnP(points_2d):
    
    correspondences = correspondence_2d_3d(points_2d)

    # Cone points
    points_3d = np.array([correspondences[i][1] for i in correspondences.keys()])
    points_3d = points_3d.astype('float32')

    # Image points
    points_2d = np.array([correspondences[i][0] for i in correspondences.keys()])
    points_2d = points_2d.astype('float32')

    # Camera Matrix
    K = get_camera_matrix()

    # Distortion coefficients
    dist_coeffs = np.zeros((4, 1))  # Assuming no distortion

    ret, rvec, tvec = cv2.solvePnP(points_3d, points_2d, K, dist_coeffs)

    # Convert rotation vector to rotation matrix
    rvec, _ = cv2.Rodrigues(rvec)

    return rvec, tvec


def get_camera_matrix():

    # IMX219-83 intrinsics
    f_mm = 2.6  # Focal length in mm
    W = 3280    # Image width in pixels
    H = 2464    # Image height in pixels
    CMOS_width = 1/4  # CMOS size in inches

    # ETH intrinsics
    f_mm = 10
    W = 1600
    H = 1200
    CMOS_width = 1/4


    # Convert focal length from mm to pixels
    f_px = f_mm * W / CMOS_width

    # Calculate principal point
    cx = W / 2
    cy = H / 2

    K = np.array([[f_px, 0, cx],
              [0, f_px, cy],
              [0, 0, 1]])
              
    
    return K

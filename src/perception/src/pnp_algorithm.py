import numpy as np
import math
import cv2

def PnP(points_2d):

    points_3d = [
        [-114, 0, 0], 
        [-69, 157, 0],
        [-39, 265, 0],
        [0, 325, 0],
        [114, 0, 0],
        [69, 157, 0],
        [39, 265, 0]
    ]

    points_3d = np.array(points_3d).astype('float32')
    points_2d = np.array(points_2d).astype('float32')

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
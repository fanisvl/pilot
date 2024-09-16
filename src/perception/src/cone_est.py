#!/usr/bin/python3
import rospy

from jetson_inference import detectNet
from jetson_utils import cudaFromNumpy, cudaToNumpy

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, VisionInfo
from message_filters import ApproximateTimeSynchronizer, Subscriber
from messages.msg import ConeEstimates, ConeEstimate

import numpy as np
import matplotlib.pyplot as plt
import time

COLORS_CV2 = {
    0: (255, 0, 0),    # Blue  - #0000FF
    1: (0, 255, 255),  # Yellow - #FFFF00
    2: (0, 165, 255),  # Orange - #FFA500
    3: (0, 165, 255),  # Orange - #FFA500
}

COLORS_HEX = {
    0: '#0000FF',
    1: '#FFFF00',
    2: '#FFA500',
    3: '#FFA500',
}

CONE_3D_POINTS = np.array([
    [-65.0, 0.0, 0.0],
    [-39.34, 89.52, 0.0],
    [-22.24, 151.10, 0.0],
    [0.0, 185.31, 0.0],
    [65.0, 0.0, 0.0],
    [39.34, 89.52, 0.0],
    [22.24, 151.10, 0.0]
], dtype=np.float32)

# Left Camera Intrinsic Matrix
LEFT_INTR = np.array([
    [915.65548816,   0.,         689.70140921],
    [  0.,         685.54019452, 365.50765549],
    [  0.,           0.,           1.        ]
], dtype=np.float32)


# Left Camera Distortion Coefficients
LEFT_DIST = np.array([
    [-0.03668314,  0.24291992,  0.00226458,  0.00444546, -0.30489909]
], dtype=np.float32)

# Right Camera Intrinsic Matrix
RIGHT_INTR = np.array([
    [846.175412,   0.,         670.2238759 ],
    [  0.,       633.75634756, 342.02144716],
    [  0.,         0.,           1.        ]
], dtype=np.float32)


# Right Camera Distortion Coefficients
RIGHT_DIST = np.array([
    [ 0.00796533, -0.02163974, -0.00371325,  0.00139172, -0.0067765 ]
], dtype=np.float32)

# Rotation Matrix between Cameras
R = np.array([
    [ 0.99706868, -0.00724228,  0.07616824],
    [-0.00185945,  0.99292238,  0.11875055],
    [-0.07648917, -0.11854409,  0.98999834]
], dtype=np.float32)

# Translation Vector between Cameras
T = np.array([
    [79.10517262],
    [5.55096856],
    [52.61707659]
], dtype=np.float32)

class ConeEstimation:
    def __init__(self):
        net = detectNet(
            model="/workspace/pilot/src/perception/src/models/mobilenet/mobilenet.onnx",
            labels="/workspace/pilot/src/perception/src/models/mobilenet/labels.txt",
            input_blob="input_0",
            output_cvg="scores",
            output_bbox="boxes",
            threshold=0.5
        )
        rospy.loginfo("Model loaded.")
        rospy.init_node('cone_estimation')

        self.bridge = CvBridge()
        # Define subscribers for left and right images
        self.left_sub = Subscriber('/stereo/left/image_raw', Image)
        self.right_sub = Subscriber('/stereo/right/image_raw', Image)

        # Define publisher for cone estimates
        self.cone_pub = rospy.Publisher('/cone_estimates', ConeEstimates, queue_size=1)

        # Set up the time synchronizer
        self.time_sync = ApproximateTimeSynchronizer([self.left_sub, self.right_sub], queue_size=1, slop=0.1)
        self.time_sync.registerCallback(self.cone_estimation)


    def cone_estimation(self, left_msg, right_msg):
        
        total_time_start = time.time()

        left_frame = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='bgr8')
        right_frame = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='bgr8')

        cone_estimates_msg = ConeEstimates()
        cone_estimates_msg.cones = []

        detections = self.net.Detect(left_frame)

        if not detections:
            print("No cone detections.")
            return

        for id, detection in enumerate(detections):

            width = int(detection.Width)
            height = int(detection.Height)
            center_x = detection.Center[0]
            center_y = detection.Center[1]
            x = int(center_x - width / 2)
            y = int(center_y - height / 2)

            bbox_left = (x, y, width, height)

            bbox_right = self.bounding_box_propagation(bbox_left, left_frame, right_frame)

            # Extract SIFT features for triangulation
            keypoints_left, descriptors_left = self.extract_sift_features(left_frame, bbox_left)
            keypoints_right, descriptors_right = self.extract_sift_features(right_frame, bbox_right)

            good_matches = self.match_features(descriptors_left, descriptors_right)
            if good_matches is None:
                continue

            # Matched points
            pts1 = np.float32([keypoints_left[m.queryIdx].pt for m in good_matches])
            pts2 = np.float32([keypoints_right[m.trainIdx].pt for m in good_matches])

            # Triangulate points
            if len(pts1) < 1 or len(pts2) < 1:
                continue
            points_3d = self.triangulate_points(pts1, pts2)

            # Apply median filtering to 3D points
            points_3d_filtered = np.median(points_3d, axis=0)
            points_3d_filtered[2] = abs(points_3d_filtered[2])  # Positive depth Z
            points_3d_filtered[0] = -points_3d_filtered[0]  # Flip sign on X
            points_3d_filtered /= 10  # Scale to cm

            # Append the result to cones list
            cone_estimate_msg = ConeEstimate()
            cone_estimate_msg.id = id
            cone_estimate_msg.x = points_3d_filtered[0]
            cone_estimate_msg.y = points_3d_filtered[2]
            cone_estimates_msg.cones.append(cone_estimate_msg)

        total_time_end = time.time()
        rospy.loginfo(f"Total Pipeline Time: {total_time_end - total_time_start:.4f}")

        self.cone_pub.publish(cone_estimates_msg)


    def bounding_box_propagation(self, bbox, left_frame, right_frame):
        tracker = cv2.TrackerCSRT_create()
        tracker.init(left_frame, bbox)
        success, new_bbox = tracker.update(right_frame)
        if success:
            return new_bbox  # format: (x, y, w, h)
        rospy.logerr("Error Tracking Bounding Box to right frame")
        return None

    def extract_sift_features(self, image, bounding_box):
        x, y, w, h = bounding_box
        cropped_image = image[y:y + h, x:x + w]

        sift = cv2.SIFT_create()
        keypoints, descriptors = sift.detectAndCompute(cropped_image, None)

        keypoints = [cv2.KeyPoint(kp.pt[0] + x, kp.pt[1] + y, kp.size) for kp in keypoints]

        return keypoints, descriptors

    def match_features(self, desc1, desc2):
        """Match features using KNN."""
        if desc1 is None or desc2 is None:
            return None

        bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)
        matches = bf.knnMatch(desc1, desc2, k=2)

        good_matches = []
        for match_pair in matches:
            if len(match_pair) != 2:
                return None
            m, n = match_pair
            if m.distance < 0.75 * n.distance:
                good_matches.append(m)

        return good_matches

    def triangulate_points(self, pts1, pts2):
        projMatrix1 = np.dot(LEFT_INTR, np.hstack((np.eye(3), np.zeros((3, 1)))))
        projMatrix1 = np.dot(LEFT_INTR, np.hstack((np.eye(3), np.zeros((3, 1)))))
        projMatrix2 = np.dot(RIGHT_INTR, np.hstack((R, T)))

        pts1 = pts1.T
        pts2 = pts2.T

        points4D = cv2.triangulatePoints(projMatrix1, projMatrix2, pts1, pts2)

        points3D = points4D[:3, :] / points4D[3, :]

        return points3D.T

if __name__ == '__main__':
    node = ConeEstimation()
    rospy.spin()

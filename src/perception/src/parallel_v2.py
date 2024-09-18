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
from scipy import stats
import matplotlib.pyplot as plt
import time
import concurrent.futures

import torch

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
    [1122.426162, 0.000000, 598.503499],
    [0.000000, 1121.925556, 365.915575],
    [0.000000, 0.000000, 1.000000]
], dtype=np.float32)

# Left Camera Distortion Coefficients
LEFT_DIST = np.array([
    -0.058484, 0.512610, 0.000971, -0.000261, -1.056148
], dtype=np.float32)

# Right Camera Intrinsic Matrix
RIGHT_INTR = np.array([
    [1120.255791, 0.000000, 579.629778],
    [0.000000, 1118.852221, 357.675228],
    [0.000000, 0.000000, 1.000000]
], dtype=np.float32)

# Right Camera Distortion Coefficients
RIGHT_DIST = np.array([
    -0.098465, 0.442262, -0.001005, -0.002640, -0.642019
], dtype=np.float32)

# Rotation Matrix
R = np.array([
    [0.999625, -0.008677, -0.025983],
    [0.008992, 0.999887, 0.012057],
    [0.025876, -0.012286, 0.999590]
], dtype=np.float32)

# Translation Vector
T = np.array([
    [63.697536],
    [-4.602118],
    [8.842539]
], dtype=np.float32)

class ConeEstimation:
    def __init__(self):
        self.net = detectNet(
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
        self.left_sub = Subscriber('/left_cam/raw', Image)
        self.right_sub = Subscriber('/right_cam/raw', Image)

        # Define publisher for cone estimates
        self.cone_pub = rospy.Publisher('/cone_estimates', ConeEstimates, queue_size=1)

        # Set up the time synchronizer
        self.time_sync = ApproximateTimeSynchronizer([self.left_sub, self.right_sub], queue_size=1, slop=0.1)
        self.time_sync.registerCallback(self.cone_estimation)

        # benchmark
        self.detect_time = []
        self.bbox_time = []
        self.sift_time = []
        self.all_cones_time = []
        self.total_time = []

        # benchmark parameters
        self.benchmark_interval = 10 #sec
        self.last_benchmark_time = time.time()

        self.visualize = False

    def cone_estimation(self, left_msg, right_msg):
        start_time = time.time()

        left_frame = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='bgr8')
        right_frame = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='bgr8')

        cone_estimates_msg = ConeEstimates()
        cone_estimates_msg.cones = []

        # Measure detection time
        detect_start = time.time()
        cuda_img_left = cudaFromNumpy(left_frame)
        cuda_img_right = cudaFromNumpy(right_frame)
        detections_left = self.net.Detect(cuda_img_left)
        detections_right = self.net.Detect(cuda_img_right)
        detect_end = time.time()
        self.detect_time.append(detect_end - detect_start)

        if not detections_left and not detections_right:
            print("No cone detections.")
            return
        elif not detections_right or not detections_right:
            print("No cone detections on left or right frame.")
            return

        # Right box indices
        bbox_match_start_time = time.time()
        right_detection_matches = self.bounding_box_matching(detections_left, detections_right)
        bbox_match_end_time = time.time()
        self.bbox_time.append(bbox_match_end_time-bbox_match_start_time)

        visualization_data = []

        processed_all_cones_start = time.time()
        with concurrent.futures.ThreadPoolExecutor() as executor:
            futures = [executor.submit(self.pipeline, id, left_detection, detections_right, right_detection_matches, left_frame, right_frame) for id, left_detection in enumerate(detections_left)]
            
            for future in concurrent.futures.as_completed(futures):
                result = future.result()
                if result:

                    cone_estimates_msg.cones.append(result['cone_estimate_msg'])

                    # Benchmarking & Visualization
                    self.sift_time.append(result['sift_time'])
                    if self.visualize: visualization_data.append(result['visualization_data'])
        
        processed_all_cones_end = time.time()
        self.all_cones_time.append(processed_all_cones_end - processed_all_cones_start)
        
        self.cone_pub.publish(cone_estimates_msg)

        total_time_end = time.time()
        total_pipeline_time = total_time_end - start_time
        self.total_time.append(total_pipeline_time)
        rospy.loginfo(f"Total Pipeline Time: {total_pipeline_time:.4f} s ({1/total_pipeline_time:.2f} Hz) - {len(cone_estimates_msg.cones)} cones")
       
        current_time = time.time()
        if current_time - self.last_benchmark_time >= self.benchmark_interval:
            self.print_benchmark_info()
            self.last_benchmark_time = current_time

        if self.visualize:
            self.visualize_frames(left_frame, right_frame)
            for data in visualization_data:
                self.visualize_bounding_box(left_frame, data['bbox_left'], "Detected Left Bounding Box")
                self.visualize_bounding_box(right_frame, data['bbox_right'], "Propagated Right Bounding Box")
                self.visualize_sift_features(left_frame, right_frame, data['keypoints_left'], data['keypoints_right'])
                self.visualize_sift_matches(left_frame, right_frame, data['keypoints_left'], data['keypoints_right'], data['good_matches'])

            self.plot_cone_estimates(cone_estimates_msg)
            rospy.signal_shutdown("Shutting down after one run. (visualization enabled)")

    def pipeline(self, id, left_detection, detections_right, right_detection_matches, left_frame, right_frame):
        """"
        Bounding Box Propagation
        SIFT Feature Extraction
        SIFT Feature Matching
        Triangulation
        Filtering
        """

        # Bounding Box Propagation
        # TODO: Clean up bbox definitions, use new model output API for all methods instead of redefining
        bbox_left = (int(left_detection.Center[0] - left_detection.Width / 2),
                    int(left_detection.Center[1] - left_detection.Height / 2),
                    int(left_detection.Width),
                    int(left_detection.Height))
        
        bbox_right = detections_right[right_detection_matches[id]]
        bbox_right = (int(bbox_right.Center[0] - bbox_right.Width / 2),
                    int(bbox_right.Center[1] - bbox_right.Height / 2),
                    int(bbox_right.Width),
                    int(bbox_right.Height))

        if bbox_right is None:
            return None

        # SIFT Feature Extraction
        sift_start = time.time()
        keypoints_left, descriptors_left = self.extract_sift_features(left_frame, bbox_left)
        keypoints_right, descriptors_right = self.extract_sift_features(right_frame, bbox_right)
        sift_end = time.time()

        # SIFT Feature matching
        good_matches = self.match_features(descriptors_left, descriptors_right)
        if good_matches is None:
            return None

        # Triangulation
        pts1 = np.float32([keypoints_left[m.queryIdx].pt for m in good_matches])
        pts2 = np.float32([keypoints_right[m.trainIdx].pt for m in good_matches])
        if len(pts1) < 1 or len(pts2) < 1:
            return
        
        points_3d = self.triangulate_points(pts1, pts2)
        
        # Remove outliers
        z_scores = np.abs(stats.zscore(points_3d, axis=0))
        threshold = 3 # 3 standard deviations from the mean
        filtered_points = points_3d[(z_scores < threshold).all(axis=1)]

        # Median
        median_points = np.median(filtered_points, axis=0)
        median_points[0] = -median_points[0]  # Flip sign on X
        median_points /= 10  # Scale to cm

        cone_estimate_msg = ConeEstimate()
        cone_estimate_msg.id = id
        cone_estimate_msg.x = median_points[0]
        cone_estimate_msg.y = median_points[1]

        result = {
            'cone_estimate_msg': cone_estimate_msg,
            'sift_time': sift_end - sift_start
        }

        if self.visualize:
            result['visualization_data'] = {
                'bbox_left': bbox_left,
                'bbox_right': bbox_right,
                'keypoints_left': keypoints_left,
                'keypoints_right': keypoints_right,
                'good_matches': good_matches
            }

        return result


    def bounding_box_matching(self, left_detections, right_detections):
        left_centers_x = torch.tensor([d.Center[0] for d in left_detections], dtype=torch.float)
        right_centers_x = torch.tensor([d.Center[0] for d in right_detections], dtype=torch.float)

        horizontal_differences = (left_centers_x[:, None] - right_centers_x[None, :]).abs()

        # best matched index for each left box
        best_right_matches = torch.argmin(horizontal_differences, dim=1)

        return best_right_matches

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
        projMatrix2 = np.dot(RIGHT_INTR, np.hstack((R, T)))

        pts1 = pts1.T
        pts2 = pts2.T

        points4D = cv2.triangulatePoints(projMatrix1, projMatrix2, pts1, pts2)

        points3D = points4D[:3, :] / points4D[3, :]

        return points3D.T

    def visualize_frames(self, left_frame, right_frame):
        """Visualize the left and right camera frames."""
        cv2.imshow("Left Frame", left_frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        cv2.imshow("Right Frame", right_frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def visualize_bounding_box(self, frame, bbox, title="Bounding Box"):
        """Visualize the bounding box on a frame."""
        x, y, w, h = bbox
        frame_copy = frame.copy()
        cv2.rectangle(frame_copy, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.imshow(title, frame_copy)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def visualize_sift_features(self, left_frame, right_frame, keypoints_left, keypoints_right):
        """Visualize the SIFT features on both frames."""
        left_with_features = cv2.drawKeypoints(left_frame, keypoints_left, None, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        right_with_features = cv2.drawKeypoints(right_frame, keypoints_right, None, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        cv2.imshow("Left SIFT Features", left_with_features)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        cv2.imshow("Right SIFT Features", right_with_features)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def visualize_sift_matches(self, left_frame, right_frame, keypoints_left, keypoints_right, matches):
        """Visualize matched SIFT features between left and right frames."""
        matches_img = cv2.drawMatches(left_frame, keypoints_left, right_frame, keypoints_right, matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        cv2.imshow("SIFT Feature Matches", matches_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def plot_cone_estimates(self, cone_estimates_msg):
        """Plot the cone estimates in a 2D plot."""

        plt.figure(figsize=(6, 6))
        xs = [cone.x for cone in cone_estimates_msg.cones]
        ys = [cone.y for cone in cone_estimates_msg.cones]
        for cone in cone_estimates_msg.cones:
            plt.scatter(cone.x, cone.y, c='orange')
            plt.text(cone.x, cone.y+3, f'{cone.id}: ({cone.x:.2f}, {cone.y:.2f})', fontsize=9, ha='right')

        plt.scatter(0, 0, c='red', marker='x')
        plt.xlabel("X (cm)")
        plt.ylabel("Z (cm)")
        plt.title("Cone Estimation Map")
        plt.legend()
        plt.grid(True)
        plt.show()

    def print_benchmark_info(self):
        def print_avg_time(name, times):
            if times:
                avg_time = np.mean(times)
                print(f"Average {name} Time: {avg_time:.4f} s ({1/avg_time:.2f} Hz)")

        print("Benchmark Information:")
        print_avg_time("Detection", self.detect_time)
        print_avg_time("Bounding Box Match Time", self.bbox_time)
        print_avg_time("SIFT Feature Extraction", self.sift_time)
        print_avg_time("Processed All cones Time", self.all_cones_time)
        print_avg_time("Total Pipeline", self.total_time)

        self.detect_time.clear()
        self.bbox_time.clear()
        self.sift_time.clear()
        self.all_cones_time.clear()
        self.total_time.clear()


if __name__ == '__main__':
    node = ConeEstimation()
    rospy.spin()

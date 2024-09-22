#!/usr/bin/python3
import rospy

from jetson_inference import detectNet
from jetson_utils import cudaFromNumpy, cudaToNumpy

import cv2
from cv_bridge import CvBridge
import torch

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, VisionInfo
from message_filters import ApproximateTimeSynchronizer, Subscriber
from messages.msg import ConeEstimates, ConeEstimate

import numpy as np
import matplotlib.pyplot as plt
import time
import yaml

class ConeEstimation:
    def __init__(self):
        self.net = detectNet(
            model="/workspace/pilot/src/perception/src/models/v1.onnx",
            labels="/workspace/pilot/src/perception/src/models/labels.txt",
            input_blob="input_0",
            output_cvg="scores",
            output_bbox="boxes",
            threshold=0.1
        )
        
        rospy.loginfo("Model loaded.")
        rospy.init_node('cone_estimation')

        self.bridge = CvBridge()
        self.left_sub = Subscriber('/left_cam/raw', Image)
        self.right_sub = Subscriber('/right_cam/raw', Image)
        self.cone_pub = rospy.Publisher('/cone_estimates', ConeEstimates, queue_size=1)

        # approximate time sync
        self.time_sync = ApproximateTimeSynchronizer([self.left_sub, self.right_sub], queue_size=1, slop=0.1)
        self.time_sync.registerCallback(self.cone_estimation)

        self.cam_info = self.get_cam_info("/workspace/pilot/src/perception/src/cam_config.yaml")

        # benchmark
        self.detect_time = []
        self.bbox_time = []
        self.sift_time = []
        self.all_cones_time = []
        self.total_time = []

        # benchmark parameters
        self.benchmark_interval = 10 #sec
        self.last_benchmark_time = time.time()

        # debug mode
        self.debug = rospy.get_param('~debug', False)

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
        right_detection_matches = self.bounding_box_matching(detections_left, detections_right)

        all_cones_time_start = time.time()
        for id, detection_left in enumerate(detections_left):
            bbox_left = self.get_bbox(detection_left) # x,y,w,h
            
            detection_right = detections_right[right_detection_matches[id]]
            bbox_right = self.get_bbox(detection_right)

            if bbox_right is None:
                continue

            # SIFT Feature Extraction
            sift_start = time.time()
            keypoints_left, descriptors_left = self.extract_sift_features(left_frame, bbox_left)
            keypoints_right, descriptors_right = self.extract_sift_features(right_frame, bbox_right)
            sift_end = time.time()
            self.sift_time.append(sift_end - sift_start)

            # SIFT Feature matching
            good_matches = self.match_features(descriptors_left, descriptors_right)

            if good_matches is None:
                continue

            # Triangulation
            pts1 = np.float32([keypoints_left[m.queryIdx].pt for m in good_matches])
            pts2 = np.float32([keypoints_right[m.trainIdx].pt for m in good_matches])
            if len(pts1) < 1 or len(pts2) < 1:
                print(f"(id: {id}): Not enough good SIFT feature matches")
                continue
            points_3d = self.triangulate_points(pts1, pts2)

            # calc. median of triangulated points
            median_points = np.median(points_3d, axis=0)
            median_points /= 10  # scale to cm
            median_points[0] = -median_points[0]  # Flip sign on X
            median_points[0] -= 3 # stereo cam baseline is 6cm, sift x to approximate vehicle world frame

            # create ros msg
            cone_estimate_msg = ConeEstimate()
            cone_estimate_msg.id = id
            cone_estimate_msg.x = median_points[0]
            cone_estimate_msg.y = median_points[2]
            cone_estimates_msg.cones.append(cone_estimate_msg)

            # visualize every step of the pipeline
            if self.debug:
                self.debug_pipeline(left_frame, right_frame, bbox_left, bbox_right, 
                                    keypoints_left, keypoints_right, good_matches)

        self.cone_pub.publish(cone_estimates_msg)

        all_cones_time_end = time.time()
        self.all_cones_time.append(all_cones_time_end - all_cones_time_start)

        total_time_end = time.time()
        total_pipeline_time = total_time_end - start_time
        self.total_time.append(total_pipeline_time)

        # Benchmarking
        current_time = time.time()
        if current_time - self.last_benchmark_time >= self.benchmark_interval:
            self.print_benchmark_info()
            self.last_benchmark_time = current_time

        if self.debug:
            self.plot_cone_estimates(cone_estimates_msg)
            rospy.signal_shutdown("Shutting down after one run.")
        
    def get_bbox(self, detection, width_scale=0.8):
        """Calculate bounding box coordinates from a detection object."""
        width = width_scale * detection.Width
        x = int(detection.Center[0] - width / 2)
        y = int(detection.Center[1] - detection.Height / 2)
        w = int(width)
        h = int(detection.Height)
        return (x, y, w, h)

    def bounding_box_matching(self, left_detections, right_detections):
        """
        Match bounding boxes from two different frames based on horizontal center dist.
        Used when performing object detection on both L and R frames seperately.
        """
        # TODO: Add edge case where the cone is visible in only one frame.
        left_centers_x = torch.tensor([d.Center[0] for d in left_detections], dtype=torch.float)
        right_centers_x = torch.tensor([d.Center[0] for d in right_detections], dtype=torch.float)
        horizontal_differences = (left_centers_x[:, None] - right_centers_x[None, :]).abs()
        # best matched index for each left box
        best_right_matches = torch.argmin(horizontal_differences, dim=1)

        return best_right_matches
    
    def bounding_box_propagation(self, bbox, left_frame, right_frame):
        """
        Propagate the detected bounding box from the L frame to the R frame.
        Used when performing obj. det. on L frame only.
        CSRT tracker is accurate, but slow.
        """
        tracker = cv2.TrackerCSRT_create()
        tracker.init(left_frame, bbox)
        success, new_bbox = tracker.update(right_frame)
        if success:
            return tuple(int(x) for x in new_bbox)  # format: (x, y, w, h)
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
        projMatrix1 = np.dot(self.cam_info["LEFT_INTR"], np.hstack((np.eye(3), np.zeros((3, 1)))))
        projMatrix2 = np.dot(self.cam_info["RIGHT_INTR"], np.hstack((self.cam_info["R"], self.cam_info["T"])))

        pts1 = pts1.T
        pts2 = pts2.T

        points4D = cv2.triangulatePoints(projMatrix1, projMatrix2, pts1, pts2)

        points3D = points4D[:3, :] / points4D[3, :]

        return points3D.T

    def plot_cone_estimates(self, cone_estimates_msg):
        """Plot cone estimates msg"""
        plt.figure(figsize=(6, 6))
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

    def debug_pipeline(self, left_frame, right_frame, bbox_left, bbox_right, 
                          keypoints_left, keypoints_right, good_matches):
        
         # Visualize bounding boxes
        left_frame_copy = left_frame.copy()
        right_frame_copy = right_frame.copy()
        
        cv2.rectangle(left_frame_copy, (bbox_left[0], bbox_left[1]), 
                    (bbox_left[0] + bbox_left[2], bbox_left[1] + bbox_left[3]), (0, 255, 0), 2)
        cv2.rectangle(right_frame_copy, (bbox_right[0], bbox_right[1]), 
                    (bbox_right[0] + bbox_right[2], bbox_right[1] + bbox_right[3]), (0, 255, 0), 2)

        # Visualize features
        left_with_features = cv2.drawKeypoints(left_frame_copy, keypoints_left, None, 
                                            flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        right_with_features = cv2.drawKeypoints(right_frame_copy, keypoints_right, None, 
                                                flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        combined_frame = cv2.hconcat([left_with_features, right_with_features])
        cv2.imshow("BBoxes and Features", combined_frame)
        cv2.waitKey(0)

        # feature matches
        matches_img = cv2.drawMatches(left_frame, keypoints_left, right_frame, keypoints_right, 
                                      good_matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        cv2.imshow("SIFT Feature Matches", matches_img)
        cv2.waitKey(0)

    def print_benchmark_info(self):
        def print_avg_time(name, times):
            if times:
                avg_time = np.mean(times)
                print(f"Average {name} Time: {avg_time:.4f} s ({1/avg_time:.2f} Hz)")

        print("Benchmark Information:")
        print_avg_time("Detection", self.detect_time)
        print_avg_time("Bounding Box Propagation", self.bbox_time)
        print_avg_time("SIFT Feature Extraction", self.sift_time)
        print_avg_time("All Cones Time", self.all_cones_time)
        print_avg_time("Total Pipeline", self.total_time)

        self.detect_time.clear()
        self.bbox_time.clear()
        self.sift_time.clear()
        self.all_cones_time.clear()
        self.total_time.clear()

    def get_cam_info(self, config_file):
        with open(config_file, 'r') as file:
            config = yaml.safe_load(file)

        left_intr = np.array(config['camera_intrinsics']['left']['intrinsic_matrix'], dtype=np.float32)
        left_dist = np.array(config['camera_intrinsics']['left']['distortion_coefficients'], dtype=np.float32)
        right_intr = np.array(config['camera_intrinsics']['right']['intrinsic_matrix'], dtype=np.float32)
        right_dist = np.array(config['camera_intrinsics']['right']['distortion_coefficients'], dtype=np.float32)
        R = np.array(config['camera_extrinsics']['rotation_matrix'], dtype=np.float32)
        T = np.array(config['camera_extrinsics']['translation_vector'], dtype=np.float32)

        return {
            'LEFT_INTR': left_intr,
            'LEFT_DIST': left_dist,
            'RIGHT_INTR': right_intr,
            'RIGHT_DIST': right_dist,
            'R': R,
            'T': T
        }

if __name__ == '__main__':
    node = ConeEstimation()
    rospy.spin()

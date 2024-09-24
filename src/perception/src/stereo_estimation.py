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
import time
import yaml
from utils import get_cam_info, plot_cone_estimates, debug_pipeline, print_benchmark_info, benchmark
from concurrent.futures import ThreadPoolExecutor, as_completed
import threading

class ConeEstimation:
    def __init__(self):
        self.net = detectNet(
            model="/workspace/pilot/src/perception/src/models/mobilenet-od.onnx",
            labels="/workspace/pilot/src/perception/src/models/mobilenet_labels.txt",
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

        self.cam_info = get_cam_info("/workspace/pilot/src/perception/src/cam_config.yaml")

        # benchmark
        self.benchmark = rospy.get_param('~benchmark', True)
        if self.benchmark:
            self.benchmark_interval = rospy.get_param('~benchmark_interval', 5)
            timer = rospy.Timer(rospy.Duration(self.benchmark_interval), print_benchmark_info)

        # debug mode
        self.debug = rospy.get_param('~debug', False)
        self.lock = threading.Lock() # needed for displaying in debug mode

    @benchmark('total')
    def cone_estimation(self, left_msg, right_msg):
        cone_estimates_msg = ConeEstimates()
        cone_estimates_msg.cones = []

        left_frame = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='bgr8')
        right_frame = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='bgr8')

        detections_left = self.detect_cones(left_frame)
        detections_right = self.detect_cones(right_frame)

        if not detections_right or not detections_right:
            print("No cone detections on left or right frame/s.")
            return
        
        # Right box indices
        right_detection_matches = self.bounding_box_matching(detections_left, detections_right)
        if right_detection_matches is None:
            return

        cone_estimates_msg = self.parallel_process_detections(detections_left, detections_right, right_detection_matches, 
                                                              left_frame, right_frame)

        self.cone_pub.publish(cone_estimates_msg)

        if self.debug:
            plot_cone_estimates(cone_estimates_msg)
            rospy.signal_shutdown("Shutting down after one run.")

    def process_detection(self, id, detection_left, detection_right, left_frame, right_frame):
        bbox_left = self.get_bbox(detection_left)
        bbox_right = self.get_bbox(detection_right)
        if bbox_right is None:
            return None

        # SIFT Feature Extraction
        keypoints_left, descriptors_left = self.extract_sift_features(left_frame, bbox_left)
        keypoints_right, descriptors_right = self.extract_sift_features(right_frame, bbox_right)

        # SIFT Feature matching
        good_matches = self.match_features(descriptors_left, descriptors_right)
        if good_matches is None:
            return None

        # Triangulation
        pts1 = np.float32([keypoints_left[m.queryIdx].pt for m in good_matches])
        pts2 = np.float32([keypoints_right[m.trainIdx].pt for m in good_matches])
        if len(pts1) < 1 or len(pts2) < 1:
            print(f"(id: {id}): Not enough good SIFT feature matches")
            return None

        points_3d = self.triangulate_points(pts1, pts2)
        
        # calc. median of triangulated points
        median_points = np.median(points_3d, axis=0)
        median_points /= 10  # scale to cm
        median_points[0] = -median_points[0] # flip sign on x (due to gstreamer limitations)

        # create ros msg
        cone_estimate_msg = ConeEstimate()
        cone_estimate_msg.id = id
        cone_estimate_msg.x = median_points[0]
        cone_estimate_msg.y = median_points[2]

        # visualize every step of the pipeline
        if self.debug:
            with self.lock:
                debug_pipeline(left_frame, right_frame, bbox_left, bbox_right,
                            keypoints_left, keypoints_right, good_matches)

        return cone_estimate_msg

    def parallel_process_detections(self, detections_left, detections_right, right_detection_matches, left_frame, right_frame):
        with ThreadPoolExecutor() as executor:
                futures = [
                    executor.submit(self.process_detection, 
                                    id, 
                                    detection_left, 
                                    detections_right[right_detection_matches[id]], 
                                    left_frame, 
                                    right_frame)
                    for id, detection_left in enumerate(detections_left)
                ]
                cone_estimates_msg = ConeEstimates()
                for future in as_completed(futures):
                    try:
                        res = future.result()
                        if res is not None:
                            cone_estimates_msg.cones.append(future.result())
                    except Exception as e:
                        print(f"exception while processing detection: {e}")

        return cone_estimates_msg

    @benchmark('detect')
    def detect_cones(self, frame):
        cuda_img = cudaFromNumpy(frame)
        detections = self.net.Detect(cuda_img)
        return detections
        
    def get_bbox(self, detection, width_scale=0.8):
        """Calculate bounding box coordinates from a detection object."""
        width = width_scale * detection.Width
        x = int(detection.Center[0] - width / 2)
        y = int(detection.Center[1] - detection.Height / 2)
        w = int(width)
        h = int(detection.Height)
        return (x, y, w, h)

    def bounding_box_matching(self, left_detections, right_detections, threshold_dist=50):
        """
        Match bounding boxes from two different frames based on horizontal center dist.
        Used when performing object detection on both L and R frames seperately.
        """
        if len(left_detections) != len(right_detections):
            return None
        
        left_centers_x = torch.tensor([d.Center[0] for d in left_detections], dtype=torch.float)
        right_centers_x = torch.tensor([d.Center[0] for d in right_detections], dtype=torch.float)
        horizontal_differences = (left_centers_x[:, None] - right_centers_x[None, :]).abs()
        # best matched index for each left box
        best_right_matches = torch.argmin(horizontal_differences, dim=1)
        min_distances = torch.min(horizontal_differences, dim=1).values

        valid_matches = [match if distance <= threshold_dist else -1
                         for match, distance in zip(best_right_matches, min_distances)]

        return valid_matches
    
    @benchmark('propagation')
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

    @benchmark('sift')
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


if __name__ == '__main__':
    node = ConeEstimation()
    rospy.spin()

#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Image
from messages.msg import ConeEstimates, ConeEstimate
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import numpy as np
import cv2
from utils import get_cam_info

class ConeEstimation:
    def __init__(self):
        rospy.init_node('cone_estimator_node', anonymous=True)
        self.bridge = CvBridge()
        self.model_src = '/workspace/pilot/src/perception/src/models/yolov8n-keypts.pt'
        self.cone_detection_model = YOLO(self.model_src)
        self.sub = rospy.Subscriber("/left_cam/raw", Image, self.cone_estimation)
        self.cone_pub = rospy.Publisher('/cone_estimates', ConeEstimates, queue_size=1)
        self.cam_info = get_cam_info()

    def cone_estimation(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert image: {e}")
            return
        
        results = self.cone_detection_model.predict(image)[0]
        cone_estimates_msg = ConeEstimates()
        cone_estimates_msg.cones = []
        for id, keypoints in enumerate(results.keypoints.data):
            # pnp for translation vector
            _, tvec = self.PnP(keypoints)
            if tvec is None:
                continue

            tvec /= 10  # Convert from mm to cm
            # PnP world frame is base of cone, 
            # so tvec corresponds to position of cone relative to cam
            x, z = -tvec[0][0], tvec[2][0] # x is flipped (due to gstreamer limitations)

            # assuming that we're using the left cam, 
            # IMX219-83 camera baseline is 6cm
            x += 3 # from left cam world frame, to vehicle world frame

            msg = self.cone_estimate_msg(id, x, z)
            cone_estimates_msg.cones.append(msg)
        
        self.cone_pub.publish(cone_estimates_msg)
        
    def PnP(self, points_2d):
        # JUMBO cones (130mm x 130mm x 240mm)
        # Keypoint coordinates with cone base as the world frame
        points_3d = np.array([
            [0, 240, 0],    # top
            [-40, 170, 0],  # l2
            [-90, 100, 0],  # l1
            [-120, 0, 0],   # l0
            [40, 170, 0],   # r2
            [90, 100, 0],   # r1
            [120, 0, 0],    # r0
        ], dtype='float32')
        # FSOCO Dataset / FSG WEMAS Cones (228mm x 228mm x 325mm)
        # points_3d = np.array([
        #     [   0.0,  568.75, 0],
        #     [ -68.25, 463.75, 0],
        #     [-120.75, 274.75, 0],
        #     [-199.5,   0.0,   0],
        #     [  68.25, 463.75, 0],
        #     [ 120.75, 274.75, 0],
        #     [ 199.5,    0.0,  0],
        # ], dtype='float32')

        points_2d = np.array(points_2d).astype('float32')
        K = self.cam_info["LEFT_INTR"]
        dist_coeffs = self.cam_info["LEFT_DIST"]
        ret, rvec, tvec = cv2.solvePnP(points_3d, points_2d, K, dist_coeffs)
        if not ret:
            rospy.logerr("PnP solution could not be found.")
            return None, None
        rvec, _ = cv2.Rodrigues(rvec)  # rotation vector to rotation matrix
        return rvec, tvec
    
    def cone_estimate_msg(self, id, x, y):
        cone_estimate_msg = ConeEstimate()
        cone_estimate_msg.id = id
        cone_estimate_msg.x = x
        cone_estimate_msg.y = y
        return cone_estimate_msg
        
if __name__ == '__main__':
    node = ConeEstimation()
    rospy.spin()

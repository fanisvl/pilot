#!/usr/bin/python3
import cv2
import matplotlib.pyplot as plt
import time
import numpy as np
from ultralytics import YOLO
from keypoint_regression.keypoint_regression_model import KeypointRegression
from pnp_algorithm import PnP
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from messages.msg import ConeEstimate, ConeEstimates

colors = {
    0: '#0000FF',
    1: '#FFFF00',
    2: '#FFA500',
    3: '#FFA500',
}

class ConeEstimation:
    def __init__(self, cone_detection_src, keypoint_regression_src):
        rospy.init_node('cone_estimation')
        rospy.loginfo("ConeEstimation node initialized.")

        self.subscription = rospy.Subscriber(
            '/camera',
            Image,
            self.cone_estimation,
            queue_size=10
        )

        self.publisher = rospy.Publisher(
            'cone_estimates',
            ConeEstimates,
            queue_size=10
        )
        self.cv_bridge = CvBridge()
        self.cone_detection_model = YOLO(cone_detection_src)
        self.keypoint_regression_model = KeypointRegression(keypoint_regression_src)

    def cone_estimation(self, image_msg):
        """
        Demo includes image and plot visualization of keypoints and cone estimates.

        > Full Img 
        > Bounding Box Detection 
        > Crop Cone Imgs to feed into keypoint regression model
        > Keypoint Regression on Cropped Images (x,y output on cropped coordinates) 
        > Translation to full image coordinates
        > PnP
        """

        total_time_start = time.time()
        full_image = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

        cone_det_start = time.time()
        result = self.cone_detection_model.predict(full_image)
        cone_det_end = time.time()
        bounding_boxes = result[0].boxes

        cone_estimates_msg = ConeEstimates()
        cropped_cones = []

        if len(bounding_boxes) <= 1:
            rospy.loginfo("Waiting to detect at least 2 cones.")
            return

        rospy.loginfo(f"{len(bounding_boxes)} cones detected.")
        for id, box in enumerate(bounding_boxes):
            label = box.cls.item()  # 2 = orange, 1 = yellow, 0 = blue
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cropped_img = full_image[y1:y2, x1:x2]
            cropped_cones.append(cropped_img)
            
            cone_estimate_msg = ConeEstimate()
            cone_estimate_msg.id = id
            cone_estimate_msg.label = int(label)
            cone_estimates_msg.cones.append(cone_estimate_msg)

        keypoint_reg_start = time.time()
        keypoints = self.keypoint_regression_model.eval(cropped_cones)
        keypoint_reg_end = time.time()
        rospy.loginfo("Keypoint regression finished")

        keypoints_2d = {}

        # Map 7 keypoints for each cone
        for id, cone in enumerate(cone_estimates_msg.cones):
            # The model output is 14 (x,y) coordinates => 7 keypoints
            x1, y1, x2, y2 = map(int, bounding_boxes[id].xyxy[0])  # bounding box coordinates in full image
            for point in range(0, len(keypoints[0]), 2):
                cropped_height = y2 - y1
                cropped_width = x2 - x1

                cropped_x = int(keypoints[cone.id][point] / 100.0 * cropped_width)
                cropped_y = int(keypoints[cone.id][point+1] / 100.0 * cropped_height)
                
                full_x = int(cropped_x + x1)
                full_y = int(cropped_y + y1)

                # Add to 2D keypoints map for PnP
                keypoints_2d[point//2] = [full_x, full_y]

                cv2.putText(full_image, str(cone.id), (x2, y2), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 1)
                cv2.circle(full_image, (full_x, full_y), 2, (0, 0, 255), -1)

            # Estimate cone position with PnP using all the 2d keypoints for this cone
            rvec, tvec = PnP(keypoints_2d)
            tvec /= 1000
            tvec[2] /= 100

            # 2D Coordinates for cone position in map
            cone.x = tvec[0][0]
            cone.y = tvec[2][0]

        total_time_end = time.time()
        rospy.loginfo(f"\nTotal Pipeline Time: {total_time_end-total_time_start:.4}")  
        rospy.loginfo(f"Cone Detection Time: {cone_det_end-cone_det_start:.4}")
        rospy.loginfo(f"Total Keypoint Regr. Time: {keypoint_reg_end-keypoint_reg_start:.4}")
        self.publisher.publish(cone_estimates_msg)

        # Visualization (if needed)
        cv2.imshow("Keypoints", full_image)
        cv2.waitKey(0)
        plt.title("Estimated Cone Position Relative to Camera")
        plt.scatter(0, 0, color='r', label='Camera')
        plt.xlabel("X-axis")
        plt.ylabel("Depth-axis")
        plt.legend()
        plt.grid(True)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.axis('equal')
        
        for cone in cone_estimates_msg.cones:   
            x = cone.x
            y = cone.y
            if cone.label in colors.keys():
                plt.scatter(x, y, color=colors[cone.label])
            plt.annotate(f'{cone.id}', (x, y))

        plt.show()
        

if __name__ == '__main__':
    cone_estimation = ConeEstimation('src/vision/vision/models/yolov8n-100e.pt', 'src/vision/vision/models/keypoint_regression.pth')
    rospy.spin()  # Node is kept alive until killed with ctrl+c


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
from messages.msg import ConeEstimates, ConeEstimate
from cv_bridge import CvBridge

CLASS_COLORS_CV2 = {
    0: (255, 0, 0),    # Blue  - #0000FF
    1: (0, 255, 255),  # Yellow - #FFFF00
    2: (0, 165, 255),  # Orange - #FFA500
    3: (0, 165, 255),  # Orange - #FFA500
}

CLASS_COLORS_HEX = {
    0: '#0000FF',  # Blue
    1: '#FFFF00',  # Yellow
    2: '#FFA500',  # Orange
    3: '#FFA500',  # Orange (Note: Labels 2 and 3 use the same color)
}

class ConeEstimation:
    def __init__(self, model_src, demo=False):
        rospy.init_node('cone_estimation')
        self.subscription = rospy.Subscriber(
            '/camera',
            Image,
            self.cone_estimation,
            queue_size=1
        )

        self.publisher = rospy.Publisher(
            'cone_estimates',
            ConeEstimates,
            queue_size=1
        )
        self.cv_bridge = CvBridge()
        self.model = YOLO(model_src)
        self.demo = demo
        rospy.loginfo("ConeEstimation node initialized.")

    def cone_estimation(self, image_msg):
        """
        Demo includes image and plot visualization of keypoints and cone estimates.

        > Image Input
        > Bounding Box and Keypoint Regression done by YOLO model
        > PnP
        """
        print(f"Starting cone_estimation pipeline..")
        total_time_start = time.time()
        image = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

        results = self.model.predict(image)
        results = results[0]

        if len(results.boxes) <= 1:
            rospy.loginfo("Waiting to detect at least 2 cones.")
            return


        cone_estimates_msg = ConeEstimates()
        cone_estimates_msg.cones = []

        for idx, cone_points in enumerate(results.keypoints.data):
            cone_points = cone_points.cpu()
            rvec, tvec = PnP(cone_points)
            
            # Magic
            tvec /= 1000
            tvec[2] /= 100

            x, y = tvec[0][0], tvec[2][0]
            
            cone_estimate_msg = ConeEstimate()
            cone_estimate_msg.id = idx 
            cone_estimate_msg.label = int(results.boxes[idx].cls.item())
            cone_estimate_msg.x = x
            cone_estimate_msg.y = y
            
            cone_estimates_msg.cones.append(cone_estimate_msg)

        self.publisher.publish(cone_estimates_msg)

        total_time_end = time.time()
        print(f"Total pipeline time: {total_time_end-total_time_start}")


        if self.demo:
            for box in results.boxes:
                x1, y1, x2, y2 = box.xyxy.cpu().numpy()[0]
                conf = box.conf.item()
                class_id = int(box.cls.item())
                print([x1, y1, x2, y2], conf, class_id)
                cv2.rectangle(image, (int(x1),int(y1)), (int(x2),int(y2)), CLASS_COLORS_CV2[class_id], 1)
            for cone in results.keypoints.data:
                for (x,y) in cone:
                    cv2.circle(image, (int(x),int(y)), 1, (0, 0, 255), 1)
            cv2.imshow("img", image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

            # Plot cone map
            plt.figure(figsize=(8, 6))
            plt.scatter(0, 0, color='red', marker='x', s=100, label='Camera')
            plt.xlabel('X-axis')
            plt.ylabel('Y-axis')
            plt.title('Scatter Plot of Cone Points')
            plt.legend()
            plt.grid(True)

            for cone in cone_estimates_msg.cones:
                plt.scatter(cone.x, cone.y, color=CLASS_COLORS_HEX[cone.label], marker='o', s=100, edgecolor='black', label=f"{cone.label}")

            plt.show()

if __name__ == '__main__':
    demo = False
    cone_estimation = ConeEstimation('/workspace/autopilot/src/perception/src/models/yolo-nano.pt', demo)
    rospy.spin()  # Node is kept alive until killed with ctrl+c

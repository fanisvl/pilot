import cv2
import matplotlib.pyplot as plt
import time
import numpy as np
import json
from ultralytics import YOLO
from .keypoint_regression_model import KeypointRegression
from .PnP_algorithm import PnP 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from messages.msg import ConeEstimate
from messages.msg import ConeEstimates


colors = {
    0: '#0000FF',
    1: '#FFFF00',
    2: '#FFA500',
    3: '#FFA500',
}

class ConeEstimation(Node):

    def __init__(self, cone_detection_src, keypoint_regression_src):
        super().__init__("cone_estimation")
        self.get_logger().info("ConeEstimation node initialized.")


        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.cone_estimation,
            10
        )

        self.publisher = self.create_publisher(
            ConeEstimates,
            'cone_estimates',
            10
        )
        self.cv_bridge = CvBridge()
        self.cone_detection_model = YOLO(cone_detection_src)
        self.keypoint_regression_model = KeypointRegression(keypoint_regression_src)


    def cone_estimation(self, image_msg, demo=False):
        """"
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

        cone_estimates = {}
        cropped_cones = []
        id = 0

        if len(bounding_boxes) <= 1:
            print("Waiting to detect at least 2 cones.")
            return

        print(f"{len(bounding_boxes)} cones detected.")
        for box in bounding_boxes:
            label = box.cls.item() # 2 = orange, 1 = yellow, 0 = blue
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cone_estimates[id] = {"id": id, "label": label}
            cropped_img = full_image[y1:y2, x1:x2]
            cropped_cones.append(cropped_img)
            id += 1

        keypoint_reg_start = time.time()
        keypoints = self.keypoint_regression_model.eval(cropped_cones)
        keypoint_reg_end = time.time()
        print("Keypoint regression finished")

        keypoints_2d = {}

        # Map 7 keypoints for each cone
        # TODO: Changing the frame size of the input image causes bugs in cone estimation (sideways path)
        for cone in cone_estimates.values():
            # Map keypoint model  output from cropped image to full image coordinates
            # The model output is 14 (x,y) coordinates => 7 keypoints

            id = cone["id"]
            x1, y1, x2, y2 = map(int, bounding_boxes[id].xyxy[0]) # bounding box coordinates in full image
            for point in range(0, len(keypoints[0]), 2):
                cropped_height = y2 - y1
                cropped_width = x2 - x1

                cropped_x = int(keypoints[cone["id"]][point] / 100.0 * cropped_width)
                cropped_y = int(keypoints[cone["id"]][point+1] / 100.0 * cropped_height)
                
                full_x = int(cropped_x + x1)
                full_y = int(cropped_y + y1)

                # Add to 2D keypoints map for PnP
                keypoints_2d[point//2] = [full_x, full_y]

                if demo:
                    cv2.putText(full_image, str(cone["id"]), (x2, y2), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 1)
                    cv2.circle(full_image, (full_x, full_y), 2, (0, 0, 255), -1)

            # Estimate cone position with PnP using all the 2d keypoints for this cone
            # TODO: Tune PnP output based on real world measurments
            rvec, tvec = PnP(keypoints_2d)
            tvec /= 1000
            tvec[2] /= 100

            # 2D Coordinates for cone position in map
            cone["X"] = tvec[0][0]
            cone["Y"] = tvec[2][0]
        

        total_time_end = time.time()
        print(f"\nTotal Pipeline Time: {total_time_end-total_time_start:.4}")  
        print(f"Cone Detection Time: {cone_det_end-cone_det_start:.4}")
        print(f"Total Keypoint Regr. Time: {keypoint_reg_end-keypoint_reg_start:.4}")


        # TODO: Remove cone_estimates dict and just use ROS msg
        cone_estimates_msg = ConeEstimates()
        cone_estimates_msg.cones = []
        for cone in cone_estimates.values():
            cone_estimate_msg = ConeEstimate()
            cone_estimate_msg.id = cone["id"]
            cone_estimate_msg.label = int(cone["label"])
            cone_estimate_msg.x = cone["X"]
            cone_estimate_msg.y = cone["Y"]
            cone_estimates_msg.cones.append(cone_estimate_msg)

        self.publisher.publish(cone_estimates_msg)

        if demo:
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
        
            for cone in cone_estimates.values():   
                x = cone["X"]
                y = cone["Y"]
                if cone['label'] in colors.keys():
                    plt.scatter(x, y, color=colors[cone['label']])
                plt.annotate(f'{cone["id"]}', (x, y))

            plt.show()
        

def main(args=None):
    rclpy.init(args=args)

    cone_estimation = ConeEstimation('src/vision/vision/models/yolov8n-100e.pt', 'src/vision/vision/models/keypoint_regression.pth')
    
    rclpy.spin(cone_estimation) # Node is kept alive until killed with ctrl+c
    cone_estimation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
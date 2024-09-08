#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from messages.msg import ConeEstimates, ConeEstimate
import cv2
import numpy as np
from ultralytics import YOLO
from std_msgs.msg import String


MODEL_SRC = 'models/yolov10n-V2.pt'

class InferenceNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('inference_node', anonymous=True)

        self.cone_detection_model = YOLO(MODEL_SRC)
        rospy.loginfo("YOLO model loaded.")

        self.bridge = CvBridge()

        self.left_sub = rospy.Subscriber('/stereo/left/image_raw', Image, self.inference)
        #self.right_sub = rospy.Subscriber('/stereo/right/image_raw', Image, self.inference)

        self.cone_pub = rospy.Publisher('/inference', String, queue_size=1)

    def inference(self, img):
        img = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
        self.cone_detection_model.predict(img)
        self.cone_pub.publish("inf done")



if __name__ == '__main__':
    node = InferenceNode()
    rospy.spin()


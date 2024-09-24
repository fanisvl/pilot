#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaver:
    def __init__(self, interval, save_dir):
        rospy.init_node('image_saver', anonymous=True)
        self.bridge = CvBridge()
        self.left_image_sub = rospy.Subscriber('left_cam/raw', Image, self.left_image_callback)
        self.right_image_sub = rospy.Subscriber('right_cam/raw', Image, self.right_image_callback)
        self.left_image_received = False
        self.right_image_received = False
        self.left_image = None
        self.right_image = None
        self.image_count = 0
        self.interval = interval
        self.save_dir = save_dir

        # Timer to save images every 5 seconds
        self.timer = rospy.Timer(rospy.Duration(self.interval), self.save_images_timer)

        if not os.path.exists(self.save_dir):
            os.mkdir(self.save_dir)

    def left_image_callback(self, msg):
        self.left_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.left_image_received = True

    def right_image_callback(self, msg):
        self.right_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.right_image_received = True

    def save_images_timer(self, event):
        if self.left_image_received and self.right_image_received:
            self.image_count += 1
            cv2.imwrite(f'{self.save_dir}/left_{self.image_count}.png', self.left_image)
            cv2.imwrite(f"{self.save_dir}/right_{self.image_count}.png", self.right_image)
            rospy.loginfo(f'Saved images: left_{self.image_count}.png, right_{self.image_count}.png')
            self.left_image_received = False
            self.right_image_received = False

if __name__ == '__main__':
    try:
        interval = 3
        save_dir = 'calibration_images'
        image_saver = ImageSaver(interval, save_dir)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

def main():
    # Initialize the ROS node
    rospy.init_node('camera_publisher', anonymous=True)

    # Create a CvBridge object for converting between ROS and OpenCV images
    bridge = CvBridge()

    # Create a publisher for the camera images
    pub_camera = rospy.Publisher('camera', Image, queue_size=1)

    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    # these are not working for some reason
    # temp fix: resize frame before publishing
    #cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280.0)
    #cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720.0)


    rospy.loginfo(f"Resolution image: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
    rospy.loginfo(f"Frames per second (attempted): {cap.get(cv2.CAP_PROP_FPS)}")


    if not cap.isOpened():
        rospy.logerr("Camera is not opened.")
        return

    # Set the loop rate
    rate = rospy.Rate(10)# Hz
  
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        frame = cv2.resize(frame, (640, 480))
        if not ret:
            rospy.loginfo("No frame info")
            break

        # Convert the OpenCV image to a ROS Image message
        image_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        # Publish the image
        pub_camera.publish(image_msg)

        # Sleep to maintain the loop rate
        rate.sleep()

    # Release the camera
    cam.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


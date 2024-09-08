#!/usr/bin/python3
import os
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from time import sleep

class FakeCameraPublisher:
    def __init__(self, image_directory, fps):
        # Initialize the ROS node
        rospy.init_node('fake_camera_publisher', anonymous=True)
        
        # Create a publisher object
        self.publisher_ = rospy.Publisher('/camera', Image, queue_size=1)
        self.bridge = CvBridge()

        # Initialize class variables
        self.image_directory = image_directory
        self.fps = fps
        self.image_files = sorted(os.listdir(self.image_directory))
        self.num_images = len(self.image_files)
        self.current_image_idx = 0

        # Timer setup for image publishing
        self.timer_period = 1.0 / self.fps
        self.timer = rospy.Timer(rospy.Duration(self.timer_period), self.publish_image)

        rospy.loginfo(f"Camera Publisher initialized with {self.num_images} images.")

    def publish_image(self, event):
        if self.current_image_idx < self.num_images:
            image_filename = os.path.join(self.image_directory, self.image_files[self.current_image_idx])
            frame = cv2.imread(image_filename)

            if frame is not None:
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.publisher_.publish(ros_image)
                rospy.loginfo(f"Published image {self.current_image_idx + 1}/{self.num_images}")
                # cv2.imshow("Camera Image", frame)
                cv2.waitKey(1)
                self.current_image_idx += 1
            else:
                rospy.logwarn(f"Failed to read image {self.image_files[self.current_image_idx]}")
        else:
            rospy.loginfo("Finished publishing all images. Reset.")
            self.current_image_idx = 0

def main():
    image_directory = '/workspace/autopilot/src/perception/src/camera_data/run1'
#!/usr/bin/python3
    fps = 1

    # Create the FakeCameraPublisher instance
    camera_publisher = FakeCameraPublisher(image_directory, fps)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    main()


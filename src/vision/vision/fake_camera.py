import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from time import sleep

class FakeCameraPublisher(Node):

    def __init__(self, image_directory, fps):
        super().__init__("FakeCameraPublisher")
        self.publisher_ = self.create_publisher(Image, '/camera', 10)
        self.bridge = CvBridge()

        self.image_directory = image_directory
        self.fps = fps

        self.image_files = sorted(os.listdir(self.image_directory))
        self.num_images = len(self.image_files)
        self.current_image_idx = 0

        timer_period = 1.0 / self.fps  # Calculate period based on desired fps
        self.timer = self.create_timer(timer_period, self.publish_image)
        # self.frame_size = frame_size

        self.get_logger().info(f"Camera Publisher initialized with {self.num_images} images.")

    def publish_image(self):
        if self.current_image_idx < self.num_images:
            image_filename = os.path.join(self.image_directory, self.image_files[self.current_image_idx])
            frame = cv2.imread(image_filename)

            if frame is not None:
                # frame = cv2.resize(frame, self.frame_size)
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.publisher_.publish(ros_image)
                self.get_logger().info(f"Published image {self.current_image_idx + 1}/{self.num_images}")
                # cv2.imshow("Camera Image", frame)
                cv2.waitKey(1) 
                self.current_image_idx += 1

            else:
                self.get_logger().warning(f"Failed to read image {self.image_files[self.current_image_idx]}")
        else:
            self.get_logger().info("Finished publishing all images. Reset.")
            self.current_image_idx = 0

def main(args=None):
    rclpy.init(args=args)

    image_directory = 'src/vision/vision/camera_data/single_img/'
    # TODO: Changing the frame size of the input image causes bugs in cone estimation
    # frame_size = (640, 480)
    fps = 1

    camera_publisher = FakeCameraPublisher(image_directory, fps)
    
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

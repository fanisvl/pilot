import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):

    def __init__(self):
        super().__init__("CameraPublisher")
        self.publisher_ = self.create_publisher(Image, '/camera', 10)
        timer_period = 0.01 # sec
        self.timer = self.create_timer(timer_period, self.publish_image)
        self.bridge = CvBridge()

        # Initialize video streams for both cameras
        self.video_stream_0 = cv2.VideoCapture('/dev/video0')

        self.get_logger().info("Camera Publisher initialized.")


    def publish_image(self):
        # Read frames from both camera streams
        read_success0, frame_0 = self.video_stream_0.read()

        if read_success0:
            # Convert frames from NumPy arrays to ROS Image messages
            ros_image_0 = self.bridge.cv2_to_imgmsg(frame_0, "bgr8")

            # Publish the ROS Image messages
            self.publisher_.publish(ros_image_0)

            cv2.imshow("Camera Image", frame_0)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    camera_publisher = CameraPublisher()
    
    rclpy.spin(camera_publisher) # Node is kept alive until killed with ctrl+c
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
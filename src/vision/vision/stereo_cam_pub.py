import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class StereoCamPub(Node):

    def __init__(self):
        super().__init__("stereo_cam_pub")
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        timer_period = 0.01 # sec
        self.timer = self.create_timer(timer_period, self.publish_image)
        self.bridge = CvBridge()

        # Initialize video streams for both cameras
        self.video_stream_0 = cv2.VideoCapture('/dev/video0')
        # self.video_stream_1 = cv2.VideoCapture('/dev/video1')

        self.get_logger().info("stereo_cam Publisher initialized.")


    def publish_image(self):
        # Read frames from both camera streams
        read_success0, frame_0 = self.video_stream_0.read()
        # read_success1, frame_1 = self.video_stream_1.read()

        # if read_success0 and read_success1:
        if read_success0:
            # Convert frames from NumPy arrays to ROS Image messages
            ros_image_0 = self.bridge.cv2_to_imgmsg(frame_0, "bgr8")
            # ros_image_1 = self.bridge.cv2_to_imgmsg(frame_1, "bgr8")

            # Publish the ROS Image messages
            self.publisher_.publish(ros_image_0)
            # self.publisher_.publish(ros_image_1)


def main(args=None):
    rclpy.init(args=args)

    stereo_cam_pub = StereoCamPub()
    
    rclpy.spin(stereo_cam_pub) # Node is kept alive until killed with ctrl+c
    stereo_cam_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
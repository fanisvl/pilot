import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class StereoCamSub(Node):

    def __init__(self):
        super().__init__("stereo_cam_sub")

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.bridge = CvBridge()
        self.get_logger().info("stereo_cam Subscriber initialized.")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error("Error converting ROS Image to OpenCV image: {0}".format(e))
            return

        # Display the image
        cv2.imshow("Camera Image", cv_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    stereo_cam_sub = StereoCamSub()
    
    rclpy.spin(stereo_cam_sub) # Node is kept alive until killed with ctrl+c
    stereo_cam_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
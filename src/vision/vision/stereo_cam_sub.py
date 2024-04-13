import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class StereoCamSub(Node):

    def __init__(self):
        super().__init__("stereo_cam_sub")

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.model = YOLO('model/yolov8s700.pt') 
        self.bridge = CvBridge()
        self.get_logger().info("stereo_cam Subscriber initialized.")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error("Error converting ROS Image to OpenCV image: {0}".format(e))
            return

        annotated_image = self.inference(cv_image)

        # Display the image
        cv2.imshow("Camera Image", annotated_image)
        cv2.waitKey(1)

    def inference(self, image):  
        result = self.model.predict(image)[0]
        boxes = result.boxes  # Boxes object for bbox outputs
        for box in boxes:
            coordinates = box.xyxy[0]
            x1, y1, x2, y2 = int(coordinates[0]), int(coordinates[1]), int(coordinates[2]), int(coordinates[3])
            conf = box.conf.item()
            class_pred = box.cls.item()
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{self.model.names[class_pred]}: conf: {conf}"
            cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        
        return image

def main(args=None):
    rclpy.init(args=args)

    stereo_cam_sub = StereoCamSub()
    
    rclpy.spin(stereo_cam_sub) # Node is kept alive until killed with ctrl+c
    stereo_cam_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
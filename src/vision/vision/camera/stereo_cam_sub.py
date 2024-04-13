import rclpy
from rclpy.node import Node

class StereoCamSub(Node):

    def __init__(self):
        super().__init__("my_node_name")
        self.get_logger().info("Hello world!!!")

def main(args=None):
    rclpy.init(args=args)

    node = MyNode()
    
    rclpy.spin(node) # Node is kept alive until killed with ctrl+c

    rclpy.shutdown()

if __name__ == '__main__':
    main()
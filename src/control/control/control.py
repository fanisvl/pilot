import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class Control(Node):

    def __init__(self):
        super().__init__("control")
        self.get_logger().info("Control Node initialized")

        self.subscription = self.create_subscription(
            Float64,
            '/pure_pursuit_steering_angle',
            self.control,
            10
        )

        self.TURN_MIN = 0.0
        self.TURN_A   = 0.25
        self.TURN_MID = 0.50
        self.TURN_B   = 0.75
        self.TURN_MAX = 1.0

    def control(self, steering_angle_msg):
        # TODO
        """
        For speed just use a very low value constant for now
        It later be mapped as a function of the steering angle
        INPUT: Steering angle from pure pursuit
        OUTPUT: Steering commands to actuators
        """
        steering_angle = float(steering_angle_msg.data)
        print(f"Normalized: {self.normalize_steering_angle(steering_angle)}")

    def normalize_steering_angle(self,steering_angle):
        """"
        Helper function for control
        TODO: Absolute steering angle from pure pursuit is biased to values above 1, that's why if steering_angle < 1 -> TURN MIN, 
        this should probably be fixed in pure pursuit

        INPUT: Steering angle output from pure pursuit ranges from (-1.57, 1.57) = (-pi/2, pi/2)
        OUTPUT: Normalize to (-1, 1)
        """

        # determine turn heading
        if steering_angle >= 0.0:
            turn_heading = 1 # right
        else:
            turn_heading = -1 # left

        normalized_steering_angle = turn_heading
        steering_angle = abs(steering_angle)
        print(steering_angle)
        
        # Normalize the steering angle
        if steering_angle < 1.0:
            normalized_steering_angle *= self.TURN_MIN
        elif 1.0 <= steering_angle < 1.1425:
            normalized_steering_angle *= self.TURN_A
        elif 1.1425 <= steering_angle < 1.285:
            normalized_steering_angle *= self.TURN_MID
        elif 1.285 <= steering_angle < 1.4275:
            normalized_steering_angle *= self.TURN_B
        elif 1.4275 <= steering_angle <= 1.57:
            normalized_steering_angle *= self.TURN_MAX

        return normalized_steering_angle


def main(args=None):
    rclpy.init(args=args)

    control = Control()
    
    rclpy.spin(control) # Node is kept alive until killed with ctrl+c

    rclpy.shutdown()

if __name__ == '__main__':
    main()
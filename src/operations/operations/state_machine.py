import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

class System(Node):
    def __init__(self):
        super().__init__('system')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('state', None),
                ('PI', None),
                ('g', None)
            ]
        )

        self.state = self.get_parameter('state').value
        self.PI = self.get_parameter('PI').value
        self.g = self.get_parameter('g').value


def system_entry_point(args=None):
    rclpy.init(args=args)

    try:
        system = System()

        rclpy.spin(system)
    except KeyboardInterrupt:
        # Log into console with logger
        pass

    system.destroy_node()
    rclpy.shutdown()


class Car(Node):
    def __init__(self):
        super().__init__('car')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('position', None),
                ('direction', None),
                ('motor', None),
                ('steering', None)
            ]
        )

        packed_list = self.get_parameter('position').value
        self.position = Vector3()
        self.position.x = packed_list[0]
        self.position.y = packed_list[1]
        self.position.z = packed_list[2]

        packed_list = self.get_parameter('direction').value
        self.direction = Vector3()
        self.direction.x = packed_list[0]
        self.direction.y = packed_list[1]
        self.direction.z = packed_list[2]

        self.motor = self.get_parameter('motor').value
        self.steering = self.get_parameter('steering').value

def car_entry_point(args=None):
    rclpy.init(args=args)

    try:
        car = Car()

        rclpy.spin(car)
    except KeyboardInterrupt:
        # Log into console with logger
        pass

    car.destroy_node()
    rclpy.shutdown()

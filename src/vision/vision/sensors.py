import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

class StereoCamera(Node):
    def __init__(self):
        super().__init__('stereo_camera')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('resolution_x', None),
                ('resolution_y', None),
                ('sensor_width', None),
                ('sensor_height', None),
                ('baseline', None),
                ('focal_length', None),
                ('angle_of_view_diagonal', None),
                ('angle_of_view_horizontal', None),
                ('angle_of_view_vertical', None),
                ('pixel_size', None)
            ]
        )

        # passing ros parameters to our python class
        self.resolution_x = self.get_parameter('resolution_x').value
        self.resolution_y = self.get_parameter('resolution_y').value
        self.sensor_width = self.get_parameter('sensor_width').value
        self.sensor_height = self.get_parameter('sensor_height').value
        self.baseline = self.get_parameter('baseline').value
        self.focal_length = self.get_parameter('focal_length').value
        self.angle_of_view_diagonal = self.get_parameter('angle_of_view_diagonal').value
        self.angle_of_view_horizontal = self.get_parameter('angle_of_view_horizontal').value
        self.angle_of_view_vertical = self.get_parameter('angle_of_view_vertical').value
        self.pixel_size = self.get_parameter('pixel_size').value

def stereo_camera_entry_point(args=None):
    rclpy.init(args=args)

    try:
        stereo_camera = StereoCamera()

        rclpy.spin(stereo_camera)
    except KeyboardInterrupt:
        # Log into console with logger
        pass

    stereo_camera.destroy_node()
    rclpy.shutdown()

class IMU(Node):
    def __init__(self):
        super().__init__('imu')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('accelerometer', None),
                ('gyroscope', None),
                ('magnetometer', None)
            ]
        )

        # unpack and store as Vector3
        packed_list = self.get_parameter('accelerometer').value
        self.accelerometer = Vector3()
        self.accelerometer.x = packed_list[0]
        self.accelerometer.y = packed_list[1]
        self.accelerometer.z = packed_list[2]

        packed_list = self.get_parameter('gyroscope').value
        self.gyroscope = Vector3()
        self.gyroscope.x = packed_list[0]
        self.gyroscope.y = packed_list[1]
        self.gyroscope.z = packed_list[2]

        packed_list = self.get_parameter('magnetometer').value
        self.magnetometer = Vector3()
        self.magnetometer.x = packed_list[0]
        self.magnetometer.y = packed_list[1]
        self.magnetometer.z = packed_list[2]

def imu_entry_point(args=None):
    rclpy.init(args=args)

    try:
        imu = IMU()

        rclpy.spin(imu)
    except KeyboardInterrupt:
        # Log into console with logger
        pass

    imu.destroy_node()
    rclpy.shutdown()

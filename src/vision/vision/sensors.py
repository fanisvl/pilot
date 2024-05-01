import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3
import rclpy.timer
from sensor_msgs.msg import Imu
from math import acos, sqrt

from rcl_interfaces.msg import SetParametersResult

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

        self.raw_data_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.raw_data_callback,
            10
        )

        # rotation estimation using accelerometer, gyroscope, magnetometer
        # initialization parameters

        self.accelerometer_biases = [0.0, 0.0, 0.0]
        self.gyroscope_biases = [0.0, 0.0, 0.0]
        self.magnetometer_biases = [0.0, 0.0, 0.0]

        self.tolerance = 1e4

        self.is_calibrating = True # we want to calibrate on startup
        self.initialize_calibration()

        self.is_gyroscope_data_in_rad = True

        self.previous_time = 0.0

        # normalization parameters

        # add in yaml file is_data_normalized
        # if it is not normalized need to provide min, max range
        # if abs(min) == abs(max) then we enable  
        self.is_data_normalized = True # this should also be in yaml file

        if(not self.is_data_normalized):
            # these values should also be in yaml file
            self.accelerometer_max_value = 1.0
            self.accelerometer_min_value = 1.0

            self.gyroscope_max_value = 1.0
            self.gyroscope_min_value = 1.0

            self.magnetometer_max_value = 1.0
            self.magnetometer_min_value = 1.0

        # accelerometer_initial_biases: [0.0, 0.0, 0.0]


        # rotation angles
        # all the point is to calculate as accurate as possible this
        self.rotation_estimations = [] # contains arrays of 3 elemnts for each axis X-Y-Z
        self.rotation_angles = [0.0, 0.0, 0.0]


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

        self.add_on_set_parameters_callback(self.parameter_callback)


    def raw_data_callback(self, msg):
        
        # throttle mechanism we dont want to read the same values over and over again
        if(self.previous_time == msg.header.stamp.sec):
            return
        
        # normalize data before processing it

        if(not self.is_data_normalized):
            msg.linear_acceleration.x /= self.accelerometer_max_value
            msg.linear_acceleration.y /= self.accelerometer_max_value
            msg.linear_acceleration.z /= self.accelerometer_max_value

            msg.angular_velocity.x /= self.gyroscope_max_value
            msg.angular_velocity.y /= self.gyroscope_max_value
            msg.angular_velocity.z /= self.gyroscope_max_value

            # magnetometer data is not available TODO
        
        if(self.is_calibrating):

            self.average_accelerometer_values[0] += msg.linear_acceleration.x / self.calibration_samples
            self.average_accelerometer_values[1] += msg.linear_acceleration.y / self.calibration_samples
            self.average_accelerometer_values[2] += (msg.linear_acceleration.z - 9.80665) / self.calibration_samples

            self.average_gyroscope_values[0] += msg.angular_velocity.x / self.calibration_samples
            self.average_gyroscope_values[1] += msg.angular_velocity.y / self.calibration_samples
            self.average_gyroscope_values[2] += msg.angular_velocity.z / self.calibration_samples
            
            # magnetometer data is not available TODO
            
            self.calibration_sample_counter += 1

            if(self.calibration_sample_counter == self.calibration_samples):
                self.is_calibrating = False
                self.deinitialize_calibration()
                
                self.get_logger().info("Accelerometer Biases: [%f," % self.accelerometer_biases[0] + "%f," % self.accelerometer_biases[1] + "%f]" % self.accelerometer_biases[2])
                self.get_logger().info("Gyroscope Biases: [%f," % self.gyroscope_biases[0] + "%f," % self.gyroscope_biases[1] + "%f]" % self.gyroscope_biases[2])
            
            return
        
        # time
        self.dt = msg.header.stamp.sec - self.previous_time
        self.previous_time = msg.header.stamp.sec
        
        # Sensor value correction 

        msg.linear_acceleration.x -= self.accelerometer_biases[0]
        msg.linear_acceleration.y -= self.accelerometer_biases[1]
        msg.linear_acceleration.z -= self.accelerometer_biases[2]

        msg.angular_velocity.x -= self.gyroscope_biases[0]
        msg.angular_velocity.y -= self.gyroscope_biases[1]
        msg.angular_velocity.z -= self.gyroscope_biases[2]

        # magnetometer data is not available TODO

        #self.get_logger().info("Accelerometer: [%f," % msg.linear_acceleration.x + "%f," % msg.linear_acceleration.y + "%f]" % msg.linear_acceleration.z)
        #self.get_logger().info("Accelerometer NO TIME: [%f," % (msg.linear_acceleration.x * msg.header.stamp.sec * msg.header.stamp.sec) + "%f," % (msg.linear_acceleration.y * msg.header.stamp.sec * msg.header.stamp.sec) + "%f]" % (msg.linear_acceleration.z * msg.header.stamp.sec * msg.header.stamp.sec))
        #self.get_logger().info("Gyroscope: [%f," % msg.angular_velocity.x + "%f," % msg.angular_velocity.y + "%f]" % msg.angular_velocity.z)
        #self.get_logger().info("Gyroscope NO TIME: [%f," % (msg.angular_velocity.x * msg.header.stamp.sec) + "%f," % (msg.angular_velocity.y * msg.header.stamp.sec) + "%f]" % (msg.angular_velocity.z * msg.header.stamp.sec))

        parameters = [rclpy.parameter.Parameter('accelerometer',
                                                rclpy.Parameter.Type.DOUBLE_ARRAY,
                                                [self.accelerometer.x,
                                                 self.accelerometer.y,
                                                 self.accelerometer.z]),
                      rclpy.parameter.Parameter('gyroscope',
                                                rclpy.Parameter.Type.DOUBLE_ARRAY,
                                                [self.gyroscope.x,
                                                 self.gyroscope.y,
                                                 self.gyroscope.z])]
        

        # magnetometer data is not available TODO

        self.set_parameters(parameters)


        # perform kalman filter on rotation_estimations

        self.kalman_filter()

    def kalman_filter():
        pass # TODO

    def parameter_callback(self, parameters):
        for parameter in parameters:
            if parameter.name == 'accelerometer':
                self.accelerometer.x = parameter.value[0]
                self.accelerometer.y = parameter.value[1]
                self.accelerometer.z = parameter.value[2]

                self.accelerometer_magnitude = sqrt(self.accelerometer.x * self.accelerometer.x + self.accelerometer.y * self.accelerometer.y + self.accelerometer.z * self.accelerometer.z)

                self.get_logger().info("Accelerometer [%f," % self.accelerometer.x + "%f," % self.accelerometer.y + "%f]" % self.accelerometer.z)

                self.rotation_estimations.append([acos(self.accelerometer.x / self.accelerometer_magnitude),
                                                  acos(self.accelerometer.y / self.accelerometer_magnitude),
                                                  acos(self.accelerometer.z / self.accelerometer_magnitude)])
            elif parameter.name == 'gyroscope':
                self.gyroscope.x = parameter.value[0]
                self.gyroscope.y = parameter.value[1]
                self.gyroscope.z = parameter.value[2]
                self.get_logger().info("Gyroscope [%f," % self.gyroscope.x + "%f," % self.gyroscope.y + "%f]" % self.gyroscope.z)

                self.rotation_estimations.append([self.rotation_angles[0] + self.gyroscope.x * self.dt, 
                                                  self.rotation_angles[1] + self.gyroscope.y * self.dt, 
                                                  self.rotation_angles[2] + self.gyroscope.z * self.dt])

        return SetParametersResult(successful=True)

    def initialize_calibration(self, samples=1000):
        # calibrate each axis
        self.average_accelerometer_values = [0.0, 0.0, 0.0]
        self.average_gyroscope_values = [0.0, 0.0, 0.0]
        self.average_magnetometer_values = [0.0, 0.0, 0.0]

        self.calibration_samples = samples
        self.calibration_sample_counter = 0

    def deinitialize_calibration(self):
        for i in range(3):
            self.accelerometer_biases[i] = self.average_accelerometer_values[i]
            self.gyroscope_biases[i] = self.average_gyroscope_values[i]
            self.magnetometer_biases[i] = self.average_magnetometer_values[i]

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

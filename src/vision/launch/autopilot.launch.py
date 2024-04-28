import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    launch_description = LaunchDescription()

    config_path = os.path.join(get_package_share_directory('vision'), 'config')

    system_params_path = os.path.join(config_path, 'system.yaml')
    car_params_path = os.path.join(config_path, 'car.yaml')
    stereo_camera_params_path = os.path.join(config_path, 'IMX219-83_Stereo.yaml')
    imu_params_path = os.path.join(config_path, 'ICM20948_IMU.yaml')

    system_node = Node(
        package='vision',
        executable='system_entry_point',
        name='system',
        parameters=[system_params_path]
    )
    launch_description.add_action(system_node)

    car_node = Node(
        package='vision',
        executable='car_entry_point',
        name='car',
        parameters=[car_params_path]
    )
    launch_description.add_action(car_node)

    stereo_camera_node = Node(
        package='vision',
        executable='stereo_camera_entry_point',
        name='stereo_camera',
        parameters=[stereo_camera_params_path]
    )
    launch_description.add_action(stereo_camera_node)

    imu_node = Node(
        package='vision',
        executable='imu_entry_point',
        name='imu',
        parameters=[imu_params_path]
    )
    launch_description.add_action(imu_node)


    return launch_description

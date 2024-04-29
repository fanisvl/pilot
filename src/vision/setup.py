import os
from glob import glob
from setuptools import setup

package_name = 'vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
	(os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = vision.test_node:main",
            "stereo_cam_pub = vision.stereo_cam_pub:main",
            "stereo_cam_sub = vision.stereo_cam_sub:main",
            "stereo_camera_entry_point = vision.sensors:stereo_camera_entry_point",
            "imu_entry_point = vision.sensors:imu_entry_point",
        ],
    },
)

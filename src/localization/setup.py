from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # required ROS index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # ✅ include launch and config files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='rheakhimgaudiano00@gmail.com',
    description='Localization and SLAM integration with IMU and LiDAR',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sensor_fusion = localization.sensor_fusion:main',
            'arduino_motor_bridge = localization.arduino_motor_bridge:main',
            'frontier_detector = localization.frontier_detector:main',
            'exploration_coordinator = localization.exploration_coordinator:main',
            'exploration_coordinator_v2 = localization.exploration_coordinator_v2:main',
            'motor_controller = localization.motor_controller:main',
            'ultrasonic_explorer = localization.ultrasonic_explorer:main',
            'ultrasonic_servo_sweeper = localization.ultrasonic_servo_sweeper:main',
            'phase5_metrics = localization.phase5_metrics:main',
        ],
    },
)

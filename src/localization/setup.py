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
            'arduino_motor_bridge_simple = localization.arduino_motor_bridge_simple:main',
            'frontier_detector = localization.frontier_detector:main',
            'exploration_coordinator_simple = localization.exploration_coordinator_simple:main',
            'lidar_explorer = localization.lidar_explorer:main',
            'map_odom_fallback = localization.map_odom_fallback:main',
            'scan_timestamp_fix = localization.scan_timestamp_fix:main',
            'dynamic_obstacle_tracker = localization.dynamic_obstacle_tracker:main',
        ],
    },
)

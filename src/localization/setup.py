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
        ],
    },
)

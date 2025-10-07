from setuptools import setup
from glob import glob
import os

package_name = 'fea_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=['slam'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rhea Gaudiano',
    maintainer_email='rheakhimgaudiano00@gmail.com',
    description='FEA-SLAM Python nodes',
    license='MIT',
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'raspi_fusion = slam.raspi_fusion:main',
        ],
    },
)
from setuptools import setup, find_packages
import os

package_name = 'visualization_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/visualization_launch.py']),
        ('share/' + package_name + '/config', ['config/visualization.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='rheakhimgaudiano00@gmail.com',
    description='Map visualization package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'map_node = visualization_pkg.map:main',
        ],
    },
)


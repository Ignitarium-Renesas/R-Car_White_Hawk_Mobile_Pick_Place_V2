from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'rcar_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Package resource index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/params', glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nirmalka',
    maintainer_email='nirmal.ka@ignitarium',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            "rcar_odom = rcar_robot.rcar_odometry:main",
            "rcar_odom_smoothed = rcar_robot.rcar_odometry_smoothed:main",
            "base_driver = rcar_robot.base_driver:main",
            "rcar_tf = rcar_robot.rcar_tf:main",
            "rcar_imu = rcar_robot.rcar_imu:main",
            "dynamic_params = rcar_robot.dynamic_params:main",
            "goal_pose_node = rcar_robot.goal_pose_node:main"
            
        ],
    },
)

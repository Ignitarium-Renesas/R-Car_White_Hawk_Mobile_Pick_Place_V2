from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # IMU is rotated 180 degrees in yaw
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0', '0.0', '0.04', '0.0', '0.0', '0.0', 'base_link', 'imu_link'],
            output='screen',
        ),

        # LiDAR is also rotated 180 degrees in yaw
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['-0.09', '0.0', '0.05', '3.14159', '0.0', '0.0', 'base_link', 'base_scan'],
            output='screen',
        ),

        # Base footprint to base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_footprint', 'base_link'],
            output='screen',
        ),
    ])

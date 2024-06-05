"""
Launch file which calls all necessary nodes for running the Lidar.

Based on: https://github.com/babakhani/rplidar_ros2/blob/ros2/launch/rplidar.launch.py
Date of Retrieval: 04.06.2024

"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    This function sets all need parameters for running the Lidar.
    See https://github.com/lukasholtkamp/b.ob/wiki/Hardware-Documentation for details about how to connect the Lidar.
    return: a Launch Description with all needed arguments for Lidar
    """
    return LaunchDescription([
        Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,  # A1 / A2
                # 'serial_baudrate': 256000, # A3
                'frame_id': 'lidar_frame',
                'inverted': False,
                'angle_compensate': True,
            }],
        ),
    ])
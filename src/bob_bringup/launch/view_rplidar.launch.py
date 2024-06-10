"""
Launch file which calls all necessary nodes for running rviz.

Based on: https://github.com/babakhani/rplidar_ros2/blob/ros2/launch/view_rplidar.launch.py
Date of Retrieval: 04.06.2024

"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    """
    This function sets all need parameters for running rviz.
    return: a Launch Description with all needed arguments for rviz
    """
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', [ThisLaunchFileDir(), '/../rviz/rplidar.rviz']],
        )
    ])
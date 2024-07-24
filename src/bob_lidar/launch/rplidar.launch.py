"""
Launch file which calls all necessary nodes for running B.ob.

Based on: https://github.com/Slamtec/rplidar_ros/blob/ros2/launch/rplidar_a1_launch.py
Date of Retrieval: 23.07.2024

"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from pathlib import Path


def generate_launch_description():
    """
    This function finds all needed parameters for the lidar node.
    return: a Launch Description with all needed arguments and nodes
    """

    # Parameters for the Lidar
    channel_type = LaunchConfiguration("channel_type", default="serial")
    serial_port = LaunchConfiguration("serial_port", default="/dev/ttyUSB0")
    serial_baudrate = LaunchConfiguration("serial_baudrate", default="115200")
    frame_id = LaunchConfiguration("frame_id", default="lidar_frame")
    inverted = LaunchConfiguration("inverted", default="false")
    angle_compensate = LaunchConfiguration("angle_compensate", default="true")
    scan_mode = LaunchConfiguration("scan_mode", default="Standard")

    # Get config path of the PID settings
    pid_config_filepath = os.path.join(
        Path.cwd(), "src", "bob_lidar", "config", "lidar.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bob_lidar_params",
                default_value=pid_config_filepath,
                description="Full path to the ROS2 parameters file to use for the filtering lidar node",
            ),
            DeclareLaunchArgument(
                "channel_type",
                default_value=channel_type,
                description="Specifying channel type of lidar",
            ),
            DeclareLaunchArgument(
                "serial_port",
                default_value=serial_port,
                description="Specifying usb port to connected lidar",
            ),
            DeclareLaunchArgument(
                "serial_baudrate",
                default_value=serial_baudrate,
                description="Specifying usb port baudrate to connected lidar",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value=frame_id,
                description="Specifying frame_id of lidar",
            ),
            DeclareLaunchArgument(
                "inverted",
                default_value=inverted,
                description="Specifying whether or not to invert scan data",
            ),
            DeclareLaunchArgument(
                "angle_compensate",
                default_value=angle_compensate,
                description="Specifying whether or not to enable angle_compensate of scan data",
            ),
            DeclareLaunchArgument(
                "scan_mode",
                default_value=scan_mode,
                description="Specifying scan mode of lidar",
            ),
            Node(
                package="bob_lidar",
                executable="rplidar_node",
                name="rplidar_node",
                parameters=[
                    pid_config_filepath,
                    {
                        "channel_type": channel_type,
                        "serial_port": serial_port,
                        "serial_baudrate": serial_baudrate,
                        "frame_id": frame_id,
                        "inverted": inverted,
                        "angle_compensate": angle_compensate,
                        "scan_mode": scan_mode,
                    },
                ],
                output="screen",
            ),
        ]
    )


"""
Launch file for running the teleoperation node

Based on: https://github.com/TheNoobInventor/lidarbot/blob/main/lidarbot_base/src/lidarbot_hardware.cpp
Date of Retrieval: 17.05.2024

"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions


def generate_launch_description():
    """
    This function finds the parameter file and passes them to the teleop Node.
    return: a Launch Description with all needed arguments and nodes
    """
    # Get config path of the teleop settings
    config_filepath = os.path.join(
        Path.cwd(), "src", "bob_teleop", "config", "ps2.config.yaml"
    )

    # Launch node
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                name="teleop_twist_joy_node",
                parameters=[
                    config_filepath,
                ],
                remappings=[("/cmd_vel", "/diffbot_base_controller/cmd_vel")],
                output="screen",
            ),
        ]
    )

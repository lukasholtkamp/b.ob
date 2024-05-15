from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from pathlib import Path

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    joy_params = os.path.join(Path.cwd(), "config", "xbox.config.yaml")

    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_node",
        parameters=[joy_params],
        output = "screen",
    )

    return LaunchDescription(
        [
            # DeclareLaunchArgument(
            #   'use_sim_time',
            #    default_value='false',
            #     description='Use sim time if true'),
            teleop_node,
            # twist_stamper
        ]
    )

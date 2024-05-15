from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from pathlib import Path

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # use_sim_time = LaunchConfiguration('use_sim_time')

    # set the Parameters from BasicDrive.yaml as joy_params
    joy_params = os.path.join(Path.cwd(), "config", "BasicDrive.yaml")

    # starting the joy node
    joy_linux_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
    )

    # starting the node for the drive modes
    sub_node = Node(
        package="joy_linux",
        executable="sub_node",
        output = "screen",
    )

    # starting the teleop node with parameters from BasicDrive.yaml
    # teleop_node = Node(
    #     package="teleop_twist_joy",
    #     executable="teleop_node",
    #     name="teleop_node",
    #     parameters=[joy_params],
    # )

    # twist_stamper = Node(
    #         package='twist_stamper',
    #         executable='twist_stamper',
    #         parameters=[{'use_sim_time': use_sim_time}],
    #         remappings=[('/cmd_vel_in','/diff_cont/cmd_vel_unstamped'),
    #                     ('/cmd_vel_out','/diff_cont/cmd_vel')]
    #      )

    return LaunchDescription(
        [
            # DeclareLaunchArgument(
            #   'use_sim_time',
            #    default_value='false',
            #     description='Use sim time if true'),
            joy_linux_node,
            sub_node,
            # teleop_node,
            # twist_stamper
        ]
    )

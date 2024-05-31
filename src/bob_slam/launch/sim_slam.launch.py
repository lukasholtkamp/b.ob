import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

from pathlib import Path


def generate_launch_description():

    pkg_slam = os.path.join(Path.cwd(), "src", "bob_slam")
    pkg_sim = os.path.join(Path.cwd(), "src", "bob_simulation")

    # Launch configuration variables specific to simulation
    rviz = LaunchConfiguration("rviz")

    declare_rviz = DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    
    sim = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    pkg_sim,'launch','launch_sim.launch.py'
                )]),
    )

    rviz_config_file = os.path.join(pkg_slam, "rviz", "slam_config.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(rviz),
    )

    slam = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    pkg_slam,'launch','online_async_launch.py'
                )])
    )


    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_rviz)

    # Add any actions
    ld.add_action(sim)
    ld.add_action(rviz_node)
    ld.add_action(slam)

    # Launch them all!
    return ld
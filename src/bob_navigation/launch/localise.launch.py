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
    pkg_navigation = os.path.join(Path.cwd(),"src","bob_navigation")

    localisation_params_file = os.path.join(pkg_navigation,"config","mapper_params_localisation.yaml")

    # Launch configuration variables specific to simulation
    rviz = LaunchConfiguration("rviz")
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_rviz = DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=localisation_params_file,
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')
    
    sim = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    pkg_sim,'launch','launch_sim_people.launch.py'
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

    continue_slam = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    pkg_slam,'launch','online_async_launch.py'
                )]),launch_arguments={'slam_params_file': slam_params_file,}.items()
    )


    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_params_file_cmd)

    # Add any actions
    ld.add_action(continue_slam)

    # Launch them all!
    return ld
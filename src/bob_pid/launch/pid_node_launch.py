# launch/pid_node_launch.py
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('bob_pid'),
        'config',
        'pid_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='bob_pid',
            executable='pid_node',
            name='pid_node',
            parameters=[config],
            output="screen",
        )
    ])


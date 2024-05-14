# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

import launch
import launch_ros.actions

from pathlib import Path
import os
import yaml

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("diffdrive_bob"), "urdf", "diffbot.urdf.xacro"]
            ),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = os.path.join(Path.cwd(),'src','diffdrive_bob','bringup','config','diffbot_controllers.yaml')
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ros2_control_demo_description"), "diffbot/rviz", "diffbot.rviz"]
    )

    joy_params = os.path.join(Path.cwd(),'src','bob_teleop','config','xbox.config.yaml')

    with open(joy_params, 'r') as file:
        params = yaml.safe_load(file)

    x_scale = params['teleop_twist_joy_node']['ros__parameters']['scale_linear']['x']

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers,{'linear.x.max_velocity': 0.122*x_scale},{'linear.x.min_velocity': -0.122*x_scale},],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[
            ("/diff_drive_controller/cmd_vel", "/cmd_vel"),
        ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    joy_config = launch.substitutions.LaunchConfiguration('joy_config')
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')
    publish_stamped_twist = launch.substitutions.LaunchConfiguration('publish_stamped_twist')
    config_filepath = launch.substitutions.LaunchConfiguration('config_filepath')

    launch.actions.DeclareLaunchArgument('joy_vel', default_value='cmd_vel'),
    launch.actions.DeclareLaunchArgument('joy_config', default_value='xbox'),
    launch.actions.DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
    launch.actions.DeclareLaunchArgument('publish_stamped_twist', default_value='true'),
    launch.actions.DeclareLaunchArgument('config_filepath', default_value=[
        launch.substitutions.TextSubstitution(text=os.path.join(
            get_package_share_directory('teleop_twist_joy'), 'config', '')),
        joy_config, launch.substitutions.TextSubstitution(text='.config.yaml')]),

    launch_ros.actions.Node(
        package='joy', executable='joy_node', name='joy_node',
        parameters=[{
            'dev': joy_dev,
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
        }]),
    launch_ros.actions.Node(
        package='teleop_twist_joy', executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[config_filepath, {'publish_stamped_twist': publish_stamped_twist}],
        remappings=[('/cmd_vel', '/diffbot_base_controller/cmd_vel')],
        )                               

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)


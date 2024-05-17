# Based on: https://github.com/ros-controls/ros2_control_demos/blob/master/example_2/bringup/launch/diffbot.launch.py
# Date of Retrieval: 17.05.2024

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)

from pathlib import Path
import os
import yaml

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )

    # Initialize Arguments
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("bob_description"), "urdf", "diffbot.urdf.xacro"]
            ),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # Get settings file paths
    robot_controllers = os.path.join(
        Path.cwd(), "src", "bob_bringup", "config", "bob_controllers.yaml"
    )

    joy_params = os.path.join(
        Path.cwd(), "src", "bob_teleop", "config", "xbox.config.yaml"
    )

    # Obtain the x scale
    with open(joy_params, "r") as file:
        params = yaml.safe_load(file)

    x_scale = params["teleop_twist_joy_node"]["ros__parameters"]["scale_linear"]["x"]


    # Implement the launching Nodes with all parameters and declaring all necessary settings
    # Scale the max and min velocities by the x_scale
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_controllers,
            {"linear.x.max_velocity": 0.122 * x_scale},
            {"linear.x.min_velocity": -0.122 * x_scale},
        ],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    # Launch state pub node to publish the transforms
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[
            ("/diff_drive_controller/cmd_vel", "/cmd_vel"),
        ],
    )

    # Launch Joint broadcaster to read all state interfaces
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Launch Controller to convert the command velocities to individual motor speeds
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diffbot_base_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Launch joy node to read gamepad commands
    joy_node = Node(
        package="joy",
        executable="joy_node",
    )

    # Launch drive selection node to be able to switch between the modes
    drive_selection_node = Node(
        package="bob_bringup",
        executable="drive_selection_node",
        output="screen",
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
            )
        )
    )

    nodes = [
        joy_node,
        drive_selection_node,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)

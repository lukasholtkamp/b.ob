"""
Launch file which calls all necessary nodes for running B.ob.

Based on: https://github.com/ros-controls/ros2_control_demos/blob/master/example_2/bringup/launch/diffbot.launch.py
Date of Retrieval: 17.05.2024

"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    LogInfo,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart

from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)

from pathlib import Path
import os
import xacro

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    This function finds all need parameter files, robot descriptions and passes them to the necessary Nodes.
    See https://github.com/lukasholtkamp/b.ob/wiki/Software-Documentation for detail about which nodes.
    return: a Launch Description with all needed arguments and nodes
    """
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use sim time if true",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            name="use_ros2_control",
            default_value="True",
            description="Use ros2_control if true",
        )
    )

    declared_arguments.append(
            DeclareLaunchArgument(
        name="use_robot_localization",
        default_value="True",
        description="Use robot_localization package if true",
    )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    use_robot_localization = LaunchConfiguration("use_robot_localization")

    # Get URDF via xacro

    # Process the URDF file
    pkg_bring_up = os.path.join(Path.cwd(), "src", "bob_bringup")
    pkg_path = os.path.join(Path.cwd(), "src", "bob_description")
    pkg_navigation = os.path.join(Path.cwd(), "src", "bob_navigation")
    xacro_file = os.path.join(pkg_path, "urdf", "bob.urdf.xacro")
    robot_description_config = xacro.process_file(xacro_file)
    ekf_params_file = os.path.join(pkg_navigation, "config/ekf.yaml")

    robot_description = {
        "robot_description": robot_description_config.toxml(),
        "use_sim_time": use_sim_time,
        "use_ros2_control": use_ros2_control,
    }

    # Get settings file paths
    robot_controllers = os.path.join(
        Path.cwd(), "src", "bob_bringup", "config", "bob_controllers.yaml"
    )

    rviz_config_file = os.path.join(
        Path.cwd(), "src", "bob_description", "rviz", "diffbot_view.rviz"
    )

    # Implement the launching Nodes with all parameters and declaring all necessary settings
    # Scale the max and min velocities by the x_scale
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_controllers,
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

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
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

    # Spawn imu_sensor_broadcaser
    start_imu_broadcaster_cmd = Node(
        # condition=IfCondition(use_ros2_control),
        package="controller_manager",
        executable="spawner",
        arguments=["imu_broadcaster"],
    )

    # Delayed imu_broadcaster_spawner action
    start_delayed_imu_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[start_imu_broadcaster_cmd],
        )
    )

    # Start robot localization using an Extended Kalman Filter
    start_robot_localization_cmd = Node(
        condition=IfCondition(use_robot_localization),
        package="robot_localization",
        executable="ekf_node",
        parameters=[ekf_params_file],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_for_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    # Launch Lidar Node from launch file
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_bring_up, "launch", "rplidar.launch.py")]
        )
    )

    # Send message to turn on Xbox controller
    turn_on_xbox = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[
                LogInfo(msg="Turn on Xbox controller"),
                LogInfo(msg="For Testing press Up-D-PAD button"),
                LogInfo(msg="For Basic driving press X button"),
            ],
        )
    )

    nodes = [
        joy_node,
        drive_selection_node,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_for_joint_state_broadcaster_spawner,
        lidar,
        turn_on_xbox,
        start_delayed_imu_broadcaster_spawner,
        start_robot_localization_cmd,
    ]

    return LaunchDescription(declared_arguments + nodes)

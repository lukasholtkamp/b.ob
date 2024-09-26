"""
Launch file which calls all necessary nodes for running B.ob.

Based on: https://github.com/stephenadhi/pedsim_ros/blob/humble/pedsim_gazebo_plugin/launch/gazebo_tb3_house_demo_launch.py
Date of Retrieval: 25.07.2024

"""

import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

from pathlib import Path


def generate_launch_description():

    scene = "new_scenario"

    pkg_path = os.path.join(Path.cwd(), "src", "bob_simulation")
    pkg_teleop = os.path.join(Path.cwd(), "src", "bob_teleop")
    pkg_navigation = os.path.join(Path.cwd(), "src", "bob_navigation")

    gazebo_params_file = os.path.join(pkg_path, "config/gazebo_params.yaml")
    twist_mux_params_file = os.path.join(pkg_teleop, "config/twist_mux.yaml")
    ekf_params_file = os.path.join(pkg_navigation, "config/ekf.yaml")
    world_filename = "empty_lidar.world"
    world_path = os.path.join(pkg_path, "worlds", world_filename)

    pedsim_gazebo_dir = FindPackageShare(package="pedsim_gazebo_plugin").find(
        "pedsim_gazebo_plugin"
    )
    pedsim_dir = get_package_share_directory("pedsim_simulator")
    pedsim_viz_dir = get_package_share_directory("pedsim_visualizer")

    world_model_path = os.path.join(
        pedsim_gazebo_dir, "worlds", "empty_world" + ".world"
    )
    default_pedsim_scene_path = os.path.join(pedsim_dir, "scenarios", scene + ".xml")
    default_pedsim_config_path = os.path.join(pedsim_dir, "config", "params.yaml")

    # Launch configuration variables specific to simulation
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    world = LaunchConfiguration("world")
    use_robot_localization = LaunchConfiguration("use_robot_localization")

    pedsim_scene_file = LaunchConfiguration("pedsim_scene_file")
    pedsim_config_file = LaunchConfiguration("pedsim_config_file")

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_use_ros2_control_cmd = DeclareLaunchArgument(
        name="use_ros2_control",
        default_value="False",
        description="Use ros2_control if true",
    )

    declare_world_cmd = DeclareLaunchArgument(
        name="world",
        default_value=world_model_path,
        description="Full path to the world model to load",
    )

    declare_use_robot_localization_cmd = DeclareLaunchArgument(
        name="use_robot_localization",
        default_value="True",
        description="Use robot_localization package if true",
    )

    declare_pedsim_scene_file_cmd = DeclareLaunchArgument(
        "pedsim_scene_file", default_value=default_pedsim_scene_path, description=""
    )

    declare_pedsim_config_file_cmd = DeclareLaunchArgument(
        "pedsim_config_file", default_value=default_pedsim_config_path, description=""
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_path, "launch", "rsp.launch.py")]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_ros2_control": use_ros2_control,
        }.items(),
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py",
                )
            ]
        ),
        launch_arguments={
            "world": world,
            "extra_gazebo_args": "--ros-args --params-file " + gazebo_params_file,
        }.items(),
    )

    agent_spawner_cmd = Node(
        package="pedsim_gazebo_plugin",
        executable="spawn_pedsim_agents",
        name="spawn_pedsim_agents",
        output="screen",
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "bob", "-topic", "robot_description"],
        output="screen",
    )

    # Start robot localization using an Extended Kalman Filter
    start_robot_localization_cmd = Node(
        condition=IfCondition(use_robot_localization),
        package="robot_localization",
        executable="ekf_node",
        parameters=[ekf_params_file, {"use_sim_time": True}],
    )

    # Launch Joint broadcaster to read all state interfaces
    joint_state_broadcaster_spawner = Node(
        condition=IfCondition(use_ros2_control),
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
        condition=IfCondition(use_ros2_control),
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
    config_filepath = os.path.join(
        Path.cwd(), "src", "bob_teleop", "config", "ps3.config.yaml"
    )

    # Launch drive selection node to be able to switch between the modes
    teleop_node_ros2_control = Node(
        condition=IfCondition(use_ros2_control),
        package="bob_teleop",
        executable="bob_teleop_node",
        parameters=[
            config_filepath,
        ],
        remappings=[("/cmd_vel", "/diffbot_base_controller/cmd_vel_unstamped")],
        output="screen",
    )

    teleop_node_gazebo_control = Node(
        condition=UnlessCondition(use_ros2_control),
        package="bob_teleop",
        executable="bob_teleop_node",
        parameters=[
            config_filepath,
        ],
        output="screen",
    )

    # Start pedsim simulator
    pedsim_launch_cmd = TimerAction(
        period=10.0,  # wait for simulator until launching pedsim
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pedsim_dir, "launch", "simulator_launch.py")
                ),
                launch_arguments={
                    "scene_file": pedsim_scene_file,
                    "config_file": pedsim_config_file,
                    "namespace": "",
                    "use_rviz": "True",
                }.items(),
            )
        ],
    )

    vizualizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pedsim_viz_dir, "launch", "visualizer_launch.py")]
        ),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_ros2_control_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_use_robot_localization_cmd)
    ld.add_action(declare_pedsim_scene_file_cmd)
    ld.add_action(declare_pedsim_config_file_cmd)

    # Add any actions
    ld.add_action(rsp)
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    ld.add_action(robot_controller_spawner)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(start_robot_localization_cmd)
    ld.add_action(joy_node)
    ld.add_action(teleop_node_ros2_control)
    ld.add_action(teleop_node_gazebo_control)

    ld.add_action(agent_spawner_cmd)
    ld.add_action(pedsim_launch_cmd)
    ld.add_action(vizualizer)

    # Launch them all!
    return ld

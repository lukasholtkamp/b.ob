import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from pathlib import Path

def generate_launch_description():
    pkg_path = os.path.join(Path.cwd(), "src", "bob_simulation")
    pkg_navigation = os.path.join(Path.cwd(), "src", "bob_navigation")

    gazebo_params_file = os.path.join(pkg_path, "config/gazebo_params.yaml")
    ekf_params_file = os.path.join(pkg_navigation, "config/ekf.yaml")
    world_filename = "inf.world"
    world_path = os.path.join(pkg_path, "worlds", world_filename)

    # Launch configuration variables specific to simulation
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    world = LaunchConfiguration("world")
    use_robot_localization = LaunchConfiguration("use_robot_localization")

    # Declare the launch arguments
    declare_gui = DeclareLaunchArgument(
        "gui",
        default_value="False",
        description="Start RViz2 automatically with this launch file.",
    )

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
        default_value=world_path,
        description="Full path to the world model to load",
    )

    declare_use_robot_localization_cmd = DeclareLaunchArgument(
        name="use_robot_localization",
        default_value="False",
        description="Use robot_localization package if true",
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
            "world": world_path,
            "extra_gazebo_args": "--ros-args --params-file " + gazebo_params_file,
        }.items(),
    )

    # Run the spawner node from the gazebo_ros package with initial pose parameters
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "bob",
            "-x", "4.0",  # x position
            "-y", "1.0",  # y position
            "-z", "0.0",  # z position
            "-Y", "0.0"   # yaw orientation
        ],
        output="screen",
    )

    # Start robot localization using an Extended Kalman Filter
    start_robot_localization_cmd = Node(
        condition=IfCondition(use_robot_localization),
        package="robot_localization",
        executable="ekf_node",
        parameters=[ekf_params_file],
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

    rviz_node = Node(
        condition=IfCondition(gui),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_ros2_control_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_use_robot_localization_cmd)
    # ld.add_action(declare_gui)

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
    # ld.add_action(rviz_node)

    # Launch them all!
    return ld
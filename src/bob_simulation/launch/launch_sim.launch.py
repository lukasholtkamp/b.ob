import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from pathlib import Path


def generate_launch_description():

    pkg_path = os.path.join(Path.cwd(), "src", "bob_simulation")
    pkg_teleop = os.path.join(Path.cwd(), "src", "bob_teleop")

    gazebo_params_file = os.path.join(pkg_path, "config/gazebo_params.yaml")
    world_filename = "obstacle.world"
    world_path = os.path.join(pkg_path, "worlds", world_filename)

    # Launch configuration variables specific to simulation
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    world = LaunchConfiguration("world")

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
        default_value=world_path,
        description="Full path to the world model to load",
    )

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    pkg_path,'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': use_sim_time,'use_ros2_control': use_ros2_control,}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),launch_arguments={
            "world": world,
            "extra_gazebo_args": "--ros-args --params-file " + gazebo_params_file,}.items(),
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'bob'],
                        output='screen',
                        remappings=[("~/robot_description", "/robot_description"),],)

    # Launch joy node to read gamepad commands
    joy_node = Node(
        package="joy",
        executable="joy_node",
    )

    config_filepath = os.path.join(pkg_teleop, "config", "ps2.config.yaml")

    # Launch drive selection node to be able to switch between the modes
    teleop_node = Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                name="teleop_twist_joy_node",
                parameters=[
                    config_filepath,
                ],
                output="screen",
            )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_ros2_control_cmd)
    ld.add_action(declare_world_cmd)

    # Add any actions
    ld.add_action(rsp)
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    ld.add_action(joy_node)
    ld.add_action(teleop_node)

    # Launch them all!
    return ld
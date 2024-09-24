import os

from ament_index_python.packages import get_package_share_directory

from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from nav2_common.launch import RewrittenYaml

from pathlib import Path

def generate_launch_description():
    # Get the launch directory
    navigation_dir = os.path.join(Path.cwd(), "src", "bob_navigation")
    slam_dir = os.path.join(Path.cwd(), "src", "bob_slam")

    lifecycle_nodes = ['filter_mask_server', 'costmap_filter_info_server']

    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    autostart = LaunchConfiguration('autostart')
    keepout_params_file = LaunchConfiguration('keepout_params_file')
    mask_yaml_file = LaunchConfiguration('mask')

    mask_file = os.path.join(slam_dir, "map", "creative_room_keepout_mask.yaml")
    keepout_file = os.path.join(navigation_dir, "config", "cost_map_filter.yaml")

    # Declare the launch arguments     
    declare_namespace_cmd = DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
            'autostart', 
            default_value='true',
            description='Automatically startup the nav2 stack')

    declare_keepout_params_file_cmd = DeclareLaunchArgument(
            'keepout_params_file',
            default_value=keepout_file,
            description='Full path to the ROS2 parameters file to use')

    declare_mask_yaml_file_cmd = DeclareLaunchArgument(
            'mask',
            default_value=mask_file,
            description='Full path to filter mask yaml file to load')

    param_substitutions = {
            'use_sim_time': use_sim_time,
            'yaml_filename': mask_yaml_file}
    
    configured_params = RewrittenYaml(
            source_file=keepout_params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True)
    
    start_lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap_filters',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])

    start_map_server_cmd = Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            namespace=namespace,
            output='screen',
            emulate_tty=True, 
            parameters=[configured_params])

    start_costmap_filter_info_server_cmd = Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  
            parameters=[configured_params])

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_keepout_params_file_cmd)
    ld.add_action(declare_mask_yaml_file_cmd)

    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(start_map_server_cmd)
    ld.add_action(start_costmap_filter_info_server_cmd)

    return ld
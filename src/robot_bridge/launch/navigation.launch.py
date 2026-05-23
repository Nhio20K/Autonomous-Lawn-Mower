import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode, SetRemap
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_bridge')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file_path = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    # ---- 1. Map Server ----
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[{'yaml_filename': os.path.join(pkg_dir, 'maps', 'empty_map.yaml'), 'use_sim_time': use_sim_time}]
    )

    # ---- 2. Lifecycle Manager ----
    map_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True, 'node_names': ['map_server']}]
    )

    # ---- 3. Nav2 Navigation Stack (Remapped to safety layer) ----
    nav2_launch = GroupAction(
        actions=[
            SetRemap(src='/cmd_vel', dst='/cmd_vel_nav_raw'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
                launch_arguments={'use_sim_time': use_sim_time, 'params_file': params_file_path, 'autostart': 'True'}.items()
            ),
        ]
    )

    return LaunchDescription([
    DeclareLaunchArgument('use_sim_time', default_value='false'),
    map_server_node,
    map_lifecycle_manager,
    nav2_launch,
])

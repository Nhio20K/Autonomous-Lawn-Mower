import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg_mower_description = get_package_share_directory('mower_bot_description')
    pkg_robot_bridge = get_package_share_directory('robot_bridge')
    
    # 1. Get URDF via xacro
    urdf_file = os.path.join(pkg_mower_description, 'urdf', 'robot.urdf.xacro')
    
    # 2. Start Robot State Publisher with use_sim_time=true
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': True
        }]
    )

    # 3. Include Gazebo launch
    world_file = os.path.join(pkg_mower_description, 'worlds', 'my_obstacle_world.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'world': world_file
        }.items()
    )

    # 4. Spawn Robot into Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'tracked_mower', '-z', '0.2'],
        output='screen'
    )
    
    # 5. Start EKF Localization (use_sim_time=True)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_bridge, 'launch', 'localization.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'start_rsp': 'false', # Gazebo sim already has RSP
            'config_file': os.path.join(pkg_robot_bridge, 'config', 'ekf_sim.yaml')
        }.items()
    )

    # 6. Start Nav2 Navigation (use_sim_time=True)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_bridge, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        TimerAction(
            period=5.0,
            actions=[localization_launch]
        ),
        TimerAction(
            period=10.0,
            actions=[navigation_launch]
        )
    ])

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 1. โมเดล 3 มิติ (จำเป็น! เพื่อบอก EKF ว่ากล้องอยู่ตรงไหนบนตัวรถ)
    mower_desc_share = get_package_share_directory('mower_bot_description')
    xacro_file = os.path.join(mower_desc_share, 'urdf', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': False}]
    )

    # 2. RealSense Camera (IMU + กล้องปกติ)
    realsense_launch_dir = get_package_share_directory('realsense2_camera')
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(realsense_launch_dir, 'launch', 'rs_launch.py')),
        launch_arguments={
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2',
            'enable_color': 'true',
            'enable_depth': 'false',
            'pointcloud.enable': 'false',
        }.items()
    )

    # 3. EKF Node (ใช้ Config แบบ IMU ตัวเดียว ไม่ต้องพึ่งล้อ)
    robot_bridge_share = get_package_share_directory('robot_bridge')
    ekf_config_path = os.path.join(robot_bridge_share, 'config', 'ekf_imu_test.yaml')

    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': False}]
    )

    # 4. RViz
    rviz_config_file = os.path.join(mower_desc_share, 'config', 'view_bot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        realsense_node,
        ekf_local_node,
        rviz_node
    ])

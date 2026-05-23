import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():
    # รับพารามิเตอร์
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='ใช้เวลาจำลอง (Gazebo) หรือไม่'
    )

    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(get_package_share_directory('robot_bridge'), 'config', 'ekf.yaml'),
        description='พาธไฟล์พารามิเตอร์ EKF'
    )

    # 1. นำเข้าโมเดล 3 มิติ และกระดูกหุ่นยนต์ (TF Tree) จากไฟล์เก่าที่คุณเคยทำไว้
    mower_desc_share = get_package_share_directory('mower_bot_description')
    xacro_file = os.path.join(mower_desc_share, 'urdf', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': use_sim_time}]
    )

    # 2. ปลุกสมองคำนวณตำแหน่ง (EKF - robot_localization) พร้อมป้อนคัมภีร์ yaml
    robot_bridge_share = get_package_share_directory('robot_bridge')
    ekf_config_path = os.path.join(robot_bridge_share, 'config', 'ekf.yaml')

    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}]
    )

    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odometry/global')]
    )

    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        remappings=[('imu', '/imu/data'),  # ใช้ BNO055 ที่เราจูนค่า Heading มาแล้ว
                    ('gps/fix', '/fix'),
                    ('odometry/filtered', 'odometry/global')]
    )

    rviz_config_file = os.path.join(mower_desc_share, 'config', 'view_bot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # 4. เชื่อมต่อกระดูก (Static TF) สำหรับ GPS (ถ้าไม่มีตัวนี้ navsat_transform จะหาหุ่นไม่เจอ)
    # สมมติว่า GPS วางอยู่ตรงกลางหุ่น (0, 0, 0)
    static_gps_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_gps_tf',
        arguments=['0', '-0.1', '0', '0', '0', '0', 'base_link', 'gps']
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_config_file,
        # เปิดหน้าจอ 3 มิติ
        rviz_node,
        # เปิดกระดูกหุ่นยนต์
        robot_state_publisher_node,
        # วงใน (ล้อ + IMU)
        ekf_local_node,
        # วงนอก (ล้อ + IMU + GPS)
        ekf_global_node,
        # ตัวแปลงพิกัดดาวเทียมเป็น X,Y
        navsat_transform_node,
        # เชื่อมต่อ GPS เข้ากับหุ่น
        static_gps_tf
    ])

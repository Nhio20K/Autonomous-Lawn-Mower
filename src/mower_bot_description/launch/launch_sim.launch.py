import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # 1. ประกาศตัวแปรหลัก
    pkg_name = 'mower_bot_description'
    pkg_share = get_package_share_directory(pkg_name)
    
    # pkg_share คือ .../install/mower_bot_description/share/mower_bot_description
    # เราต้องถอยออกมา 1 ชั้น เพื่อให้ได้ folder 'share' รวม
    gazebo_models_path = os.path.join(pkg_share, '..')

    # สร้างคำสั่งเพื่อเพิ่ม Path นี้เข้าไปในระบบชั่วคราวตอนรัน
    set_gazebo_model_path = AppendEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        gazebo_models_path
    )
    # ========================================================================

    # 2. ตั้งค่า Configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use sim time if true'
    )

    # 3. เตรียมไฟล์ Robot Description (Xacro)
    # (หมายเหตุ: ตรวจสอบชื่อไฟล์ให้ตรงกับไฟล์จริงของคุณด้วยนะครับ ว่าชื่อ mower_core.xacro หรือ robot.urdf.xacro)
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro') 
    robot_description_config = xacro.process_file(xacro_file)
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # 4. เตรียมไฟล์ World
    world_file_name = 'my_obstacle_world.world' 
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_path}.items()
    )

    # 5. Node สำหรับเสกหุ่นยนต์ (Spawn)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_mower_bot',
                   '-z', '0.5'], # ยกสูงกันระเบิด
        output='screen'
    )

    # 6. เตรียมไฟล์ RViz
    rviz_config_file = os.path.join(pkg_share, 'config', 'view_bot.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    # ส่งคืน Description (อย่าลืมใส่ set_gazebo_model_path เข้าไปใน list!)
    return LaunchDescription([
        set_gazebo_model_path,    # <--- ใส่ไว้ตัวแรกสุดเลยครับ สำคัญมาก!
        declare_use_sim_time_cmd,
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        rviz_node, 
    ])
#!/bin/bash

# --- Robot Master Startup Script ---
source /opt/ros/humble/setup.bash

if [ ! -d "src/robot_bridge" ]; then
    echo "Error: กรุณารันสคริปต์นี้จากโฟลเดอร์ ~/ros2_ws เท่านั้น"
    exit 1
fi

echo "🛑 Stopping existing ROS2 nodes..."
killall -9 teleop_stm arduino_reader nmea_serial_driver 2>/dev/null
pkill -9 -f "rplidar_node" 2>/dev/null
pkill -9 -f "realsense2_camera" 2>/dev/null
pkill -9 -f "rs_launch" 2>/dev/null
pkill -9 -f "robot_localization" 2>/dev/null
pkill -9 -f "nav2" 2>/dev/null
pkill -9 -f "rviz2" 2>/dev/null
pkill -9 -f "component_container" 2>/dev/null

ros2 daemon stop
ros2 daemon start

# เปิดสิทธิ์พอร์ต USB ทั้งหมด
#sudo chmod 666 /dev/ttyUSB* 2>/dev/null

sleep 3

echo "🔨 Building Workspace (with Symlink for easy updates)..."
colcon build --symlink-install --packages-select robot_bridge mower_bot_description

WS_DIR=$(pwd)
SOURCE_CMD="source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash"

echo "🚀 Opening Tabs in a single terminal window..."

# เปิด 3 แท็บในหน้าต่างเดียว
# แต่ละแท็บ sleep รอให้ตัวก่อนหน้าเปิดขึ้นก่อน
# "exec bash" ที่ท้ายทำให้แท็บไม่ปิดตัวเองเมื่อคำสั่ง launch จบ
gnome-terminal --tab --title="1. HARDWARE" -- bash -c "$SOURCE_CMD && ros2 launch robot_bridge hardware_bringup.launch.py; exec bash"
sleep 10

# 2. Terminal สำหรับ Localization (EKF Fusion)
gnome-terminal --tab --title="2. LOCALIZATION" -- bash -c "$SOURCE_CMD && ros2 launch robot_bridge localization.launch.py; exec bash"
sleep 16

# 3. Terminal สำหรับ Navigation (Nav2 Stack)
gnome-terminal --tab --title="3. NAVIGATION" -- bash -c "$SOURCE_CMD && ros2 launch robot_bridge navigation.launch.py; exec bash"
sleep 8

# 4. Terminal สำหรับ Geofence & Planner (Safety + Path Planning)
gnome-terminal --tab --title="4. SAFETY & PLANNER" -- bash -c "$SOURCE_CMD && ros2 run robot_bridge geofence_and_planner --ros-args -p geofence_file:=${WS_DIR}/lawn_geofence.yaml; exec bash"
sleep 5

 #5. Terminal สำหรับ Mowing Executor
gnome-terminal --tab --title="5. MOWING" -- bash -c "$SOURCE_CMD && echo '🚜 รอรับพิกัดสนาม... เมื่อทุกอย่างพร้อมพิมพ์ go แล้วกด Enter' && ros2 run robot_bridge mow_zigzag; exec bash"
 #6. Terminal สำหรับ Dashboard (ดูสถานะภาพรวมแบบ Real-time)
gnome-terminal --tab --title="6. DASHBOARD" -- bash -c "$SOURCE_CMD && ros2 run robot_bridge robot_dashboard; exec bash"

echo ""
echo "✅ เริ่มระบบหุ่นยนต์คันจริงสำเร็จ!"

echo "   📌 แท็บ 1-3 → ระบบขับเคลื่อนและนำทาง"
echo "   📌 แท็บ 4-5 → ระบบรั้วไฟฟ้าและแผนผังเส้นทาง"
echo "   📌 แท็บ 6   → หน้าจอสำหรับสั่งงาน [go]"
echo ""

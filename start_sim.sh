#!/bin/bash

# --- Robot Simulation Startup Script ---

if [ ! -d "src/robot_bridge" ]; then
    echo "Error: กรุณารันสคริปต์นี้จากโฟลเดอร์ ~/ros2_ws เท่านั้น"
    exit 1
fi

echo "🛑 Stopping existing ROS2 nodes..."
killall -9 gzserver gzclient
killall -9 teleop_stm arduino_reader nmea_serial_driver 2>/dev/null
pkill -9 -f "realsense2_camera" 2>/dev/null
pkill -9 -f "rs_launch" 2>/dev/null
pkill -9 -f "robot_localization" 2>/dev/null
pkill -9 -f "nav2" 2>/dev/null
pkill -9 -f "rviz2" 2>/dev/null
pkill -9 -f "gzserver" 2>/dev/null
pkill -9 -f "gzclient" 2>/dev/null
pkill -9 -f "lawn_planner" 2>/dev/null

ros2 daemon stop
ros2 daemon start

sleep 2

echo "🔨 Building Workspace..."
colcon build --symlink-install

WS_DIR=$(pwd)
SOURCE_CMD="source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash"

echo "🚀 Launching Gazebo Simulation & Systems..."

gnome-terminal --tab --title="1. GAZEBO_SIM" -- bash -c "$SOURCE_CMD && ros2 launch robot_bridge simulation.launch.py; exec bash" 
sleep 8
gnome-terminal --tab --title="2. GEOFENCE (SAFETY)" -- bash -c "sleep 5 && $SOURCE_CMD && ros2 run robot_bridge geofence_enforcer --ros-args -p geofence_file:=${WS_DIR}/lawn_geofence_sim.yaml; exec bash"
sleep 10
gnome-terminal --tab --title="3. LAWN_PLANNER" -- bash -c "sleep 8 && $SOURCE_CMD && ros2 run robot_bridge lawn_planner --ros-args -p geofence_file:=${WS_DIR}/lawn_geofence_sim.yaml; exec bash"
sleep 12
gnome-terminal --tab --title="4. MOWING_EXECUTOR" -- bash -c "sleep 12 && $SOURCE_CMD && echo 'พิมพ์ go แล้วกด Enter เพื่อเริ่มตัดหญ้าตามเส้นทาง' && ros2 run robot_bridge mow_zigzag; exec bash"

echo ""
echo "✅ เริ่มระบบจำลองสำเร็จ! (รอสักครู่โปรแกรมในแต่ละแท็บจะโหลดขึ้นมาตามลำดับ)"
echo "   📌 แท็บ 1 (GAZEBO)   → โลกเสมือนและระบบนำทาง (รอให้โหลด 2D Map เสร็จก่อน)"
echo "   📌 แท็บ 2 (GEOFENCE) → ตัวควบคุมความปลอดภัย (รอรับคำสั่ง เลือกระบบ 1 หรือ 2)"
echo "   📌 แท็บ 3 (PLANNER)  → ตัวสร้างเส้นทางรอคำนวณ"
echo "   📌 แท็บ 4 (MOWING)   → สำหรับสั่งพิมพ์ 'go' เพื่อวิ่ง!"
echo ""

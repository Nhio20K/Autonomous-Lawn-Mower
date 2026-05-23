#!/bin/bash

# =================================================================
# Autonomous Mower - Full Dependency Installer
# =================================================================

echo "🚀 Starting Full Dependency Installation..."

# 1. Update System
echo "--- Updating System Packages ---"
sudo apt update && sudo apt upgrade -y

# 2. Install ROS 2 Dependencies (Humble)
echo "--- Installing ROS 2 System Dependencies ---"
sudo apt install -y \
    ros-humble-robot-localization \
    ros-humble-nmea-msgs \
    ros-humble-cv-bridge \
    ros-humble-realsense2-camera \
    ros-humble-nav2-msgs \
    ros-humble-nav2-bringup \
    ros-humble-xacro \
    ros-humble-diagnostic-msgs \
    ros-humble-diagnostic-updater \
    python3-pip \
    python3-colcon-common-extensions \
    librealsense2-utils

# 3. Install Python Libraries via pip
echo "--- Installing Python Libraries ---"
if [ -f "requirements.txt" ]; then
    pip3 install -r requirements.txt
else
    echo "❌ requirements.txt not found! Installing core libs manually..."
    pip3 install "numpy==1.23.5" opencv-python pyserial shapely ultralytics PyYAML torch torchvision torchaudio
fi

# 4. Install Udev Rules (For STM32 and GPS)
echo "--- Installing Udev Rules ---"
if [ -f "99-stm32.rules" ]; then
    sudo cp 99-stm32.rules /etc/udev/rules.d/
    echo "✅ 99-stm32.rules installed."
fi

if [ -f "98-gps.rules" ]; then
    sudo cp 98-gps.rules /etc/udev/rules.d/
    echo "✅ 98-gps.rules installed."
fi

echo "--- Reloading Udev Rules ---"
sudo udevadm control --reload-rules && sudo udevadm trigger

# 5. Optional: Hailo-8L SDK (For Raspberry Pi 5 AI)
echo "--- Checking for Hailo SDK (Optional) ---"
echo "NOTE: If you are using Raspberry Pi 5 with Hailo-8L AI kit,"
echo "you should run: sudo apt install hailo-all"

echo "✅ ALL-IN-ONE Installation Complete!"
echo "🚀 Next Steps:"
echo "   1. source /opt/ros/humble/setup.bash"
echo "   2. colcon build --symlink-install"
echo "   3. source install/setup.bash"

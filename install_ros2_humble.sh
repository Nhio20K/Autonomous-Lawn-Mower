#!/bin/bash
set -e

echo "🚀 Starting ROS 2 Humble Base Installation..."

# 1. Setup Locale
echo "--- Setting up Locale ---"
sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. Setup Sources
echo "--- Setting up Sources ---"
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. Install ROS 2 packages
echo "--- Installing ROS 2 Humble Desktop ---"
sudo apt update
sudo apt install ros-humble-desktop -y

echo "--- Installing Dev Tools (colcon, etc.) ---"
sudo apt install ros-dev-tools -y
sudo apt install python3-colcon-common-extensions -y

echo "✅ ROS 2 Humble Installation Complete!"
echo "Now you can run the dependency script again."

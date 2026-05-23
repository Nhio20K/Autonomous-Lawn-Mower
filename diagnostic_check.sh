#!/bin/bash
# diagnostic_check.sh
# Script to capture logs from 3 main components for 20 seconds and analyze errors.

# 1. Kill any existing nodes
echo "Stopping existing nodes..."
killall -9 teleop_stm arduino_reader nmea_serial_driver 2>/dev/null
pkill -9 -f "realsense2_camera" 2>/dev/null
pkill -9 -f "robot_localization" 2>/dev/null
pkill -9 -f "nav2" 2>/dev/null

sleep 2

# 2. Start logs
echo "Starting Diagnostic Logs (20 seconds)..."
WS_DIR=$(pwd)
LOG_DIR="${WS_DIR}/logs_diag"
mkdir -p $LOG_DIR

# Sourcing
source /opt/ros/humble/setup.bash
source ${WS_DIR}/install/setup.bash

# Hardware
echo "Launching Hardware..."
ros2 launch robot_bridge hardware_bringup.launch.py > $LOG_DIR/hw.log 2>&1 &
HW_PID=$!
sleep 5

# Localization
echo "Launching Localization..."
ros2 launch robot_bridge localization.launch.py > $LOG_DIR/loc.log 2>&1 &
LOC_PID=$!
sleep 5

# Navigation
echo "Launching Navigation..."
ros2 launch robot_bridge navigation.launch.py > $LOG_DIR/nav.log 2>&1 &
NAV_PID=$!

echo "Capturing data..."
sleep 20

# 3. Stop everything
echo "Stopping capture..."
kill $HW_PID $LOC_PID $NAV_PID
killall -9 teleop_stm arduino_reader realsense2_camera 2>/dev/null

echo "Analysis complete. Logs saved in $LOG_DIR"

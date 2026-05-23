#!/bin/bash
echo "=== ROS2 Communication Debugger ==="
echo "Press Ctrl+C to stop"
echo ""

# 1. Check Nav2 Output
(ros2 topic echo /cmd_vel --tail 1 &)
echo "[1] Subscribed to /cmd_vel (Nav2 -> teleop_stm)"

# 2. Check STM32 Feedback
(ros2 topic echo /odom_raw --tail 1 &)
echo "[2] Subscribed to /odom_raw (STM32 -> ROS2)"

# 3. Check Hardware Logs (Looking for TX lines)
echo "[3] Monitoring Hardware Terminal Logs for Serial TX..."
# Note: This is harder to do in a script without knowing the pid, 
# so we advise the user to check Terminal 1 or use ros2 node info.

wait

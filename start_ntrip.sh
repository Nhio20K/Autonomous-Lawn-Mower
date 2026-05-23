#!/bin/bash

# สคริปต์สำหรับรัน NTRIP Client แยกเพื่อ Warm-up RTK-GPS
# รันตัวนี้ทิ้งไว้เพื่อให้ GPS ทำระดับ Fixed ตลอดเวลา

WS_DIR=$(pwd)
source /opt/ros/humble/setup.bash
source ${WS_DIR}/install/setup.bash

echo "🚀 Starting NTRIP Client for RTK-GPS Warm-up..."

ros2 run robot_bridge ntrip_client --ros-args \
    --params-file ${WS_DIR}/src/robot_bridge/config/ntrip_params.yaml

# ถ้าโปรแกรมหลุด ให้รอ 5 วินาทีแล้วจบ
sleep 5

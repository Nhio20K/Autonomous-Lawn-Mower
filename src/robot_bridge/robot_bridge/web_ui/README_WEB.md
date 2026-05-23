# 🚜 Mower Bot Web Dashboard

A premium, responsive Web UI for controlling and monitoring an autonomous mower robot. This dashboard provides real-time 3D visualization, camera streaming, and manual control capabilities.

## 📋 Features
- **3D World View**: Live Map, Path, and Robot pose rendering.
- **AI Camera Stream**: Real-time object detection visualization.
- **Telemetry**: Battery, GPS, IMU, and Speed monitoring.
- **Manual Control**: Virtual joystick for teleoperation.

---

## 🛠️ Installation

### 1. Install ROS 2 Dependencies
Run the following command on your robot or PC:
```bash
sudo apt update
sudo apt install ros-humble-rosbridge-suite ros-humble-web-video-server ros-humble-tf2-web-republisher
```

### 2. Build the Package
Ensure you are in the root of your workspace:
```bash
cd ~/ros2_ws
colcon build --packages-select robot_bridge
source install/setup.bash
```

---

## 🚀 How to Use

### Step 1: Start the ROS 2 Backend
**For Real Robot:**
```bash
ros2 launch robot_bridge hardware_bringup.launch.py
```
**For Simulation:**
```bash
./start_sim.sh
```

### Step 2: Start the Web Server
In a new terminal, run:
```bash
python3 -m http.server 8000 --directory ~/ros2_ws/src/robot_bridge/web_ui
```

### Step 3: Access the Dashboard
Open your web browser and navigate to:
`http://localhost:8000` (or the IP address of your robot)

---

## 📂 File Structure
- `index.html`: Main dashboard layout.
- `style.css`: Premium dark theme styling.
- `app.js`: ROS logic and 3D visualization code.
- `requirements_web.txt`: List of dependencies.

---

## 💡 Notes
- Ensure your browser and the robot are on the same network.
- For the joystick to work, toggle the **Manual Mode** switch in the UI.
- If the camera stream is offline, check if `/camera/camera/color/image_raw` is being published.

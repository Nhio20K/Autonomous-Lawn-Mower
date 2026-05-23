# 🚜 Autonomous Lawn Mower (ROS2 & STM32)

An advanced, high-precision autonomous lawn mower built on top of **ROS2 (Humble)** and real-time **STM32/ESP32** hardware controllers. This project integrates LiDAR obstacle avoidance, RTK-GPS centimeter-level navigation, IMU sensor fusion, and AI-powered vision safety logic.

---

## 🛠️ System Architecture & Core Technologies

The mower operates using a split-architecture design:
1. **High-Level Control (Raspberry Pi 5 / PC):** Runs **ROS2 Humble** for sensor fusion (`robot_localization`), path planning (Zigzag lawn planner), geofencing, and AI vision safety (YOLO).
2. **Low-Level Control (STM32 Bluepill & ESP32):** Manages high-frequency motor PWM control, wheel encoders, INA226 battery monitoring, and BNO055 IMU serial communication.

---

## 📚 Credits & Open-Source Acknowledgments

This project is built upon the incredible work of the open-source robotics and maker community. We would like to express our gratitude and give credit to the following libraries and frameworks:

### 🤖 High-Level & ROS2 Libraries
* **[ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/)** - The core middleware framework powering the entire robot.
* **[Slamtec RPLIDAR ROS2 Driver](https://github.com/Slamtec/rplidar_ros)** - The official ROS2 package for RPLIDAR laser scanners (Apache-2.0 License).
* **[robot_localization](https://github.com/craig-o-meara/robot_localization)** - Used for EKF sensor fusion (IMU + Odometry + GPS).
* **[Shapely](https://github.com/shapely/shapely)** - Python package used for geofencing polygon calculations (BSD 3-Clause License).
* **[Ultralytics YOLO](https://github.com/ultralytics/ultralytics)** - Advanced real-time object detection used for the vision-based safety system (AGPL-3.0 License).
* **[OpenCV Python](https://github.com/opencv/opencv-python)** - Used for camera frames and vision pipelines (Apache-2.0 License).

### ⚡ Low-Level & Firmware (PlatformIO / Arduino)
* **[PlatformIO](https://platformio.org/)** - The development environment used for STM32 and ESP32 firmware.
* **[Adafruit BNO055 Driver](https://github.com/adafruit/Adafruit_BNO055)** - Driver library for the BNO055 9-DOF IMU (MIT License).
* **[Adafruit Unified Sensor Library](https://github.com/adafruit/Adafruit_Sensor)** - Unified sensor abstraction layer (Apache-2.0 License).
* **[INA226_WE Library](https://github.com/wollewald/INA226_WE)** - wollewald's library for high-accuracy current and power monitoring (MIT License).

---

## 📄 License & Usage

This project's custom logic is licensed under the **MIT License**. Third-party packages listed above retain their respective open-source licenses (MIT, BSD, Apache-2.0, AGPL-3.0).

---
*Created by [Nhio20K](https://github.com/Nhio20K) - Shaping the future of agricultural automation.*

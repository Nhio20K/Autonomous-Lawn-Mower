# Project Task Checklist

## Phase 1: Planning and Protocol Clarification
- [x] Review existing STM32 codebase
- [x] Analyze the new protocol tables (STM32 <-> Raspberry Pi 5)
- [x] **User Review Required**: Clarify missing commands and hardware layout
- [x] Finalize the communication protocol specification

## Phase 2: ROS2 Node Updates (teleop_stm.py & arduino_reader.py)
- [x] Update `teleop_stm.py` to map `/cmd_vel` to the new `C,<Dir>,<Speed>` command (F, B, L, R, S)
- [x] Hook up Emergency stop topic to send `E,1` / `E,0`
- [x] Make `arduino_reader.py` relay `U` strings over serial back to STM32, while also publishing to ROS2
- [x] Prepare `teleop_stm.py` to read incoming `D`, `I`, `P`, `G` messages and publish them as ROS2 topics

## Phase 3: STM32 Firmware Updates
- [x] **User action required:** STM32 team has created parsing logic with checksums in the `20260228_Final_Project_Main` project.
- [ ] STM32 team to implement parsing for `U`, `P`, and `G` elements.

## Phase 4: Sensor Integration
- [x] Setup and verify RTK L29h GPS with `nmea_navsat_driver`
- [ ] Integration of RPLiDAR (Pending Hardware)
- [ ] Integration of RealSense D435 (Pending Hardware)
- [ ] Integration of dedicated IMU (Pending Hardware)

## Phase 5: Workspace Cleanup and Launch Files
- [x] Delete redundant node `serial_bridge.py`
- [x] Delete unused scripts `gen_packet.py`, `mower_teleop_key.py`
- [x] Remove `mower_teleop_key` from `robot_bridge/setup.py` entry points
- [x] Create `hardware_bringup.launch.py` to start `teleop_stm`, `arduino_reader`, and GPS with one command

## Phase 6: ROS2 Teleop Protocol Update (Again) 
- [x] Edit `teleop_stm.py` to append Checksum (`ord(Dir) + Speed`) to `C` command
- [x] Edit `teleop_stm.py` to append Checksum (`69 + state`) to `E` command

## Phase 7: ROS2 Topic Integration
- [x] Create `sensor_msgs/Imu` publisher for STM32 `I` command logic (`/imu/data_raw`)
- [x] Create `nav_msgs/Odometry` publisher for STM32 `D` command logic (`/odom_raw`)
- [ ] Determine appropriate ROS2 Message Type for Power `P` data (e.g., `sensor_msgs/BatteryState`) and implement publisher
- [ ] Design custom ROS2 Message Type for General `G` status data (e.g., `RobotState.msg`) and implement publisher

## Phase 8: Localization (EKF) and RViz Visualization
- [x] Locate existing simulation assets (`mower_bot_description` package)
- [x] Review and adapt `mower_core.xacro` / `robot.urdf.xacro` to ensure physical sensor locations match reality (`base_link` -> `imu_link`, `gps_link`)
- [x] Create `config/ekf.yaml` to configure `robot_localization` parameters (Odom + IMU fusion)
- [x] Create `launch/localization.launch.py` to start `robot_state_publisher` (using existing xacro) and `ekf_node`
- [x] Reuse the existing `view_bot.rviz` for the default visualization state

## Phase 9: Nav2 Waypoint Navigation (Blind Run)
- [x] Install `navigation2` and `nav2_bringup` packages
- [x] Create `config/nav2_params.yaml` tuned for Ackermann/Differential drive without obstacle layers (Blind Mode)
- [x] Create an empty/dummy map file (`map.yaml` and `map.pgm`) since we don't have LiDAR mapped walls
- [x] Create `launch/navigation.launch.py` to start the Nav2 stack
- [x] Test publishing a 2D Goal Pose in RViz and verify `cmd_vel` outputs

## Phase 10: Full Autonomous Navigation (RTK + Ultrasonic + IMU)
- [x] **Ultrasonic to LaserScan:** Create `ultrasonic_converter.py` to parse Arduino strings into ROS2 `LaserScan` messages (simulated Lidar).
- [x] **URDF Update:** Add ultrasonic sensor frames to the robot's Xacro/URDF file.
- [x] **Dual EKF Setup:** Update `ekf.yaml` to run two instances: Local EKF (Odom+IMU) and Global EKF (Odom+IMU+GPS).
- [x] **RTK Integration:** Add `navsat_transform_node` to convert GPS Lat/Lon into XY map coordinates.
- [x] **Nav2 Obstacle Layer:** Update `nav2_params.yaml` to enable the `obstacle_layer` and subscribe to the simulated LaserScan.
- [x] **Sensor Fusion Bugfix:** Fix catastrophic TF explosion in RViz by implementing MPU6050 scaling factors (LSB to $m/s^2$ / $rad/s$) inside `teleop_stm.py`.
## Phase 11: Calibration, Filtering and Testing
- [x] **Advanced Odometry:** Update `teleop_stm.py` to parse 4 variables (vL, vR, pL, pR) and implement mathematical integration for absolute X, Y, Z coordinates.
- [x] **IMU Smoothing:** Implement a software Exponential Moving Average (EMA) filter in `teleop_stm.py` to reduce sensor jitter.
- [x] **Nav2 Performance Tuning:** Reduce maximum linear and angular velocities in `nav2_params.yaml` to ensure smooth turning and stopping.
- [x] **Path Planning Fix:** Enlarge the Global Costmap rolling window (50x50m) to allow path planning to distant goals.
- [x] **Testing Isolation:** Configure `ekf.yaml` to successfully perform suspended-wheel testing by disabling IMU/GPS conflict.
- [x] **Physical Calibration (TICKS_PER_METER):** Measured real-world encoder distance and computed `TICKS_PER_METER = 13450` (was 10000).
- [ ] **Physical Calibration (track_width):** Confirmed 0.5m matches physical measurement. Needs rotation test verification.
- [/] **Speed Mapping Calibration:** At cmd_vel 0.1 m/s, odom reads 0.03 m/s → `calculate_speed` multiplier needs tuning to match Nav2 expectations.

## Phase 12: Geofencing & Advanced Perception (RealSense) [/]
- [x] **IMU Migration:** Integrate Intel RealSense D435i IMU topics into `ekf.yaml`, replacing or augmenting the MPU6050 for better stability.
    - [x] *Troubleshooting:* Resolve "No such device" and USB disconnect issues on Raspberry Pi 5 (Laptop test successful).
    - [x] *Launch Integration:* Added RealSense node to `hardware_bringup.launch.py` with IMU interpolation enabled.
    - [x] *Thermal Optimization:* Disabled unused Stereo/Depth modules to reduce RealSense heat.
- [x] **EKF IMU Debugging:** Identified that RealSense orientation is always 0 (normal behavior), EKF uses Angular Velocity to compute orientation. Fixed local EKF Yaw bug.
- [x] **EKF-Only IMU Test:** Created `ekf_imu_test.yaml` and `imu_test.launch.py` for standalone IMU testing without STM32 hardware. Discovered missing `robot_state_publisher` was root cause of silent EKF data rejection.
- [x] **Per-Wheel Encoder Topics:** Added `/encoder/left_velocity`, `/encoder/right_velocity`, `/encoder/left_position`, `/encoder/right_position` for calibration.
- [x] **Rotation Safety:** Added 1.5-revolution (540°) rotation limiter with auto-stop and Error logging.
- [x] **Emergency Stop Enhancement:** Emergency Stop now sends `cmd_vel = 0` to Nav2, preventing RViz model from spinning post-stop.
- [x] **Startup Automation:** Created `start_robot.sh` to handle port cleanup and multi-terminal orchestration.
- [ ] **Geofencing (Keepout Zones):** Implement Nav2 Keepout Filter to define working boundaries and restricted areas on the map.
- [ ] **Visual Obstacle Avoidance:** Convert RealSense Depth Data to `PointCloud2` and integrate into `local_costmap` for robust collision prevention.
- [ ] **Object Recognition:** Implement a vision node (OpenCV/YOLO) to identify specific objects and trigger emergency stops.

## Phase 13: Calibration & Tuning [/]
- [x] **TICKS_PER_METER:** Calibrated to 13450 via physical measurement (1m test × 4 runs).
- [x] **track_width:** Confirmed 0.5m matches physical spur-gear-to-spur-gear measurement.
- [x] **Speed Mapping & Deadzone:** Implemented `MIN_PWM_OFFSET = 60` and `DEADBAND_MS = 0.05` on STM32 firmware to instantly overcome static friction.
- [x] **URDF Track Fix:** Adjusted `track_offset_y` to 0.25 (0.5m total width) in `mower_core.xacro`.
- [/] **Nav2 Controller Tuning:** Increased `min_approach_linear_velocity` to 0.1 m/s, removed `AMCL` parameters.
- [ ] **RTK Auto-Calibration Script:** Write automated calibration script using GPS waypoints.
- [ ] **Roll/Pitch Safety Monitor:** IMU-based tilt detection for slope/rollover emergency stop.

## Phase 14: Future Hardware Expansion
- [ ] **LiDAR Integration:** Add 360-degree LiDAR support for high-fidelity SLAM and long-range obstacle detection.

## Phase 15: Power Monitoring & Ultrasonic Refinement (Current) [ ]
- [x] **Battery Monitoring (INA226):**
    - [x] Implemented modular `power_monitor` code for ESP32/STM32.
    - [x] Calibrated with PSU: `voltFactor = 1.04`, `calFactor = 0.915` (50A/75mV shunt).
    - [x] Added `/battery_status` publisher in `teleop_stm.py` using `DiagnosticStatus`.
- [x] **Ultrasonic Optimization (JSN-SR04T):**
    - [x] Solved "No Pulse" issue using 5-attempt Retry logic.
    - [x] Implemented Sequential Firing (50ms delay) to eliminate crosstalk from JSN's 75° beam.
    - [x] Calibrated `DISTANCE_OFFSET = 3.0cm` to match physical tape measurement.
    - [x] Cleaned up ESP32 firmware to support 2-sensor (L/R) output: `U,Left,Right`.
- [x] **System Dashboard:**
    - [x] Created `robot_dashboard.py` (TUI) to monitor Battery, IMU, and Camera status in real-time.
- [ ] **Hardware Migration:**
    - [ ] Port `power_monitor` and `ultrasonic` modules from ESP32 test environment to main STM32 firmware.
    - [ ] Update `teleop_stm.py` parser to match new `U,L,R` (2-value) sonar protocol.

## Phase 16: Vision & Object Detection [x]
- [x] **RealSense Stability:** Verified with Powered USB Hub and 640x480x15fps profile.
- [x] **YOLOv11 Integration:** Implemented detection (Person/Dog/Cat) with Depth-aware E-Stop.
- [x] **AI Optimization:** RTX 3050 Ti running at ~10ms/frame (Forced CUDA Mode).

## Phase 17: Control Loop Integration [x]
- [x] **E-Stop Binding:** Linked `/cmd_emergency` (AI) to `teleop_stm.py` for physical braking.
- [x] **Dashboard Safety HUD:** Integrated AI and LiDAR status into `robot_dashboard.py` with 2.0s stability timeout.
- [x] **Field Verification:** Verified real-time braking logic with People/Pet detection.

## Phase 18: Integrated Safety Architecture & LiDAR [x]
- [x] **LiDAR Hardware Recovery:** Fixed `rplidar_node` crash (Error 255) and stabilized serial port access.
- [x] **180° Orientation Correction:** Aligned LiDAR detection zone with physical sensor mounting (PI-offset).
- [x] **Plant-Aware Safety:** Integrated YOLO foliage detection with Depth Fence to eliminate false stops.
- [x] **Visual Safety HUD (RViz):** Created `lidar_safety_marker` arc for real-time Red/Green safety zone status.
- [x] **AI Safety Refinement:** Calibrated AI stop threshold to 2.0m for humans and animals.

## Phase 19: Navigation & Mapping (Next) [ ]
- [ ] **SLAM/Mapping:** Record LaserScan and Odom data to generate a persistent 2D map of the garden.
- [ ] **Nav2 Global Planning:** Test long-distance waypoint navigation using the newly integrated safety layers.

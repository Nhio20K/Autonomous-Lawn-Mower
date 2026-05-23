# 🚜 Autonomous Lawn Mower Robot — Project README

> **ROS2 Humble | Gazebo Classic | Nav2 | EKF | RTK GPS**
>
> Last Updated: 2026-04-02

---

## 📖 Table of Contents

1. [Project Overview](#1-project-overview)
2. [Package Structure](#2-package-structure)
3. [Architecture & Data Flow](#3-architecture--data-flow)
4. [Quick Start](#4-quick-start)
5. [Key Configuration Files](#5-key-configuration-files)
6. [Current Status & Known Issues](#6-current-status--known-issues)
7. [Tested Workflows](#7-tested-workflows)
8. [For Future Agents / Developers](#8-for-future-agents--developers)

---

## 1. Project Overview

A tracked autonomous lawn mower robot running on ROS2 Humble. It uses:
- **`robot_bridge`** — the primary package: Nav2 config, localization, geofence, path planner, launch files
- **`mower_bot_description`** — URDF, meshes, Gazebo plugins, worlds

The robot currently has **two modes**:
1. **Real Hardware Mode** (`./start_robot.sh`) — physical STM32 motor driver, RTK GPS, RealSense D435i IMU
2. **Simulation Mode** (`./start_sim.sh`) — Gazebo Classic with virtual diff-drive, GPS, LiDAR, and IMU plugins

---

## 2. Package Structure

```
ros2_ws/
├── src/
│   ├── robot_bridge/               # PRIMARY PACKAGE
│   │   ├── config/
│   │   │   ├── nav2_params.yaml    # Nav2 controller, planner, smoother config
│   │   │   ├── ekf.yaml            # EKF for REAL hardware (sideways IMU)
│   │   │   ├── ekf_sim.yaml        # EKF for Gazebo (standard REP-103 axes)
│   │   │   └── empty_map.yaml      # Blank map for Nav2 (no costmap obstacles)
│   │   ├── launch/
│   │   │   ├── simulation.launch.py     # MAIN SIM LAUNCH
│   │   │   ├── navigation.launch.py     # Nav2 + Map Server
│   │   │   ├── localization.launch.py   # EKF + NavSat Transform
│   │   │   └── hardware_bringup.launch.py
│   │   ├── maps/
│   │   │   └── empty_map.yaml
│   │   └── robot_bridge/           # Python Nodes
│   │       ├── geofence_enforcer.py     ⭐ SAFETY GATEKEEPER
│   │       ├── lawn_planner.py          ⭐ PATH GENERATOR
│   │       ├── mow_zigzag.py            ⭐ PATH EXECUTOR
│   │       ├── teleop_stm.py            # Real hardware teleop
│   │       ├── arduino_reader.py        # Ultrasonic sensor bridge
│   │       └── ntrip_client.py          # RTK GPS NTRIP corrections
│   └── mower_bot_description/
│       ├── urdf/
│       │   ├── robot.urdf.xacro         # Main URDF entry point
│       │   ├── mower_core.xacro         # Physical links (76x60cm, 0.66m track)
│       │   └── mower_gazebo.xacro       # Gazebo plugins (diff_drive, GPS, IMU, LiDAR)
│       └── worlds/
│           └── my_obstacle_world.world
│
├── docs/                           # 📚 ALL DOCUMENTATION HERE
│   ├── README.md                   ← You are here
│   ├── STATUS.md                   # Detailed current status & issues
│   └── walkthrough.md              # Session walkthrough (auto-generated)
│
├── start_sim.sh                    # Simulation quick-start script
├── start_robot.sh                  # Real hardware quick-start script
├── lawn_geofence.yaml              # Real-world geofence points (GPS)
├── lawn_geofence_sim.yaml          # Simulation geofence (near 0,0)
└── topic_monitor.py                # Debug tool to monitor all key topics
```

---

## 3. Architecture & Data Flow

### Simulation Control Flow (Safety-Mandatory)

```
Nav2 (Path Controller)
        │
        ▼  /cmd_vel  (remapped by GroupAction in navigation.launch.py)
        │
        ▼  /cmd_vel_nav_raw
        │
Velocity Smoother (Nav2 internal)
        │
        ▼  /cmd_vel_nav
        │
╔════════════════════════════════╗
║  geofence_enforcer.py          ║  ← SAFETY GATEKEEPER
║  (Subscribes: /cmd_vel_nav,    ║     Checks GPS against Shapely polygon
║               /cmd_vel_teleop) ║     Blocks movement if out-of-bounds
╚═══════════╤════════════════════╝
            │
            ▼  /cmd_vel_filtered
            │
╔═══════════════════════╗
║ Gazebo diff_drive     ║  ← Only accepts /cmd_vel_filtered
║ (mower_gazebo.xacro)  ║
╚═══════════════════════╝
```

### Teleop Override

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/cmd_vel_teleop
```

Teleop has **1-second priority window** — Nav2 yields whenever keyboard input is detected.

### Localization Pipeline

```
Gazebo GPS (/fix)  →  navsat_transform_node  →  /odometry/gps
Gazebo IMU (/camera/camera/imu)     ──────┐
Gazebo Odom (/odom_raw)             ──────┤→  EKF (ekf_sim.yaml)  →  /odometry/filtered
                                          │
                                  navsat odom
```

---

## 4. Quick Start

### First Time Setup

Before running for the first time, install dependencies and build the workspace:

```bash
cd ~/ros2_ws
chmod +x install_dependencies.sh
./install_dependencies.sh

source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### Simulation

```bash
cd ~/ros2_ws
./start_sim.sh
```

This builds `robot_bridge`, launches Gazebo + Nav2, and opens RViz.

**After Sim starts, in separate terminals:**

```bash
# 1. Start the safety geofence system (choose mode 1 = enforce)
ros2 run robot_bridge geofence_enforcer

# 2. Start the zigzag path generator
ros2 run robot_bridge lawn_planner --ros-args -p geofence_file:=~/ros2_ws/lawn_geofence_sim.yaml

# 3. Start the mowing executor (type 'go' to begin)
ros2 run robot_bridge mow_zigzag
```

### Send a One-Off Goal (RViz)

1. In RViz, click **"2D Goal Pose"** in the toolbar
2. Click and drag on the map to set position + orientation
3. Nav2 will plan and execute

### Cancel a Goal

- **RViz UI:** Panels → Add New Panel → **Nav2 Panel** → Click **Cancel Navigation**
- **Terminal:** `ros2 topic pub /navigate_to_pose/_action/cancel_all_goals  std_msgs/Empty "{}"` (workaround)

---

## 5. Key Configuration Files

| File | Purpose | Critical Settings |
|---|---|---|
| `config/nav2_params.yaml` | Nav2 all-in-one config | `xy_goal_tolerance: 0.20`, `cmd_vel_out_topic: /cmd_vel_nav` |
| `config/ekf_sim.yaml` | EKF for Gazebo | REP-103 standard axes (X-forward, Z-yaw) |
| `config/ekf.yaml` | EKF for real hardware | Sideways IMU remapping (Robot X = IMU Z) |
| `urdf/mower_gazebo.xacro` | Gazebo plugins | `command_topic: cmd_vel_filtered` (safety lock) |
| `launch/simulation.launch.py` | SIM main launch | Includes localization + navigation |
| `launch/navigation.launch.py` | Nav2 launch | Uses `GroupAction + SetRemap` to lock cmd_vel |

---

## 6. Current Status & Known Issues

See **[STATUS.md](STATUS.md)** for full details.

**Quick Summary:**
- ✅ Simulation runs stably with correct TF frames
- ✅ Geofence boundary displays correctly in RViz
- ✅ Zigzag path is generated and aligned with geofence
- ✅ Waypoint follower (`mow_zigzag.py`) works
- ⚠️ Geofence **Stop** only works when `geofence_enforcer` is running (not auto-started)
- ⚠️ Teleop requires custom topic (`/cmd_vel_teleop`) — does NOT work with default topic during Nav2

---

## 7. Tested Workflows

- [x] `start_sim.sh` → Gazebo + Nav2 + RViz launches correctly
- [x] `2D Goal Pose` in RViz → Robot navigates to goal
- [x] `mow_zigzag` → Robot follows complete Zigzag coverage path
- [x] `geofence_enforcer` detects out-of-bounds GPS and blocks velocity commands
- [ ] Full end-to-end: Geofence record → Plan → Execute → Auto-stop at boundary *(in progress)*

---

## 8. For Future Agents / Developers

> **Read `docs/STATUS.md` for the detailed engineering context.**

### What Works Well
- The `GroupAction + SetRemap` in `navigation.launch.py` successfully reroutes `/cmd_vel` → `/cmd_vel_nav` at the launch level, so no YAML hacks are needed.
- `ekf_sim.yaml` is completely separate from real-hardware EKF — do not merge them.
- The Geofence polygon coordinate system uses the **first recorded GPS point** as the datum `(0,0)` origin. Both `lawn_planner.py` and `geofence_enforcer.py` MUST use the same datum logic (currently `points[0]`).

### Known Traps
- If `geofence_enforcer` is not running, the robot receives **zero commands** because Gazebo only listens on `/cmd_vel_filtered`. Always start the enforcer first.
- `navsat_transform_node` needs `wait_for_datum: false` in sim — it uses the first GPS fix as its origin automatically.
- The `velocity_smoother` inside Nav2 has its own `cmd_vel_in_topic` / `cmd_vel_out_topic` settings in `nav2_params.yaml`. These are currently set to `cmd_vel_nav_raw` → `/cmd_vel_nav`.

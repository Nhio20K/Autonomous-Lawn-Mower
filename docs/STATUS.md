# 🔧 Project Status — Autonomous Mower Simulation

> Last Updated: 2026-04-02
> Version Tag: `sim-stable-v1` (before this commit)

---

## ✅ What Works (Confirmed Stable)

| Feature | Status | Notes |
|---|---|---|
| Gazebo simulation launch | ✅ Working | `./start_sim.sh` |
| Robot spawning (URDF) | ✅ Working | Tracked model, 76×60cm body |
| TF chain (map→odom→base) | ✅ Working | No extrapolation errors |
| EKF localization (sim) | ✅ Working | `ekf_sim.yaml` with REP-103 axes |
| Nav2 path planning | ✅ Working | Regulated Pure Pursuit controller |
| 2D Goal Pose (RViz) | ✅ Working | Robot navigates to manually clicked goal |
| Zigzag path generation | ✅ Working | `lawn_planner.py` — Shapely-based |
| Path visualization (RViz) | ✅ Working | `/mowing_path` (purple) + `/mowing_markers` (green) |
| Geofence boundary display | ✅ Working | Red polygon on `/geofence_viz` |
| Waypoint follower | ✅ Working | `mow_zigzag.py` — Nav2 Simple Commander |
| Geofence GPS detection | ✅ Works | Logs "STOP" correctly when out of bounds |

---

## ⚠️ Known Issues / Limitations

### 1. Geofence Enforcer Must Be Started Manually

**Problem:** The `geofence_enforcer` node is not auto-started by `start_sim.sh` or any launch file.

**Impact:** If the enforcer is not running, the robot receives **zero velocity commands** because Gazebo only listens on `/cmd_vel_filtered` (the filtered output of the enforcer). **The robot will not move at all.**

**Workaround:** Always start the enforcer first in a separate terminal:
```bash
ros2 run robot_bridge geofence_enforcer
# Choose 1 (Enforce mode)
```

**Ideal Fix:** Add `geofence_enforcer` as an auto-started node in `simulation.launch.py`.

---

### 2. Teleop Requires Custom Topic

**Problem:** The standard `teleop_twist_keyboard` publishes to `/cmd_vel` which is now remapped away from the motors.

**Workaround:** Must use:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/cmd_vel_teleop
```

The `geofence_enforcer` subscribes to `/cmd_vel_teleop` and gives it 1-second priority over Nav2 commands.

---

### 3. Path Offset vs. Geofence Boundary (Partially Fixed)

**Problem:** The zigzag path (`lawn_planner`) was shifted relative to the geofence boundary (`geofence_enforcer`) in RViz because both used different center-of-polygon datums.

**Fix Applied:** Both scripts now use `points[0]` (the first recorded GPS point) as the ENU coordinate origin `(0,0)`. Both files must remain consistent on this.

**Remaining Issue:** In simulation — the path still appears slightly outside the red rectangle because the sim geofence (`lawn_geofence_sim.yaml`) was recorded at dummy GPS coordinates that don't match the actual robot spawn origin perfectly. The robot spawns at Gazebo `(0, 0)` but the GPS plugin reports `(0.0, 0.0)` lat/lon which are valid ocean coordinates. This creates a tiny offset that is currently acceptable.

---

### 4. No "Cancel Goal" Button by Default

**Problem:** `ros2 action cancel` doesn't support the `cancel` subcommand in Humble.

**Workaround Options:**
1. In RViz: **Panels → Add New Panel → Nav2 Panel** — gives Cancel button
2. Send a new goal to override (Nav2 replaces active goal)
3. Kill and restart Nav2 nodes

---

### 5. Hunting / Jitter at Goal (Partially Fixed)

**Status:** Goal tolerances were increased from `0.05m / 0.02rad` to `0.20m / 0.10rad` in `nav2_params.yaml`. Robot now stops cleanly at most goals, but some waypoints in the zigzag sequence may still cause brief rotation before accepting.

---

## 🏗️ Architecture Decisions

### Why `cmd_vel_filtered` instead of `cmd_vel`?

The standard `/cmd_vel` topic is now used internally by Nav2. We needed a "final output" topic that ONLY the safety node writes to. The Gazebo diff_drive plugin was configured with `<command_topic>cmd_vel_filtered</command_topic>` so **nothing except the geofence_enforcer can move the physical robot**.

### Why separate `ekf_sim.yaml` vs `ekf.yaml`?

The real robot has a RealSense D435i IMU mounted **sideways** (X-axis points up). The Gazebo IMU outputs standard REP-103 axes. Using the same config in both environments causes massive drift or complete failure. The two files are intentionally separate and must never be merged.

### Why `GroupAction + SetRemap` in navigation.launch.py?

`IncludeLaunchDescription` alone does not propagate remappings to child nodes. Using `GroupAction` with `SetRemap` ensures the `/cmd_vel` → `/cmd_vel_nav` remapping applies to every node spawned by the Nav2 bringup launch, including velocity smoother and controller server.

---

## 📂 Important File Locations

| File | Path | Role |
|---|---|---|
| Sim Geofence | `~/ros2_ws/lawn_geofence_sim.yaml` | GPS points for sim boundary |
| Real Geofence | `~/ros2_ws/lawn_geofence.yaml` | GPS points for real field |
| Safety Node | `src/robot_bridge/robot_bridge/geofence_enforcer.py` | Velocity gatekeeper |
| Path Gen | `src/robot_bridge/robot_bridge/lawn_planner.py` | Zigzag path from geofence |
| Path Exec | `src/robot_bridge/robot_bridge/mow_zigzag.py` | Nav2 Waypoint Follower |
| Nav2 Config | `src/robot_bridge/config/nav2_params.yaml` | All Nav2 tuning |
| EKF Sim | `src/robot_bridge/config/ekf_sim.yaml` | Localization for Gazebo |
| EKF Real | `src/robot_bridge/config/ekf.yaml` | Localization for hardware |
| Gazebo URDF | `src/mower_bot_description/urdf/mower_gazebo.xacro` | Plugin config incl. cmd_vel_filtered |

---

## 🔜 Next Steps (Recommended)

1. **Auto-start `geofence_enforcer`** in `simulation.launch.py` in enforce mode (non-interactive)
2. **Add `geofence_file` param** to `simulation.launch.py` so the sim uses `lawn_geofence_sim.yaml` automatically
3. **Full field test** on real hardware with RTK Fixed lock and re-recorded geofence
4. **Investigate compass/IMU heading** — the GY-273 I2C compass causes NaN crashes on real hardware, consider upgrading to dual-antenna RTK
5. **Integrate `Fields2Cover`** library for more efficient coverage path planning

---

## 🧩 Topic Reference

| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/cmd_vel_teleop` | Twist | `teleop_twist_keyboard` | `geofence_enforcer` |
| `/cmd_vel_nav` | Twist | `geofence_enforcer` (pass-through from Nav2) | `geofence_enforcer` processes it |
| `/cmd_vel_nav_raw` | Twist | Nav2 controller | `velocity_smoother` |
| `/cmd_vel_filtered` | Twist | `geofence_enforcer` | Gazebo motors |
| `/fix` | NavSatFix | Gazebo GPS plugin | `geofence_enforcer`, `navsat_transform` |
| `/odometry/filtered` | Odometry | EKF | Nav2, velocity smoother |
| `/mowing_path` | Path | `lawn_planner` | `mow_zigzag`, RViz |
| `/mowing_markers` | Marker | `lawn_planner` | RViz |
| `/geofence_viz` | Marker | `geofence_enforcer` | RViz |
| `/emergency_stop` | Int8 | `geofence_enforcer` | (future: motor driver) |

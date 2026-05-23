#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu, LaserScan, Image
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticStatus
from std_msgs.msg import Float64, String
from rclpy.qos import qos_profile_sensor_data
import time
import os
import math

class RobotDashboard(Node):
    def __init__(self):
        super().__init__('robot_dashboard')
        
        # --- ตัวแปรเก็บเวลาล่าสุดที่ได้รับข้อมูล ---
        self.last_gps_time = 0.0
        self.last_imu_time = 0.0
        self.last_odom_time = 0.0
        self.last_lidar_time = 0.0
        self.last_battery_time = 0.0
        self.last_camera_time = 0.0
        
        # --- ตัวแปรเก็บสถานะพิเศษ ---
        self.gps_status = -1  # -1 = No Data, 0 = Standalone, 1 = Float, 2 = Fixed
        self.gps_lat = 0.0
        self.gps_lon = 0.0
        self.gps_cov = [99.0] * 9
        
        self.odom_v_x = 0.0
        self.odom_v_z = 0.0
        
        self.compass_heading = 0.0
        self.imu_cal = "S:? G:? A:? M:?"
        
        self.lidar_safe = True
        self.lidar_min_dist = 10.0
        
        self.battery_v = 0.0
        self.battery_a = 0.0

        # --- Subscribers (ใช้ QoS Profile แบบ Sensor Data เพื่อให้รับข้อมูลได้ทุุกประเภท) ---
        self.create_subscription(NavSatFix, '/fix', self.gps_callback, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom_raw', self.odom_callback, qos_profile_sensor_data)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.lidar_scan_callback, 10)
        self.sub_lidar_status = self.create_subscription(String, '/lidar_safety_status', self.lidar_status_callback, 10)
        self.create_subscription(Float64, '/compass/heading', self.compass_callback, qos_profile_sensor_data)
        self.create_subscription(String, '/imu/calibration', self.cal_callback, qos_profile_sensor_data)
        self.create_subscription(DiagnosticStatus, '/battery_status', self.battery_callback, 10)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.camera_callback, qos_profile_sensor_data)
        
        # --- 🚨 AI Vision Emergency Status ---
        self.vision_safe = True
        self.vision_perf = "--- FPS | ---ms"
        self.last_vision_time = 0.0
        self.create_subscription(String, '/cmd_emergency', self.vision_emergency_callback, 10)
        self.create_subscription(String, '/camera/yolo/performance', self.vision_perf_callback, 10)
        
        # --- Timer สำหรับวาดหน้าจอ (ทุกๆ 0.5 วินาที) ---
        self.timer = self.create_timer(0.5, self.draw_dashboard)

    def gps_callback(self, msg):
        self.last_gps_time = time.time()
        self.gps_status = msg.status.status
        self.gps_lat = msg.latitude
        self.gps_lon = msg.longitude
        self.gps_cov = msg.position_covariance

    def imu_callback(self, msg):
        self.last_imu_time = time.time()

    def odom_callback(self, msg):
        self.last_odom_time = time.time()
        self.odom_v_x = msg.twist.twist.linear.x
        self.odom_v_z = msg.twist.twist.angular.z

    def lidar_callback(self, msg):
        self.last_lidar_time = time.time()
    def lidar_status_callback(self, msg):
        """รับสถานะความปลอดภัยที่ผ่านการตัดสินใจมาแล้วจาก teleop_stm"""
        self.last_lidar_time = time.time()
        self.lidar_safe = (msg.data == "SAFE")

    def lidar_scan_callback(self, msg):
        """ใช้สำหรับแสดงระยะวัตถุที่ใกล้ที่สุดเท่านั้น (ไม่ต้องคำนวณ Safety เอง)"""
        self.last_lidar_time = time.time()
        valid_ranges = [r for r in msg.ranges if r > 0.15 and r < 12.0]
        if valid_ranges:
            self.lidar_min_dist = min(valid_ranges)
        else:
            self.lidar_min_dist = 99.0

    def compass_callback(self, msg):
        self.compass_heading = msg.data

    def cal_callback(self, msg):
        self.imu_cal = msg.data

    def battery_callback(self, msg):
        self.last_battery_time = time.time()
        for kv in msg.values:
            if kv.key == 'voltage': self.battery_v = float(kv.value)
            if kv.key == 'current': self.battery_a = float(kv.value)

    def camera_callback(self, msg):
        self.last_camera_time = time.time()

    def get_status_str(self, last_time, timeout=2.5):
        if time.time() - last_time < timeout:
            return "🟢 ONLINE "
        else:
            return "🔴 OFFLINE"

    def vision_emergency_callback(self, msg):
        self.last_vision_time = time.time()
        self.vision_safe = (msg.data != "E,1")

    def vision_perf_callback(self, msg):
        self.vision_perf = msg.data

    def draw_dashboard(self):
        # ล้างหน้าจอแบบข้ามแพลตฟอร์ม
        os.system('cls' if os.name == 'nt' else 'clear')
        
        # เช็คสถานะแต่ละตัว
        imu_status = self.get_status_str(self.last_imu_time)
        odom_status = self.get_status_str(self.last_odom_time)
        lidar_status = self.get_status_str(self.last_lidar_time)
        
        # จัดรูปแบบข้อมูลย่อย
        # จัดรูปแบบข้อมูลย่อย
        odom_detail = f"(Speed: {self.odom_v_x:+.2f} m/s | Turn: {self.odom_v_z:+.2f} rad/s)" if "ONLINE" in odom_status else "(Hunting Port / Disconnected)"
        lidar_indicator = "🟢 SAFE   " if self.lidar_safe else "🚨 STOP   "
        lidar_detail = f"(Closest Obj: {self.lidar_min_dist:.2f} m)" if self.lidar_min_dist < 10.0 else "(No Obstacle)"
        if time.time() - self.last_lidar_time > 2.0:
            lidar_indicator = "🔴 OFFLINE"
            lidar_detail = "(No Data)"
        
        imu_detail = f"(Head: {self.compass_heading:03.0f}° | CAL: {self.imu_cal})" if "ONLINE" in imu_status else "(No Data)"
        
        # จัดรูปแบบ AI Vision
        vision_status = self.get_status_str(self.last_vision_time)
        vision_indicator = "🟢 SAFE   " if self.vision_safe else "🚨 EMERGENCY"
        vision_detail = f"({self.vision_perf})" if "ONLINE" in vision_status else ""
        if "OFFLINE" in vision_status:
            vision_indicator = "🔴 OFFLINE"
            vision_detail = ""

        # เช็ค GPS พิเศษหน่อย เพราะมีหลายระดับ
        gps_indicator = "🔴 OFFLINE"
        gps_detail = "(No Data / Disconnected)"
        gps_accuracy_cm = 0.0
        
        if time.time() - self.last_gps_time < 2.0:
            # คำนวณความแม่นยำจาก Covariance (RMS Accuracy)
            # Covariance[0] คือความแปรปรวนแกน X (เมตร^2)
            gps_accuracy_cm = math.sqrt(self.gps_cov[0]) * 100.0
            
            if self.gps_status >= 2: # มีค่าแก้ RTK
                if gps_accuracy_cm < 5.0: # แม่นยำน้อยกว่า 5cm
                    gps_indicator = "🟢 RTK-FIX "
                else:
                    gps_indicator = "🟡 RTK-FLOAT"
            else:
                gps_indicator = "🟠 SINGLE    "
            
            gps_detail = f"(Acc: {gps_accuracy_cm:.1f} cm | Lat: {self.gps_lat:.6f}, Lon: {self.gps_lon:.6f})"

        # --- Battery Bar Calculation ---
        bat_status = self.get_status_str(self.last_battery_time)
        bat_pct = 0
        if self.battery_v > 0:
            # สมมติว่าเป็นแบต 24V (Full ~28V, Low ~22V) 
            # หรือ 12V (Full ~13.5V, Low ~11V)
            # ในที่นี้ขอกลางๆ ไว้ก่อน ปรับได้ครับ
            if self.battery_v > 16.0: # น่าจะเป็นระบบ 24V
                bat_pct = int(((self.battery_v - 22.0) / (26.0 - 22.0)) * 100)
            else: # น่าจะเป็นระบบ 12V
                bat_pct = int(((self.battery_v - 10.5) / (13.5 - 10.5)) * 100)
            bat_pct = max(0, min(100, bat_pct))
        
        bar_len = 20
        filled_len = int(bar_len * bat_pct / 100)
        bat_bar = "█" * filled_len + "-" * (bar_len - filled_len)
        
        bat_detail = f"[{bat_bar}] {bat_pct}% ({self.battery_v:.2f}V | {self.battery_a:+.2f}A)" if "ONLINE" in bat_status else "(No Data)"

        # --- Camera Status ---
        cam_status = self.get_status_str(self.last_camera_time, timeout=2.0)
        cam_detail = "(D435i Streaming...)" if "ONLINE" in cam_status else "(Disconnected / Sleeping)"

        # วาดหน้าจอ
        dashboard = f"""
========================================================================
 🚜 MOWER BOT DASHBOARD - REALTIME HARDWARE STATUS 🚜
========================================================================
 [ 📡 RTK-GPS ]      : {gps_indicator} {gps_detail}
 [ 🧭 BNO055 IMU ]   : {imu_status} {imu_detail}
 [ 🛞 STM32 Motor ]  : {odom_status} {odom_detail}
 [ 🎯 Lidar Scan ]   : {lidar_indicator} {lidar_detail}
 [ 👁️  AI SAFETY ]   : {vision_indicator} {vision_detail}
 [ 📷 RealSense ]    : {cam_status} {cam_detail}
 [ 🔋 Battery ]      : {bat_status} {bat_detail}
========================================================================
 Press Ctrl+C to exit dashboard.
"""
        print(dashboard)

def main(args=None):
    rclpy.init(args=args)
    node = RobotDashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        os.system('cls' if os.name == 'nt' else 'clear')
        print("\n👋 Dashboard Closed.\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

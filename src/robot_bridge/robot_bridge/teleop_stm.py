import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from std_msgs.msg import Int8, Float64, String, Empty
from sensor_msgs.msg import Imu, LaserScan, NavSatFix
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue, DiagnosticArray
import math
import serial
import serial.tools.list_ports
import threading
import time
import numpy as np
import json
import os

# 🟢 ตั้งค่า Offset สำหรับ BNO055 (องศา)
# ปรับเลขนี้เพื่อให้ทิศหน้าของหุ่นยนต์ตรงกับทิศเหนือจริง
BNO_OFFSET_DEG = -190.0

class TeleopSTMNode(Node):
    def __init__(self):
        super().__init__('teleop_stm')

        # พารามิเตอร์ของพอร์ต (เผื่อไว้ในกรณีที่หา Auto ไม่เจอ)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('max_speed_ms', 1.25) # ⚠️ TODO: วัดจริงแล้วใส่ค่า max speed (m/s)
        self.declare_parameter('bypass_safety', False) # โหมดทดสอบ RTK-only (ข้ามระบบ Safety)

        self.baudrate = self.get_parameter('baudrate').value
        self.max_speed_ms = self.get_parameter('max_speed_ms').value
        self.bypass_safety = self.get_parameter('bypass_safety').value

        # ลบการล็อกพอร์ตตายตัว เปลี่ยนเป็น Dynamic Port Hunting
        self.port = None
        self.serial_conn = None
        self.is_connected = False

        # Publishers ของ Sensor กลับไปให้ ROS2
        self.pub_odom = self.create_publisher(Odometry, '/odom_raw', 10)
        self.pub_imu = self.create_publisher(Imu, '/imu/data', 10)

        # Publishers แยกล้อซ้าย-ขวา (สำหรับ Calibrate และเทียบกับ IMU)
        self.pub_enc_left_vel = self.create_publisher(Float64, '/encoder/left_velocity', 10)
        self.pub_enc_right_vel = self.create_publisher(Float64, '/encoder/right_velocity', 10)
        self.pub_enc_left_pos = self.create_publisher(Float64, '/encoder/left_position', 10)
        self.pub_enc_right_pos = self.create_publisher(Float64, '/encoder/right_position', 10)
        
        # --- Visual Debugging Topics ---
        self.pub_heading_visual = self.create_publisher(PoseStamped, '/robot/heading_visual', 10)
        self.pub_heading_marker = self.create_publisher(Marker, '/robot/heading_marker', 10)

        # Publisher สำหรับ Compass (True Heading 0-360 องศา จาก QMC5883L)
        self.heading_pub = self.create_publisher(Float64, '/compass/heading', 10)
        self.cal_pub = self.create_publisher(String, '/imu/calibration', 10)

        self.pub_serial_tx = self.create_publisher(String, '/serial/raw_tx', 10)
        self.pub_serial_rx = self.create_publisher(String, '/serial/raw_rx', 10)

        # 🚀 ย้ายมาไว้ตรงนี้เพื่อให้ระบบเริ่มส่ง Heartbeat ทันทีที่เปิดโปรแกรม
        self.timer = self.create_timer(0.1, self.heartbeat_loop)
        self.pub_serial_rx = self.create_publisher(String, '/serial/raw_rx', 10)

        # สร้าง Thread สำหรับจัดการการเชื่อมต่อ (Auto-Reconnect)
        self.monitor_thread = threading.Thread(target=self.connection_monitor_loop)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()

        # --- ระบบจัดการ Offset ของ IMU ---
        self.offset_file = os.path.expanduser('~/ros2_ws/imu_offset.json')
        self.bno_offset = self.load_offset()
        self.latest_raw_yaw = 0.0 # เก็บค่าดิบไว้คำนวณ

        # สร้าง Thread สำหรับอ่านข้อมูลจาก STM32 ตลอดเวลา
        self.read_thread = threading.Thread(target=self.read_serial_data)
        self.read_thread.daemon = True
        self.read_thread.start()

        # ติดตามผลลัพธ์จาก EKF เพื่อเอามาโชว์ใน Marker
        self.sub_ekf = self.create_subscription(Odometry, '/odometry/global', self.ekf_callback, 10)
        
        self.sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.sub_emergency = self.create_subscription(Int8, 'emergency_stop', self.emergency_callback, 10)
        
        # Topic สำหรับสั่งให้รถ "จำว่านี่คือทิศเหนือ"
        self.sub_set_north = self.create_subscription(Empty, '/imu/set_north', self.set_north_callback, 10)

        # รับข้อมูล U ดิบจาก Arduino Reader เพื่อส่งต่อให้ STM32
        self.sub_ultrasonic_raw = self.create_subscription(String, 'ultrasonic_raw', self.ultra_raw_callback, 10)
        
        # --- 🚨 AI Vision Emergency Stop Subscription ---
        self.vision_safe = True
        self.last_vision_msg_time = self.get_clock().now()
        self.sub_vision_emergency = self.create_subscription(String, '/cmd_emergency', self.vision_emergency_callback, 10)
        
        # --- 🛡️ LiDAR Visual Safety Marker ---
        self.marker_pub = self.create_publisher(Marker, 'lidar_safety_marker', 10)
        self.safety_angle_rad = 0.785 # +/- 45 องศา
        self.stop_dist_lidar = 1.2   # ระยะเบรก 1.2 ม.
        
        # --- 🛑 Nav2 Safety Sync & Verification ---
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.is_paused = False
        self.is_verifying = False
        self.verification_start_time = None
        self.pause_start_time = None
        self.VERIFY_DURATION = 2.0     # วินาทีที่ต้องปลอดภัยต่อเนื่องก่อนเดินต่อ
        self.MAX_PAUSE_DURATION = 60.0  # หยุดรอนานสุด 60 วินาทีก่อนยกเลิกงาน
    
        self.current_vL = 0.0  # m/s ล้อซ้าย
        self.current_vR = 0.0  # m/s ล้อขวา
        self.last_cmd_time = self.get_clock().now()

        # เก็บสถานะพิกัดสะสมของ Odom (X, Y, Theta)
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_th = 0.0
        self.latest_compass_yaw = 0.0  # เก็บค่า Yaw ล่าสุดจากเข็มทิศ (Radian)
        self.last_odom_time = self.get_clock().now()

        # Safety: ดักหมุนเกิน 1.5 รอบ (540°)
        self.rotation_accumulator = 0.0  # สะสมมุมหมุน (rad)
        self.ROTATION_LIMIT = 3.0 * 3.14159  # 1.5 รอบ = 540° = 3π rad
        self.rotation_safety_triggered = False
        self.pub_cmd_vel_override = self.create_publisher(Twist, 'cmd_vel', 10)

        # --- ระบบความปลอดภัยจาก LiDAR ---
        self.lidar_safe = True
        self.lidar_min_dist = 10.0 # เก็บระยะใกล้ที่สุดที่เจอ
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        
        # --- 🔋 Battery Monitoring Setup ---
        self.battery_pub = self.create_publisher(DiagnosticStatus, '/battery_status', 10)
        self.battery_volt = 0.0
        self.battery_curr = 0.0

        # --- 📢 Safety Status Publishers ---
        self.pub_lidar_status = self.create_publisher(String, '/lidar_safety_status', 10)
        self.pub_status = self.create_publisher(String, '/robot/status', 10)
        self.pub_battery = self.create_publisher(DiagnosticArray, '/robot/battery', 10)
        self.pub_volt = self.create_publisher(Float64, '/battery/voltage', 10)
        self.pub_curr = self.create_publisher(Float64, '/battery/current', 10)
        self.last_reported_status = ""

    def load_offset(self):
        """โหลดค่า Offset จากไฟล์ ถ้าไม่มีให้ใช้ค่า Default (160.0)"""
        if os.path.exists(self.offset_file):
            try:
                with open(self.offset_file, 'r') as f:
                    data = json.load(f)
                    self.get_logger().info(f"🟢 โหลด IMU Offset สำเร็จ: {data['offset']}°")
                    return data['offset']
            except:
                pass
        return 160.0 # Default เดิมของคุณ

    def save_offset(self, value):
        """บันทึกค่า Offset ลงไฟล์"""
        try:
            with open(self.offset_file, 'w') as f:
                json.dump({'offset': value}, f)
            self.get_logger().info(f"💾 บันทึก IMU Offset ใหม่เรียบร้อย: {value}°")
        except Exception as e:
            self.get_logger().error(f"🔴 บันทึก Offset พลาด: {e}")

    def vision_emergency_callback(self, msg):
        """รับสัญญาณหยุดฉุกเฉินจาก AI Vision"""
        self.last_vision_msg_time = self.get_clock().now()
        if msg.data == "E,1":
            if self.vision_safe:
                self.get_logger().error("🛑 [AI VISION] EMERGENCY STOP TRIGGERED!")
            self.vision_safe = False
        else:
            if not self.vision_safe:
                self.get_logger().info("🟢 [AI VISION] CLEAR - Resuming Operation")
            self.vision_safe = True

    def set_north_callback(self, msg):
        """คำสั่งตั้งค่าหน้าหุ่นให้เป็นทิศเหนือ (0 องศา)"""
        # สูตร: Offset ใหม่ = (360 - ค่าดิบปัจจุบัน) % 360
        new_offset = (360.0 - self.latest_raw_yaw) % 360.0
        self.bno_offset = new_offset
        self.save_offset(new_offset)
        self.get_logger().warn(f"🎯 ตั้งค่าทิศเหนือใหม่แล้ว! Offset ใหม่คือ: {new_offset:.2f}°")

        # รับข้อมูล U ดิบจาก Arduino Reader เพื่อส่งต่อให้ STM32
        self.sub_ultrasonic_raw = self.create_subscription(String, 'ultrasonic_raw', self.ultra_raw_callback, 10)

        # Timer ส่งคำสั่งซ้ำๆ (Heartbeat) ที่ 10Hz (0.1 วิ) เหมือนใน main_control.py
        self.timer = self.create_timer(0.1, self.heartbeat_loop)
    
        self.current_vL = 0.0  # m/s ล้อซ้าย
        self.current_vR = 0.0  # m/s ล้อขวา
        self.last_cmd_time = self.get_clock().now()

        # เก็บสถานะพิกัดสะสมของ Odom (X, Y, Theta)
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_th = 0.0
        self.latest_compass_yaw = 0.0  # เก็บค่า Yaw ล่าสุดจากเข็มทิศ (Radian)
        self.last_odom_time = self.get_clock().now()

        # Safety: ดักหมุนเกิน 1.5 รอบ (540°)
        self.rotation_accumulator = 0.0  # สะสมมุมหมุน (rad)
        self.ROTATION_LIMIT = 3.0 * 3.14159  # 1.5 รอบ = 540° = 3π rad
        self.rotation_safety_triggered = False
        self.pub_cmd_vel_override = self.create_publisher(Twist, 'cmd_vel', 10)
        
    def find_stm32_port(self):
        """ค้นหาพอร์ตอัตโนมัติ (หาเฉพาะ CH340 เท่านั้น)"""
        ports = serial.tools.list_ports.comports()
        
        # ลองหาตามเลขประจำตัวชิป (CH340) เท่านั้น
        for p in ports:
            hwid_str = str(p.hwid).upper()
            if (p.vid == 0x1A86 and p.pid == 0x7523) or ('1A86' in hwid_str and '7523' in hwid_str):
                self.get_logger().info(f"🟢 [Port Search] Found STM32 (CH340) at: {p.device}")
                return p.device
        
        # ❗ ไม่ Fallback ไปหยิบพอร์ตมั่วๆ เพื่อไม่ให้ไปแย่งพอร์ต GPS/Lidar
        return None

    def connection_monitor_loop(self):
        """วนลูปตรวจสอบและพยายามเชื่อมต่อพอร์ต USB อัตโนมัติ"""
        while rclpy.ok():
            if not self.is_connected or self.serial_conn is None or not self.serial_conn.is_open:
                # ทำการเคลียร์พอร์ตเก่าที่พัง
                if self.serial_conn:
                    try:
                        self.serial_conn.close()
                    except:
                        pass
                    self.serial_conn = None
                
                self.is_connected = False
                self.port = self.find_stm32_port()
                
                if self.port:
                    try:
                        self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=0.05)
                        self.is_connected = True
                        self.get_logger().info(f"🟢 [STM32] ต่อพอร์ตสำเร็จที่: {self.port}")
                    except serial.SerialException as e:
                        self.get_logger().warn(f"🟡 [STM32] เจอพอร์ต {self.port} แต่เปิดไม่ได้: {e} (รอ 1 วิ)")
                else:
                    self.get_logger().warn(f"🔴 [STM32] สายหลุด! กำลังค้นหาบอร์ด (CH340)...")
                
                time.sleep(1.0) # รอ 1 วิก่อนลองเชื่อมใหม่
            else:
                time.sleep(2.0) # เช็คสุขภาพทุกๆ 2 วิ


    def cmd_vel_callback(self, msg):
        v_linear = msg.linear.x
        v_angular = msg.angular.z

        # Differential Drive: แปลง linear.x / angular.z → vL / vR (m/s)
        track_width = 0.50  # เมตร (แก้ไขจาก 0.70 ตามค่าจริง 50cm)
        vL = v_linear - (v_angular * track_width / 2.0)
        vR = v_linear + (v_angular * track_width / 2.0)

        # 🚀 Deadband Compensation: ชดเชยแรงเสียดทานของสายพานตอนหมุนตัว
        # ถ้ารถพยายามหมุนตัวอยู่กับที่ (v_linear ใกล้ 0) แต่แรงไม่พอ
        if abs(v_linear) < 0.05 and abs(v_angular) > 0.05:
            MIN_SPIN_SPEED = 0.15  # ความเร็วล้อขั้นต่ำที่ทำให้สายพานขยับได้บนหญ้า (m/s)
            if abs(vL) < MIN_SPIN_SPEED:
                vL = -MIN_SPIN_SPEED if vL < 0 else MIN_SPIN_SPEED
            if abs(vR) < MIN_SPIN_SPEED:
                vR = -MIN_SPIN_SPEED if vR < 0 else MIN_SPIN_SPEED

        # 🔍 DEBUG: เช็คว่าทำไมความเร็วถึงเหลือ 0.06
        self.get_logger().info(f"DEBUG: Recv v_ang={v_angular:.2f}, vL_raw={vL:.3f}, vR_raw={vR:.3f}")

        # ✅ อนุญาตให้ความเร็วล้อตอนเลี้ยวสูงกว่า max_speed_ms ได้ (สูงสุดที่ 1.2 m/s เพื่อความแรง)
        # เพื่อรองรับการหมุนตัวที่ 2.5 Rad/s
        abs_max = 0.4
        self.current_vL = max(-abs_max, min(abs_max, vL))
        self.current_vR = max(-abs_max, min(abs_max, vR))
        self.last_cmd_time = self.get_clock().now()

    def ultra_raw_callback(self, msg):
        # ส่งข้อความ U,x,y,z\n ตรงๆ ไปยัง STM32 เลย
        self.send_serial(msg.data)

    def emergency_callback(self, msg):
        state = msg.data # 0 = Normal, 1 = Emergency
        if not hasattr(self, 'last_emergency_state'):
            self.last_emergency_state = -1
            
        if state in [0, 1] and state != self.last_emergency_state:
            self.last_emergency_state = state
            # STM32 ต้องการ Checksum: state + 69
            chk = state + 69  
            cmd_str = f"E,{state},{chk}\n"
            self.send_serial(cmd_str)
            self.get_logger().warn(f"Emergency command sent: {state}")
            
            if state == 1:  # Emergency Stop
                # หยุดรถทันที + รีเซ็ต Safety
                self.current_vL = 0.0
                self.current_vR = 0.0
                self.rotation_accumulator = 0.0
                self.rotation_safety_triggered = False
                # ส่ง cmd_vel = 0 ให้ Nav2 รู้ว่าต้องหยุด
                stop_msg = Twist()
                self.pub_cmd_vel_override.publish(stop_msg)
                self.get_logger().warn('🛑 Emergency: หยุดรถ + ยกเลิกคำสั่ง Nav2')


    def heartbeat_loop(self):
        # ลด Safety Timeout ลงเหลือ 0.3 วินาที เพื่อไม่ให้หุ่นวิ่งไหลเวลาคอมมานด์หยุดครับ
        now = self.get_clock().now()
        if (now - self.last_cmd_time).nanoseconds > 3e8: # 0.3 seconds (300ms)
            self.current_vL = 0.0
            self.current_vR = 0.0

        # --- 🛡️ AI Vision Heartbeat Safety ---
        status = "OK"
        if not self.bypass_safety:
            if (now - self.last_vision_msg_time).nanoseconds > 2e9: # 2.0 seconds
                if self.vision_safe:
                    self.get_logger().error("⚠️ [AI VISION] Timeout! AI might have crashed. Stopping for safety.")
                    self.vision_safe = False
                status = "VISION_TIMEOUT"
        else:
            self.vision_safe = True  # bypass mode → ไม่สนใจ vision

        # --- 🔍 Determine Overall Status ---
        if not self.vision_safe: status = "VISION_TIMEOUT"
        elif not self.lidar_safe: status = "LIDAR_OBSTACLE"
        elif self.is_paused: status = "WAITING_VERIFY"
        elif (now - self.last_cmd_time).nanoseconds > 5e8: # 0.5s Timeout for Nav2
            status = "WAITING_NAV2"
        
        if hasattr(self, 'last_emergency_state') and self.last_emergency_state == 1:
            status = "EMERGENCY_STOP"

        # Publish Status (Only when changed to avoid spam)
        if status != self.last_reported_status:
            msg = String()
            msg.data = status
            self.pub_status.publish(msg)
            # เปลี่ยนเป็น warn เพื่อให้เป็นสีเหลือง สังเกตง่าย
            self.get_logger().warn(f"🔄 STATUS CHANGED: {status}")
            self.last_reported_status = status

        # เก็บสถานะปัจจุบันไว้ส่งไปพร้อม TX
        self.current_status_str = status

        # ส่ง Marker ไปโชว์ใน RViz
        self.publish_safety_marker()

        # ยิงคำสั่งปัจจุปันส่งไปหา STM32 รัวๆ (Continuous Heartbeat)
        self.send_control_cmd(self.current_vL, self.current_vR)

    def send_control_cmd(self, vL, vR):
        now = self.get_clock().now()
        if self.bypass_safety:
            is_safe = True
        else:
            is_safe = self.lidar_safe and self.vision_safe

        # --- 🛡️ ตรรกะ Pause & Verify ---
        if not is_safe:
            # ตรวจเจออุปสรรค -> สั่งหยุดทันที
            vL, vR = 0.0, 0.0
            if not self.is_paused:
                self.get_logger().warn("🚨 OBSTACLE DETECTED: Pausing and waiting for clear path...")
                self.is_paused = True
                self.pause_start_time = now
            self.is_verifying = False
            
            # เช็คว่ารอจนหมดความอดทนหรือยัง (60 วินาที)
            if self.pause_start_time and (now - self.pause_start_time).nanoseconds > (self.MAX_PAUSE_DURATION * 1e9):
                self.cancel_nav2_goal()
                self.pause_start_time = None 
        
        elif self.is_paused:
            # อุปสรรคหายไปแล้ว -> เริ่มขั้นตอน Verify
            vL, vR = 0.0, 0.0
            if not self.is_verifying:
                self.get_logger().info(f"🔍 Path Cleared: Verifying for {self.VERIFY_DURATION}s...")
                self.is_verifying = True
                self.verification_start_time = now
            
            # ตรวจสอบว่าปลอดภัยต่อเนื่องจนครบเวลาหรือยัง
            elapsed_verify = (now - self.verification_start_time).nanoseconds / 1e9
            if elapsed_verify >= self.VERIFY_DURATION:
                self.get_logger().info("🟢 Verification Complete: Resuming path following.")
                self.is_paused = False
                self.is_verifying = False

        vL_rounded = round(vL, 2)
        vR_rounded = round(vR, 2)
        vL_int = int(round(vL_rounded * 100))
        vR_int = int(round(vR_rounded * 100))
        
        chk = vL_int + vR_int
        cmd_str = f"C,{vL_rounded:.2f},{vR_rounded:.2f},{chk}\n"
        self.send_serial(cmd_str)

    def cancel_nav2_goal(self):
        """ยกเลิกงาน Nav2 เมื่อหยุดรอนานเกินไป"""
        self.get_logger().error("🛑 Obstacle persisted too long. Resetting pause state.")
        self.is_paused = False
        self.is_verifying = False
        self.pause_start_time = None

    def send_serial(self, data: str):
        # เพิ่มสถานะเข้าไปใน Log เพื่อให้เห็นชัดๆ ทุกบรรทัด
        status_tag = getattr(self, 'current_status_str', 'UNKNOWN')
        if status_tag == "OK":
            self.get_logger().info(f"📤 [TX][{status_tag}] -> {data.strip()}")
        else:
            # ถ้าไม่ปกติ ให้เป็นสีเหลือง
            self.get_logger().warn(f"📤 [TX][{status_tag}] -> {data.strip()}")
        
        if self.is_connected and self.serial_conn is not None and self.serial_conn.is_open:
            try:
                self.serial_conn.write(data.encode('utf-8'))
                # Mirror to ROS2 topic
                msg = String()
                msg.data = data.strip()
                self.pub_serial_tx.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Serial write error: {e}")
                self.is_connected = False # สั่งปลดเพื่อให้ Thread ช่วยหาวิธีต่อใหม่
                
    def read_serial_data(self):
        while rclpy.ok():
            if self.is_connected and self.serial_conn and self.serial_conn.is_open:
                try:
                    if self.serial_conn.in_waiting > 0:
                        line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            # Mirror to ROS2 topic
                            msg = String()
                            msg.data = line
                            self.pub_serial_rx.publish(msg)
                            self.process_stm32_data(line)
                    else:
                        time.sleep(0.01)
                except Exception as e:
                    self.get_logger().debug(f"Error reading STM32 serial: {e}")
                    self.is_connected = False
            else:
                time.sleep(0.1)

    def process_stm32_data(self, line):
        line = line.strip()
        # 🔋 รับข้อมูลแบตเตอรี่ (Format: B,Volt,Curr)
        if line.startswith("B,"):
            parts = line.split(',')
            if len(parts) >= 3:
                try:
                    # ค่าดิบจาก STM
                    raw_volt = float(parts[1])
                    # 1. Calibration: ปรับจูนให้ตรงกับมิเตอร์ (25.5 / 26.43 = 0.965)
                    calibrated_volt = raw_volt * 0.965
                    
                    # 2. Filter: กรองให้นิ่ง (Low-pass filter / EMA)
                    # ใช้ค่าใหม่ 10% ค่าเดิม 90% เพื่อลดการกระโดด
                    # 2. Filter Volt: กรองให้นิ่ง
                    if self.battery_volt == 0.0:
                        self.battery_volt = calibrated_volt
                    else:
                        self.battery_volt = (self.battery_volt * 0.90) + (calibrated_volt * 0.10)
                    
                    # --- จูน Amp (INA226) ---
                    raw_curr = float(parts[2])
                    # 1. Calibration: ปรับจูนใหม่ตามค่าจริง (Idle 0.46->0.4, Load 3.45->4.44)
                    # สูตรใหม่: (raw * 1.35) - 0.22
                    calibrated_curr = (raw_curr * 1.35) - 0.22
                    
                    # ป้องกันค่าติดลบตอน Idle
                    if calibrated_curr < 0.05: calibrated_curr = 0.0
                    
                    # 2. Filter Amp: กรองกระแสนิ่งๆ
                    if self.battery_curr == 0.0:
                        self.battery_curr = calibrated_curr
                    else:
                        self.battery_curr = (self.battery_curr * 0.85) + (calibrated_curr * 0.15)
                    
                    # ส่งข้อมูลออกไปให้ Dashboard
                    diag = DiagnosticStatus()
                    diag.level = DiagnosticStatus.OK
                    diag.name = "Battery"
                    diag.message = f"{self.battery_volt:.2f}V {self.battery_curr:.2f}A"
                    # ใส่ค่าตัวเลขลงใน values เพื่อให้ดึงง่ายๆ
                    diag.values = [
                        KeyValue(key="voltage", value=str(self.battery_volt)),
                        KeyValue(key="current", value=str(self.battery_curr))
                    ]
                    self.battery_pub.publish(diag)
                except ValueError:
                    pass
            return

        # รับข้อมูลจาก STM32 ไปกระจายต่อใน ROS2
        if line.startswith('P,'):
            pass # TODO: publish Power
        elif line.startswith('G,'):
            pass # TODO: publish General
        elif line.startswith('I,'):
            self.process_imu_data(line)
        elif line.startswith('H,'):
            try:
                parts = line.split(',')
                if len(parts) >= 2:
                    raw_yaw = float(parts[1])
                    self.latest_raw_yaw = raw_yaw
                    # 1. Apply Offset
                    corrected_yaw = raw_yaw + self.bno_offset
                    if corrected_yaw >= 360.0: corrected_yaw -= 360.0
                    if corrected_yaw < 0.0: corrected_yaw += 360.0
                    
                    self.latest_dashboard_heading = corrected_yaw
                    
                    # Publish Heading สำหรับ Dashboard
                    compass_msg = Float64()
                    compass_msg.data = corrected_yaw
                    self.heading_pub.publish(compass_msg)
                    
                    # 2. แปลงเป็นระบบ ROS (0=East CCW) เพื่อใช้ใน Odom (ใช้ค่าที่ Correct แล้ว)
                    import math
                    yaw_deg = 90.0 - corrected_yaw
                    if yaw_deg > 180.0:
                        yaw_deg -= 360.0
                    elif yaw_deg < -180.0:
                        yaw_deg += 360.0
                    self.latest_compass_yaw = math.radians(yaw_deg)
            except Exception as e:
                self.get_logger().debug(f"Compass Parse Error: {e}")
        elif line.startswith('D,'):
            try:
                parts = line.split(',')
                if len(parts) >= 5:
                    current_time = self.get_clock().now()
                    msg = Odometry()
                    msg.header.stamp = current_time.to_msg()
                    msg.header.frame_id = "odom"
                    msg.child_frame_id = "base_link"
                    
                    # 1. อ่านค่า Ticks (พัลส์) จาก STM32
                    import math
                    vL_ticks = float(parts[1]) if parts[1] else 0.0
                    vR_ticks = float(parts[2]) if parts[2] else 0.0
                    if math.isnan(vL_ticks) or math.isnan(vR_ticks):
                        return
                        
                    pL_ticks = int(parts[3]) if parts[3] else 0
                    pR_ticks = int(parts[4]) if parts[4] else 0
                    
                    # 2. สเกลค่า Ticks ให้กลายเป็น เมตร (Meters) และ m/s
                    # ⚠️ จูนค่า TICKS_PER_METER ล่าสุด (จากสคริปต์ 3ม. แต่เดินจริงได้ 2.82ม.)
                    TICKS_PER_METER = 13298.0 
                    track_width = 0.50 # เมตร (แก้ไขจาก 0.70 ตามค่าจริง 50cm)
                    
                    vL = vL_ticks / TICKS_PER_METER
                    vR = vR_ticks / TICKS_PER_METER
                    pL = pL_ticks / TICKS_PER_METER  # ตำแหน่งล้อซ้าย (เมตร)
                    pR = pR_ticks / TICKS_PER_METER  # ตำแหน่งล้อขวา (เมตร)
                    
                    # 2.5 Publish ค่าแยกล้อซ้าย-ขวา (สำหรับ Calibrate)
                    enc_msg = Float64()
                    enc_msg.data = vL
                    self.pub_enc_left_vel.publish(enc_msg)
                    enc_msg.data = vR
                    self.pub_enc_right_vel.publish(enc_msg)
                    enc_msg.data = pL
                    self.pub_enc_left_pos.publish(enc_msg)
                    enc_msg.data = pR
                    self.pub_enc_right_pos.publish(enc_msg)
                    
                    # 3. คำนวณความเร็ว (Twist) ในหน่วย m/s และ rad/s
                    v_x = (vR + vL) / 2.0
                    v_th = (vR - vL) / track_width
                    msg.twist.twist.linear.x = v_x
                    msg.twist.twist.angular.z = v_th
                    
                    # 4. คำนวณพิกัดสะสมล้อ (Odometry Position)
                    dt = (current_time - self.last_odom_time).nanoseconds / 1e9
                    import math
                    
                    # 🚀 ไฮไลต์: ใช้ทิศทางจาก 'เข็มทิศ' ตรึงเป๊ะๆ แทนการบวกสะสมจากล้อ (แก้การหลงทิศ!)
                    self.odom_th = self.latest_compass_yaw 
                    
                    delta_x = (v_x * math.cos(self.odom_th)) * dt
                    delta_y = (v_x * math.sin(self.odom_th)) * dt
                    # delta_th = v_th * dt # ❌ เลิกใช้ delta ทิศทางจากล้อ
                    
                    self.odom_x += delta_x
                    self.odom_y += delta_y
                    # self.odom_th += delta_th # ❌ เลิกบวกสะสมค่าทิศ (สาเหตุของ Drift)
                    self.last_odom_time = current_time
                    
                    msg.pose.pose.position.x = self.odom_x
                    msg.pose.pose.position.y = self.odom_y
                    
                    # แปลงมุม odom_th (Euler Yaw) เป็น Quaternion สำหรับ ROS2
                    msg.pose.pose.orientation.z = math.sin(self.odom_th / 2.0)
                    msg.pose.pose.orientation.w = math.cos(self.odom_th / 2.0)
                    
                    # เพิ่ม Covariance (EKF Tuning) เพื่อระบุความไม่แน่นอน
                    msg.pose.covariance[0] = 0.20  # x
                    msg.pose.covariance[7] = 0.01  # y
                    msg.pose.covariance[35] = 5.0 # Yaw (🔴 Extreme - Let IMU dominate 100%)
                    msg.twist.covariance[0] = 0.20 # linear.x
                    msg.twist.covariance[35] = 0.1 # angular.z (🔴 Extreme)

                    self.pub_odom.publish(msg)
            except Exception as e:
                self.get_logger().debug(f"Encoder Parse Error: {e}")

    def process_imu_data(self, line):
        try:
            parts = line.split(',')
            if len(parts) >= 15:
                # I, ax, ay, az, gx, gy, gz, qx, qy, qz, qw, s, g, a, m
                ax = float(parts[1])
                ay = float(parts[2])
                az = float(parts[3])
                gx = float(parts[4])
                gy = float(parts[5])
                gz = float(parts[6])
                qx = float(parts[7])
                qy = float(parts[8])
                qz = float(parts[9])
                qw = float(parts[10])

                # --- 1. คำนวณ Rotation Quaternion สำหรับ Offset ---
                # สูตร: q_new = q_old * q_offset
                # 💡 เพิ่ม math.pi/2 เพื่อให้ทิศเหนือ (0°) ไปตรงกับแกนเขียว ROS (+90°)
                offset_rad = math.radians(self.bno_offset) + (math.pi / 2.0)
                # q_offset รอบแกน Z: [0, 0, sin(theta/2), cos(theta/2)]
                orz = math.sin(offset_rad / 2.0)
                orw = math.cos(offset_rad / 2.0)

                # คูณ Quaternion (เฉพาะแกน Z)
                # new_x = x*rw + y*rz
                # new_y = y*rw - x*rz
                # new_z = z*rw + w*rz
                # new_w = w*rw - z*rz
                final_qx = qx * orw + qy * orz
                final_qy = qy * orw - qx * orz
                final_qz = qz * orw + qw * orz
                final_qw = qw * orw - qz * orz

                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = "base_link"

                # --- 2. Coordinate Transform (BNO055 -> ROS base_link) ---
                # แปลงหน่วยจาก Degrees/s เป็น Radians/s เพื่อให้ ROS2 เข้าใจถูกทิศทาง
                imu_msg.linear_acceleration.x = ay
                imu_msg.linear_acceleration.y = -ax
                imu_msg.linear_acceleration.z = az

                imu_msg.angular_velocity.x = math.radians(gy)
                imu_msg.angular_velocity.y = math.radians(-gx)
                imu_msg.angular_velocity.z = math.radians(gz)

                imu_msg.orientation.x = final_qx
                imu_msg.orientation.y = final_qy
                imu_msg.orientation.z = final_qz
                imu_msg.orientation.w = final_qw

                # --- 3. ส่งค่า Calibration ไป Dashboard ---
                cal_str = f"S:{parts[11]} G:{parts[12]} A:{parts[13]} M:{parts[14].strip()}"
                cal_msg = String()
                cal_msg.data = cal_str
                self.cal_pub.publish(cal_msg)

                # ใส่ Covariance ต่ำๆ เพราะ BNO055 นิ่งมาก
                imu_msg.orientation_covariance[0] = 0.001
                imu_msg.orientation_covariance[4] = 0.001
                imu_msg.orientation_covariance[8] = 0.001
                
                imu_msg.angular_velocity_covariance[0] = 0.001
                imu_msg.angular_velocity_covariance[4] = 0.001
                imu_msg.angular_velocity_covariance[8] = 0.001

                # 🟢 ส่งข้อมูลออกไปยัง ROS2 (จุดที่หายไป)
                self.pub_imu.publish(imu_msg)

                # --- 4. ส่งข้อมูลทิศทางไป Rviz (เพื่อตรวจสอบความตรงกัน) ---
                # 4.1 ส่งลูกศรทิศทาง
                h_msg = PoseStamped()
                h_msg.header = imu_msg.header
                h_msg.pose.orientation = imu_msg.orientation
                self.pub_heading_visual.publish(h_msg)

                # 4.2 คำนวณองศาจากสิ่งที่ Rviz กำลังหันอยู่จริงๆ (EKF Output)
                # แปลง Quaternion ของรถใน Rviz เป็น Euler Yaw (องศา)
                # และแปลงจากระบบ ROS (East=0, CCW) กลับเป็นระบบเข็มทิศ (North=0, CW)
                rviz_yaw_deg = 0.0
                if hasattr(self, 'latest_ekf_pose'):
                    q = self.latest_ekf_pose.orientation
                    # สูตรแปลง Quaternion -> Euler Yaw
                    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
                    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
                    yaw_rad = math.atan2(siny_cosp, cosy_cosp)
                    # แปลงเป็นองศาเข็มทิศ: Compass = 90 - Yaw
                    rviz_yaw_deg = 90.0 - math.degrees(yaw_rad)
                    if rviz_yaw_deg < 0: rviz_yaw_deg += 360.0
                    if rviz_yaw_deg >= 360.0: rviz_yaw_deg -= 360.0

                # 4.3 ส่งตัวเลขอ้างอิง (Text Marker)
                t_msg = Marker()
                t_msg.header = imu_msg.header
                t_msg.ns = "heading_debug"
                t_msg.id = 0
                t_msg.type = Marker.TEXT_VIEW_FACING
                t_msg.action = Marker.ADD
                t_msg.pose.position.z = 1.2 # ลอยสูงขึ้นอีกนิด
                t_msg.scale.z = 0.25
                t_msg.color.r = 1.0; t_msg.color.g = 1.0; t_msg.color.b = 0.0; t_msg.color.a = 1.0
                
                # โชว์เทียบกัน 2 ค่า
                line1 = f"Sensor Head: {getattr(self, 'latest_dashboard_heading', 0.0):03.0f}°"
                line2 = f"RViz (EKF) Head: {rviz_yaw_deg:03.0f}°"
                t_msg.text = f"{line1}\n{line2}"
                self.pub_heading_marker.publish(t_msg)

        except Exception as e:
            self.get_logger().debug(f"IMU Parse Error: {e}")

    # ตัวแปรช่วยเก็บค่าสำหรับ Debug
    latest_dashboard_heading = 0.0
    
    def lidar_callback(self, msg):
        """วิเคราะห์ข้อมูลจาก LiDAR เพื่อตรวจจับสิ่งกีดขวางด้านหน้า"""
        # 1. กรองข้อมูลเฉพาะโซนอันตราย (ด้านหน้าหุ่นยนต์)
        # ใช้ค่าที่ตั้งไว้ใน __init__ (self.safety_angle_rad)
        stop_dist = self.stop_dist_lidar
        
        danger_points = []
        min_dist = 10.0
        
        for i, dist in enumerate(msg.ranges):
            # ข้ามค่าที่วัดไม่ได้ หรือระยะที่ใกล้เกินไป (น่าจะเป็นตัวรถเอง)
            if dist < 0.15 or dist > msg.range_max or math.isinf(dist) or math.isnan(dist):
                continue
            
            # คำนวณมุมของจุดนี้
            angle = msg.angle_min + (i * msg.angle_increment)
            
            # ปรับมุมให้อยู่ในช่วง -pi ถึง pi
            while angle > math.pi: angle -= 2.0 * math.pi
            while angle < -math.pi: angle += 2.0 * math.pi
            
            # ถ้าอยู่ในมุมด้านหน้าหุ่นยนต์ (เนื่องจาก LiDAR หมุน 180 องศาใน URDF 
            # ดังนั้นหน้าหุ่นยนต์จริงคือมุมที่ใกล้กับ PI หรือ -PI ของ LiDAR)
            is_front = abs(angle) > (math.pi - self.safety_angle_rad)
            
            if is_front:
                if dist < min_dist:
                    min_dist = dist
                if dist < stop_dist:
                    danger_points.append(dist)
        
        self.lidar_min_dist = min_dist
        
        # 2. ตัดสินใจ (ต้องเจออุปสรรคติดต่อกันเพื่อกัน Noise/Grass)
        # ปรับจาก 10 เป็น 25 เพื่อลดผลกระทบจากเศษหญ้าหรือฝุ่น
        if len(danger_points) >= 10: 
            if self.lidar_safe:
                self.get_logger().warn(f"🚨 [LiDAR] OBSTACLE DETECTED at {min_dist:.2f}m ({len(danger_points)} pts)")
                self.lidar_safe = False
        else:
            # ถ้าไม่เจอจุดอันตรายเลย ให้รีเซ็ตกลับเป็น Safe
            if not self.lidar_safe:
                self.get_logger().info("🟢 [LiDAR] Path Cleared")
                self.lidar_safe = True
        # 3. ส่งสถานะออกไปให้คนอื่น (Dashboard) ฟัง
        status_msg = String()
        status_msg.data = "SAFE" if self.lidar_safe else "STOP"
        self.pub_lidar_status.publish(status_msg)

    def ekf_callback(self, msg):
        """เก็บค่าพิกัดล่าสุดที่ EKF คำนวณเสร็จแล้ว"""
        self.latest_ekf_pose = msg.pose.pose

    def publish_safety_marker(self):
        """วาดพัดสีแดง/เขียว ใน RViz โดยหันไปทางด้านหน้าหุ่นยนต์ (มุม 180 ของ LiDAR)"""
        marker = Marker()
        marker.header.frame_id = "laser_frame"
        # 🛠️ แก้ปัญหา extrapolation: ใช้เวลาเป็น 0 เพื่ออิงพิกัดล่าสุดที่ RViz มี
        from rclpy.time import Time
        marker.header.stamp = Time().to_msg()
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # ปรับสีตามสถานะจริง
        if self.lidar_safe:
            # สีเขียวใส (Safe)
            marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0; marker.color.a = 0.4
        else:
            # สีแดงเข้ม (Danger)
            marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 0.8
            
        marker.scale.x = 0.05 # ความหนาเส้น
        
        # วาดพัดหันไปทางด้านหน้า (มุม PI ของ LiDAR)
        points = []
        center = Point(x=0.0, y=0.0, z=0.0)
        points.append(center)
        
        steps = 20 # เพิ่มความละเอียดให้โค้งสวยขึ้น
        # หน้าหุ่นยนต์คือมุม PI (180 องศา) ดังนั้นเราจะวาดจาก PI - safety_angle ถึง PI + safety_angle
        start_angle = math.pi - self.safety_angle_rad
        end_angle = math.pi + self.safety_angle_rad
        
        for i in range(steps + 1):
            angle = start_angle + ((end_angle - start_angle) * i / steps)
            px = self.stop_dist_lidar * math.cos(angle)
            py = self.stop_dist_lidar * math.sin(angle)
            points.append(Point(x=px, y=py, z=0.0))
            
        points.append(center)
        marker.points = points
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopSTMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # สั่งหยุดรถก่อนปิดโหนด
        if hasattr(node, 'serial_conn') and node.serial_conn and node.serial_conn.is_open:
            node.serial_conn.write(b"C,0.00,0.00,0\n")
            time.sleep(0.1)
            node.serial_conn.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

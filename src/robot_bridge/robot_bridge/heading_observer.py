#!/usr/bin/env python3
"""
heading_observer.py — Smart Heading Observer with EMI-Resistant Kalman Filter

แก้ปัญหา: เข็มทิศแม่เหล็ก (Magnetometer) ในชิป BNO055 ถูกรบกวนจากคลื่น EMI
          ของมอเตอร์ใบมีดตัดหญ้า ทำให้ทิศทาง (Heading) เพี้ยนขณะเครื่องยนต์ทำงาน

วิธีแก้:  ใช้ 1D Kalman Filter หลอมรวมข้อมูล 3 แหล่งที่ปลอดจาก EMI:
  Source 1: IMU Gyro Z (ω_z)    — Dead-reckoning, ทนทาน EMI 100%
  Source 2: RTK GPS COG          — ทิศทางจริงเมื่อเคลื่อนที่ตรง, ทนทาน EMI 100%
  Source 3: Magnetometer Yaw     — ใช้เฉพาะเมื่อ EMI ต่ำเท่านั้น

ฟีเจอร์เพิ่มพิเศษ (Fault-Tolerant):
  หากตรวจจับได้ว่าชิป BNO055 ค้าง (Hardware Freeze) ผ่านการนับค่าไม่ขยับ หรือสัญญาณไทม์เอาต์
  ระบบจะตัดสัญญาณ BNO055 ออกโดยอัตโนมัติ และสลับไปใช้ความเร็วเชิงมุมจากล้อ (Encoder / Odom) สำรองทันที!
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String, Empty
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math
import numpy as np


def normalize_angle(angle: float) -> float:
    """ปรับมุมให้อยู่ในช่วง -π ถึง π"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class HeadingObserver(Node):
    """
    1D Kalman Filter สำหรับ Heading ที่ทนทานต่อคลื่นแม่เหล็กไฟฟ้า และอาการเซนเซอร์ค้าง
    """

    # --- ค่าคงที่ของ Filter (Noise Tuning) ---
    Q_THETA = 0.001   # rad²  — ความไม่แน่นอนของ heading rate
    Q_BIAS  = 0.0001  # rad²  — ความไม่แน่นอนของ gyro bias drift

    # Measurement Noise
    R_MAG   = 0.10    # rad²  — เข็มทิศ (ค่าสูง = ไม่ค่อยเชื่อ)
    R_GPS   = 0.01    # rad²  — GPS COG (ค่าต่ำ = เชื่อมาก)

    # --- เกณฑ์การตัดสินใจ ---
    SPEED_GPS_THRESHOLD   = 0.15   # m/s — ต้องเร็วกว่านี้จึงใช้ GPS COG
    SPEED_STATIC_THRESH   = 0.05   # m/s — ช้ากว่านี้ถือว่า "จอดนิ่ง"
    OMEGA_STATIC_THRESH   = 0.02   # rad/s — นิ่งกว่านี้ถือว่า "ไม่หมุน"
    MAG_CORRUPT_DEG       = 3.0    # องศา — ถ้าเข็มทิศขยับเกินนี้ขณะนิ่ง = EMI กวน
    MAG_CORRUPT_DEG_RAD   = math.radians(MAG_CORRUPT_DEG)
    GPS_COG_MIN_DIST      = 0.05   # เมตร — ต้องขยับอย่างน้อยนี้ระหว่าง GPS samples

    def __init__(self):
        super().__init__('heading_observer')

        # --- Kalman Filter State ---
        # x = [theta (rad), gyro_bias (rad/s)]
        self.x = np.array([0.0, 0.0])
        self.P = np.diag([1.0, 0.01])
        self.Q = np.diag([self.Q_THETA, self.Q_BIAS])

        # --- Sensor Data Storage ---
        self.last_gyro_z     = 0.0    # rad/s (จาก /imu/data)
        self.last_mag_yaw_rad = None  # rad (จาก /compass/heading)
        self.last_mag_time   = None

        # ตำแหน่งล่าสุดจาก EKF map frame (สำหรับคำนวณ GPS COG)
        self.last_ekf_x      = None
        self.last_ekf_y      = None
        self.last_ekf_time   = None
        self.last_cog_rad    = None   # Course-over-Ground ล่าสุด

        # ความเร็วหุ่นยนต์ (สำหรับตัดสินใจ Source)
        self.current_speed   = 0.0   # m/s
        self.current_omega_z = 0.0   # rad/s
        self.odom_omega_z    = 0.0   # rad/s (แบบมีเครื่องหมาย +/-)

        # ตรวจจับ BNO055 ค้าง (Hardware Freeze Detection)
        self.bno_frozen      = False
        self.gyro_history    = []
        self.last_imu_msg_time = None

        # สถานะการตรวจจับ EMI
        self.mag_corrupted   = False
        self.mag_corrupt_count = 0
        self.MAG_CORRUPT_FRAMES = 3

        # Timing
        self.last_predict_time = self.get_clock().now()

        # --- Publishers ---
        self.pub_fused   = self.create_publisher(Float64, '/heading/fused',   10)
        self.pub_source  = self.create_publisher(String,  '/heading/source',  10)
        self.pub_reset_imu = self.create_publisher(Empty, '/cmd_reset_imu', 10)

        # --- Subscribers ---
        self.create_subscription(Imu,      '/imu/data',        self.imu_cb,       10)
        self.create_subscription(Float64,  '/compass/heading', self.compass_cb,   10)
        self.create_subscription(Odometry, '/odometry/global', self.ekf_global_cb, 10)
        self.create_subscription(Odometry, '/odom_raw',        self.odom_raw_cb,  10)

        # Timer สำหรับ Predict Step (50 Hz)
        self.create_timer(0.02, self.predict_and_publish)

        self.get_logger().info("✅ Heading Observer started (Kalman Filter with Auto-Encoder Fallback)")

    # ─────────────────────────── Callbacks ───────────────────────────

    def imu_cb(self, msg: Imu):
        """รับค่า Gyro Z จาก BNO055 และคอยเช็คการทำงาน"""
        self.last_gyro_z = msg.angular_velocity.z
        self.last_imu_msg_time = self.get_clock().now()
        
        # บันทึกประวัติเพื่อตรวจสอบค่าแข็งค้าง (Static Freeze Check)
        self.gyro_history.append(msg.angular_velocity.z)
        if len(self.gyro_history) > 30:
            self.gyro_history.pop(0)

    def odom_raw_cb(self, msg: Odometry):
        """รับความเร็วเชิงเส้นและเชิงมุมจากล้อ (Encoder)"""
        self.current_speed  = abs(msg.twist.twist.linear.x)
        self.current_omega_z = abs(msg.twist.twist.angular.z)
        self.odom_omega_z    = msg.twist.twist.angular.z  # แบบมีทิศทาง +/- สำหรับฟอลแบ็ก

    def compass_cb(self, msg: Float64):
        """รับ Heading จาก BNO055 Magnetometer และตรวจ EMI"""
        mag_deg = msg.data
        ros_deg = 90.0 - mag_deg
        mag_rad_ros = math.radians(ros_deg)

        # ─── ตรวจจับ EMI (Corruption Detection) ───
        is_static = (
            self.current_speed < self.SPEED_STATIC_THRESH and
            self.current_omega_z < self.OMEGA_STATIC_THRESH
        )

        if self.last_mag_yaw_rad is not None and is_static:
            # รถจอดนิ่ง แต่เข็มทิศกระโดด?
            delta = abs(normalize_angle(mag_rad_ros - self.last_mag_yaw_rad))
            if delta > self.MAG_CORRUPT_DEG_RAD:
                self.mag_corrupt_count += 1
                if self.mag_corrupt_count >= self.MAG_CORRUPT_FRAMES:
                    if not self.mag_corrupted:
                        self.get_logger().warn(
                            f"🚨 [EMI] Magnetometer corrupted! Δmag={math.degrees(delta):.1f}° "
                            f"while static. Switching to Gyro-only mode."
                        )
                    self.mag_corrupted = True
            else:
                self.mag_corrupt_count = max(0, self.mag_corrupt_count - 1)
                if self.mag_corrupt_count == 0 and self.mag_corrupted:
                    self.get_logger().info("🟢 [EMI] Magnetometer signal restored.")
                    self.mag_corrupted = False
        else:
            if not is_static:
                self.mag_corrupt_count = max(0, self.mag_corrupt_count - 1)
                if self.mag_corrupt_count == 0:
                    self.mag_corrupted = False

        self.last_mag_yaw_rad = mag_rad_ros

    def ekf_global_cb(self, msg: Odometry):
        """รับพิกัดเพื่อใช้หาทิศทางเคลื่อนที่จริง (GPS COG)"""
        now_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        cx = msg.pose.pose.position.x
        cy = msg.pose.pose.position.y

        if self.last_ekf_x is not None:
            dx = cx - self.last_ekf_x
            dy = cy - self.last_ekf_y
            dist = math.hypot(dx, dy)

            # คำนวณ COG เฉพาะเมื่อขยับได้ระยะขั้นต่ำ
            if dist >= self.GPS_COG_MIN_DIST:
                self.last_cog_rad = math.atan2(dy, dx)

        self.last_ekf_x    = cx
        self.last_ekf_y    = cy
        self.last_ekf_time = now_sec

    # ─────────────────────────── Kalman Core ───────────────────────────

    def predict_and_publish(self):
        """
        1. Predict Step: ทำนายทิศทางจาก Gyro (หรือล้อสำรอง)
        2. Update Step:  ปรับมุมอ้างอิงให้ตรงโลกจริง (GPS COG / Mag)
        3. Publish:      ส่งออกค่าผลลัพธ์
        """
        now = self.get_clock().now()
        dt = (now - self.last_predict_time).nanoseconds * 1e-9
        self.last_predict_time = now

        if dt <= 0.0 or dt > 0.5:
            return

        # ─── 0. DETECT BNO055 HARDWARE FREEZE ───
        bno_was_frozen = self.bno_frozen
        self.bno_frozen = False

        # เช็คแบบที่ 1: ค่า Gyro แข็งค้าง (ส่งค่าเดิมทศนิยมเท่าเดิมเป๊ะๆ 30 ครั้งติดต่อกัน)
        if len(self.gyro_history) >= 30 and len(set(self.gyro_history)) == 1:
            self.bno_frozen = True
        
        # เช็คแบบที่ 2: สัญญาณ IMU ขาดหายไปเลย (Timeout เกิน 1.5 วินาที)
        if self.last_imu_msg_time is not None:
            time_diff = (now - self.last_imu_msg_time).nanoseconds * 1e-9
            if time_diff > 1.5:
                self.bno_frozen = True

        # หากเจออาการค้าง บล็อกเข็มทิศและสลับใช้ล้อทันที
        if self.bno_frozen:
            if not bno_was_frozen:
                self.get_logger().error("🚨 [BNO055 CRASHED] BNO055 Hardware Freeze Detected! Falling back to Encoders (Wheel Odom) Gyro backup!")
                self.trigger_imu_reset()
                self.last_reset_trigger_time = now
            else:
                # ลองส่งสัญญาณรีเซ็ตซ้ำทุกๆ 5.0 วินาที เพื่อกู้คืนพอร์ตแบบวนซ้ำจนกว่าจะฟื้น
                if hasattr(self, 'last_reset_trigger_time'):
                    elapsed = (now - self.last_reset_trigger_time).nanoseconds * 1e-9
                    if elapsed >= 5.0:
                        self.get_logger().warn("🔄 [IMU Rescue] BNO055 is still frozen. Retrying hardware reset command...")
                        self.trigger_imu_reset()
                        self.last_reset_trigger_time = now
                else:
                    self.last_reset_trigger_time = now
            self.mag_corrupted = True
        else:
            if bno_was_frozen:
                self.get_logger().info("🟢 [BNO055 RESTORED] BNO055 Hardware recovered.")
                self.mag_corrupted = False

        # ─── 1. PREDICT STEP ───
        theta, bias = self.x[0], self.x[1]
        
        if self.bno_frozen:
            # 🚨 ค้าง! บังคับใช้ความเร็วเชิงมุมจากล้อ (Encoder) แทน
            # และใช้ Process Noise ที่เพิ่มขึ้นเพื่อเน้นความสำคัญของ GPS COG
            gyro_corrected = self.odom_omega_z
            current_Q = np.diag([self.Q_THETA * 4.0, 0.0])
            F = np.array([[1.0, 0.0],
                          [0.0, 1.0]])
        else:
            gyro_corrected = self.last_gyro_z - bias
            current_Q = self.Q
            F = np.array([[1.0, -dt],
                          [0.0,  1.0]])

        x_pred = np.array([
            normalize_angle(theta + gyro_corrected * dt),
            bias
        ])
        P_pred = F @ self.P @ F.T + current_Q

        # ─── 2. UPDATE STEP (เลือก Source ที่ดีที่สุด) ───
        if self.bno_frozen:
            source = "ODOM_BACKUP"
        else:
            source = "GYRO"

        # Priority 1: GPS COG
        if (self.current_speed >= self.SPEED_GPS_THRESHOLD and
                self.last_cog_rad is not None):
            z   = self.last_cog_rad
            H   = np.array([[1.0, 0.0]])
            R   = np.array([[self.R_GPS]])
            source = "GPS_COG"
            x_pred, P_pred = self._kalman_update(x_pred, P_pred, z, H, R)

        # Priority 2: Magnetometer (ใช้เมื่อไม่ค้างและไม่มี EMI)
        elif (not self.mag_corrupted and
              self.last_mag_yaw_rad is not None):
            z   = self.last_mag_yaw_rad
            H   = np.array([[1.0, 0.0]])
            R   = np.array([[self.R_MAG]])
            source = "MAG"
            x_pred, P_pred = self._kalman_update(x_pred, P_pred, z, H, R)

        # Priority 3: Gyro Dead-Reckoning
        else:
            if not self.bno_frozen:
                source = "GYRO"

        self.x = x_pred
        self.P = P_pred

        # ─── 3. PUBLISH ───
        heading_rad  = normalize_angle(self.x[0])
        heading_deg_compass = (90.0 - math.degrees(heading_rad)) % 360.0

        fused_msg = Float64()
        fused_msg.data = heading_deg_compass
        self.pub_fused.publish(fused_msg)

        source_msg = String()
        source_msg.data = source
        if self.mag_corrupted and not self.bno_frozen:
            source_msg.data += " [EMI!]"
        if self.bno_frozen:
            source_msg.data += " [BNO DEAD!]"
        self.pub_source.publish(source_msg)

    def trigger_imu_reset(self):
        msg = Empty()
        self.pub_reset_imu.publish(msg)
        self.get_logger().info("📤 [IMU Rescue] Published IMU Reset Request on /cmd_reset_imu")

    def _kalman_update(self, x_pred, P_pred, z_scalar, H, R):
        innovation = normalize_angle(z_scalar - (H @ x_pred)[0])
        S = H @ P_pred @ H.T + R
        K = P_pred @ H.T @ np.linalg.inv(S)
        x_new = x_pred + K.flatten() * innovation
        x_new[0] = normalize_angle(x_new[0])
        I = np.eye(2)
        KH = K @ H
        P_new = (I - KH) @ P_pred @ (I - KH).T + K @ R @ K.T
        return x_new, P_new


def main(args=None):
    rclpy.init(args=args)
    node = HeadingObserver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Auto Datum Node
---------------
อ่านค่า Datum (จุดอ้างอิง 0,0) จากไฟล์รั้ว lawn_geofence.yaml
แล้วตั้งค่าให้ navsat_transform_node ผ่าน Service Call อัตโนมัติ

ผลลัพธ์: ทุกครั้งที่ Restart ระบบ map frame จะเริ่มที่จุดเดิมเสมอ
          แม้จะย้ายไปสนามอื่น ก็แค่บันทึกรั้วใหม่ ไม่ต้องแก้ไฟล์ใดๆ
"""
import rclpy
from rclpy.node import Node
import yaml
import os
import math


class AutoDatumNode(Node):
    def __init__(self):
        super().__init__('auto_datum_node')

        self.declare_parameter('geofence_file', '~/ros2_ws/lawn_geofence.yaml')
        self.geofence_file = self.get_parameter('geofence_file').value

        self._done = False
        self._client = None
        self._future = None

        # รอ 3 วิให้ navsat_transform_node เริ่มทำงานก่อนค่อย set datum
        self.timer = self.create_timer(3.0, self._start_set_datum)
        self.get_logger().info("🛰️ Auto Datum Node started. Waiting for navsat_transform...")

    def _start_set_datum(self):
        """เรียกครั้งเดียว (One-shot) หลังจาก 3 วินาที"""
        if self._done:
            return
        self.timer.cancel()

        datum_lat, datum_lon = self._load_datum()

        if datum_lat is None:
            self.get_logger().warn(
                "⚠️ ไม่พบไฟล์ Geofence หรือ Datum ในไฟล์ "
                "→ navsat_transform จะใช้ GPS แรกที่รับได้เป็น Datum (Fallback)"
            )
            self._done = True
            return

        # Import service type
        try:
            from robot_localization.srv import SetDatum
        except ImportError:
            self.get_logger().error("❌ ไม่พบ robot_localization package! ตรวจสอบ dependencies")
            self._done = True
            return

        self._client = self.create_client(SetDatum, '/datum')

        if not self._client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                "❌ Service '/datum' ไม่พร้อม! "
                "ตรวจสอบว่า navsat_transform_node กำลังทำงานอยู่"
            )
            self._done = True
            return

        request = SetDatum.Request()
        request.geo_pose.position.latitude = datum_lat
        request.geo_pose.position.longitude = datum_lon
        request.geo_pose.position.altitude = 0.0
        # Yaw = 0 (หน้าหุ่นชี้ไปทาง East เป็น default ของ ENU)
        request.geo_pose.orientation.w = 1.0
        request.geo_pose.orientation.x = 0.0
        request.geo_pose.orientation.y = 0.0
        request.geo_pose.orientation.z = 0.0

        self._future = self._client.call_async(request)
        self._datum_lat = datum_lat
        self._datum_lon = datum_lon

        # ใช้ Timer เช็คผลลัพธ์ เพื่อไม่ให้ Block Event Loop
        self._check_timer = self.create_timer(0.1, self._check_result)

    def _check_result(self):
        """เช็คผลลัพธ์จาก Service Call"""
        if self._future is None or not self._future.done():
            return

        self._check_timer.cancel()

        if self._future.result() is not None:
            self.get_logger().info(
                f"✅ ตั้งค่า Datum สำเร็จ! "
                f"lat={self._datum_lat:.8f}, lon={self._datum_lon:.8f} "
                f"→ map frame 0,0 ล็อกแล้วครับ"
            )
        else:
            self.get_logger().error(
                f"❌ ตั้งค่า Datum ล้มเหลว: {self._future.exception()}"
            )

        self._done = True
        # Node ทำหน้าที่เสร็จแล้ว สามารถ shutdown ได้
        # แต่ยังทิ้งไว้ให้ ROS2 จัดการ lifecycle เองครับ

    def _load_datum(self):
        """อ่าน datum จากไฟล์ geofence"""
        if not os.path.exists(self.geofence_file):
            self.get_logger().warn(f"⚠️ ไม่พบไฟล์: {self.geofence_file}")
            return None, None

        try:
            with open(self.geofence_file, 'r') as f:
                data = yaml.safe_load(f)

            datum = data.get('datum', {})
            lat = datum.get('lat')
            lon = datum.get('lon')

            if lat is not None and lon is not None:
                self.get_logger().info(
                    f"📐 โหลด Datum จากไฟล์รั้ว: lat={float(lat):.8f}, lon={float(lon):.8f}"
                )
                return float(lat), float(lon)
            else:
                self.get_logger().warn("⚠️ ไม่พบ lat/lon ใน datum section ของไฟล์รั้ว")
                return None, None

        except Exception as e:
            self.get_logger().error(f"❌ อ่านไฟล์ geofence ล้มเหลว: {e}")
            return None, None


def main(args=None):
    rclpy.init(args=args)
    node = AutoDatumNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

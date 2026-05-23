#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int8
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Twist
import yaml
import math
import os
import threading
import tf2_ros
import time

class GeofenceSystem(Node):
    def __init__(self, start_mode='enforce'):
        super().__init__('geofence_system')
        
        default_geofence = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'lawn_geofence.yaml')
        self.declare_parameter('geofence_file', default_geofence)
        self.save_path = self.get_parameter('geofence_file').get_parameter_value().string_value
        
        self.safety_margin_meters = 0.20 # ✅ ขยายระยะให้หุ่นมีช่องว่างเลี้ยว ไม่เบรกจุกจิกจนกระตุก
        self.mode = start_mode 
        
        self.current_fix = None
        self.recorded_lat_lon = []
        self.recorded_points_xy = []
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.emergency_pub = self.create_publisher(Int8, '/emergency_stop', 10)
        self.marker_pub = self.create_publisher(Marker, '/geofence_viz', 10)
        # GPS sub ยังคงไว้สำหรับ Record Mode (บันทึกจุดรั้ว)
        self.gps_sub = self.create_subscription(NavSatFix, '/fix', self.gps_callback, 10)
        
        # Velocity Mux & Safety Filter
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_filtered', 10)
        self.nav_sub = self.create_subscription(Twist, '/cmd_vel_nav_smoothed', self.nav_callback, 10)
        self.teleop_sub = self.create_subscription(Twist, '/cmd_vel_teleop', self.teleop_callback, 10)
        
        self.last_teleop_time = self.get_clock().now()
        self.is_out_of_bounds = False

        self.valid_poly = None
        self.polygon_gps = []
        
        if self.mode == 'enforce':
            self.init_enforce_mode()
            # ✅ ใช้ Timer 10Hz ตรวจสอบตำแหน่งจาก TF แทน GPS ดิบ
            self.enforce_timer = self.create_timer(0.1, self.enforce_check_loop)

    def init_enforce_mode(self):
        data = self.load_geofence()
        if not data:
            self.valid_poly = None
            return

        self.polygon_gps = data.get('geofence', [])
        if not self.polygon_gps or len(self.polygon_gps) < 3:
            self.get_logger().error(f"❌ โหลดไฟล์รั้วล้มเหลว หรือจุดน้อยกว่า 3 จุด! โปรดรันโปรแกรมใหม่และเลือกโหมดสร้างรั้ว (2)")
            self.valid_poly = None
            return
        # ✅ ต้องโหลดจุดอ้างอิง (Datum) ทุกครั้งเพื่อให้ gps_to_enu ทำงานได้
        self.datum_lat = data.get('datum', {}).get('lat', self.polygon_gps[0]['lat'] if self.polygon_gps else 0.0)
        self.datum_lon = data.get('datum', {}).get('lon', self.polygon_gps[0]['lon'] if self.polygon_gps else 0.0)
        self.lat_to_m = 111320.0
        self.lon_to_m = 111320.0 * math.cos(math.radians(self.datum_lat))

        # 1. พยายามดึงพิกัด X,Y (แม่นยำที่สุด)
        polygon_xy = []
        if 'geofence_xy' in data:
            self.get_logger().info("📐 ใช้งานพิกัด X,Y เมตรจากไฟล์รั้ว")
            polygon_xy = [(p['x'], p['y']) for p in data['geofence_xy']]
        
        # 2. ถ้าไม่มี X,Y ให้แปลงจาก GPS (Lat/Lon)
        elif self.polygon_gps:
            self.get_logger().warn("🌐 ไม่พบพิกัด X,Y เมตร... กำลังแปลงจาก GPS (อาจมีความคลาดเคลื่อน)")
            # ใช้พิกัดกลางแผนที่ Gazebo เป็น Datum พื้นฐาน (เพื่อให้พิกัดเมตรสัมพันธ์กับ map)
            self.datum_lat = data.get('datum', {}).get('lat', 0.0)
            self.datum_lon = data.get('datum', {}).get('lon', 0.0)
            
            self.lat_to_m = 111320.0
            self.lon_to_m = 111320.0 * math.cos(math.radians(self.datum_lat))
            polygon_xy = [self.gps_to_enu(p['lat'], p['lon']) for p in self.polygon_gps]

        if not polygon_xy:
            self.get_logger().error("❌ ไม่มีข้อมูลพิกัดในไฟล์รั้ว!")
            self.valid_poly = None
            return
        
        try:
            from shapely.geometry import Polygon
            from shapely.validation import make_valid
            raw_poly = Polygon(polygon_xy)
            self.valid_poly = make_valid(raw_poly)
            self.get_logger().info(f"✅ โหลดรั้วสำเร็จ! (จำนวนจุด: {len(polygon_xy)})")
        except Exception as e:
            self.get_logger().error(f"❌ สร้างรูปทรง Polygon ล้มเหลว: {e}")
            self.valid_poly = None
            return
            
        self.get_logger().info(f"➡️ ระยะเบรกปลอดภัย: {self.safety_margin_meters} เมตร")

    def load_geofence(self):
        if not os.path.exists(self.save_path): return None
        with open(self.save_path, 'r') as f:
            return yaml.safe_load(f)

    def save_geofence(self):
        if len(self.recorded_lat_lon) < 3:
            print("⚠️ มีจุดน้อยกว่า 3 จุด ไม่สามารถเป็นรั้วได้! ยกเลิกการบันทึก")
            return False
            
        points_xy_list = [{'x': p.x, 'y': p.y} for p in self.recorded_points_xy]
        with open(self.save_path, 'w') as f:
            yaml.dump({
                'geofence': self.recorded_lat_lon,
                'geofence_xy': points_xy_list,
                'datum': self.recorded_lat_lon[0] if self.recorded_lat_lon else {}
            }, f)
        print(f"\n💾 เซฟไฟล์สำเร็จ! ทั้งหมด {len(self.recorded_lat_lon)} จุด ไปที่ {self.save_path}")
        return True

    def gps_to_enu(self, lat, lon):
        x = (lon - self.datum_lon) * self.lon_to_m
        y = (lat - self.datum_lat) * self.lat_to_m
        return (x, y)

    def gps_callback(self, msg):
        """เก็บค่า GPS ล่าสุดไว้สำหรับ Record Mode เท่านั้น"""
        self.current_fix = msg
        
        if self.mode == 'record':
            self.publish_marker_record()

    def enforce_check_loop(self):
        """ตรวจสอบตำแหน่งหุ่นยนต์จาก TF map->base_link ที่ 10Hz"""
        if not self.valid_poly:
            return

        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now)
            x = trans.transform.translation.x
            y = trans.transform.translation.y
        except Exception:
            # TF ยังไม่พร้อม (ปกติตอนเพิ่งเริ่มรัน)
            return

        from shapely.geometry import Point
        point = Point(x, y)
        is_in_bounds = self.valid_poly.contains(point)
        dist_to_edge = self.valid_poly.boundary.distance(point)

        stop_msg = Int8()
        if not is_in_bounds:
            self.get_logger().error("🚨 [STOP] นอกเขต! หุ่นยนต์เลยเส้นแดงออกไปแล้ว")
            self.is_out_of_bounds = True
            stop_msg.data = 1
        elif dist_to_edge < self.safety_margin_meters:
            self.get_logger().warn(f"⚠️ [SLOW] เข้าใกล้ขอบเขตอันตราย! ({dist_to_edge:.2f}m)")
            self.is_out_of_bounds = False
            stop_msg.data = 0
        else:
            self.is_out_of_bounds = False
            stop_msg.data = 0

        self.emergency_pub.publish(stop_msg)
        self.publish_marker_enforce()

    def teleop_callback(self, msg):
        self.last_teleop_time = self.get_clock().now()
        self.process_and_publish(msg, "TELEOP")

    def nav_callback(self, msg):
        # Priority logic: ignore Nav2 if Teleop was used in the last 1.0 second
        now = self.get_clock().now()
        if (now - self.last_teleop_time).nanoseconds < 1e9: # 1 second timeout
            return
            
        self.process_and_publish(msg, "NAV2")

    def process_and_publish(self, twist_msg, source):
        # Always allow stop commands (linear x == 0 and angular z == 0)
        is_stop_cmd = (abs(twist_msg.linear.x) < 0.001 and abs(twist_msg.angular.z) < 0.001)

        if self.mode == 'enforce' and self.is_out_of_bounds and not is_stop_cmd:
            # Force zero velocity if out of bounds and trying to move
            safe_twist = Twist()
            self.cmd_pub.publish(safe_twist)
            # Only log every 10th time to avoid spam
            if not hasattr(self, '_log_cnt'): self._log_cnt = 0
            self._log_cnt += 1
            if self._log_cnt % 10 == 0:
                self.get_logger().error(f"🛑 Blocking {source} command: Robot is OUT OF BOUNDS!")
        else:
            # Pass through (or allow stopping even if out of bounds)
            self.cmd_pub.publish(twist_msg)

    def record_point(self):
        if self.current_fix is None:
            print("❌ ยังไม่มีสัญญาณ GPS... กรุณารอสักครู่")
            return

        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now)
            
            p_x = trans.transform.translation.x
            p_y = trans.transform.translation.y
            
            if self.recorded_points_xy:
                last_p = self.recorded_points_xy[-1]
                dist_m = math.hypot(p_x - last_p.x, p_y - last_p.y)
                if dist_m < 0.05:
                    print(f"⚠️ จุดบันทึกใกล้เกินไป ({dist_m:.2f}m) ขยับรถออกไปอีกนิดครับ (เกิน 20cm)")
                    return

            p_xy = Point()
            p_xy.x = p_x
            p_xy.y = p_y
            p_xy.z = 0.0
            self.recorded_points_xy.append(p_xy)

            point_gps = {
                'lat': self.current_fix.latitude,
                'lon': self.current_fix.longitude,
                'alt': self.current_fix.altitude
            }
            self.recorded_lat_lon.append(point_gps)
            print(f"✅ บันทึกจุดที่ {len(self.recorded_lat_lon)} สำเร็จ! (Lat: {point_gps['lat']:.8f}, Lon: {point_gps['lon']:.8f})")
            self.publish_marker_record()

        except Exception as e:
            print(f"❌ ไม่สามารถระบุตำแหน่งบนแผนที่ได้: {e}")

    def publish_marker_record(self):
        if not self.recorded_points_xy: return
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "geofence"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1 
        marker.color.a = 1.0 
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.points = list(self.recorded_points_xy) 
        if len(self.recorded_points_xy) > 1:
             marker.points.append(self.recorded_points_xy[0])
        self.marker_pub.publish(marker)

    def publish_marker_enforce(self):
        if not self.valid_poly: return
        
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "geofence"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1 
        marker.color.a = 1.0 
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # ✅ ดึงจุด X,Y จาก Polygon ตัวจริงที่ใช้ประมวลผลมาวาด
        # วิธีนี้จะทำให้ Rviz ตรงกับสิ่งที่หุ่น "รู้สึก" จริงๆ
        coords = list(self.valid_poly.exterior.coords)
        for x, y in coords:
            p_xy = Point()
            p_xy.x = x
            p_xy.y = y
            p_xy.z = 0.0
            marker.points.append(p_xy)
            
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    
    print("\n================================")
    print("🛡️ ระบบจัดการรั้วไฟฟ้าอัจฉริยะ (Geofence System - All in One) 🛡️")
    print("1. [Enforce] โหมดกันชน (สแกนแนวรั้วและเบรกอัตโนมัติ)")
    print("--- ระบบควบคุมเขตปลอดภัย ---")
    print("1: โหลดและบังคับใช้รั้วทันที (Enforce)")
    print("2: สร้างรั้วใหม่ (Teach-in)")
    print("-------------------------")
    
    # ใช้งานโหมด 1 ทันทีถ้าไม่อยากให้บล็อก CI หรือการรันแบบอัตโนมัติ
    try:
        import select, sys
        i, o, e = select.select([sys.stdin], [], [], 2.0)
        if (i):
            choice = sys.stdin.readline().strip()
        else:
            print("⏳ หมดเวลาเลือก โหลดใช้งานรั้วอัตโนมัติ (1)")
            choice = '1'
    except Exception:
        choice = '1'
    
    mode = 'record' if choice == '2' else 'enforce'
    system = GeofenceSystem(start_mode=mode)
    
    if mode == 'record':
        spin_thread = threading.Thread(target=rclpy.spin, args=(system,), daemon=True)
        spin_thread.start()
        try:
            print("\n🚨 [โหมดสร้างรั้ว] ระบบเบรกถูกปิดชั่วคราว ขับรถหลุดขอบเขตได้อิสระ 🚨")
            while rclpy.ok() and system.mode == 'record':
                val = input("\n💬 ขับไปที่มุมสนาม -> กด [Enter] เพื่อบันทึกจุด (พิมพ์ 's' แล้วกด Enter ค้างไว้เพื่อเซฟ): ").strip().lower()
                if val == 's':
                    if system.save_geofence():
                        print("✅ โหลดแผนที่เข้าสู่สมองกล...")
                        system.mode = 'enforce'
                        system.init_enforce_mode()
                else:
                    system.record_point()
        except KeyboardInterrupt:
            pass
            
    if system.mode == 'enforce':
        print("\n🚀 [โหมดกันชน] หุ่นยนต์พร้อมทำงานและรักษาระยะห่างจากรั้วแล้ว (กด Ctrl+C เพื่อออก) 🚀")
        if 'spin_thread' not in locals(): 
            try:
                rclpy.spin(system)
            except KeyboardInterrupt:
                pass
        else: 
            try:
                while rclpy.ok():
                    time.sleep(1)
            except KeyboardInterrupt:
                pass

    system.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

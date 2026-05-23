#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int8
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Twist, PoseStamped
from nav_msgs.msg import Path
import yaml
import math
import os
import threading
import tf2_ros
import time
import select
import sys
from shapely.geometry import Polygon, LineString, Point as SPoint
from shapely.affinity import rotate
from shapely.validation import make_valid

class GeofenceAndPlanner(Node):
    def __init__(self, start_mode='enforce'):
        super().__init__('geofence_and_planner')
        
        # --- 1. Parameters (จากทั้งสองโค้ด) ---
        # กำหนด path ของ geofence ให้สัมพันธ์กับ workspace
        current_dir = os.path.dirname(os.path.abspath(__file__))
        workspace_dir = os.path.abspath(os.path.join(current_dir, "../../../"))
        default_geofence = os.path.join(workspace_dir, "lawn_geofence.yaml")
        
        self.declare_parameter('geofence_file', default_geofence)
        self.save_path = os.path.expanduser(self.get_parameter('geofence_file').get_parameter_value().string_value)
        self.safety_margin_meters = 0.20
        self.mode = start_mode 
        
        self.declare_parameter('mower_width', 0.60)
        self.declare_parameter('overlap', 0.24)
        self.declare_parameter('fence_offset', 0.30)
        
        self.update_parameters()
        
        # --- 2. Variables ---
        self.current_fix = None
        self.recorded_lat_lon = []
        self.recorded_points_xy = []
        self.is_out_of_bounds = False
        self.valid_poly = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.last_teleop_time = self.get_clock().now()

        # --- 3. Publishers & Subscribers ---
        self.emergency_pub = self.create_publisher(Int8, '/emergency_stop', 10)
        self.marker_pub = self.create_publisher(Marker, '/geofence_viz', 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/fix', self.gps_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_filtered', 10)
        self.nav_sub = self.create_subscription(Twist, '/cmd_vel_nav_smoothed', self.nav_callback, 10)
        self.teleop_sub = self.create_subscription(Twist, '/cmd_vel_teleop', self.teleop_callback, 10)
        
        self.path_pub = self.create_publisher(Path, '/mowing_path', 10)
        self.planner_marker_pub = self.create_publisher(Marker, '/mowing_markers', 10)
        
        if self.mode == 'enforce':
            self.init_enforce_mode()
            self.enforce_timer = self.create_timer(0.1, self.enforce_check_loop)
            self.get_logger().info("⏳ Waiting 1s for Planner to start...")
            threading.Timer(1.0, self.start_planner_timer).start()

    def update_parameters(self):
        self.mower_width = self.get_parameter('mower_width').value
        self.overlap = self.get_parameter('overlap').value
        self.fence_offset = self.get_parameter('fence_offset').value
        self.lane_spacing = self.mower_width - self.overlap

    def start_planner_timer(self):
        self.create_timer(5.0, self.plan_logic)
        self.get_logger().info("🚀 Planner Ready with Full Path Data!")

    def init_enforce_mode(self):
        if not os.path.exists(self.save_path): return
        with open(self.save_path, 'r') as f:
            data = yaml.safe_load(f)
            polygon_xy = []
            if 'geofence_xy' in data:
                polygon_xy = [(p['x'], p['y']) for p in data['geofence_xy']]
            if polygon_xy:
                self.valid_poly = make_valid(Polygon(polygon_xy))
                self.get_logger().info(f"✅ โหลดรั้วสำเร็จ! {len(polygon_xy)} จุด")

    def gps_callback(self, msg):
        self.current_fix = msg

    def enforce_check_loop(self):
        if not self.valid_poly: return
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x, y = trans.transform.translation.x, trans.transform.translation.y
            pt = SPoint(x, y)
        except: return

        is_in = self.valid_poly.contains(pt)
        dist = self.valid_poly.boundary.distance(pt)
        
        if not is_in:
            self.get_logger().error("🚨 [STOP] นอกเขต! หุ่นยนต์เลยเส้นออกไปแล้ว")
            self.is_out_of_bounds = True
        elif dist < self.safety_margin_meters:
            self.get_logger().warn(f"⚠️ [SLOW] เข้าใกล้ขอบเขตอันตราย! ({dist:.2f}m)")
            self.is_out_of_bounds = False
        else:
            self.is_out_of_bounds = False

        stop_msg = Int8()
        stop_msg.data = 1 if self.is_out_of_bounds else 0
        self.emergency_pub.publish(stop_msg)
        self.publish_marker_enforce()

    def teleop_callback(self, msg):
        self.last_teleop_time = self.get_clock().now()
        self.process_and_publish(msg, "TELEOP")

    def nav_callback(self, msg):
        now = self.get_clock().now()
        if (now - self.last_teleop_time).nanoseconds < 1e9: return
        self.process_and_publish(msg, "NAV2")

    def process_and_publish(self, twist_msg, source):
        is_stop = (abs(twist_msg.linear.x) < 0.001 and abs(twist_msg.angular.z) < 0.001)
        if self.mode == 'enforce' and self.is_out_of_bounds and not is_stop:
            self.cmd_pub.publish(Twist())
        else:
            self.cmd_pub.publish(twist_msg)

    def record_point(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            p_x, p_y = trans.transform.translation.x, trans.transform.translation.y
            self.recorded_points_xy.append(Point(x=p_x, y=p_y, z=0.0))
            if self.current_fix:
                self.recorded_lat_lon.append({'lat':self.current_fix.latitude, 'lon':self.current_fix.longitude})
            print(f"✅ บันทึกจุดที่ {len(self.recorded_points_xy)} สำเร็จ!")
        except: print("❌ รอตำแหน่งจาก TF...")

    def save_geofence(self):
        if len(self.recorded_points_xy) < 3: return False
        points_xy_list = [{'x': p.x, 'y': p.y} for p in self.recorded_points_xy]
        with open(self.save_path, 'w') as f:
            yaml.dump({'geofence': self.recorded_lat_lon, 'geofence_xy': points_xy_list}, f)
        return True

    def publish_marker_enforce(self):
        if not self.valid_poly: return
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.1
        marker.color.a, marker.color.r = 1.0, 1.0
        coords = list(self.valid_poly.exterior.coords)
        for x, y in coords:
            marker.points.append(Point(x=float(x), y=float(y), z=0.0))
        self.marker_pub.publish(marker)

    def plan_logic(self):
        self.update_parameters() # ดึงค่าล่าสุด
        if not self.valid_poly: return
        inner_poly = self.valid_poly.buffer(-self.fence_offset)
        if inner_poly.is_empty: return
        
        coords = list(self.valid_poly.exterior.coords)
        best_angle = 0.0
        max_dist = 0.0
        for i in range(len(coords) - 1):
            dist = math.hypot(coords[i+1][0]-coords[i][0], coords[i+1][1]-coords[i][1])
            if dist > max_dist:
                max_dist = dist
                best_angle = math.degrees(math.atan2(coords[i+1][1]-coords[i][1], coords[i+1][0]-coords[i][0]))

        rotated_poly = rotate(inner_poly, -best_angle, origin=(0, 0))
        min_x, min_y, max_x, max_y = rotated_poly.bounds
        path_points = []
        current_y = min_y + (self.lane_spacing / 2.0)
        reverse = False
        
        while current_y < max_y:
            line = LineString([(min_x - 1, current_y), (max_x + 1, current_y)])
            intersection = rotated_poly.intersection(line)
            if not intersection.is_empty:
                if intersection.geom_type == 'LineString':
                    c = list(intersection.coords)
                    if reverse: c.reverse()
                    path_points.extend(c)
                elif intersection.geom_type == 'MultiLineString':
                    lines = sorted(list(intersection.geoms), key=lambda l: l.bounds[0], reverse=reverse)
                    for l in lines:
                        c = list(l.coords)
                        if reverse: c.reverse()
                        path_points.extend(c)
                reverse = not reverse
            current_y += self.lane_spacing
            
        final_points = []
        for pt in path_points:
            p_obj = rotate(SPoint(pt), best_angle, origin=(0, 0))
            final_points.append((p_obj.x, p_obj.y))
        
        # --- Densify และคำนวณ Yaw (Orientation) ตามต้นฉบับ lawn_planner ---
        dense_points = []
        resolution = 0.1  # 10 cm
        for i in range(len(final_points) - 1):
            p1, p2 = final_points[i], final_points[i+1]
            dist = math.hypot(p2[0]-p1[0], p2[1]-p1[1])
            num_steps = max(1, int(dist / resolution))
            for step in range(num_steps):
                alpha = step / float(num_steps)
                x = p1[0] + alpha * (p2[0] - p1[0])
                y = p1[1] + alpha * (p2[1] - p1[1])
                dense_points.append((x, y))
        dense_points.append(final_points[-1])
        
        self.publish_path(dense_points)

    def publish_path(self, dense_points):
        if not dense_points: return
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for i, pt in enumerate(dense_points):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(pt[0])
            pose.pose.position.y = float(pt[1])
            
            # คำนวณ Yaw เพื่อให้รถหันหน้าถูกทาง (ORIENTATION FIX)
            yaw = 0.0
            if i < len(dense_points) - 1:
                next_pt = dense_points[i+1]
                yaw = math.atan2(next_pt[1] - pt[1], next_pt[0] - pt[0])
            elif i > 0:
                prev_pt = dense_points[i-1]
                yaw = math.atan2(pt[1] - prev_pt[1], pt[0] - prev_pt[0])
            
            pose.pose.orientation.w = float(math.cos(yaw / 2.0))
            pose.pose.orientation.z = float(math.sin(yaw / 2.0))
            path_msg.poses.append(pose)
            
        self.path_pub.publish(path_msg)
        
        # Marker สำหรับ Rviz
        marker = Marker()
        marker.header = path_msg.header
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.1
        marker.color.a, marker.color.g = 1.0, 1.0
        for pt in dense_points:
            marker.points.append(Point(x=float(pt[0]), y=float(pt[1]), z=0.15))
        self.planner_marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    print("\n=== ระบบ Geofence & Planner รวมร่าง (V.Full) ===")
    print("1: ใช้งานรั้วเดิม (Enforce + Planner)")
    print("2: สร้างรั้วใหม่ (Teach-in)")
    
    i, o, e = select.select([sys.stdin], [], [], 5.0)
    choice = sys.stdin.readline().strip() if i else '1'
    
    mode = 'record' if choice == '2' else 'enforce'
    node = GeofenceAndPlanner(start_mode=mode)
    
    if mode == 'record':
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()
        print("\n[โหมดสร้างรั้ว] กด Enter เพื่อเก็บจุด, พิมพ์ 's' เพื่อเซฟ")
        while rclpy.ok():
            val = input().strip().lower()
            if val == 's':
                if node.save_geofence():
                    print("💾 เซฟสำเร็จ! กรุณารันใหม่เพื่อเข้าโหมด Enforce")
                    break
            else: node.record_point()
    else:
        rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

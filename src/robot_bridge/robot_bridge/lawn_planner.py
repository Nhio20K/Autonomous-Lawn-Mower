#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point as MsgPoint
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
import yaml
import math
import os
from shapely.geometry import Polygon, LineString, Point
from shapely.affinity import rotate

class LawnPlanner(Node):
    def __init__(self):
        super().__init__('lawn_planner')
        

        self.declare_parameter('geofence_file', '~/ros2_ws/lawn_geofence.yaml')
        self.declare_parameter('mower_width', 0.60)
        self.declare_parameter('overlap', 0.10)
        self.declare_parameter('fence_offset', 0.0)  # 👈 ระยะห่างจากขอบรั้วถึงจุดกลางรถ
        
        self.geofence_file = self.get_parameter('geofence_file').get_parameter_value().string_value
        self.mower_width = self.get_parameter('mower_width').get_parameter_value().double_value
        self.overlap = self.get_parameter('overlap').get_parameter_value().double_value
        self.fence_offset = self.get_parameter('fence_offset').get_parameter_value().double_value
        self.lane_spacing = self.mower_width - self.overlap
        
        self.path_pub = self.create_publisher(Path, '/mowing_path', 10)
        self.marker_pub = self.create_publisher(Marker, '/mowing_markers', 10)
        
        self.timer = self.create_timer(2.0, self.plan_and_publish)
        self.get_logger().info("🚀 Lawn Planner Node Started! (Zig-Zag Mode)")

    def load_geofence(self):
        if not os.path.exists(self.geofence_file):
            self.get_logger().error(f"❌ Geofence file not found: {self.geofence_file}")
            return None
        
        with open(self.geofence_file, 'r') as f:
            data = yaml.safe_load(f)
            
            poly_points = []
            if 'geofence_xy' in data:
                # ✅ ใช้พิกัด X,Y เมตรโดยตรง (แม่นยำที่สุด)
                self.get_logger().info("📐 ใช้งานพิกัด X,Y เมตรจากไฟล์รั้ว")
                poly_points = [(p['x'], p['y']) for p in data['geofence_xy']]
            else:
                # 🌐 หากไม่มี X,Y ให้ใช้ GPS (แบบเดิม)
                points = data.get('geofence', [])
                if not points: return None
                
                datum = data.get('datum', {})
                datum_lat = datum.get('lat', points[0]['lat'])
                datum_lon = datum.get('lon', points[0]['lon'])
                
                lat_to_m = 111320.0
                lon_to_m = 111320.0 * math.cos(math.radians(datum_lat))
                
                for p in points:
                    x = (p['lon'] - datum_lon) * lon_to_m
                    y = (p['lat'] - datum_lat) * lat_to_m
                    poly_points.append((x, y))
            
            return Polygon(poly_points)

    def generate_coverage_path(self, polygon):
        if not polygon or polygon.is_empty:
            return []

        # Buffer polygon inwards: ใช้ fence_offset แทน mower_width/2
        inner_poly = polygon.buffer(-self.fence_offset)
        if inner_poly.is_empty:
            self.get_logger().warn("❌ Geofence is too narrow for this mower width!")
            return []

        # 1. Find the optimal angle (longest edge)
        # We use the original polygon to find the best angle so the lines are parallel to the main boundaries
        best_angle = 0.0
        max_dist = 0.0
        coords = list(polygon.exterior.coords)
        for i in range(len(coords) - 1):
            p1 = coords[i]
            p2 = coords[i+1]
            dist = math.hypot(p2[0]-p1[0], p2[1]-p1[1])
            if dist > max_dist:
                max_dist = dist
                best_angle = math.degrees(math.atan2(p2[1]-p1[1], p2[0]-p1[0]))

        # 2. Rotate polygon to align with the X-axis
        rotated_poly = rotate(inner_poly, -best_angle, origin=(0, 0))
        min_x, min_y, max_x, max_y = rotated_poly.bounds
        
        path_points = []
        current_y = min_y + (self.lane_spacing / 2.0)
        reverse = False
        
        while current_y < max_y:
            # Create a horizontal line across the bounding box
            line = LineString([(min_x - 1, current_y), (max_x + 1, current_y)])
            intersection = rotated_poly.intersection(line)
            
            if not intersection.is_empty:
                self._add_intersection_to_path(intersection, path_points, reverse)
                reverse = not reverse # Zig-Zag effect
                
            current_y += self.lane_spacing

        # ✅ [FIX] ตรวจสอบแถวสุดท้าย (Final Gap Fill)
        # หากจุด current_y ล่าสุด ยังคลุมไม่ถึงขอบสนาม (max_y) ให้เพิ่มแถวสุดท้ายชิดขอบรั้ว
        last_coverage_y = (current_y - self.lane_spacing) + (self.mower_width / 2.0)
        if last_coverage_y < max_y:
            final_y = max_y - (self.mower_width / 2.0)
            line = LineString([(min_x - 1, final_y), (max_x + 1, final_y)])
            intersection = rotated_poly.intersection(line)
            if not intersection.is_empty:
                self.get_logger().info(f"✨ Adding final safety row at Y={final_y:.2f} to cover the edge!")
                self._add_intersection_to_path(intersection, path_points, reverse)

    def _add_intersection_to_path(self, intersection, path_points, reverse):
        if intersection.geom_type == 'LineString':
            coords = list(intersection.coords)
            if reverse: coords.reverse()
            for pt in coords:
                path_points.append(pt)
        elif intersection.geom_type == 'MultiLineString':
            # Handle polygons with holes or multiple segments
            lines = sorted(list(intersection.geoms), key=lambda l: l.bounds[0], reverse=reverse)
            for l in lines:
                coords = list(l.coords)
                if reverse: coords.reverse()
                for pt in coords:
                    path_points.append(pt)
            
        # 3. Rotate back to original frame
        final_points = []
        for pt in path_points:
            # Re-implement rotation for points manually or create temporary geometry
            p_obj = rotate(Point(pt), best_angle, origin=(0, 0))
            final_points.append((p_obj.x, p_obj.y))
            
        return final_points

    def plan_and_publish(self):
        poly = self.load_geofence()
        if not poly: return
        
        final_points = self.generate_coverage_path(poly)
        if not final_points: return
        
        # Densify the path points with linear interpolation
        dense_points = []
        resolution = 0.1  # 10 cm apart
        for i in range(len(final_points) - 1):
            p1 = final_points[i]
            p2 = final_points[i+1]
            dist = math.hypot(p2[0]-p1[0], p2[1]-p1[1])
            num_steps = max(1, int(dist / resolution))
            
            for step in range(num_steps):
                alpha = step / float(num_steps)
                x = p1[0] + alpha * (p2[0] - p1[0])
                y = p1[1] + alpha * (p2[1] - p1[1])
                dense_points.append((x, y))
        dense_points.append(final_points[-1])
        
        # 1. Publish Path message
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for i, pt in enumerate(dense_points):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(pt[0])
            pose.pose.position.y = float(pt[1])
            pose.pose.position.z = 0.0
            
            yaw = 0.0
            if i < len(dense_points) - 1:
                next_pt = dense_points[i+1]
                yaw = math.atan2(next_pt[1] - pt[1], next_pt[0] - pt[0])
            elif i > 0:
                prev_pt = dense_points[i-1]
                yaw = math.atan2(pt[1] - prev_pt[1], pt[0] - prev_pt[0])
                
            pose.pose.orientation.w = float(math.cos(yaw / 2.0))
            pose.pose.orientation.z = float(math.sin(yaw / 2.0))
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            
            path_msg.poses.append(pose)
            
        self.path_pub.publish(path_msg)

        # 2. Publish Marker for visualization (S-Curve points)
        marker = Marker()
        marker.header = path_msg.header
        marker.ns = "mowing_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        
        for pt in dense_points:
            p = MsgPoint()
            p.x = float(pt[0])
            p.y = float(pt[1])
            p.z = 0.15
            marker.points.append(p)
            
        self.marker_pub.publish(marker)
        self.get_logger().info(f"📍 Published mowing path with {len(dense_points)} waypoints")

def main(args=None):
    rclpy.init(args=args)
    node = LawnPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

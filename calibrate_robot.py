#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import sys
import time

def quaternion_to_euler(q):
    # Convert quaternion to yaw (z-axis rotation)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class UniversalCalibrator(Node):
    def __init__(self, mode, target):
        super().__init__('universal_calibrator')
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        
        self.mode = mode # 'linear' or 'angular'
        
        if mode == 'linear':
            self.target = float(target)
            unit = "meters"
        else:
            # แปลง องศา -> เรเดียน (เพื่อให้คนใช้ง่ายขึ้น)
            self.target = math.radians(float(target))
            unit = "degrees"
            
        self.start_p = None
        self.start_yaw = None
        self.last_yaw: float = 0.0
        self.total_yaw = 0.0
        self.current_val = 0.0
        self.finished = False
        
        self.timer = self.create_timer(0.1, self.loop)
        
        log_target = target if mode == 'angular' else self.target
        self.get_logger().info(f"CALIBRATION START: Mode={mode}, Target={log_target} {unit}")

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        
        if self.mode == 'linear':
            if self.start_p is None:
                self.start_p = (pos.x, pos.y)
                return
            dist = math.sqrt((pos.x - self.start_p[0])**2 + (pos.y - self.start_p[1])**2)
            self.current_val = dist
        else:
            yaw = quaternion_to_euler(ori)
            if self.start_yaw is None:
                self.last_yaw = yaw
                self.start_yaw = yaw
                return
                
            # Calculate incremental diff and handle wrap-around
            diff = yaw - self.last_yaw
            if diff > math.pi:
                diff -= 2 * math.pi
            elif diff < -math.pi:
                diff += 2 * math.pi
                
            self.total_yaw += diff
            self.last_yaw = yaw
            self.current_val = abs(self.total_yaw)

    def loop(self):
        if (self.mode == 'linear' and self.start_p is None) or (self.mode == 'angular' and self.start_yaw is None):
            return
            
        msg = Twist()
        # ใช้ค่าสัมบูรณ์ (abs) ในการเทียบระยะทาง เพื่อให้รองรับค่าติดลบ (ถอยหลัง) ครับ
        if self.current_val < abs(self.target):
            if self.mode == 'linear':
                msg.linear.x = 0.15 if self.target > 0 else -0.15
            else:
                # หมุนซ้ายถ้าค่าเป็นบวก หมุนขวาถ้าค่าเป็นลบ
                msg.angular.z = 0.3 if float(sys.argv[2]) > 0 else -0.3
            
            self.pub_cmd.publish(msg)
            
            if self.mode == 'linear':
                print(f"\r[RUNNING] Current: {self.current_val:.3f} m / Target: {abs(self.target):.3f} m", end="")
            else:
                curr_deg = math.degrees(self.current_val)
                target_deg = abs(float(sys.argv[2]))
                print(f"\r[RUNNING] Current: {curr_deg:.1f}° / Target: {target_deg}°", end="")
        else:
            if not self.finished:
                # ส่งคำสั่งหยุด 0.0 ทันที
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.pub_cmd.publish(msg)
                
                final_val = self.current_val if self.mode == 'linear' else math.degrees(self.current_val)
                unit = "m" if self.mode == 'linear' else "deg"
                direction = "Forward" if (self.mode == 'linear' and self.target > 0) else "Backward" if (self.mode == 'linear') else "Rotation"
                
                print(f"\n[GOAL REACHED] {direction} final: {final_val:.3f} {unit}")
                self.finished = True
                
                # ส่งหยุดซ้ำๆ สั้นๆ เพื่อเคลียร์บัฟเฟอร์ แล้วปิดโหนดทันทีครับ
                for _ in range(5):
                    self.pub_cmd.publish(msg)
                    time.sleep(0.05)
                
                print("Cleanup complete. Ready for next command.")
                # ใช้ระบบปิดโหนดที่สะอาด
                self.destroy_node()
                rclpy.shutdown()
                import os
                os._exit(0)

def print_usage():
    print("Usage: python3 calibrate_robot.py <mode> <target>")
    print("Modes:")
    print("  linear <meters>  : Move forward/backward (e.g. 1.0 or -1.0)")
    print("  angular <degrees>: Rotate in place (e.g. 90 or -90)")

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print_usage()
        sys.exit(1)
        
    mode = sys.argv[1]
    target = sys.argv[2]
    
    rclpy.init()
    node = UniversalCalibrator(mode, target)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class LinearCalibrator(Node):
    def __init__(self):
        super().__init__('linear_calibrator')
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom_raw', self.odom_callback, 10)
        
        self.target_distance = 1.0  # เมตร
        self.start_p = None
        self.current_distance = 0.0
        self.finished = False
        
        self.timer = self.create_timer(0.1, self.loop)
        self.get_logger().info(f"Targeting: {self.target_distance} meters. Monitoring /odom_raw...")

    def odom_callback(self, msg):
        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y
        
        if self.start_p is None:
            self.start_p = (curr_x, curr_y)
            self.get_logger().info(f"Started at: {self.start_p}")
            return
            
        self.current_distance = math.sqrt((curr_x - self.start_p[0])**2 + (curr_y - self.start_p[1])**2)

    def loop(self):
        if self.start_p is None: return
        
        msg = Twist()
        if self.current_distance < self.target_distance:
            msg.linear.x = 0.15  # ความเร็วช้าๆ เพื่อความชัวร์
            self.pub_cmd.publish(msg)
            print(f"\rMoving... Distance: {self.current_distance:.3f}m / {self.target_distance}m", end="")
        else:
            if not self.finished:
                msg.linear.x = 0.0
                self.pub_cmd.publish(msg)
                print(f"\n[GOAL REACHED] Odom says: {self.current_distance:.3f}m")
                print("Stopping robot...")
                self.finished = True
                # หยุดส่งคำสั่ง 2 วินาทีเพื่อให้ Smoother/Bridge มั่นใจว่าต้องหยุด
                for _ in range(20):
                    self.pub_cmd.publish(msg)
                    time.sleep(0.1)
                print("Finished. Please measure the physical distance.")
                rclpy.shutdown()

def main():
    rclpy.init()
    node = LinearCalibrator()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass

if __name__ == '__main__':
    main()

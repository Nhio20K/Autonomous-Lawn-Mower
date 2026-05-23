#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
import time

class RobotIMUMonitor(Node):
    def __init__(self):
        super().__init__('robot_imu_monitor')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.sub = self.create_subscription(Imu, '/camera/camera/imu', self.callback, qos)
        
        self.get_logger().info("Robot-Centric IMU Monitor Started")
        self.get_logger().info("This script integrates raw IMU but maps them to ROBOT FRAME (base_link)")

        # Robot Frame Accumulators
        self.r_roll = 0.0
        self.r_pitch = 0.0
        self.r_yaw = 0.0
        
        self.last_time = None

    def callback(self, msg):
        current_time = time.time()
        if self.last_time is None:
            self.last_time = current_time
            return
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Raw IMU (camera_imu_optical_frame)
        # Based on user observation & TF matrix:
        # Robot_Yaw (Z) = IMU_Y 
        # Robot_Pitch (Y) = -IMU_Z (approx)
        # Robot_Roll (X) = -IMU_X (approx)
        
        imu_x = msg.angular_velocity.x
        imu_y = msg.angular_velocity.y
        imu_z = msg.angular_velocity.z
        
        # Apply Mapping (Tailored to this robot's mounting)
        # NOTE: Using a slightly different mapping to match user's "Pitch is Yaw" observation
        self.r_yaw += imu_y * dt
        self.r_pitch += (-imu_z) * dt
        self.r_roll += (-imu_x) * dt
        
        y_deg = math.degrees(self.r_yaw)
        p_deg = math.degrees(self.r_pitch)
        r_deg = math.degrees(self.r_roll)
        
        print(f"\r[ROBOT FRAME] Yaw: {y_deg: 6.2f}° | Pitch: {p_deg: 6.2f}° | Roll: {r_deg: 6.2f}°", end="")

def main():
    rclpy.init()
    node = RobotIMUMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

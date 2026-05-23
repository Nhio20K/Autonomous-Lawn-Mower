#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import math

class UltrasonicToPointCloud(Node):
    def __init__(self):
        super().__init__('ultrasonic_converter')
        
        # Subscribe to the 3 ultrasonic topics published by arduino_reader
        self.sub_left = self.create_subscription(Range, 'ultrasonic/left', self.left_cb, 10)
        self.sub_center = self.create_subscription(Range, 'ultrasonic/center', self.center_cb, 10)
        self.sub_right = self.create_subscription(Range, 'ultrasonic/right', self.right_cb, 10)
        
        # Publish combined PointCloud2
        self.pc_pub = self.create_publisher(PointCloud2, 'ultrasonic_pointcloud', 10)
        
        # Cache the latest distances and timestamps
        self.dist_l = float('inf')
        self.time_l = 0.0
        self.dist_c = float('inf')
        self.time_c = 0.0
        self.dist_r = float('inf')
        self.time_r = 0.0

        # Timer to publish cloud at 10Hz
        self.timer = self.create_timer(0.1, self.publish_cloud)

    def filter_dist(self, new_dist, current_dist, last_time):
        """
        Temporal Hold Filter (Tuned for JSN-SR04T)
        If the sensor goes blind (-1 or inf) recently after seeing an obstacle closer than 50cm,
        we HOLD that obstacle on the map for 2.0 seconds.
        """
        now = self.get_clock().now().nanoseconds / 1e9
        if not math.isinf(new_dist) and new_dist > 0:
            return new_dist, now
        else:
            # Blind spot hold logic
            if current_dist < 0.50 and (now - last_time) < 2.0:
                # Keep holding the ghost obstacle
                return current_dist, last_time
            else:
                return float('inf'), last_time

    def left_cb(self, msg):
        self.dist_l, self.time_l = self.filter_dist(msg.range, self.dist_l, self.time_l)

    def center_cb(self, msg):
        self.dist_c, self.time_c = self.filter_dist(msg.range, self.dist_c, self.time_c)

    def right_cb(self, msg):
        self.dist_r, self.time_r = self.filter_dist(msg.range, self.dist_r, self.time_r)

    def create_points_for_sensor(self, distance, fov, center_angle, offset_x, offset_y):
        points = []
        # Allow points starting from 2cm (0.02) to 4.0m
        if distance > 4.0 or math.isinf(distance):
            return points # Ignore out of bounds
        
        # We will create an arc of points at the given distance to represent the FOV of the sonic sensor
        num_points = 5 # 5 points per sensor arc
        half_fov = fov / 2.0
        start_angle = center_angle - half_fov
        angle_step = fov / (num_points - 1)

        for i in range(num_points):
            angle = start_angle + (i * angle_step)
            # Transform from sensor frame to base_link roughly.
            # Real TF is better, but since sensors are fixed, we can hardcode the approx transformation
            # to keep it simple and fast in base_link.
            px = offset_x + distance * math.cos(angle)
            py = offset_y + distance * math.sin(angle)
            pz = 0.15 # Height of sensor
            points.append([px, py, pz])
        return points

    def publish_cloud(self):
        points = []
        
        # Sonar Center (Origin: x=0.50, y=0, yaw=0) FOV=0.26 rad
        points.extend(self.create_points_for_sensor(self.dist_c, 0.26, 0.0, 1, 0.0))
        
        # Sonar Left (Origin: x=0.45, y=0.24, yaw=0.785)
        points.extend(self.create_points_for_sensor(self.dist_l, 0.26, 0.785, 1, 0.24))
        
        # Sonar Right (Origin: x=0.45, y=-0.24, yaw=-0.785)
        points.extend(self.create_points_for_sensor(self.dist_r, 0.26, -0.785, 1, -0.24))

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_link' # We publish directly in base_link

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        # Even if empty, publish it so Nav2 clears the old points as we move
        cloud_msg = pc2.create_cloud(header, fields, points)
        self.pc_pub.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicToPointCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

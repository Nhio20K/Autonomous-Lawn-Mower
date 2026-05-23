import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge

import numpy as np
import cv2
import json
from ultralytics import YOLO
from message_filters import ApproximateTimeSynchronizer, Subscriber


class YoloDepthNode(Node):
    def __init__(self):
        super().__init__('yolo_depth_node')

        # --- Parameters ---
        self.declare_parameter('model', 'yolo11n.pt')
        self.declare_parameter('conf', 0.5)
        self.declare_parameter('warn_distance', 1.5)

        model_path    = self.get_parameter('model').value
        self.conf     = self.get_parameter('conf').value
        self.warn_dist = self.get_parameter('warn_distance').value

        # --- โหลด YOLO ---
        self.model  = YOLO(model_path)
        self.bridge = CvBridge()

        # --- Subscribe ภาพสี + Depth (Sync) ---
        color_sub = Subscriber(self, Image, '/camera/color/image_raw')
        depth_sub = Subscriber(self, Image, '/camera/depth/image_rect_raw')

        self.sync = ApproximateTimeSynchronizer(
            [color_sub, depth_sub],
            queue_size=10,
            slop=0.05          # ยอมให้ Timestamp ต่างกันได้ 50ms
        )
        self.sync.registerCallback(self.callback)

        # --- Publish ผลลัพธ์ ---
        # Topic 1: JSON string ของ detection ทั้งหมด
        self.det_pub = self.create_publisher(String, '/yolo/detections', 10)

        self.get_logger().info(
            f'YoloDepthNode started | model={model_path} '
            f'conf={self.conf} warn_dist={self.warn_dist}m'
        )

    # -----------------------------------------------------------------------
    def callback(self, color_msg: Image, depth_msg: Image):
        # แปลง ROS Image → OpenCV
        color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')  # uint16 mm

        # --- YOLO Detection ---
        results = self.model(color_image, conf=self.conf, verbose=False)

        detections = []

        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                label = self.model.names[int(box.cls[0])]
                conf  = float(box.conf[0])

                # วัดระยะทาง Median ใน Bounding Box
                roi_depth    = depth_image[y1:y2, x1:x2]
                valid_depths = roi_depth[roi_depth > 0]

                if len(valid_depths) == 0:
                    continue

                dist_m = float(np.median(valid_depths)) * 0.001  # mm → m

                # แจ้งเตือนใน Terminal
                if label == 'person' and dist_m < self.warn_dist:
                    self.get_logger().warn(
                        f'⚠️  Person detected at {dist_m:.2f}m'
                    )

                detections.append({
                    'label':    label,
                    'conf':     round(conf, 3),
                    'distance': round(dist_m, 3),   # เมตร
                    'bbox':     [x1, y1, x2, y2],
                    'warning':  dist_m < self.warn_dist
                })

        # Publish JSON
        msg      = String()
        msg.data = json.dumps(detections)
        self.det_pub.publish(msg)


# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = YoloDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

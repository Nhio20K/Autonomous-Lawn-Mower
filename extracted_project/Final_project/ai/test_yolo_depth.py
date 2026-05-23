import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO

# --- 1. Setup กล้อง RealSense ---
pipeline = rs.pipeline()
config = rs.config()

# เปิด Stream ภาพสีและภาพความลึก (640x480 เป็นระยะที่ประมวลผลได้ลื่นบนคอมทั่วไป)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# เริ่มต้นการทำงาน
profile = pipeline.start(config)

# สร้างตัว Align เพื่อเลื่อนภาพ Depth ให้ตรงกับพิกเซลของภาพสี (สำคัญมาก!)
align_to = rs.stream.color
align = rs.align(align_to)

# โหลด Model YOLOv8 (จะโหลดไฟล์ .pt มาให้อัตโนมัติในครั้งแรก)
model = YOLO('yolo11n.pt')

print("--- ระบบเริ่มต้น: กด 'q' เพื่อออกจากโปรแกรม ---")

try:
    while True:
        # รับเฟรมภาพและทำ Align
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            continue

        # แปลงข้อมูลเป็น Numpy Array
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # --- 2. ส่งภาพไปให้ AI ตรวจจับ ---
        # conf=0.5 คือเอาความมั่นใจ 50% ขึ้นไป
        results = model(color_image, conf=0.5, verbose=False)

        for r in results:
            for box in r.boxes:
                # พิกัด Bounding Box
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                label = model.names[int(box.cls[0])]

                # --- 3. ดึงค่าความลึกเฉพาะในพื้นที่ Bounding Box ---
                roi_depth = depth_image[y1:y2, x1:x2]
                
                # กรองจุดที่เป็น 0 (จุดที่กล้องอ่านไม่ได้/เป็นเงา)
                valid_depths = roi_depth[roi_depth > 0]
                
                if len(valid_depths) > 0:
                    # ใช้ค่า Median เพื่อความเสถียร (กัน Noise)
                    # คูณ 0.001 เพื่อเปลี่ยนจากหน่วย mm เป็น เมตร
                    dist = np.median(valid_depths) * 0.001 
                    
                    # --- 4. การแสดงผล (Visual Feedback) ---
                    # ถ้าใกล้กว่า 1.5 เมตร ให้เป็นสีแดง (อันตราย) ถ้าไกลกว่านั้นเป็นสีเขียว
                    line_color = (0, 0, 255) if dist < 1.5 else (0, 255, 0)
                    
                    # วาดกรอบและชื่อวัตถุพร้อมระยะทาง
                    cv2.rectangle(color_image, (x1, y1), (x2, y2), line_color, 2)
                    text = f"{label} {dist:.2f}m"
                    cv2.putText(color_image, text, (x1, y1 - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, line_color, 2)

                    # พิมพ์แจ้งเตือนใน Terminal เฉพาะตอนเจอคนใกล้ๆ
                    if label == "person" and dist < 1.5:
                        print(f"⚠️ Warning: Person detected at {dist:.2f}m")

        # แสดงภาพ
        cv2.imshow('PST LAB: RealSense + YOLO Vision Test', color_image)

        # กด q เพื่อเลิก
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # ปิดระบบ
    pipeline.stop()
    cv2.destroyAllWindows()
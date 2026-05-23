import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String
import serial
import threading
import time

class ArduinoReaderNode(Node):
    def __init__(self):
        super().__init__('arduino_reader')

        # ประกาศพารามิเตอร์สำหรับ Serial Port
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value

        # สร้าง Publisher สำหรับ Ultrasonic ทั้ง 3 ตัว (สำหรับทำ Navigation)
        self.pub_left = self.create_publisher(Range, 'ultrasonic/left', 10)
        self.pub_center = self.create_publisher(Range, 'ultrasonic/center', 10)
        self.pub_right = self.create_publisher(Range, 'ultrasonic/right', 10)

        # สร้าง Publisher สำหรับส่ง String ดิบไปให้ teleop_stm คุยกับ STM32
        self.pub_raw_ultra = self.create_publisher(String, 'ultrasonic_raw', 10)

        # เชื่อมต่อ Serial
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1.0)
            self.get_logger().info(f"Successfully connected to Arduino on {self.port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to {self.port}: {e}")
            raise SystemExit

        # สร้าง Thread แยกสำหรับอ่าน Serial เพื่อไม่ให้บล็อก Node
        self.read_thread = threading.Thread(target=self.read_serial_data)
        self.read_thread.daemon = True
        self.read_thread.start()

    def create_range_msg(self, frame_id, distance_cm):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26  # ประมาณ 15 องศา
        msg.min_range = 0.02      # 2 cm
        msg.max_range = 4.0       # 400 cm
        
        # ถ้า Arduino ส่งค่าติดลบมา (แปลว่าไม่เจอวัตถุ หรือ เกินระยะ)
        if distance_cm <= 0:
            msg.range = float('inf')  # บอก ROS 2 ว่าทางโล่งสุดลูกหูลูกตา
        else:
            msg.range = distance_cm / 100.0  # แปลง cm เป็น meter ปกติ
            
        return msg

    def read_serial_data(self):
        while rclpy.ok():
            if self.serial_conn.in_waiting > 0:
                try:
                    line = self.serial_conn.readline().decode('utf-8').strip()
                    self.process_line(line)
                except Exception as e:
                    self.get_logger().warn(f"Error reading serial: {e}")
            else:
                time.sleep(0.01)

    def process_line(self, line):
        # คาดหวังรูปแบบ: U,<dL>,<dC>,<dR>
        if line.startswith('U,'):
            # โยน string ดิบ U,... นี้ออกไปเป็น Topic เพื่อให้ Node อื่นเอาไปส่งเข้า STM32
            raw_msg = String()
            raw_msg.data = line + '\n' # เติม enter กลับเข้าไปด้วย
            self.pub_raw_ultra.publish(raw_msg)

            parts = line.split(',')
            if len(parts) == 4:
                try:
                    dL = float(parts[1])
                    dC = float(parts[2])
                    dR = float(parts[3])

                    # Publish แบบ Range ด้วย
                    self.pub_left.publish(self.create_range_msg('sonar_left', dL))
                    self.pub_center.publish(self.create_range_msg('sonar_center', dC))
                    self.pub_right.publish(self.create_range_msg('sonar_right', dR))
                    
                except ValueError:
                    self.get_logger().warn(f"Invalid data format from Arduino: {line}")

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'serial_conn') and node.serial_conn.is_open:
            node.serial_conn.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

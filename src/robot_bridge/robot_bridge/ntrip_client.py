#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nmea_msgs.msg import Sentence
import socket
import serial
import serial.tools.list_ports
import threading
import base64
import time

class NtripClient(Node):
    def __init__(self):
        super().__init__('ntrip_client')
        
        from rcl_interfaces.msg import ParameterDescriptor, ParameterType
        str_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        
        self.declare_parameter('ip', '127.0.0.1', str_descriptor)
        self.declare_parameter('port', 2101)
        self.declare_parameter('mountpoint', 'VRS_RTCM32', str_descriptor)
        self.declare_parameter('user', 'user')
        from rcl_interfaces.msg import ParameterDescriptor
        self.declare_parameter('password', 'password', ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('serial_port', '/dev/gps_rtk', str_descriptor)
        self.declare_parameter('baudrate', 460800)
        
        self.ip = str(self.get_parameter('ip').value)
        self.port = self.get_parameter('port').value
        self.mountpoint = str(self.get_parameter('mountpoint').value)
        self.user = str(self.get_parameter('user').value)
        self.password = str(self.get_parameter('password').value)
        self.serial_port = str(self.get_parameter('serial_port').value)
        self.baudrate = self.get_parameter('baudrate').value
        self.tcp_sock = None
        self.ser = None
        self.serial_port = None # แก้เป็น serial_port ป้องกันชื่อซ้ำ
        self.is_connected = False
        self.running = True
        self.has_standalone_fix = False # ตัวแปรเช็คว่าหุ่นมีพิกัดพื้นฐานหรือยัง
        
        # 1. สร้าง Thread สำหรับจัดการการเชื่อมต่อ (Auto-Reconnect)
        self.monitor_thread = threading.Thread(target=self.connection_monitor_loop, daemon=True)
        self.monitor_thread.start()
            
        # 2. เตรียม Publisher เฉพาะสำหรับ NMEA (เอาไว้ส่งต่อให้โหนดคำนวณพิกัด)
        self.nmea_pub = self.create_publisher(Sentence, '/nmea_sentence', 10)

        # 3. เตรียม Subscription สำหรับรับค่า GGA จากตัวเอง (เพื่อส่งกลับไปหาเซิร์ฟเวอร์ NTRIP)
        self.nmea_sub = self.create_subscription(
            Sentence,
            '/nmea_sentence',
            self.nmea_callback,
            10
        )
        
        # 4. เชื่อมต่อ NTRIP Caster
        self.connect_ntrip()
        
        # 5. เริ่ม Thread สำหรับ "อ่าน NMEA จากบอร์ด"
        self.serial_read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.serial_read_thread.start()

        # 6. เริ่ม Thread สำหรับ "อ่าน RTCM จากเซิร์ฟเวอร์และยิงลงบอร์ด"
        self.ntrip_read_thread = threading.Thread(target=self.ntrip_read_loop, daemon=True)
        self.ntrip_read_thread.start()

    def find_gps_port(self):
        """ค้นหาพอร์ตที่มีชิป FTDI (VID:0403, PID:6001)"""
        ports = serial.tools.list_ports.comports()
        for p in ports:
            hwid_str = str(p.hwid).upper()
            if (p.vid == 0x0403 and p.pid == 0x6001) or ('0403' in hwid_str and '6001' in hwid_str):
                return p.device
        return None

    def connection_monitor_loop(self):
        """วนลูปตรวจสอบและพยายามเชื่อมต่อพอร์ต USB อัตโนมัติ"""
        while self.running and rclpy.ok():
            if not self.is_connected or self.ser is None or not self.ser.is_open:
                if self.ser:
                    try:
                        self.ser.close()
                    except:
                        pass
                    self.ser = None
                
                self.is_connected = False
                self.serial_port = self.find_gps_port() # ใช้ serial_port ป้องกันชื่อซ้ำกับ TCP port
                
                if self.serial_port:
                    try:
                        self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.1)
                        self.is_connected = True
                        self.get_logger().info(f"🟢 [GPS] ต่อพอร์ตสำเร็จที่: {self.serial_port}")
                    except serial.SerialException as e:
                        self.get_logger().warn(f"🟡 [GPS] เจอพอร์ต {self.serial_port} แต่เปิดไม่ได้: {e} (รอ 1 วิ)")
                else:
                    self.get_logger().warn(f"🔴 [GPS] สายหลุด! กำลังค้นหาบอร์ด (FTDI)...")
                
                time.sleep(1.0)
            else:
                time.sleep(2.0)

    def serial_read_loop(self):
        """อ่านข้อมูล NMEA ดิบทจากบอร์ด GPS แล้ว Publish ลง Topic"""
        self.get_logger().info("📖 เริ่มการอ่านข้อมูล NMEA จากบอร์ด GPS...")
        buffer = b""
        while self.running and rclpy.ok():
            if self.is_connected and self.ser and self.ser.is_open:
                try:
                    # อ่านข้อมูลดิบ (ใช้ bytes เพื่อความแม่นยำ)
                    raw_data = self.ser.read(1024)
                    if raw_data:
                        buffer += raw_data
                        # แยกบรรทัด NMEA
                        while b"\n" in buffer:
                            line, buffer = buffer.split(b"\n", 1)
                            try:
                                decoded_line = line.decode('ascii', errors='ignore').strip()
                                if decoded_line.startswith('$'):
                                    # --- ตรวจสอบสถานะ FIX จาก GNGGA/GPGGA ---
                                    if 'GGA' in decoded_line:
                                        parts = decoded_line.split(',')
                                        if len(parts) > 6:
                                            fix_type = parts[6]
                                            # ถ้า Fix Type = 0 คือไม่มีสัญญาณ
                                            if fix_type == '0':
                                                if self.has_standalone_fix:
                                                    self.get_logger().warn("📡 สัญญาณ GPS ขาดหาย! หยุดส่ง RTK ชั่วคราวเพื่อลดสัญญาณรบกวน")
                                                self.has_standalone_fix = False
                                            else:
                                                if fix_type == '4':
                                                    status_str = "🟢 RTK Fixed (แม่นยำระดับเซนติเมตร)"
                                                elif fix_type == '5':
                                                    status_str = "🟡 RTK Float (แม่นยำระดับเดซิเมตร รอแพร็บ)"
                                                elif fix_type == '2':
                                                    status_str = "🟠 DGPS (Differential GPS)"
                                                elif fix_type == '1':
                                                    status_str = "🔴 Standalone GPS (แม่นยำ 1-3 เมตร)"
                                                else:
                                                    status_str = f"⚪ Type {fix_type}"
                                                    
                                                if not getattr(self, 'last_fix_status', None) == fix_type:
                                                    if fix_type in ['4', '5']:
                                                        self.get_logger().info(f"✨ สถานะ GPS เปลี่ยนเป็น: {status_str}")
                                                    else:
                                                        self.get_logger().warn(f"✨ สถานะ GPS เปลี่ยนเป็น: {status_str}")
                                                    self.last_fix_status = fix_type
                                                self.has_standalone_fix = True

                                    # Publish ข้อมูลลง Topic
                                    msg = Sentence()
                                    msg.header.stamp = self.get_clock().now().to_msg()
                                    msg.header.frame_id = "gps"
                                    msg.sentence = decoded_line
                                    self.nmea_pub.publish(msg)
                            except:
                                continue
                except Exception as e:
                    self.get_logger().debug(f"⚠️ ข้อผิดพลาดในการอ่าน Serial: {e}")
                    self.is_connected = False
                    time.sleep(1)
            else:
                time.sleep(0.5)

    def connect_ntrip(self):
        while self.running and rclpy.ok():
            try:
                self.get_logger().info(f"🔄 กำลังเชื่อมต่อ NTRIP {self.ip}:{self.port}/{self.mountpoint}...")
                self.tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.tcp_sock.settimeout(5.0)
                self.tcp_sock.connect((self.ip, self.port))
                
                user_pass = f"{self.user}:{self.password}"
                b64_auth = base64.b64encode(user_pass.encode('utf-8')).decode('utf-8')
                
                req = (
                    f"GET /{self.mountpoint} HTTP/1.1\r\n"
                    f"User-Agent: NTRIP ROS2Client/1.0\r\n"
                    f"Accept: */*\r\n"
                    f"Connection: close\r\n"
                    f"Authorization: Basic {b64_auth}\r\n"
                    "\r\n"
                )
                self.tcp_sock.sendall(req.encode('utf-8'))
                
                # อ่าน Response
                head = self.tcp_sock.recv(1024)
                if b'ICY 200 OK' in head or b'HTTP/1.1 200 OK' in head:
                    self.get_logger().info("✅ เชื่อมต่อ NTRIP Caster สำเร็จ! (รอรับข้อมูล RTCM)")
                    return
                else:
                    self.get_logger().error(f"❌ เซิร์ฟเวอร์ปฏิเสธการเชื่อมต่อ: {head.decode('utf-8', 'ignore')}")
                    time.sleep(5)
            except Exception as e:
                self.get_logger().error(f"❌ เชื่อมต่อ NTRIP ล้มเหลว: {e}")
                time.sleep(5)

    def nmea_callback(self, msg):
        """รับ NMEA (GGA) เพื่อส่งกลับไปให้เซิร์ฟเวอร์ VRS"""
        if self.tcp_sock is None:
            return
            
        sentence = msg.sentence
        # เซิร์ฟเวอร์ VRS ต้องการตำแหน่งคร่าวๆ ของเราผ่าน GGA
        if '$GPGGA' in sentence or '$GNGGA' in sentence:
            try:
                # ส่ง GGA กลับหาเซิร์ฟเวอร์แม้ไม่มี Fix (เพื่อแจ้งสถานะ)
                nmea_cmd = sentence.strip() + "\r\n"
                self.tcp_sock.sendall(nmea_cmd.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"⚠️ ส่งพิกัดให้เซิร์ฟเวอร์ไม่ได้: {e}")
                self.tcp_sock.close()
                self.tcp_sock = None

    def ntrip_read_loop(self):
        """อ่านข้อมูล RTCM จากเซิร์ฟเวอร์และเขียนลงบอร์ด GPS"""
        self.get_logger().info("📡 เริ่มทำงานการยิงข้อมูล RTCM เข้าบอร์ด GPS...")
        while self.running and rclpy.ok():
            if self.tcp_sock is None:
                self.connect_ntrip()
                time.sleep(1)
                continue
                
            try:
                data = self.tcp_sock.recv(4096)
                if len(data) == 0:
                    self.get_logger().error("⚠️ สัญญาณ NTRIP ขาดหาย กำลังเชื่อมต่อใหม่...")
                    self.tcp_sock.close()
                    self.tcp_sock = None
                    continue
                    
                # 🚀 จุดตายคือตรงนี้ครับ: 
                # ถ้าหุ่น "ยังไม่มีพิกัดพื้นฐานเลย" เราจะไม่ฉีด RTCM เข้าไปกวนบอร์ดครับ
                # เพื่อป้องกันบอร์ด GPS "ตาพร่า" จากข้อมูลขยะที่ยิงมาตอนที่มันกำลังพยายามหาดาวเทียม
                if self.is_connected and self.ser and self.ser.is_open and self.has_standalone_fix:
                    try:
                        self.ser.write(data)
                    except Exception as e:
                        self.get_logger().debug(f"⚠️ Serial write error in RTCM: {e}")
                        self.is_connected = False
                elif not self.has_standalone_fix:
                    # พักไว้ก่อน รอพิกัดมาค่อยฉีดครับ
                    pass
                    
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"💥 ข้อผิดพลาดในการรับสัญญาณ NTRIP: {e}")
                if self.tcp_sock:
                    self.tcp_sock.close()
                self.tcp_sock = None

    def destroy_node(self):
        self.running = False
        if self.tcp_sock:
            self.tcp_sock.close()
        if self.ser:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = NtripClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

import serial
import serial.tools.list_ports
import time

def find_stm32_port():
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        description = port.description.lower()
        device = port.device.lower()
        if "usb" in description or "serial" in description or "ch340" in description or "ttyusb" in device or "ttyacm" in device:
            return port.device
    return None

def verify_imu_checksum(parts):
    """ ตรวจสอบ Checksum ข้อมูล IMU ที่ส่งมาจาก STM32 """
    try:
        # I, ax, ay, az, gx, gy, gz, chk (รวม 8 ส่วน)
        data_values = [int(p) for p in parts[1:7]]
        received_checksum = int(parts[7])
        calculated_checksum = sum(data_values) & 0xFF
        return calculated_checksum == received_checksum
    except (ValueError, IndexError):
        return False

def send_control_command(ser, direction, speed):
    """ 
    ส่งคำสั่งคุมทิศทาง (C) ตามโปรโตคอลใหม่: C,Dir,Speed,Chk
    direction: 'F', 'B', 'L', 'R', 'S'
    speed: 0-100
    """
    try:
        # Checksum Logic: ASCII(Dir) + Speed
        checksum = ord(direction) + int(speed)
        cmd = f"C,{direction},{speed},{checksum}\n"
        ser.write(cmd.encode('utf-8'))
    except Exception as e:
        print(f" ❌ [Control Error]: {e}")

def send_emergency(ser, state):
    """ ส่งคำสั่งหยุดฉุกเฉิน (E): E,state,chk """
    try:
        # Checksum Logic: State + ASCII('E' = 69)
        checksum = int(state) + 69
        cmd = f"E,{state},{checksum}\n"
        ser.write(cmd.encode('utf-8'))
    except Exception as e:
        print(f" ❌ [Emergency Error]: {e}")

def parse_incoming_data(line):
    """ แยกแยะข้อมูล I (IMU) และ D (Encoder) """
    if not line: return
    
    parts = line.split(',')
    header = parts[0]

    # --- IMU (I) ---
    if header == 'I' and len(parts) == 8:
        if verify_imu_checksum(parts):
            print(f" ✅ [IMU]: Acc({parts[1]},{parts[2]},{parts[3]}) Gyro({parts[4]},{parts[5]},{parts[6]})")
        else:
            print(" ❌ [IMU]: Checksum mismatch!")

    # --- Encoder (D) ---
    elif header == 'D' and len(parts) == 5:
        # D, vL, vR, pL, pR
        print(f" ⚙️ [Encoder]: Spd({parts[1]}, {parts[2]}) Pos({parts[3]}, {parts[4]})")

# --- เริ่มการทำงาน ---
target_port = find_stm32_port()

if target_port:
    try:
        # ปรับ timeout ให้เร็วขึ้นเพื่อให้ Loop ไม่กระตุก
        #ser = serial.Serial(target_port, 115200, timeout=0.05)
        # แก้จาก target_port เป็น 'COM5' (หรือเลขที่มึงเจอ)
        ser = serial.Serial('COM9', 115200, timeout=0.1)
        print(f"Connected to {target_port}")
        time.sleep(2) 

        while True:
            # 1. รับข้อมูลจาก STM32
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                parse_incoming_data(line)
            
            # 2. ส่งคำสั่งควบคุม (ตัวอย่าง: เดินหน้า)
            # มึงสามารถเอาค่าจากจอยสติ๊กหรืออัลกอริทึมตัดหญ้ามาใส่ตรงนี้
            send_control_command(ser, 'F', 40)
            
            # หน่วงเวลาเล็กน้อยเพื่อให้ Buffer ไม่เต็ม
            time.sleep(0.05) 

    except Exception as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        print("\nStopping...")
        # ส่งหยุดรถก่อนปิด
        send_control_command(ser, 'S', 0)
        time.sleep(0.1)
        ser.close()
else:
    print("STM32 Port not found!")
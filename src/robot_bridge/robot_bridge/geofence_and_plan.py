#!/usr/bin/env python3
import sys, os, subprocess, threading, time
import rclpy

sys.path.insert(0, os.path.expanduser('~/ros2_ws/src/robot_bridge'))
from robot_bridge.geofence_enforcer import GeofenceSystem

GEOFENCE_FILE = os.path.expanduser('~/ros2_ws/lawn_geofence.yaml')
MOWER_WIDTH_M = 0.60
OVERLAP_M     = 0.07


def print_header():
    print()
    print("=" * 52)
    print("   Geofence + Planner + Mowing (Auto)")
    print("=" * 52)
    print("  [Enter]  = บันทึกจุดรั้ว")
    print("  [s]      = เซฟรั้ว -> Planner -> Mowing อัตโนมัติ")
    print("  [Ctrl+C] = ยกเลิก")
    print("=" * 52)
    print()


def run_record_mode():
    rclpy.init()
    system = GeofenceSystem(start_mode='record')
    spin_thread = threading.Thread(target=rclpy.spin, args=(system,), daemon=True)
    spin_thread.start()

    print(">> กำลังรอสัญญาณ GPS และ TF map->base_link...")
    time.sleep(2.0)

    saved = False
    try:
        while rclpy.ok():
            try:
                val = input("[Enter=บันทึก | s=เซฟ] >> ").strip().lower()
            except EOFError:
                break

            if val == 's':
                if len(system.recorded_lat_lon) < 3:
                    print(f"!! ต้องมีอย่างน้อย 3 จุด (ตอนนี้มี {len(system.recorded_lat_lon)} จุด)")
                else:
                    if system.save_geofence():
                        saved = True
                        break
            else:
                system.record_point()

    except KeyboardInterrupt:
        print("\n!! ยกเลิกโดยผู้ใช้")
    finally:
        system.destroy_node()
        rclpy.shutdown()

    return saved


def start_planner_background():
    print()
    print(">> [Planner] เริ่มทำงาน background...")
    print(f"   แถวกว้าง: {MOWER_WIDTH_M} ม. | Overlap: {OVERLAP_M} ม.")

    cmd = [
        'ros2', 'run', 'robot_bridge', 'lawn_planner',
        '--ros-args',
        '-p', f'geofence_file:={GEOFENCE_FILE}',
        '-p', f'mower_width:={MOWER_WIDTH_M}',
        '-p', f'overlap:={OVERLAP_M}',
    ]
    return subprocess.Popen(cmd)


def run_mow_zigzag():
    print()
    print("=" * 52)
    print("   [Mow Zigzag] พร้อมแล้ว")
    print("   รอรับเส้นทางจาก Planner... แล้วพิมพ์ [go]")
    print("   กด Ctrl+C เพื่อหยุดภารกิจ")
    print("=" * 52)
    print()
    cmd = ['ros2', 'run', 'robot_bridge', 'mow_zigzag']
    try:
        subprocess.run(cmd)
    except KeyboardInterrupt:
        print("\n>> Mow Zigzag หยุดทำงาน")


def main():
    print_header()

    saved = run_record_mode()
    if not saved:
        print("\n!! ยกเลิก - ไม่มีการบันทึกรั้ว")
        sys.exit(0)

    print("\n>> บันทึกรั้วสำเร็จ! เริ่มลำดับถัดไปใน 2 วินาที...")
    time.sleep(2.0)

    planner_proc = start_planner_background()

    print("\n>> รอ Planner สร้างเส้นทาง (3 วิ)...")
    time.sleep(3.0)

    try:
        run_mow_zigzag()
    finally:
        print("\n>> ปิด Lawn Planner...")
        planner_proc.terminate()
        planner_proc.wait()
        print(">> จบภารกิจทั้งหมด")


if __name__ == '__main__':
    main()

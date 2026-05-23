#!/bin/bash
# ============================================================
#  snapshot_env.sh
#  สคริปต์สรุปสภาพแวดล้อมปัจจุบัน เพื่อใช้ย้ายไป Pi 5
# ============================================================

OUTPUT_FILE="$HOME/ros2_ws/environment_snapshot.txt"
echo "⏳ กำลังสแกนสภาพแวดล้อม..."
echo "" > "$OUTPUT_FILE"

# ─── ส่วนที่ 1: ข้อมูลระบบ ────────────────────────────────
echo "========================================" >> "$OUTPUT_FILE"
echo " ข้อมูลระบบ (System Info)"               >> "$OUTPUT_FILE"
echo "========================================" >> "$OUTPUT_FILE"
echo "OS:"          >> "$OUTPUT_FILE"
lsb_release -a 2>/dev/null >> "$OUTPUT_FILE"
echo ""              >> "$OUTPUT_FILE"
echo "Architecture:" >> "$OUTPUT_FILE"
uname -m             >> "$OUTPUT_FILE"
echo ""              >> "$OUTPUT_FILE"
echo "Kernel:"       >> "$OUTPUT_FILE"
uname -r             >> "$OUTPUT_FILE"
echo ""              >> "$OUTPUT_FILE"

# ─── ส่วนที่ 2: ROS2 ──────────────────────────────────────
echo "========================================" >> "$OUTPUT_FILE"
echo " ROS2 Environment"                        >> "$OUTPUT_FILE"
echo "========================================" >> "$OUTPUT_FILE"
echo "ROS_DISTRO: $ROS_DISTRO"                  >> "$OUTPUT_FILE"
echo "ROS_VERSION: $ROS_VERSION"                >> "$OUTPUT_FILE"
echo ""                                         >> "$OUTPUT_FILE"
echo "--- ROS2 Packages (apt) ---"              >> "$OUTPUT_FILE"
dpkg -l | grep -i ros | awk '{print $2, $3}'   >> "$OUTPUT_FILE"
echo ""                                         >> "$OUTPUT_FILE"

# ─── ส่วนที่ 3: Python Libraries ──────────────────────────
echo "========================================" >> "$OUTPUT_FILE"
echo " Python Libraries (pip)"                  >> "$OUTPUT_FILE"
echo "========================================" >> "$OUTPUT_FILE"
python3 -m pip list 2>/dev/null                >> "$OUTPUT_FILE"
echo ""                                        >> "$OUTPUT_FILE"
echo "Python Version:"                         >> "$OUTPUT_FILE"
python3 --version                              >> "$OUTPUT_FILE"
echo ""                                        >> "$OUTPUT_FILE"

# ─── ส่วนที่ 4: Hardware & Port ───────────────────────────
echo "========================================" >> "$OUTPUT_FILE"
echo " USB Devices ที่เชื่อมต่ออยู่"           >> "$OUTPUT_FILE"
echo "========================================" >> "$OUTPUT_FILE"
lsusb 2>/dev/null                              >> "$OUTPUT_FILE"
echo ""                                        >> "$OUTPUT_FILE"
echo "--- Serial Ports ---"                    >> "$OUTPUT_FILE"
ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null  >> "$OUTPUT_FILE"
echo ""                                        >> "$OUTPUT_FILE"

# ─── ส่วนที่ 5: Udev Rules (ชื่อ Port ที่ตั้งเอง) ─────────
echo "========================================" >> "$OUTPUT_FILE"
echo " Custom Udev Rules (/etc/udev/rules.d/)" >> "$OUTPUT_FILE"
echo "========================================" >> "$OUTPUT_FILE"
ls /etc/udev/rules.d/ 2>/dev/null             >> "$OUTPUT_FILE"
echo ""                                        >> "$OUTPUT_FILE"
# ดัมพ์เนื้อหาของ rules ที่เราน่าจะตั้งเอง (ไม่ใช่ของ system)
for f in /etc/udev/rules.d/*.rules; do
    if [[ "$f" != *"70-snap"* ]] && [[ "$f" != *"73-seat"* ]]; then
        echo "--- $f ---"                      >> "$OUTPUT_FILE"
        cat "$f" 2>/dev/null                   >> "$OUTPUT_FILE"
        echo ""                                >> "$OUTPUT_FILE"
    fi
done

# ─── ส่วนที่ 6: ROS Workspace Packages ────────────────────
echo "========================================" >> "$OUTPUT_FILE"
echo " ROS2 Workspace Packages (src/)"         >> "$OUTPUT_FILE"
echo "========================================" >> "$OUTPUT_FILE"
find "$HOME/ros2_ws/src" -name "package.xml" 2>/dev/null | while read f; do
    echo "Package: $(dirname $f | xargs basename)"  >> "$OUTPUT_FILE"
    grep -E "<depend>|<exec_depend>|<build_depend>" "$f" >> "$OUTPUT_FILE"
    echo ""                                         >> "$OUTPUT_FILE"
done

# ─── ส่วนที่ 7: User Groups (สิทธิ์ Port) ──────────────────
echo "========================================" >> "$OUTPUT_FILE"
echo " User Groups (Permission)"               >> "$OUTPUT_FILE"
echo "========================================" >> "$OUTPUT_FILE"
groups                                         >> "$OUTPUT_FILE"
echo ""                                        >> "$OUTPUT_FILE"

# ─── ส่วนที่ 8: YOLO Model files ─────────────────────────
echo "========================================" >> "$OUTPUT_FILE"
echo " YOLO Model Files (.pt)"                 >> "$OUTPUT_FILE"
echo "========================================" >> "$OUTPUT_FILE"
find "$HOME" -name "*.pt" -maxdepth 5 2>/dev/null >> "$OUTPUT_FILE"
echo ""                                           >> "$OUTPUT_FILE"

# ─── ส่วนที่ 9: Maps & Config files ──────────────────────
echo "========================================" >> "$OUTPUT_FILE"
echo " Maps & Config Files"                    >> "$OUTPUT_FILE"
echo "========================================" >> "$OUTPUT_FILE"
find "$HOME/ros2_ws" -name "*.yaml" -o -name "*.pgm" -o -name "*.json" 2>/dev/null | grep -v "__pycache__" >> "$OUTPUT_FILE"
echo ""                                        >> "$OUTPUT_FILE"

# ─── สรุป ─────────────────────────────────────────────────
echo ""
echo "✅ เสร็จแล้วครับ! ไฟล์บันทึกอยู่ที่:"
echo "   📄 $OUTPUT_FILE"
echo ""
echo "เปิดดูได้ด้วยคำสั่ง:"
echo "   cat $OUTPUT_FILE | less"

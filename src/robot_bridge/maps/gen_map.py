#!/usr/bin/env python3
"""
สร้าง empty map สำหรับ Nav2
ขนาด 60x60 เมตร ครอบคลุมพื้นที่ตัดหญ้า ±30m จาก origin
"""
import struct
import os

# ──────────────────────────────────────────────
WIDTH_M  = 60.0   # เมตร
HEIGHT_M = 60.0   # เมตร
RES      = 0.05   # เมตร/pixel (เดิมคือ 0.05)
ORIGIN_X = -30.0  # จุดเริ่มต้นแผนที่ (ซ้ายล่าง)
ORIGIN_Y = -30.0
# ──────────────────────────────────────────────

W = int(WIDTH_M / RES)   # 1200 pixels
H = int(HEIGHT_M / RES)  # 1200 pixels

out_pgm  = os.path.join(os.path.dirname(__file__), 'empty_map.pgm')
out_yaml = os.path.join(os.path.dirname(__file__), 'empty_map.yaml')

# สร้าง PGM (P5 binary): 254 = free space (สีขาว)
print(f"Generating {W}x{H} px map ({WIDTH_M}x{HEIGHT_M}m)...")
with open(out_pgm, 'wb') as f:
    header = f'P5\n{W} {H}\n255\n'.encode('ascii')
    f.write(header)
    # เขียน row by row เพื่อประหยัด RAM
    row = bytes([254] * W)
    for _ in range(H):
        f.write(row)
print(f"Saved: {out_pgm}")

# อัปเดต YAML
yaml_content = f"""image: empty_map.pgm
mode: trinary
resolution: {RES}
origin: [{ORIGIN_X}, {ORIGIN_Y}, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
with open(out_yaml, 'w') as f:
    f.write(yaml_content)
print(f"Saved: {out_yaml}")
print(f"Map covers X: {ORIGIN_X} to {ORIGIN_X + WIDTH_M}, Y: {ORIGIN_Y} to {ORIGIN_Y + HEIGHT_M}")
print("Done!")

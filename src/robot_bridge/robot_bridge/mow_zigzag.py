#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import NavSatFix
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.qos import qos_profile_sensor_data
import time
import math
import sys
import os
import json
from datetime import datetime

# ──────────────────────────────────────────────────────────────
#  ค่าพารามิเตอร์การตัดแถว (ปรับได้)
# ──────────────────────────────────────────────────────────────
TURN_ANGLE_THRESHOLD = 1.2   # ~70° - มุมที่ถือว่าเป็น "จุดเลี้ยว"
MIN_CHUNK_LENGTH     = 1.0   # เมตร  - ก้อนที่สั้นกว่านี้จะรวมกับก้อนถัดไป
REST_BETWEEN_CHUNKS  = 0.5   # วินาที - พักหลังจบแต่ละก้อนเพื่อ Reset Error (ลดจาก 2.0)
# ── Report settings ───────────────────────────────────────────
MISSION_NAME  = 'ZigzagMission'              # ชื่อ mission (ปรับได้)
REPORT_DIR    = os.path.expanduser('~/mower_reports')  # โฟลเดอร์เก็บ report
LOG_INTERVAL  = 0.5                          # วินาที - บันทึกข้อมูลทุก N วินาที
MOWER_WIDTH   = 0.4                          # เมตร - หน้ากว้างของใบตัดหญ้า
# ──────────────────────────────────────────────────────────────


# ══════════════════════════════════════════════════════════════
#  MissionLogger — เก็บข้อมูลทุกอย่างระหว่าง mission
# ══════════════════════════════════════════════════════════════
class MissionLogger:
    def __init__(self):
        self.start_time      = time.time()
        self._last_log_time  = 0.0

        # GPS datum (จุดอ้างอิงสำหรับแปลง lat/lon → เมตร)
        self.datum_lat = None
        self.datum_lon = None

        # Timeseries (ทุก entry ตรงกับ index เดียวกัน)
        self.timestamps          = []   # วินาทีนับจากเริ่ม mission
        self.brain_path          = []   # [{'x':…,'y':…}]  จาก TF/Odometry
        self.rtk_path            = []   # [{'x':…,'y':…}]  จาก GPS (อาจเป็น None)
        self.cross_track_errors  = []   # เมตร
        self.voltages            = []   # V  (None ถ้ายังไม่ได้รับ)
        self.currents            = []   # A  (None ถ้ายังไม่ได้รับ)
        self.gps_fix_quality     = []   # NavSatFix.status.status (-1/0/1/2)

        # GPS ดิบสำหรับแสดง Start/End ใน report
        self.gps_lat_history = []
        self.gps_lon_history = []

        # Planned path (list of (x, y)) สำหรับคำนวณ cross-track error
        self.planned_poses = []

        # Chunk summary
        self.chunk_results = []   # [{'chunk':i, 'success':bool, 'duration':s, 'points':n}]

        # Emergency events
        self.emergency_events = [] # [{'time':s, 'source':str, 'reason':str}]

    # ── Datum & Coordinate conversion ──────────────────────────
    def set_datum(self, lat: float, lon: float):
        """ใช้ GPS fix แรกเป็นจุดอ้างอิง"""
        if self.datum_lat is None:
            self.datum_lat = lat
            self.datum_lon = lon

    def gps_to_local(self, lat: float, lon: float):
        """แปลง lat/lon → เมตร (local X,Y) เทียบกับ datum"""
        if self.datum_lat is None:
            return None, None
        lat_to_m = 111320.0
        lon_to_m = 111320.0 * math.cos(math.radians(self.datum_lat))
        x = (lon - self.datum_lon) * lon_to_m
        y = (lat - self.datum_lat) * lat_to_m
        return x, y

    # ── Cross-track error ───────────────────────────────────────
    def compute_cross_track_error(self, px: float, py: float) -> float:
        """ระยะห่างตั้งฉากจากจุด (px,py) ไปยัง segment ที่ใกล้ที่สุดบน planned path"""
        if len(self.planned_poses) < 2:
            return 0.0
        min_dist = float('inf')
        for i in range(len(self.planned_poses) - 1):
            ax, ay = self.planned_poses[i]
            bx, by = self.planned_poses[i + 1]
            dx, dy = bx - ax, by - ay
            seg_len_sq = dx * dx + dy * dy
            if seg_len_sq < 1e-12:
                dist = math.hypot(px - ax, py - ay)
            else:
                t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg_len_sq))
                cx, cy = ax + t * dx, ay + t * dy
                dist = math.hypot(px - cx, py - cy)
            if dist < min_dist:
                min_dist = dist
        return min_dist

    # ── Log snapshot ────────────────────────────────────────────
    def log(self, brain_x: float, brain_y: float,
            gps_lat=None, gps_lon=None, gps_status=None,
            voltage=None, current=None):
        """บันทึก 1 snapshot ทุก LOG_INTERVAL วินาที"""
        now = time.time()
        if now - self._last_log_time < LOG_INTERVAL:
            return
        self._last_log_time = now

        self.timestamps.append(round(now - self.start_time, 2))

        # Brain path (TF)
        self.brain_path.append({'x': round(brain_x, 4), 'y': round(brain_y, 4)})

        # RTK path
        if gps_lat is not None and gps_lon is not None:
            lx, ly = self.gps_to_local(gps_lat, gps_lon)
            if lx is not None:
                self.rtk_path.append({'x': round(lx, 4), 'y': round(ly, 4)})
                self.gps_lat_history.append(gps_lat)
                self.gps_lon_history.append(gps_lon)
            else:
                self.rtk_path.append(None)
        else:
            self.rtk_path.append(None)

        # Cross-track error
        self.cross_track_errors.append(
            round(self.compute_cross_track_error(brain_x, brain_y), 6))

        # Battery
        self.voltages.append(round(voltage, 2) if voltage is not None else None)
        self.currents.append(round(current, 2) if current is not None else None)
        self.gps_fix_quality.append(gps_status if gps_status is not None else -1)

    def log_emergency(self, source: str, reason: str):
        """บันทึกเหตุการณ์หยุดฉุกเฉิน"""
        now = time.time()
        self.emergency_events.append({
            'time': round(now - self.start_time, 2),
            'source': source,
            'reason': reason
        })

    # ── KPI computation ─────────────────────────────────────────
    def compute_kpis(self) -> dict:
        # ระยะทางรวม (brain path)
        total_dist = 0.0
        bp = self.brain_path
        for i in range(1, len(bp)):
            total_dist += math.hypot(bp[i]['x'] - bp[i-1]['x'],
                                     bp[i]['y'] - bp[i-1]['y'])

        # RMSE และ Max deviation
        errs = [e for e in self.cross_track_errors if e is not None]
        rmse    = math.sqrt(sum(e**2 for e in errs) / len(errs)) if errs else 0.0
        max_err = max(errs) if errs else 0.0

        # Battery
        cur_clean  = [c for c in self.currents if c is not None]
        volt_clean = [v for v in self.voltages  if v is not None]
        avg_cur    = sum(cur_clean)  / len(cur_clean)  if cur_clean  else 0.0
        avg_volt   = sum(volt_clean) / len(volt_clean) if volt_clean else 0.0
        min_volt   = min(volt_clean) if volt_clean else 0.0

        # GPS fix quality
        good_fix = self.gps_fix_quality.count(1) + self.gps_fix_quality.count(2)
        fix_pct  = round(good_fix / len(self.gps_fix_quality) * 100, 1) \
                   if self.gps_fix_quality else 0.0

        # Duration
        duration = time.time() - self.start_time
        chunks_ok   = sum(1 for c in self.chunk_results if c['success'])
        chunks_fail = len(self.chunk_results) - chunks_ok

        return {
            'total_dist'  : round(total_dist, 2),
            'total_area'  : round(total_dist * MOWER_WIDTH, 2),
            'rmse'        : round(rmse, 4),
            'max_err'     : round(max_err, 4),
            'avg_cur'     : round(avg_cur, 2),
            'avg_volt'    : round(avg_volt, 2),
            'min_volt'    : round(min_volt, 2),
            'fix_pct'     : fix_pct,
            'duration'    : round(duration, 1),
            'chunks_ok'   : chunks_ok,
            'chunks_fail' : chunks_fail,
        }

    # ── Downsample helper ───────────────────────────────────────
    @staticmethod
    def _dsample(lst, max_pts=300):
        step = max(1, len(lst) // max_pts)
        return lst[::step]

    @staticmethod
    def _fill_none(lst, fallback=0.0):
        return [x if x is not None else fallback for x in lst]


# ══════════════════════════════════════════════════════════════
#  HTML Report Generator
# ══════════════════════════════════════════════════════════════
def generate_html_report(logger: MissionLogger, mission_name: str,
                         output_dir: str) -> str:
    os.makedirs(output_dir, exist_ok=True)
    now_str  = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    file_ts  = datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = os.path.join(output_dir, f'{mission_name}_{file_ts}.html')

    kpi = logger.compute_kpis()

    # Downsample ข้อมูลสำหรับ chart
    planned_pts = [{'x': round(x, 4), 'y': round(y, 4)}
                   for x, y in logger._dsample(logger.planned_poses, 300)]
    brain_pts   = logger._dsample(logger.brain_path, 300)
    rtk_pts     = [p for p in logger._dsample(logger.rtk_path, 300) if p is not None]

    ts_ds      = logger._dsample(logger.timestamps, 300)
    err_ds     = logger._dsample(logger.cross_track_errors, 300)
    volt_ds    = logger._fill_none(logger._dsample(logger.voltages, 300))
    curr_ds    = logger._fill_none(logger._dsample(logger.currents, 300))

    # Start / End GPS
    start_lat = f"{logger.gps_lat_history[0]:.10f}"  if logger.gps_lat_history else 'N/A'
    start_lon = f"{logger.gps_lon_history[0]:.10f}"  if logger.gps_lat_history else 'N/A'
    end_lat   = f"{logger.gps_lat_history[-1]:.10f}" if logger.gps_lat_history else 'N/A'
    end_lon   = f"{logger.gps_lon_history[-1]:.10f}" if logger.gps_lat_history else 'N/A'
    sx = f"{logger.brain_path[0]['x']:.4f}"  if logger.brain_path else 'N/A'
    sy = f"{logger.brain_path[0]['y']:.4f}"  if logger.brain_path else 'N/A'
    ex = f"{logger.brain_path[-1]['x']:.4f}" if logger.brain_path else 'N/A'
    ey = f"{logger.brain_path[-1]['y']:.4f}" if logger.brain_path else 'N/A'

    # Chunk rows
    chunk_rows = ''
    for c in logger.chunk_results:
        badge = ('✅ Success' if c['success']
                 else '❌ Failed')
        badge_cls = 'badge-ok' if c['success'] else 'badge-fail'
        dist = c.get("distance", 0)
        area = round(dist * MOWER_WIDTH, 2)
        chunk_rows += (
            f'<tr>'
            f'<td><b>Chunk {c["chunk"]+1}</b></td>'
            f'<td><span class="{badge_cls}">{badge}</span></td>'
            f'<td>{c["duration"]:.1f} s</td>'
            f'<td>{dist:.2f} m</td>'
            f'<td>{area:.2f} m²</td>'
            f'</tr>\n'
        )
    if not chunk_rows:
        chunk_rows = '<tr><td colspan="5" style="text-align:center;color:#999">No data</td></tr>'

    # Emergency rows
    emer_rows = ''
    for e in logger.emergency_events:
        emer_rows += (
            f'<tr>'
            f'<td>{e["time"]} s</td>'
            f'<td><span class="badge-fail">{e["source"]}</span></td>'
            f'<td>{e["reason"]}</td>'
            f'</tr>\n'
        )
    if not emer_rows:
        emer_rows = '<tr><td colspan="3" style="text-align:center;color:#999">No emergency events recorded</td></tr>'

    html = f"""<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>Mower Mission Report - {mission_name}</title>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <style>
        body {{ font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
               margin: 20px; background: #f0f2f5; color: #1c1e21; }}
        .container {{ max-width: 1000px; margin: auto; }}
        .card {{ background: white; padding: 25px; border-radius: 12px;
                box-shadow: 0 4px 12px rgba(0,0,0,0.08); margin-bottom: 25px; }}
        h1 {{ color: #1a73e8; text-align: center; margin-bottom: 4px; }}
        .subtitle {{ text-align:center; color:#5f6368; margin-top:0; }}
        .stats {{ display: flex; gap: 15px; flex-wrap: wrap; }}
        .stat-box {{ flex: 1; min-width: 140px; padding: 18px; border-radius: 10px;
                    border-bottom: 4px solid #4CAF50; background: #f8f9fa; }}
        .stat-label {{ font-size: 13px; color: #5f6368; margin-bottom: 6px; }}
        .stat-val   {{ font-size: 24px; font-weight: bold; color: #202124; }}
        .stat-sub   {{ font-size: 11px; color: #9aa0a6; margin-top: 3px; }}
        table {{ width: 100%; border-collapse: collapse; margin-top: 10px; }}
        th, td {{ padding: 11px 14px; text-align: left; border-bottom: 1px solid #e8eaed; }}
        th {{ background: #f1f3f4; color: #5f6368; font-size: 13px; }}
        td {{ font-size: 14px; }}
        canvas {{ width: 100% !important; }}
        .badge-ok   {{ background:#e8f5e9; color:#2e7d32; padding:3px 10px;
                      border-radius:12px; font-size:13px; }}
        .badge-fail {{ background:#ffebee; color:#c62828; padding:3px 10px;
                      border-radius:12px; font-size:13px; }}
        .legend-note {{ font-size:12px; color:#666; margin-top:10px; }}
        .section-title {{ margin: 0 0 16px 0; color: #202124; cursor: pointer; }}
        summary {{ display: block; outline: none; }}
        summary::-webkit-details-marker {{ display: none; }}
        .dropdown-btn {{ padding: 10px; background: #e8eaed; border-radius: 8px; 
                         margin-bottom: 10px; font-weight: bold; color: #1a73e8; cursor: pointer; }}
        .dropdown-btn:hover {{ background: #d1d3d4; }}
    </style>
</head>
<body>
<div class="container">

    <h1>🚜 Mower Performance Report</h1>
    <p class="subtitle">Mission: <b>{mission_name}</b> &nbsp;|&nbsp; {now_str}</p>

    <!-- ── KPIs ─────────────────────────────────────────────── -->
    <div class="card">
        <h3 class="section-title">📊 Key Performance Indicators</h3>
        <div class="stats">
            <div class="stat-box">
                <div class="stat-label">Total Distance</div>
                <div class="stat-val">{kpi['total_dist']} m</div>
            </div>
            <div class="stat-box" style="border-color:#4CAF50;">
                <div class="stat-label">Total Mowed Area</div>
                <div class="stat-val">{kpi['total_area']} m²</div>
                <div class="stat-sub">based on {MOWER_WIDTH}m width</div>
            </div>
            <div class="stat-box" style="border-color:#2196F3;">
                <div class="stat-label">RMSE Accuracy</div>
                <div class="stat-val">{kpi['rmse']} m</div>
                <div class="stat-sub">cross-track</div>
            </div>
            <div class="stat-box" style="border-color:#f44336;">
                <div class="stat-label">Max Deviation</div>
                <div class="stat-val">{kpi['max_err']} m</div>
            </div>
            <div class="stat-box" style="border-color:#FF9800;">
                <div class="stat-label">Avg. Voltage</div>
                <div class="stat-val">{kpi['avg_volt']} V</div>
                <div class="stat-sub">min {kpi['min_volt']} V</div>
            </div>
            <div class="stat-box" style="border-color:#9c27b0;">
                <div class="stat-label">Avg. Current</div>
                <div class="stat-val">{kpi['avg_cur']} A</div>
            </div>
            <div class="stat-box" style="border-color:#00bcd4;">
                <div class="stat-label">Mission Duration</div>
                <div class="stat-val">{kpi['duration']} s</div>
            </div>
            <div class="stat-box" style="border-color:#4CAF50;">
                <div class="stat-label">GPS Fix Quality</div>
                <div class="stat-val">{kpi['fix_pct']}%</div>
                <div class="stat-sub">RTK/DGPS fix time</div>
            </div>
            <div class="stat-box" style="border-color:#607d8b;">
                <div class="stat-label">Chunks</div>
                <div class="stat-val">✅{kpi['chunks_ok']} / ❌{kpi['chunks_fail']}</div>
            </div>
        </div>
    </div>

    <!-- ── GPS Coordinates ──────────────────────────────────── -->
    <div class="card">
        <h3 class="section-title">📍 Coordinates Summary (RTK-GPS)</h3>
        <table>
            <tr>
                <th>Point</th><th>Latitude</th><th>Longitude</th>
                <th>Local X (m)</th><th>Local Y (m)</th>
            </tr>
            <tr>
                <td><b>Start (A)</b></td>
                <td>{start_lat}</td><td>{start_lon}</td>
                <td>{sx}</td><td>{sy}</td>
            </tr>
            <tr>
                <td><b>End (B)</b></td>
                <td>{end_lat}</td><td>{end_lon}</td>
                <td>{ex}</td><td>{ey}</td>
            </tr>
        </table>
    </div>

    <!-- ── Chunk Summary ─────────────────────────────────────── -->
    <div class="card">
        <details>
            <summary class="dropdown-btn">📂 Click to view Chunk Execution Summary</summary>
            <h3 class="section-title">📋 Chunk Execution Summary</h3>
            <table>
                <tr>
                    <th>Chunk</th><th>Status</th><th>Duration</th>
                    <th>Distance</th><th>Mowed Area</th>
                </tr>
                {chunk_rows}
            </table>
        </details>
    </div>

    <!-- ── Emergency Events ──────────────────────────────────── -->
    <div class="card" style="border-left: 5px solid #f44336;">
        <details>
            <summary class="dropdown-btn">🚨 Click to view Emergency & Safety Events</summary>
            <h3 class="section-title">🚨 Emergency & Safety Events</h3>
            <table>
                <tr>
                    <th>Time</th><th>Source</th><th>Reason / Message</th>
                </tr>
                {emer_rows}
            </table>
        </details>
    </div>

    <!-- ── Trajectory Map ───────────────────────────────────── -->
    <div class="card">
        <h3 class="section-title">🗺️ Trajectory Map (Reference vs. Brain vs. Reality)</h3>
        <canvas id="pathChart"></canvas>
        <p class="legend-note">
            🔴 <b>Planned:</b> เส้นทางที่วางแผน &nbsp;|&nbsp;
            🔵 <b>Brain (TF):</b> สิ่งที่หุ่นยนต์คิดว่าตัวเองเดิน &nbsp;|&nbsp;
            🟢 <b>Reality (RTK):</b> พิกัดโลกจริงจากดาวเทียม
        </p>
    </div>

    <!-- ── Error Profile ─────────────────────────────────────── -->
    <div class="card">
        <h3 class="section-title">📈 Cross-Track Error Profile</h3>
        <canvas id="errorChart"></canvas>
    </div>

    <!-- ── Battery & Power ──────────────────────────────────── -->
    <div class="card">
        <h3 class="section-title">🔋 Battery & Power Usage</h3>
        <canvas id="batteryChart"></canvas>
    </div>

</div><!-- /container -->

<script>
// ── Trajectory Chart ──────────────────────────────────────────
new Chart(document.getElementById('pathChart').getContext('2d'), {{
    type: 'scatter',
    data: {{
        datasets: [
            {{
                label: 'Planned Path',
                data: {json.dumps(planned_pts)},
                borderColor: 'rgba(255,0,0,0.55)',
                borderDash: [6, 4],
                showLine: true,
                pointRadius: 0,
                borderWidth: 2
            }},
            {{
                label: 'Actual Robot Path (Brain/TF)',
                data: {json.dumps(brain_pts)},
                borderColor: '#1a73e8',
                backgroundColor: 'rgba(26,115,232,0.08)',
                showLine: true,
                pointRadius: 0,
                borderWidth: 2
            }},
            {{
                label: 'RTK Ground Truth (Reality)',
                data: {json.dumps(rtk_pts)},
                borderColor: '#2ecc71',
                borderDash: [2, 3],
                showLine: true,
                pointRadius: 0,
                borderWidth: 2
            }}
        ]
    }},
    options: {{
        responsive: true,
        scales: {{
            x: {{ title: {{ display: true, text: 'X East (meters)' }},
                  grid: {{ color: '#e8eaed' }} }},
            y: {{ title: {{ display: true, text: 'Y North (meters)' }},
                  grid: {{ color: '#e8eaed' }} }}
        }},
        plugins: {{ legend: {{ position: 'top' }} }}
    }}
}});

// ── Cross-Track Error Chart ───────────────────────────────────
new Chart(document.getElementById('errorChart').getContext('2d'), {{
    type: 'line',
    data: {{
        labels: {json.dumps(ts_ds)},
        datasets: [{{
            label: 'Cross-track Error (m)',
            data: {json.dumps(err_ds)},
            borderColor: '#f44336',
            backgroundColor: 'rgba(244,67,54,0.10)',
            fill: true,
            tension: 0.15,
            pointRadius: 0
        }}]
    }},
    options: {{
        responsive: true,
        plugins: {{ legend: {{ position: 'top' }} }},
        scales: {{
            x: {{ title: {{ display: true, text: 'Time (seconds)' }} }},
            y: {{ title: {{ display: true, text: 'Error (meters)' }}, min: 0 }}
        }}
    }}
}});

// ── Battery Chart ─────────────────────────────────────────────
new Chart(document.getElementById('batteryChart').getContext('2d'), {{
    type: 'line',
    data: {{
        labels: {json.dumps(ts_ds)},
        datasets: [
            {{
                label: 'Voltage (V)',
                data: {json.dumps(volt_ds)},
                borderColor: '#FF9800',
                yAxisID: 'y',
                pointRadius: 0,
                tension: 0.1
            }},
            {{
                label: 'Current (A)',
                data: {json.dumps(curr_ds)},
                borderColor: '#9c27b0',
                yAxisID: 'y1',
                pointRadius: 0,
                tension: 0.1
            }}
        ]
    }},
    options: {{
        responsive: true,
        plugins: {{ legend: {{ position: 'top' }} }},
        scales: {{
            y:  {{ type: 'linear', display: true, position: 'left',
                   title: {{ display: true, text: 'Voltage (V)' }} }},
            y1: {{ type: 'linear', display: true, position: 'right',
                   title: {{ display: true, text: 'Current (A)' }},
                   grid: {{ drawOnChartArea: false }} }}
        }}
    }}
}});
</script>
</body>
</html>"""

    with open(filename, 'w', encoding='utf-8') as f:
        f.write(html)
    return filename


# ══════════════════════════════════════════════════════════════
#  MowZigzag Node
# ══════════════════════════════════════════════════════════════
class MowZigzag(Node):
    def __init__(self):
        super().__init__('mow_zigzag_executor', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])

        # ── Subscriptions ──────────────────────────────────────
        self.create_subscription(Path, '/mowing_path', self.path_callback, 10)
        self.create_subscription(NavSatFix, '/fix', self.gps_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(DiagnosticStatus, '/battery_status',
                                 self.battery_callback, 10)
        
        # --- Safety Subscriptions ---
        from std_msgs.msg import Int8, String
        self.create_subscription(Int8, '/emergency_stop', self.emergency_stop_callback, 10)
        self.create_subscription(String, '/cmd_emergency', self.vision_emergency_callback, 10)

        # ── TF & Nav2 ──────────────────────────────────────────
        self.tf_buffer    = Buffer()
        self.tf_listener  = TransformListener(self.tf_buffer, self)
        self.action_client = ActionClient(self, FollowPath, 'follow_path')

        # ── State ──────────────────────────────────────────────
        self.current_path     = None
        self.is_mission_active = False   # 🔒 ล็อคระหว่าง mission
        self._last_path_len   = None

        # ── Sensor state (อัปเดตจาก callback) ─────────────────
        self._gps_lat    : float | None = None
        self._gps_lon    : float | None = None
        self._gps_status : int   | None = None   # NavSatFix.status.status
        self._battery_v  : float | None = None
        self._battery_a  : float | None = None

        # --- Safety Internal State ---
        self._last_emergency_state = 0

        # ── Logger ─────────────────────────────────────────────
        self.log = MissionLogger()

        self.get_logger().info("🚜 Mow Zigzag Executor Ready! (with Report)")
        self.get_logger().info("Waiting for path on /mowing_path...")

    # ── Callbacks ──────────────────────────────────────────────

    def path_callback(self, msg):
        if self.is_mission_active:
            return   # 🔇 ปิดหูระหว่าง mission ไม่ให้ spam
        self.current_path = msg
        if self._last_path_len != len(msg.poses):
            self._last_path_len = len(msg.poses)
            self.get_logger().info(f"📥 Received new path with {len(msg.poses)} points.")
            self.get_logger().info("To start mowing, please type 'go' in this terminal.")

    def gps_callback(self, msg: NavSatFix):
        self._gps_lat    = msg.latitude
        self._gps_lon    = msg.longitude
        self._gps_status = msg.status.status
        # ใช้ fix แรกเป็น datum (ถ้ามี fix แล้ว)
        if msg.status.status >= 0:
            self.log.set_datum(msg.latitude, msg.longitude)

    def battery_callback(self, msg: DiagnosticStatus):
        for kv in msg.values:
            if kv.key == 'voltage':
                self._battery_v = float(kv.value)
            if kv.key == 'current':
                self._battery_a = float(kv.value)

    def emergency_stop_callback(self, msg):
        """Callback สำหรับระบบ Geofence หรือปุ่มหยุด"""
        if msg.data == 1 and self._last_emergency_state == 0:
            self.get_logger().error("🛑 [SAFETY] Emergency Stop Triggered!")
            self.log.log_emergency("System", "Emergency Stop (Int8=1)")
        self._last_emergency_state = msg.data

    def vision_emergency_callback(self, msg):
        """Callback สำหรับระบบ Vision หรือ Lidar (บอกสาเหตุเป็น String)"""
        if self.is_mission_active:
            self.get_logger().error(f"🚨 [VISION/LIDAR] {msg.data}")
            self.log.log_emergency("Vision/Safety", msg.data)

    # ── TF helper ──────────────────────────────────────────────

    def get_robot_pose(self) -> PoseStamped | None:
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp    = self.get_clock().now().to_msg()
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            pose.pose.orientation = trans.transform.rotation
            return pose
        except Exception:
            return None

    # ── Log helper ─────────────────────────────────────────────

    def _snapshot(self, pose: PoseStamped | None):
        """บันทึกข้อมูล 1 จุด (ถ้าถึงเวลา)"""
        if pose is None:
            return
        self.log.log(
            brain_x    = pose.pose.position.x,
            brain_y    = pose.pose.position.y,
            gps_lat    = self._gps_lat,
            gps_lon    = self._gps_lon,
            gps_status = self._gps_status,
            voltage    = self._battery_v,
            current    = self._battery_a,
        )

    # ── Nav2 feedback ──────────────────────────────────────────

    def follow_path_callback(self, feedback_msg):
        fb    = feedback_msg.feedback
        state = "[ROTATING]" if fb.speed < 0.02 else "[MOWING] "
        sys.stdout.write(
            f"\r📉 {state} | Dist: {fb.distance_to_goal:6.2f}m"
            f" | Speed: {fb.speed:.2f}m/s"
            f" | V:{self._battery_v or 0:.1f}V"
            f" | A:{self._battery_a or 0:.1f}A   "
        )
        sys.stdout.flush()
        # บันทึก snapshot ทุก feedback
        self._snapshot(self.get_robot_pose())

    # ── Path utilities (เหมือนเดิม) ────────────────────────────

    def split_into_chunks(self, poses):
        """แบ่ง poses ออกเป็นก้อนๆ โดยหั่น "ก่อน" จุดเลี้ยว"""
        if len(poses) < 3:
            return [poses]
        cut_indices = []
        for i in range(1, len(poses) - 1):
            p_prev = poses[i - 1].pose.position
            p_curr = poses[i].pose.position
            p_next = poses[i + 1].pose.position
            v1 = (p_curr.x - p_prev.x, p_curr.y - p_prev.y)
            v2 = (p_next.x - p_curr.x, p_next.y - p_curr.y)
            if math.hypot(*v1) < 1e-6 or math.hypot(*v2) < 1e-6:
                continue
            a1 = math.atan2(v1[1], v1[0])
            a2 = math.atan2(v2[1], v2[0])
            diff = abs(a2 - a1)
            if diff > math.pi:
                diff = 2 * math.pi - diff
            if diff > TURN_ANGLE_THRESHOLD:
                cut_at = max(i - 1, 1)
                if not cut_indices or cut_at > cut_indices[-1] + 5:
                    cut_indices.append(cut_at)
        chunks = []
        start = 0
        for cut in cut_indices:
            chunk = poses[start:cut + 1]
            if len(chunk) >= 2:
                p0 = chunk[0].pose.position
                pe = chunk[-1].pose.position
                if math.hypot(pe.x - p0.x, pe.y - p0.y) >= MIN_CHUNK_LENGTH:
                    chunks.append(chunk)
            start = cut
        final_chunk = poses[start:]
        if len(final_chunk) >= 2:
            chunks.append(final_chunk)
        return chunks if chunks else [poses]

    def build_path_with_connection(self, from_pose, chunk_poses, path_header):
        """สร้าง Path โดย prepend จุดเชื่อมจากตำแหน่งหุ่นปัจจุบัน"""
        now = self.get_clock().now().to_msg()
        path_msg = Path()
        path_msg.header.stamp    = now
        path_msg.header.frame_id = 'map'
        first_goal = chunk_poses[0]
        p1 = (from_pose.pose.position.x, from_pose.pose.position.y)
        p2 = (first_goal.pose.position.x, first_goal.pose.position.y)
        dist      = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        conn_yaw  = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        num_steps = max(1, int(dist / 0.1))
        for step in range(num_steps):
            alpha = step / float(num_steps)
            cp = PoseStamped()
            cp.header = path_header
            cp.header.stamp = now
            cp.pose.position.x    = p1[0] + alpha * (p2[0] - p1[0])
            cp.pose.position.y    = p1[1] + alpha * (p2[1] - p1[1])
            cp.pose.position.z    = 0.0
            cp.pose.orientation.z = math.sin(conn_yaw / 2.0)
            cp.pose.orientation.w = math.cos(conn_yaw / 2.0)
            path_msg.poses.append(cp)
        for pose in chunk_poses:
            pose.header.stamp    = now
            pose.header.frame_id = 'map'
            path_msg.poses.append(pose)
        return path_msg

    def send_chunk(self, path_msg, chunk_index: int, total_chunks: int,
                   chunk_poses) -> bool:
        """ส่ง Path ก้อนเดียวให้ Nav2, รอให้เสร็จ, บันทึก result"""
        goal_msg = FollowPath.Goal()
        goal_msg.path             = path_msg
        goal_msg.controller_id    = 'FollowPath'
        goal_msg.goal_checker_id  = 'general_goal_checker'

        self.action_client.wait_for_server()
        t_start     = time.time()
        send_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.follow_path_callback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error(
                f"❌ [Chunk {chunk_index+1}/{total_chunks}] REJECTED by Nav2!")
            self._record_chunk(chunk_index, False, t_start, chunk_poses)
            return False

        self.get_logger().info(
            f"✅ [Chunk {chunk_index+1}/{total_chunks}] Accepted — Mowing...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        print("")   # ขึ้นบรรทัดใหม่หลัง \r

        success = (result.status == 4)  # SUCCEEDED
        self._record_chunk(chunk_index, success, t_start, chunk_poses)

        if success:
            self.get_logger().info(
                f"🏁 [Chunk {chunk_index+1}/{total_chunks}] Completed! "
                f"({time.time()-t_start:.1f}s) — Resting {REST_BETWEEN_CHUNKS}s...")
        else:
            self.get_logger().error(
                f"❌ [Chunk {chunk_index+1}/{total_chunks}] FAILED "
                f"status={result.status}")
        return success

    def _record_chunk(self, idx: int, success: bool, t_start: float, chunk_poses):
        """บันทึกผล chunk ลง logger"""
        # คำนวณระยะทางของ chunk
        dist = 0.0
        for i in range(1, len(chunk_poses)):
            p0 = chunk_poses[i-1].pose.position
            p1 = chunk_poses[i].pose.position
            dist += math.hypot(p1.x - p0.x, p1.y - p0.y)
        self.log.chunk_results.append({
            'chunk'   : idx,
            'success' : success,
            'duration': round(time.time() - t_start, 1),
            'points'  : len(chunk_poses),
            'distance': round(dist, 2),
        })


# ══════════════════════════════════════════════════════════════
#  main()
# ══════════════════════════════════════════════════════════════
def main():
    rclpy.init()
    node = MowZigzag()

    # รอ path
    while rclpy.ok() and node.current_path is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    path = node.current_path

    # บันทึก planned path สำหรับ cross-track error
    node.log.planned_poses = [
        (p.pose.position.x, p.pose.position.y) for p in path.poses
    ]

    def _save_report(name_suffix=''):
        """สร้างและบันทึก HTML report"""
        m_name = MISSION_NAME + name_suffix
        node.get_logger().info("📝 Generating mission report...")
        report_path = generate_html_report(node.log, m_name, REPORT_DIR)
        node.get_logger().info(f"📄 Report saved → {report_path}")
        print(f"\n{'='*60}")
        print(f"  📄 Report: {report_path}")
        print(f"{'='*60}\n")

    try:
        cmd = input(
            "\n🚜 [READY] พิกัดทางเดินพร้อมแล้ว! "
            "พิมพ์ 'go' แล้วกด Enter เพื่อเริ่มภารกิจ: "
        ).strip().lower()
        # cmd = 'go'  # ✅ uncomment เพื่อวิ่งอัตโนมัติในการทดสอบ

        if cmd == 'go':
            node.is_mission_active = True
            node.log.start_time    = time.time()
            node.get_logger().info("🚀 Starting Mowing Sequence...")

            # 1. หาตำแหน่งเริ่มต้น
            node.get_logger().info("Locating robot in TF map...")
            robot_pose = None
            while rclpy.ok() and robot_pose is None:
                rclpy.spin_once(node, timeout_sec=0.1)
                robot_pose = node.get_robot_pose()
            node.get_logger().info(
                f"Robot found at ({robot_pose.pose.position.x:.2f}, "
                f"{robot_pose.pose.position.y:.2f}).")

            # 2. แบ่ง Path เป็นก้อนๆ
            chunks = node.split_into_chunks(path.poses)
            node.get_logger().info(
                f"📊 Path split into {len(chunks)} chunks. Starting row-by-row mowing...")

            # 3. เริ่มวิ่ง
            node.action_client.wait_for_server()
            node.get_logger().info("FollowPath Action Server is up!")

            current_pose = robot_pose
            for i, chunk_poses in enumerate(chunks):
                path_msg = node.build_path_with_connection(
                    current_pose, chunk_poses, path.header)
                success = node.send_chunk(path_msg, i, len(chunks), chunk_poses)

                if not success:
                    node.get_logger().error(f"⚠️ Aborting mission at chunk {i+1}")
                    break

                time.sleep(REST_BETWEEN_CHUNKS)

                # อัปเดตตำแหน่งหุ่นสำหรับก้อนถัดไป
                new_pose = None
                for _ in range(20):
                    rclpy.spin_once(node, timeout_sec=0.1)
                    new_pose = node.get_robot_pose()
                    if new_pose:
                        break
                current_pose = new_pose if new_pose else current_pose

            node.is_mission_active = False
            node.get_logger().info("🏆 All chunks completed! Mowing Done.")
            _save_report()

    except KeyboardInterrupt:
        node.is_mission_active = False
        node.get_logger().info("⚠️ Mission interrupted by user.")
        _save_report('_partial')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

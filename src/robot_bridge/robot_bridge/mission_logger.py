import os
import json
import math
import time
from datetime import datetime

class MissionLogger:
    def __init__(self, mission_name="Mission"):
        self.mission_name = mission_name
        self.start_time = datetime.now()
        self.data_points = [] # List of dicts: {t, x, y, lat, lon, fix, satellites, error}
        self.is_recording = False
        
        # สำหรับเส้นอ้างอิง (A-B)
        self.ref_pointA = None 
        self.ref_pointB = None
        
        # สำหรับคำนวณสถิติ
        self.total_distance = 0.0
        self.last_pos = None
        
    def set_reference_points(self, p1, p2, p1_gps=(0.0,0.0), p2_gps=(0.0,0.0)):
        """ตั้งค่าจุดเริ่มและจุดจบ (x, y) และพิกัดโลก (lat, lon) สำหรับวาดเส้นอ้างอิง"""
        self.ref_pointA = p1
        self.ref_pointB = p2
        self.ref_pointA_gps = p1_gps
        self.ref_pointB_gps = p2_gps

    def start(self):
        self.is_recording = True
        self.start_time = datetime.now()
        self.data_points = []
        self.total_distance = 0.0
        self.last_pos = None
        print(f"🚀 [Logger] Started recording: {self.mission_name}")

    def stop(self):
        self.is_recording = False
        print(f"🛑 [Logger] Stopped recording. Total points: {len(self.data_points)}")
        return self.save_report()

    def log(self, x, y, lat, lon, fix, satellites, target_x=None, target_y=None, volt=0.0, curr=0.0):
        if not self.is_recording:
            return

        error = 0.0
        if target_x is not None and target_y is not None:
            error = math.sqrt((x - target_x)**2 + (y - target_y)**2)

        point = {
            "t": time.time() - self.start_time.timestamp(),
            "x": x,
            "y": y,
            "lat": lat,
            "lon": lon,
            "fix": fix,
            "satellites": satellites,
            "error": error,
            "volt": volt,
            "curr": curr
        }
        self.data_points.append(point)
        
        # คำนวณระยะทางรวม
        if self.last_pos:
            d = math.sqrt((x - self.last_pos[0])**2 + (y - self.last_pos[1])**2)
            if d < 5.0: # กรองกรณี GPS กระโดด
                self.total_distance += d
        self.last_pos = (x, y)

    def save_report(self):
        if not self.data_points:
            return None

        # คำนวณสถิติสรุป
        avg_error = sum(p["error"] for p in self.data_points) / len(self.data_points)
        rmse = math.sqrt(sum(p["error"]**2 for p in self.data_points) / len(self.data_points))
        avg_sats = sum(p["satellites"] for p in self.data_points) / len(self.data_points)
        
        # กำหนด report_dir ให้สัมพันธ์กับตำแหน่งของ workspace
        current_dir = os.path.dirname(os.path.abspath(__file__))
        workspace_dir = os.path.abspath(os.path.join(current_dir, "../../../"))
        report_dir = os.path.join(workspace_dir, "reports")
        
        if not os.path.exists(report_dir):
            os.makedirs(report_dir)
            
        timestamp = self.start_time.strftime("%Y%m%d_%H%M%S")
        filename = f"{self.mission_name}_{timestamp}.html"
        filepath = os.path.join(report_dir, filename)

        # สร้างข้อมูลสำหรับ Chart.js
        labels = [round(p["t"], 1) for p in self.data_points]
        x_coords = [p['x'] for p in self.data_points]
        y_coords = [p['y'] for p in self.data_points]
        
        # คำนวณพิกัดจาก RTK จริงๆ (แปลง Lat/Lon เป็นเมตร)
        gps_path = []
        if self.data_points and self.data_points[0]['lat'] != 0:
            origin_lat = self.data_points[0]['lat']
            origin_lon = self.data_points[0]['lon']
            for p in self.data_points:
                dy = (p['lat'] - origin_lat) * 111319.5
                dx = (p['lon'] - origin_lon) * 111319.5 * math.cos(math.radians(origin_lat))
                gps_path.append({"x": dx + x_coords[0], "y": dy + y_coords[0]})
        
        labels = [round(p['t'], 1) for p in self.data_points]
        sats = [p["satellites"] for p in self.data_points]
        errors = [p["error"] for p in self.data_points]

        # เตรียมข้อมูลเส้นอ้างอิง
        ref_line_json = "[]"
        if self.ref_pointA and self.ref_pointB:
            ref_line_json = json.dumps([
                {"x": self.ref_pointA[0], "y": self.ref_pointA[1]},
                {"x": self.ref_pointB[0], "y": self.ref_pointB[1]}
            ])

        html_template = f"""
<!DOCTYPE html>
<html>
<head>
    <title>Mower Mission Report - {self.mission_name}</title>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <style>
        body {{ font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; margin: 20px; background: #f0f2f5; color: #1c1e21; }}
        .container {{ max-width: 1000px; margin: auto; }}
        .card {{ background: white; padding: 25px; border-radius: 12px; box-shadow: 0 4px 12px rgba(0,0,0,0.08); margin-bottom: 25px; }}
        h1 {{ color: #1a73e8; text-align: center; }}
        .stats {{ display: flex; gap: 15px; flex-wrap: wrap; }}
        .stat-box {{ flex: 1; min-width: 180px; padding: 20px; border-radius: 10px; border-bottom: 4px solid #4CAF50; background: #f8f9fa; }}
        .stat-label {{ font-size: 14px; color: #5f6368; margin-bottom: 8px; }}
        .stat-val {{ font-size: 26px; font-weight: bold; color: #202124; }}
        .table-card {{ overflow-x: auto; }}
        table {{ width: 100%; border-collapse: collapse; margin-top: 10px; }}
        th, td {{ padding: 12px; text-align: left; border-bottom: 1px solid #ddd; }}
        th {{ background-color: #f1f3f4; color: #5f6368; }}
        canvas {{ width: 100% !important; }}
    </style>
</head>
<body>
    <div class="container">
        <h1>🚜 Mower Performance Report</h1>
        <p style="text-align: center; color: #5f6368;">Mission: {self.mission_name} | Date: {self.start_time.strftime('%Y-%m-%d %H:%M:%S')}</p>

        <div class="card">
            <h3>📊 Key Performance Indicators</h3>
            <div class="stats">
                <div class="stat-box">
                    <div class="stat-label">Total Distance</div>
                    <div class="stat-val">{self.total_distance:.2f} m</div>
                </div>
                <div class="stat-box" style="border-color: #2196F3;">
                    <div class="stat-label">RMSE Accuracy</div>
                    <div class="stat-val">{rmse:.3f} m</div>
                </div>
                <div class="stat-box" style="border-color: #f44336;">
                    <div class="stat-label">Max Deviation</div>
                    <div class="stat-val">{max(errors) if errors else 0:.3f} m</div>
                </div>
                <div class="stat-box" style="border-color: #FF9800;">
                    <div class="stat-label">Avg. Satellites</div>
                    <div class="stat-val">{avg_sats:.1f}</div>
                </div>
                <div class="stat-box" style="border-color: #9c27b0;">
                    <div class="stat-label">Avg. Current</div>
                    <div class="stat-val">{sum(p['curr'] for p in self.data_points)/len(self.data_points) if self.data_points else 0:.2f} A</div>
                </div>
            </div>
        </div>

        <div class="card">
            <h3>📍 Mission Coordinates Comparison</h3>
            <table>
                <tr>
                    <th>Point Role</th>
                    <th>Latitude</th>
                    <th>Longitude</th>
                    <th>Local X (m)</th>
                    <th>Local Y (m)</th>
                </tr>
                <tr style="background: #fff8f8;">
                    <td><b>Target A (Planned)</b></td>
                    <td>{self.ref_pointA_gps[0]:.10f}</td>
                    <td>{self.ref_pointA_gps[1]:.10f}</td>
                    <td>{self.ref_pointA[0]:.4f}</td>
                    <td>{self.ref_pointA[1]:.4f}</td>
                </tr>
                <tr>
                    <td><b>Actual Start</b></td>
                    <td>{self.data_points[0]['lat'] if self.data_points else '-'}</td>
                    <td>{self.data_points[0]['lon'] if self.data_points else '-'}</td>
                    <td>{self.data_points[0]['x'] if self.data_points else '-'}</td>
                    <td>{self.data_points[0]['y'] if self.data_points else '-'}</td>
                </tr>
                <tr style="background: #fff8f8; border-top: 2px solid #eee;">
                    <td><b>Target B (Planned)</b></td>
                    <td>{self.ref_pointB_gps[0]:.10f}</td>
                    <td>{self.ref_pointB_gps[1]:.10f}</td>
                    <td>{self.ref_pointB[0]:.4f}</td>
                    <td>{self.ref_pointB[1]:.4f}</td>
                </tr>
                <tr>
                    <td><b>Actual End</b></td>
                    <td>{self.data_points[-1]['lat'] if self.data_points else '-'}</td>
                    <td>{self.data_points[-1]['lon'] if self.data_points else '-'}</td>
                    <td>{self.data_points[-1]['x'] if self.data_points else '-'}</td>
                    <td>{self.data_points[-1]['y'] if self.data_points else '-'}</td>
                </tr>
            </table>
        </div>

        <div class="card">
            <h3>🔵 1. Navigation View (What the Robot Thinks)</h3>
            <canvas id="brainChart"></canvas>
            <p style="font-size: 12px; color: #666; margin-top: 10px;">
                เปรียบเทียบเส้นสีแดง (Planned) กับเส้นสีน้ำเงิน (Brain) เพื่อดูว่าคอนโทรลเลอร์คุมรถได้ตรงตามที่มันเข้าใจไหม
            </p>
        </div>

        <div class="card">
            <h3>🟢 2. Physical View (What GPS Reality Says)</h3>
            <canvas id="realityChart"></canvas>
            <p style="font-size: 12px; color: #666; margin-top: 10px;">
                เปรียบเทียบเส้นสีแดง (Planned) กับเส้นสีเขียว (RTK GPS) เพื่อดูว่าตำแหน่งจริงบนโลกเบี้ยวจากเส้นไปเท่าไหร่
            </p>
        </div>

        <div class="card">
            <h3>📈 Error Profile</h3>
            <canvas id="errorChart"></canvas>
        </div>

        <div class="card">
            <h3>🔋 Battery & Power Usage</h3>
            <canvas id="batteryChart"></canvas>
        </div>
    </div>

    <script>
        // --- กราฟที่ 1: Brain View ---
        const brainCtx = document.getElementById('brainChart').getContext('2d');
        new Chart(brainCtx, {{
            type: 'scatter',
            data: {{
                datasets: [
                    {{
                        label: 'Planned Path (A-B)',
                        data: {ref_line_json},
                        borderColor: 'red',
                        borderDash: [5, 5],
                        showLine: true,
                        pointRadius: 5
                    }},
                    {{
                        label: 'Actual Robot Path (Brain)',
                        data: {json.dumps([{"x": x, "y": y} for x, y in zip(x_coords, y_coords)])},
                        borderColor: '#1a73e8',
                        showLine: true,
                        pointRadius: 0,
                        borderWidth: 2
                    }}
                ]
            }},
            options: {{
                responsive: true,
                scales: {{
                    x: {{ title: {{ display: true, text: 'X East (meters)' }} }},
                    y: {{ title: {{ display: true, text: 'Y North (meters)' }} }}
                }}
            }}
        }});

        // --- กราฟที่ 2: Reality View ---
        const realityCtx = document.getElementById('realityChart').getContext('2d');
        new Chart(realityCtx, {{
            type: 'scatter',
            data: {{
                datasets: [
                    {{
                        label: 'Planned Path (A-B)',
                        data: {ref_line_json},
                        borderColor: 'red',
                        borderDash: [5, 5],
                        showLine: true,
                        pointRadius: 5
                    }},
                    {{
                        label: 'RTK Ground Truth (Reality)',
                        data: {json.dumps(gps_path)},
                        borderColor: '#2ecc71',
                        showLine: true,
                        pointRadius: 0,
                        borderWidth: 2
                    }}
                ]
            }},
            options: {{
                responsive: true,
                scales: {{
                    x: {{ title: {{ display: true, text: 'X East (meters)' }} }},
                    y: {{ title: {{ display: true, text: 'Y North (meters)' }} }}
                }}
            }}
        }});

        const errorCtx = document.getElementById('errorChart').getContext('2d');
        new Chart(errorCtx, {{
            type: 'line',
            data: {{
                labels: {json.dumps(labels)},
                datasets: [{{
                    label: 'Cross-track Error (m)',
                    data: {json.dumps(errors)},
                    borderColor: '#f44336',
                    backgroundColor: 'rgba(244, 67, 54, 0.1)',
                    fill: true,
                    tension: 0.1,
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

        const batteryCtx = document.getElementById('batteryChart').getContext('2d');
        new Chart(batteryCtx, {{
            type: 'line',
            data: {{
                labels: {json.dumps([round(p["t"], 1) for p in self.data_points])},
                datasets: [
                    {{
                        label: 'Voltage (V)',
                        data: {json.dumps([p["volt"] for p in self.data_points])},
                        borderColor: '#FF9800',
                        yAxisID: 'y',
                        pointRadius: 0
                    }},
                    {{
                        label: 'Current (A)',
                        data: {json.dumps([p["curr"] for p in self.data_points])},
                        borderColor: '#9c27b0',
                        yAxisID: 'y1',
                        pointRadius: 0
                    }}
                ]
            }},
            options: {{
                responsive: true,
                scales: {{
                    y: {{ type: 'linear', display: true, position: 'left', title: {{ display: true, text: 'Voltage (V)' }} }},
                    y1: {{ type: 'linear', display: true, position: 'right', title: {{ display: true, text: 'Current (A)' }}, grid: {{ drawOnChartArea: false }} }}
                }}
            }}
        }});
    </script>
</body>
</html>
"""
        with open(filepath, 'w') as f:
            f.write(html_template)
            
        print(f"✅ [Logger] Report saved to: {filepath}")
        return filepath

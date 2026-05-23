import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from diagnostic_msgs.msg import DiagnosticStatus
from nmea_msgs.msg import Sentence
import tf2_ros
from geometry_msgs.msg import Point, Twist
import math
import sys
import termios
import tty
import threading
import select
from .mission_logger import MissionLogger

# ══════════════════════════════════════════════════════════════
#  Terminal Mode Manager (Safety Net)
# ══════════════════════════════════════════════════════════════
class RawMode:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)

    def __enter__(self):
        tty.setraw(self.fd)
        new_settings = termios.tcgetattr(self.fd)
        new_settings[3] |= termios.ISIG  # ยอมรับ Ctrl+C
        termios.tcsetattr(self.fd, termios.TCSADRAIN, new_settings)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

class StraightLineTestNode(Node):
    def __init__(self):
        super().__init__('straight_line_tester')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        self.sub_gps = self.create_subscription(NavSatFix, '/fix', self.gps_callback, 10)
        self.sub_nmea = self.create_subscription(Sentence, '/nmea_sentence', self.nmea_callback, 10)
        self.sub_battery = self.create_subscription(DiagnosticStatus, '/battery_status', self.battery_callback, 10)
        
        self.logger = MissionLogger("StraightLineTest")
        self.pos_x, self.pos_y = 0.0, 0.0
        self.lat, self.lon = 0.0, 0.0
        self.heading = 0.0
        self.fix_status, self.satellites = 0, 0
        self.battery_volt, self.battery_curr = 0.0, 0.0
        
        self.prev_lat, self.prev_lon = 0.0, 0.0
        self.gps_heading = 0.0
        self.use_gps_heading = True
        self.ui_enabled = True
        self.running = True
        
        self.pointA, self.pointB = None, None
        self.pointA_gps, self.pointB_gps = (0.0, 0.0), (0.0, 0.0)
        self.state = "IDLE"
        self.start_point, self.target_point = None, None
        self.start_point_gps, self.target_point_gps = (0.0, 0.0), (0.0, 0.0)
        
        self.linear_speed, self.look_ahead_dist, self.kp_angular = 0.35, 0.6, 1.8
        
        self.thread = threading.Thread(target=self.keyboard_loop)
        self.thread.daemon = True
        self.thread.start()
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.print_menu()

    def print_menu(self):
        print("\n" + "="*45)
        print("🚜 STRAIGHT LINE TESTER (RESILIENT)")
        print("="*45)
        print(" [1,2] Set Point A/B  [M] Manual Entry")
        print(" [S] START Mission    [H] STOP / HALT")
        print(" [G] Toggle GPS Mode  [Q] ABORT / BACK")
        print(" [Ctrl+C] SAVE & EXIT")
        print("="*45)

    def get_pos_from_tf(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.pos_x, self.pos_y = trans.transform.translation.x, trans.transform.translation.y
            q = trans.transform.rotation
            self.heading = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            return True
        except: return False

    def gps_callback(self, msg):
        self.lat, self.lon, self.fix_status = msg.latitude, msg.longitude, msg.status.status
        if self.prev_lat != 0:
            dy, dx = (self.lat - self.prev_lat) * 111319.5, (self.lon - self.prev_lon) * 111319.5 * math.cos(math.radians(self.lat))
            if math.sqrt(dx**2 + dy**2) > 0.1:
                self.gps_heading, self.last_gps_dist = math.atan2(dy, dx), math.sqrt(dx**2 + dy**2)
                self.prev_lat, self.prev_lon = self.lat, self.lon
        else: self.prev_lat, self.prev_lon = self.lat, self.lon

    def nmea_callback(self, msg):
        if "$GNGGA" in msg.sentence or "$GPGGA" in msg.sentence:
            parts = msg.sentence.split(',')
            if len(parts) > 7 and parts[7]: self.satellites = int(parts[7])

    def battery_callback(self, msg):
        try:
            parts = msg.message.replace('V','').replace('A','').split()
            if len(parts) >= 2: self.battery_volt, self.battery_curr = float(parts[0]), float(parts[1])
        except: pass

    def get_distance(self, p1, p2): return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

    def control_loop(self):
        self.get_pos_from_tf()
        cmd = Twist()
        if self.state == "ALIGNING":
            target_yaw = math.atan2(self.start_point[1] - self.pos_y, self.start_point[0] - self.pos_x)
            curr_h = self.gps_heading if (self.use_gps_heading and hasattr(self, 'last_gps_dist') and self.last_gps_dist > 0.1) else self.heading
            diff = target_yaw - curr_h
            while diff > math.pi: diff -= 2*math.pi
            while diff < -math.pi: diff += 2*math.pi
            if abs(diff) < 0.1: self.state = "GOTO_START"
            else: cmd.angular.z = 0.4 if diff > 0 else -0.4
        elif self.state == "GOTO_START":
            dist = self.get_distance((self.pos_x, self.pos_y), self.start_point)
            target_yaw = math.atan2(self.start_point[1] - self.pos_y, self.start_point[0] - self.pos_x)
            curr_h = self.gps_heading if (self.use_gps_heading and hasattr(self, 'last_gps_dist') and self.last_gps_dist > 0.1) else self.heading
            diff = target_yaw - curr_h
            while diff > math.pi: diff -= 2*math.pi
            while diff < -math.pi: diff += 2*math.pi
            if dist < 0.2:
                self.state = "MOVING"; self.logger.start()
                self.logger.set_reference_points(self.start_point, self.target_point, self.start_point_gps, self.target_point_gps)
            else: cmd.linear.x, cmd.angular.z = 0.2, diff * 1.0
        elif self.state == "MOVING":
            ldx, ldy = self.target_point[0]-self.start_point[0], self.target_point[1]-self.start_point[1]
            llen = math.sqrt(ldx**2 + ldy**2)
            if llen < 0.1: self.state = "IDLE"; return
            rdx, rdy = self.pos_x-self.start_point[0], self.pos_y-self.start_point[1]
            proj = (rdx*(ldx/llen) + rdy*(ldy/llen))
            if proj >= (llen - 0.1) or self.get_distance((self.pos_x, self.pos_y), self.target_point) < 0.2:
                self.state = "IDLE"; self.logger.stop()
            else:
                look_dist = min(proj + self.look_ahead_dist, llen)
                lx, ly = self.start_point[0] + look_dist*(ldx/llen), self.start_point[1] + look_dist*(ldy/llen)
                curr_h = self.gps_heading if (self.use_gps_heading and hasattr(self, 'last_gps_dist') and self.last_gps_dist > 0.1) else self.heading
                err = math.atan2(ly-self.pos_y, lx-self.pos_x) - curr_h
                while err > math.pi: err -= 2*math.pi
                while err < -math.pi: err += 2*math.pi
                speed = self.linear_speed
                if (llen - proj) < 1.0: speed = max(0.15, self.linear_speed * (llen - proj))
                cmd.linear.x, cmd.angular.z = speed, err * self.kp_angular
                self.logger.log(self.pos_x, self.pos_y, self.lat, self.lon, self.fix_status, self.satellites, 0, 0, volt=self.battery_volt, curr=self.battery_curr)

        # ✨ อัปเดตบรรทัดเดียวแบบเรียบง่าย (เหมือนสคริปต์อื่นๆ)
        if self.ui_enabled:
            sys.stdout.write(f"\r🔋 {self.battery_volt:.1f}V | XY:({self.pos_x:.1f},{self.pos_y:.1f}) | {self.state} | GPS:{self.fix_status}   ")
            sys.stdout.flush()
        self.pub_cmd.publish(cmd)

    def keyboard_loop(self):
        while rclpy.ok() and self.running:
            dr, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not dr: continue
            ch = sys.stdin.read(1)
            if ch == '1': self.pointA, self.pointA_gps = (self.pos_x, self.pos_y), (self.lat, self.lon); print(f"\n[SET] A:({self.pos_x:.2f},{self.pos_y:.2f})")
            elif ch == '2': self.pointB, self.pointB_gps = (self.pos_x, self.pos_y), (self.lat, self.lon); print(f"\n[SET] B:({self.pos_x:.2f},{self.pos_y:.2f})")
            elif ch.lower() == 'm':
                self.ui_enabled = False
                termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self.old_settings_main)
                try:
                    print("\n--- MANUAL ENTRY ---")
                    xa, ya = float(input("A_X: ")), float(input("A_Y: "))
                    xb, yb = float(input("B_X: ")), float(input("B_Y: "))
                    self.pointA, self.pointB = (xa, ya), (xb, yb)
                    print("✅ Points Updated.")
                except: print("❌ Invalid!")
                finally: tty.setraw(sys.stdin.fileno()); self.ui_enabled = True; self.print_menu()
            elif ch.lower() == 's':
                if self.pointA and self.pointB:
                    da, db = self.get_distance((self.pos_x,self.pos_y), self.pointA), self.get_distance((self.pos_x,self.pos_y), self.pointB)
                    if da < db: self.start_point, self.target_point, self.start_point_gps, self.target_point_gps = self.pointA, self.pointB, self.pointA_gps, self.pointB_gps
                    else: self.start_point, self.target_point, self.start_point_gps, self.target_point_gps = self.pointB, self.pointA, self.pointB_gps, self.pointA_gps
                    self.state = "ALIGNING"; print("\n🚀 Started!")
            elif ch.lower() == 'h': self.state = "IDLE"; self.logger.stop(); print("\n🛑 HALTED")
            elif ch.lower() == 'g': self.use_gps_heading = not self.use_gps_heading; print(f"\n>>> Heading: {'GPS' if self.use_gps_heading else 'TF'}")
            elif ch.lower() == 'q':
                if self.state != "IDLE": self.state = "IDLE"; self.logger.stop(); print("\n⚠️ Aborted.")
                else: print("\nℹ️ Already at Main Menu. Use Ctrl+C to Exit.")

def main():
    rclpy.init()
    node = StraightLineTestNode()
    node.old_settings_main = termios.tcgetattr(sys.stdin.fileno())
    with RawMode():
        try: rclpy.spin(node)
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): print("\n🛑 Hard Shutdown (Ctrl+C)")
        finally:
            node.running = False; node.logger.stop()
            try: node.pub_cmd.publish(Twist())
            except: pass
            node.destroy_node()
            if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()

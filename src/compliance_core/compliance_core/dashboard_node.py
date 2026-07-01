"""Admin web dashboard node.

Serves a password-protected admin console from the robot itself using only
the Python standard library (no extra dependencies - runs identically on the
dev PC and the Raspberry Pi). Open  http://<robot-ip>:8080  on any device on
the network.

Features:
  - live status: mode (autonomous / admin-manual), FSM state, room, nav state
  - mode switch: pause autonomy for joystick driving / resume normal patrol
  - return-to-base and emergency STOP buttons
  - on-screen drive pad (backup for the Bluetooth joystick)
  - violation history with statistics - METADATA ONLY, no photos or video
    are ever served (privacy by design; keyframes stay on the robot disk)

Auth: single admin password (config) -> session token held in server memory.
"""

import csv
import json
import os
import secrets
import socket
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy)
from sensor_msgs.msg import CompressedImage, Image, Joy
from std_msgs.msg import Bool, Float32, String
from tf2_ros import Buffer, TransformListener

from compliance_core.dashboard_page import PAGE_HTML


class DashboardNode(Node):

    def __init__(self):
        super().__init__('dashboard_node')

        self.declare_parameter('port', 8080)
        self.declare_parameter('admin_password', 'change-me')
        self.declare_parameter('log_dir', os.path.expanduser('~/compliance_robot_logs'))
        self.declare_parameter('drive_speed', 0.2)
        self.declare_parameter('turn_speed', 0.8)
        self.declare_parameter(
            'evidence_dir', os.path.expanduser('~/yoru_robot/evidence/output'))

        self.lock = threading.Lock()
        self.tokens = set()
        self.state = {
            'fsm': {}, 'paused': False, 'nav': '', 'base': '',
            'battery': None, 'joy_seen': 0.0,
        }
        self.drive_cmd = (0.0, 0.0)
        self.drive_time = 0.0
        self.estop_until = 0.0

        # Camera feeds
        self.bridge = CvBridge()
        self.cctv_jpg = b''
        self.robot_jpg = b''
        self.cctv_seen = 0.0
        self.robot_seen = 0.0

        self.pause_pub = self.create_publisher(Bool, '/compliance/autonomy_paused', 10)
        self.home_pub = self.create_publisher(Bool, '/compliance/return_to_base', 10)
        # cmd_vel_tracker: twist_mux priority 20 (above Nav2, below joystick)
        self.drive_pub = self.create_publisher(Twist, 'cmd_vel_tracker', 10)
        # Relocalisation: consumed by AMCL / slam_toolbox localization mode
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # Saved spots / map save / safety / auto-relocalise (Pi-side nodes)
        self.save_spot_pub = self.create_publisher(
            String, '/compliance/save_spot', 10)
        self.delete_spot_pub = self.create_publisher(
            String, '/compliance/delete_spot', 10)
        self.goto_spot_pub = self.create_publisher(
            String, '/compliance/goto_spot', 10)
        self.save_map_pub = self.create_publisher(
            String, '/compliance/save_map', 10)
        self.safety_resume_pub = self.create_publisher(
            Bool, '/compliance/safety_resume', 10)
        self.relocalise_pub = self.create_publisher(
            Bool, '/compliance/relocalise_request', 10)
        # Heartbeat: the Pi's safety monitor latches an e-stop when this stops
        self.heartbeat_pub = self.create_publisher(
            Bool, '/compliance/heartbeat', 10)
        self.create_timer(0.5, lambda: self.heartbeat_pub.publish(Bool(data=True)))

        self.spots = {}
        self.location_status = {}
        self.safety_status = {}
        self.loc_monitor_status = {}
        latched = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/compliance/locations',
                                 self.locations_callback, latched)
        self.create_subscription(String, '/compliance/location_status',
                                 lambda m: self._json_into('location_status', m), 10)
        self.create_subscription(String, '/compliance/safety_status',
                                 lambda m: self._json_into('safety_status', m), 10)
        self.create_subscription(String, '/compliance/localization_status',
                                 lambda m: self._json_into('loc_monitor_status', m), 10)

        # Live map (slam_toolbox publishes /map latched / transient local)
        self.map_png = b''
        self.map_meta = {}
        map_qos = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_target_xy = None
        self.create_subscription(PoseStamped, '/compliance/navigation_targets',
                                 self.target_callback, 10)

        self.costmap_clients = {}
        try:
            from nav2_msgs.srv import ClearEntireCostmap
            for name in ('/global_costmap/clear_entirely_global_costmap',
                         '/local_costmap/clear_entirely_local_costmap'):
                self.costmap_clients[name] = self.create_client(
                    ClearEntireCostmap, name)
        except ImportError:
            pass

        self.create_subscription(String, '/compliance/fsm_status',
                                 self.fsm_callback, 10)
        self.create_subscription(Bool, '/compliance/autonomy_paused',
                                 self.paused_callback, 10)
        self.create_subscription(String, '/compliance/nav_status',
                                 lambda m: self.json_state('nav', m), 10)
        self.create_subscription(String, '/compliance/return_to_base_status',
                                 lambda m: self.json_state('base', m), 10)
        self.create_subscription(Float32, '/compliance/battery_level',
                                 self.battery_callback, 10)
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        cam_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Image, '/compliance/cctv1/debug_image',
                                 self.cctv_callback, cam_qos)
        self.create_subscription(CompressedImage, '/camera/image_raw/compressed',
                                 self.robot_cam_callback, cam_qos)

        self.create_timer(0.1, self.drive_tick)  # 10 Hz drive/e-stop keepalive

        port = int(self.get_parameter('port').value)
        self.server = ThreadingHTTPServer(('0.0.0.0', port), self.make_handler())
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.get_logger().info(
            f'Admin dashboard at http://localhost:{port}  |  '
            f'from your phone: http://{self.lan_ip()}:{port} '
            '(same Wi-Fi; password in dashboard_node config)')

    @staticmethod
    def lan_ip():
        """Best-effort LAN IP (no packets are actually sent)."""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(('10.255.255.255', 1))
                return s.getsockname()[0]
        except OSError:
            return '<robot-ip>'

    # ----------------------------------------------------------- ROS callbacks

    def fsm_callback(self, msg):
        try:
            with self.lock:
                self.state['fsm'] = json.loads(msg.data)
        except ValueError:
            pass

    def paused_callback(self, msg):
        with self.lock:
            self.state['paused'] = msg.data

    def json_state(self, key, msg):
        try:
            with self.lock:
                self.state[key] = json.loads(msg.data).get('state', '')
        except ValueError:
            pass

    def battery_callback(self, msg):
        with self.lock:
            self.state['battery'] = round(msg.data, 1)

    def joy_callback(self, _msg):
        with self.lock:
            self.state['joy_seen'] = time.monotonic()

    def _img_to_jpg(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w = frame.shape[:2]
            if w > 640:
                frame = cv2.resize(frame, (640, int(h * 640 / w)))
            ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            return buf.tobytes() if ok else b''
        except Exception as e:
            self.get_logger().warn(f'img_to_jpg failed: {e}', throttle_duration_sec=5.0)
            return b''

    def _compressed_to_jpg(self, msg):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is None:
                return b''
            h, w = frame.shape[:2]
            if w > 640:
                frame = cv2.resize(frame, (640, int(h * 640 / w)))
            ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            return buf.tobytes() if ok else b''
        except Exception as e:
            self.get_logger().warn(f'compressed_to_jpg failed: {e}', throttle_duration_sec=5.0)
            return b''

    def cctv_callback(self, msg):
        jpg = self._img_to_jpg(msg)
        with self.lock:
            if jpg:
                self.cctv_jpg = jpg
            self.cctv_seen = time.monotonic()

    def robot_cam_callback(self, msg):
        jpg = self._compressed_to_jpg(msg)
        with self.lock:
            if jpg:
                self.robot_jpg = jpg
            self.robot_seen = time.monotonic()

    def target_callback(self, msg):
        self.latest_target_xy = (round(msg.pose.position.x, 2),
                                 round(msg.pose.position.y, 2))

    def locations_callback(self, msg):
        try:
            with self.lock:
                self.spots = json.loads(msg.data).get('spots', {})
        except ValueError:
            pass

    def _json_into(self, attr, msg):
        try:
            with self.lock:
                setattr(self, attr, json.loads(msg.data))
        except ValueError:
            pass

    def map_callback(self, msg):
        """Renders the occupancy grid to a PNG (free=white, occupied=dark,
        unknown=transparent) and caches the world metadata for the client."""
        w, h = msg.info.width, msg.info.height
        grid = np.array(msg.data, dtype=np.int8).reshape(h, w)
        rgba = np.zeros((h, w, 4), dtype=np.uint8)
        free = (grid >= 0) & (grid < 50)
        occ = grid >= 50
        rgba[free] = (246, 247, 250, 255)
        rgba[occ] = (66, 74, 96, 255)
        rgba = cv2.flip(rgba, 0)  # grid origin is bottom-left; images top-left
        ok, png = cv2.imencode('.png', cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGRA))
        if not ok:
            return
        with self.lock:
            self.map_png = png.tobytes()
            self.map_meta = {
                'width': w, 'height': h,
                'resolution': msg.info.resolution,
                'origin_x': msg.info.origin.position.x,
                'origin_y': msg.info.origin.position.y,
                'stamp': msg.header.stamp.sec,
            }

    def robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            q = tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return {'x': round(tf.transform.translation.x, 3),
                    'y': round(tf.transform.translation.y, 3),
                    'yaw': round(yaw, 3)}
        except Exception:  # noqa: BLE001 - TF not available yet
            return None

    def drive_tick(self):
        """Republishes the web drive command with a 0.5 s deadman timeout."""
        now = time.monotonic()
        if now < self.estop_until:
            self.drive_pub.publish(Twist())  # zeros override Nav2
            return
        lx, az = self.drive_cmd
        if (lx or az) and now - self.drive_time < 0.5:
            t = Twist()
            t.linear.x = lx
            t.angular.z = az
            self.drive_pub.publish(t)
        elif (lx or az):
            self.drive_cmd = (0.0, 0.0)
            self.drive_pub.publish(Twist())

    # -------------------------------------------------------------- API logic

    def api_status(self):
        with self.lock:
            s = dict(self.state)
        return {
            'mode': 'MANUAL' if s['paused'] else 'AUTONOMOUS',
            'fsm_state': s['fsm'].get('state', 'unknown'),
            'room': s['fsm'].get('room', '') or '-',
            'stage': s['fsm'].get('stage_reached', '-'),
            'nav': s['nav'] or '-',
            'return_to_base': s['base'] or '-',
            'battery': s['battery'],
            'joystick': time.monotonic() - s['joy_seen'] < 2.0,
        }

    def api_cam_status(self):
        now = time.monotonic()
        with self.lock:
            cctv_seen = self.cctv_seen
            robot_seen = self.robot_seen
        return {
            'cctv': now - cctv_seen < 3.0,
            'robot': now - robot_seen < 3.0,
        }

    def api_incidents(self):
        path = os.path.join(self.get_parameter('log_dir').value, 'incidents.jsonl')
        incidents = []
        if os.path.isfile(path):
            with open(path, encoding='utf-8') as f:
                for line in f:
                    try:
                        incidents.append(json.loads(line))
                    except ValueError:
                        continue
        incidents = incidents[-500:][::-1]  # newest first

        per_room = {}
        complied = 0
        last24h = 0
        now = time.time()
        for inc in incidents:
            room = inc.get('room') or inc.get('room_id') or 'unknown'
            per_room[room] = per_room.get(room, 0) + 1
            if inc.get('outcome') == 'complied':
                complied += 1
            try:
                from datetime import datetime
                ts = datetime.fromisoformat(inc['timestamp']).timestamp()
                if now - ts < 86400:
                    last24h += 1
            except (KeyError, ValueError):
                pass
        total = len(incidents)
        return {
            'stats': {
                'total': total,
                'complied': complied,
                'compliance_rate': round(100.0 * complied / total, 1) if total else 0.0,
                'last24h': last24h,
                'per_room': per_room,
            },
            'incidents': incidents,
        }

    def api_evidence_data(self):
        """Aggregates report-evidence artifacts (evidence/output/) + the
        incident log into one JSON payload for the dashboard Evidence tab.
        Everything is read live from disk so the tab reflects the latest run."""
        ev = os.path.expanduser(self.get_parameter('evidence_dir').value)
        log_dir = self.get_parameter('log_dir').value

        incidents = []
        path = os.path.join(log_dir, 'incidents.jsonl')
        if os.path.isfile(path):
            with open(path, encoding='utf-8') as f:
                for line in f:
                    try:
                        incidents.append(json.loads(line))
                    except ValueError:
                        continue
        sim_inc = os.path.join(ev, 'sim', 'incidents.json')
        if os.path.isfile(sim_inc):
            try:
                with open(sim_inc, encoding='utf-8') as f:
                    incidents += json.load(f)
            except ValueError:
                pass

        def tally(*keys):
            out = {}
            for inc in incidents:
                val = None
                for k in keys:
                    val = val or inc.get(k)
                val = val or 'unknown'
                out[val] = out.get(val, 0) + 1
            return out

        summary = self._read_json(os.path.join(ev, 'summary.json'), {})
        run = self._read_json(os.path.join(ev, 'sim', 'run.json'), {})

        bins = [0] * 10
        csv_path = os.path.join(ev, 'detections.csv')
        if os.path.isfile(csv_path):
            with open(csv_path, encoding='utf-8') as f:
                for row in csv.DictReader(f):
                    try:
                        c = float(row['confidence'])
                        bins[min(int(c * 10), 9)] += 1
                    except (KeyError, ValueError):
                        continue

        images = {}
        for label in ('annotated', 'sim', 'eval'):
            d = os.path.join(ev, label)
            if os.path.isdir(d):
                images[label] = sorted(
                    n for n in os.listdir(d)
                    if n.lower().endswith(('.jpg', '.jpeg', '.png')))

        return {
            'n_incidents': len(incidents),
            'outcomes': tally('outcome'),
            'rooms': tally('room', 'room_id'),
            'events': tally('event_class'),
            'class_counts': summary.get('class_counts', {}),
            'conf_bins': bins,
            'detection_summary': {
                'images': summary.get('images'),
                'total_detections': summary.get('total_detections'),
                'mean_latency_ms': summary.get('mean_latency_ms'),
                'mean_fps': summary.get('mean_fps'),
                'primary_model': summary.get('primary_model'),
                'smoking_model': summary.get('smoking_model'),
            },
            'fsm_timeline': run.get('fsm_timeline', []),
            'images': images,
        }

    @staticmethod
    def _read_json(path, default):
        if os.path.isfile(path):
            try:
                with open(path, encoding='utf-8') as f:
                    return json.load(f)
            except ValueError:
                pass
        return default

    def api_spots(self):
        with self.lock:
            return {'spots': dict(self.spots),
                    'status': dict(self.location_status),
                    'safety': dict(self.safety_status),
                    'localization': dict(self.loc_monitor_status)}

    def api_spot_save(self, body):
        name = str(body.get('name', '')).strip()
        if not name:
            return {'ok': False, 'error': 'name required'}
        self.save_spot_pub.publish(String(data=name))
        self.get_logger().info(f'DASHBOARD: save spot "{name}"')
        return {'ok': True}

    def api_spot_delete(self, body):
        name = str(body.get('name', '')).strip()
        self.delete_spot_pub.publish(String(data=name))
        return {'ok': True}

    def api_spot_goto(self, body):
        name = str(body.get('name', '')).strip()
        self.goto_spot_pub.publish(String(data=name))
        self.get_logger().warn(f'DASHBOARD: go to spot "{name}"')
        return {'ok': True}

    def api_map_save(self, body):
        name = str(body.get('name', '')).strip()
        self.save_map_pub.publish(String(data=name))
        self.get_logger().warn('DASHBOARD: save map requested')
        return {'ok': True}

    def api_safety_resume(self):
        self.safety_resume_pub.publish(Bool(data=True))
        self.get_logger().warn('DASHBOARD: safety-stop resume requested')
        return {'ok': True}

    def api_relocalise_auto(self):
        self.relocalise_pub.publish(Bool(data=True))
        self.get_logger().warn('DASHBOARD: auto global relocalisation requested')
        return {'ok': True}

    def api_set_mode(self, body):
        paused = bool(body.get('paused'))
        self.pause_pub.publish(Bool(data=paused))
        self.get_logger().warn(
            f'DASHBOARD: autonomy {"PAUSED (admin manual)" if paused else "RESUMED"}')
        return {'ok': True}

    def api_home(self):
        self.home_pub.publish(Bool(data=True))
        self.get_logger().warn('DASHBOARD: return-to-base requested')
        return {'ok': True}

    def api_stop(self):
        self.pause_pub.publish(Bool(data=True))
        self.home_pub.publish(Bool(data=False))  # also cancels a base trip
        self.estop_until = time.monotonic() + 2.0
        self.get_logger().warn('DASHBOARD: EMERGENCY STOP')
        return {'ok': True}

    def api_drive(self, body):
        scale_l = self.get_parameter('drive_speed').value
        scale_a = self.get_parameter('turn_speed').value
        lx = max(-1.0, min(1.0, float(body.get('lx', 0.0)))) * scale_l
        az = max(-1.0, min(1.0, float(body.get('az', 0.0)))) * scale_a
        self.drive_cmd = (lx, az)
        self.drive_time = time.monotonic()
        return {'ok': True}

    def api_map_info(self):
        with self.lock:
            meta = dict(self.map_meta)
        meta['robot'] = self.robot_pose()
        meta['target'] = self.latest_target_xy
        meta['has_map'] = bool(self.map_png)
        return meta

    def api_relocalise(self, body):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = float(body.get('x', 0.0))
        pose.pose.pose.position.y = float(body.get('y', 0.0))
        yaw = float(body.get('yaw', 0.0))
        pose.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.pose.orientation.w = math.cos(yaw / 2.0)
        pose.pose.covariance[0] = 0.25   # x
        pose.pose.covariance[7] = 0.25   # y
        pose.pose.covariance[35] = 0.068  # yaw
        self.initialpose_pub.publish(pose)
        listeners = self.initialpose_pub.get_subscription_count()
        self.get_logger().warn(
            f'DASHBOARD: relocalise to ({pose.pose.pose.position.x:.2f}, '
            f'{pose.pose.pose.position.y:.2f}, yaw {yaw:.2f}) - '
            f'{listeners} localisation node(s) listening')
        note = '' if listeners else \
            'No localisation node is listening (SLAM mapping mode localises itself).'
        return {'ok': True, 'listeners': listeners, 'note': note}

    def api_clear_costmaps(self):
        cleared = 0
        for name, client in self.costmap_clients.items():
            if client.service_is_ready():
                client.call_async(client.srv_type.Request())
                cleared += 1
        self.get_logger().warn(f'DASHBOARD: clear costmaps ({cleared} services)')
        return {'ok': True, 'cleared': cleared}

    # ------------------------------------------------------------ HTTP server

    def make_handler(self):
        node = self

        class Handler(BaseHTTPRequestHandler):

            def log_message(self, *args):  # silence per-request stderr spam
                pass

            def _send_jpg(self, jpg):
                if not jpg:
                    # Return a 1x1 transparent placeholder so the <img> doesn't break
                    placeholder = (
                        b'\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x00\x00\x01'
                        b'\x00\x01\x00\x00\xff\xdb\x00C\x00\x08\x06\x06\x07\x06'
                        b'\x05\x08\x07\x07\x07\t\t\x08\n\x0c\x14\r\x0c\x0b\x0b'
                        b'\x0c\x19\x12\x13\x0f\x14\x1d\x1a\x1f\x1e\x1d\x1a\x1c'
                        b'\x1c $.\' ",#\x1c\x1c(7),01444\x1f\'9=82<.342\x1e\x1c'
                        b'=47\x17\x1f\x1f\x1f\xff\xc0\x00\x0b\x08\x00\x01\x00'
                        b'\x01\x01\x01\x11\x00\xff\xc4\x00\x1f\x00\x00\x01\x05'
                        b'\x01\x01\x01\x01\x01\x01\x00\x00\x00\x00\x00\x00\x00'
                        b'\x00\x01\x02\x03\x04\x05\x06\x07\x08\t\n\x0b\xff\xc4'
                        b'\x00\xb5\x10\x00\x02\x01\x03\x03\x02\x04\x03\x05\x05'
                        b'\x04\x04\x00\x00\x01}\x01\x02\x03\x00\x04\x11\x05\x12'
                        b'!1A\x06\x13Qa\x07"q\x142\x81\x91\xa1\x08#B\xb1\xc1'
                        b'\x15R\xd1\xf0$3br\x82\t\n\x16\x17\x18\x19\x1a%&\'()*'
                        b'456789:CDEFGHIJSTUVWXYZcdefghijstuvwxyz\x83\x84\x85'
                        b'\x86\x87\x88\x89\x8a\x92\x93\x94\x95\x96\x97\x98\x99'
                        b'\x9a\xa2\xa3\xa4\xa5\xa6\xa7\xa8\xa9\xaa\xb2\xb3\xb4'
                        b'\xb5\xb6\xb7\xb8\xb9\xba\xc2\xc3\xc4\xc5\xc6\xc7\xc8'
                        b'\xc9\xca\xd2\xd3\xd4\xd5\xd6\xd7\xd8\xd9\xda\xe1\xe2'
                        b'\xe3\xe4\xe5\xe6\xe7\xe8\xe9\xea\xf1\xf2\xf3\xf4\xf5'
                        b'\xf6\xf7\xf8\xf9\xfa\xff\xda\x00\x08\x01\x01\x00\x00'
                        b'?\x00\xfb\xd4P\x00\x00\x00\x1f\xff\xd9'
                    )
                    jpg = placeholder
                self.send_response(HTTPStatus.OK)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(jpg)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(jpg)

            def _send_evidence_img(self, rel):
                """Serves an image from evidence/output/{annotated,sim} only,
                with a strict name check (no path traversal)."""
                rel = rel.split('?', 1)[0]
                label, _, name = rel.partition('/')
                ev = os.path.expanduser(node.get_parameter('evidence_dir').value)
                bases = {'annotated': os.path.join(ev, 'annotated'),
                         'sim': os.path.join(ev, 'sim'),
                         'eval': os.path.join(ev, 'eval')}
                base = bases.get(label)
                if (not base or not name or '/' in name or '\\' in name
                        or name.startswith('.')):
                    self.send_json({'error': 'not found'}, HTTPStatus.NOT_FOUND)
                    return
                path = os.path.join(base, name)
                if not os.path.isfile(path):
                    self.send_json({'error': 'not found'}, HTTPStatus.NOT_FOUND)
                    return
                with open(path, 'rb') as f:
                    data = f.read()
                ctype = 'image/png' if name.lower().endswith('.png') else 'image/jpeg'
                self.send_response(HTTPStatus.OK)
                self.send_header('Content-Type', ctype)
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(data)

            def send_json(self, payload, code=HTTPStatus.OK):
                data = json.dumps(payload).encode()
                self.send_response(code)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def authed(self):
                token = self.headers.get('X-Auth', '')
                return token in node.tokens

            def read_body(self):
                length = int(self.headers.get('Content-Length', 0))
                try:
                    return json.loads(self.rfile.read(length) or b'{}')
                except ValueError:
                    return {}

            def do_GET(self):
                if self.path == '/' or self.path.startswith('/index'):
                    data = PAGE_HTML.encode()
                    self.send_response(HTTPStatus.OK)
                    self.send_header('Content-Type', 'text/html; charset=utf-8')
                    self.send_header('Content-Length', str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                    return
                if not self.authed():
                    self.send_json({'error': 'unauthorized'}, HTTPStatus.UNAUTHORIZED)
                    return
                if self.path == '/api/status':
                    self.send_json(node.api_status())
                elif self.path == '/api/incidents':
                    self.send_json(node.api_incidents())
                elif self.path == '/api/map_info':
                    self.send_json(node.api_map_info())
                elif self.path.startswith('/api/map.png'):
                    with node.lock:
                        png = node.map_png
                    if not png:
                        self.send_json({'error': 'no map yet'},
                                       HTTPStatus.NOT_FOUND)
                        return
                    self.send_response(HTTPStatus.OK)
                    self.send_header('Content-Type', 'image/png')
                    self.send_header('Content-Length', str(len(png)))
                    self.send_header('Cache-Control', 'no-store')
                    self.end_headers()
                    self.wfile.write(png)
                elif self.path == '/api/spots':
                    self.send_json(node.api_spots())
                elif self.path == '/api/evidence/data':
                    self.send_json(node.api_evidence_data())
                elif self.path.startswith('/api/evidence/img/'):
                    self._send_evidence_img(self.path[len('/api/evidence/img/'):])
                elif self.path == '/api/cam/status':
                    self.send_json(node.api_cam_status())
                elif self.path.startswith('/api/cam/cctv.jpg'):
                    with node.lock:
                        jpg = node.cctv_jpg
                    self._send_jpg(jpg)
                elif self.path.startswith('/api/cam/robot.jpg'):
                    with node.lock:
                        jpg = node.robot_jpg
                    self._send_jpg(jpg)
                else:
                    self.send_json({'error': 'not found'}, HTTPStatus.NOT_FOUND)

            def do_POST(self):
                body = self.read_body()
                if self.path == '/api/login':
                    password = node.get_parameter('admin_password').value
                    if body.get('password') == password:
                        token = secrets.token_hex(16)
                        node.tokens.add(token)
                        self.send_json({'token': token})
                    else:
                        node.get_logger().warn('Dashboard: failed login attempt')
                        self.send_json({'error': 'wrong password'},
                                       HTTPStatus.UNAUTHORIZED)
                    return
                if not self.authed():
                    self.send_json({'error': 'unauthorized'}, HTTPStatus.UNAUTHORIZED)
                    return
                if self.path == '/api/mode':
                    self.send_json(node.api_set_mode(body))
                elif self.path == '/api/home':
                    self.send_json(node.api_home())
                elif self.path == '/api/stop':
                    self.send_json(node.api_stop())
                elif self.path == '/api/drive':
                    self.send_json(node.api_drive(body))
                elif self.path == '/api/relocalise':
                    self.send_json(node.api_relocalise(body))
                elif self.path == '/api/clear_costmaps':
                    self.send_json(node.api_clear_costmaps())
                elif self.path == '/api/spots/save':
                    self.send_json(node.api_spot_save(body))
                elif self.path == '/api/spots/delete':
                    self.send_json(node.api_spot_delete(body))
                elif self.path == '/api/spots/goto':
                    self.send_json(node.api_spot_goto(body))
                elif self.path == '/api/map/save':
                    self.send_json(node.api_map_save(body))
                elif self.path == '/api/safety/resume':
                    self.send_json(node.api_safety_resume())
                elif self.path == '/api/relocalise_auto':
                    self.send_json(node.api_relocalise_auto())
                else:
                    self.send_json({'error': 'not found'}, HTTPStatus.NOT_FOUND)

        return Handler

    def destroy_node(self):
        self.server.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

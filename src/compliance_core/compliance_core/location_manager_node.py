"""Location manager node (runs on the Pi).

Named map-frame spots ("dock", "room_1", "start", ...) + map saving +
startup pose memory:

  - /compliance/save_spot    (String: name)  save the robot's CURRENT pose
  - /compliance/delete_spot  (String: name)
  - /compliance/goto_spot    (String: name)  Nav2 NavigateToPose to the spot
  - /compliance/save_map     (String: name, '' = main_map)  runs
                              nav2 map_saver_cli into maps_dir
  - /compliance/locations    (String JSON, transient-local) {"spots": {name: [x,y,yaw]}}
  - /compliance/location_status (String JSON) result of the last operation

Spots persist in <maps_dir>/locations.json. The robot's pose is also written
to <maps_dir>/last_pose.json every few seconds; on startup in localization
mode (AMCL present) the saved pose is published to /initialpose so the robot
re-localises where it was shut down. Consumers: return_to_base_node uses
spots named dock*; patrol_node patrols spots named room*/patrol*.
"""

import json
import math
import os
import subprocess
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class LocationManagerNode(Node):

    def __init__(self):
        super().__init__('location_manager_node')

        self.declare_parameter('maps_dir', os.path.expanduser('~/yoru_robot/maps'))
        self.declare_parameter('default_map_name', 'main_map')
        self.declare_parameter('restore_pose_on_start', True)
        self.declare_parameter('pose_save_period', 3.0)

        self.maps_dir = os.path.expanduser(self.get_parameter('maps_dir').value)
        os.makedirs(self.maps_dir, exist_ok=True)
        self.locations_file = os.path.join(self.maps_dir, 'locations.json')
        self.last_pose_file = os.path.join(self.maps_dir, 'last_pose.json')
        self.spots = self._read_json(self.locations_file, {})

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        latched = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.locations_pub = self.create_publisher(
            String, '/compliance/locations', latched)
        self.status_pub = self.create_publisher(
            String, '/compliance/location_status', 10)
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        self.create_subscription(String, '/compliance/save_spot',
                                 self.save_spot_cb, 10)
        self.create_subscription(String, '/compliance/delete_spot',
                                 self.delete_spot_cb, 10)
        self.create_subscription(String, '/compliance/goto_spot',
                                 self.goto_spot_cb, 10)
        self.create_subscription(String, '/compliance/save_map',
                                 self.save_map_cb, 10)

        self.publish_locations()
        self.create_timer(self.get_parameter('pose_save_period').value,
                          self.save_last_pose)
        if self.get_parameter('restore_pose_on_start').value:
            threading.Thread(target=self.restore_pose, daemon=True).start()

        self.get_logger().info(
            f'Location manager ready ({len(self.spots)} spots, {self.maps_dir})')

    # -------------------------------------------------------------- persistence

    @staticmethod
    def _read_json(path, default):
        try:
            with open(path, encoding='utf-8') as f:
                return json.load(f)
        except (OSError, ValueError):
            return default

    def _write_json(self, path, data):
        tmp = path + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2)
        os.replace(tmp, path)

    def publish_locations(self):
        msg = String()
        msg.data = json.dumps({'spots': self.spots})
        self.locations_pub.publish(msg)

    def status(self, ok, action, detail=''):
        msg = String()
        msg.data = json.dumps({'ok': ok, 'action': action, 'detail': detail,
                               'stamp': time.time()})
        self.status_pub.publish(msg)
        log = self.get_logger().info if ok else self.get_logger().warn
        log(f'{action}: {"ok" if ok else "FAILED"} {detail}')

    # ------------------------------------------------------------------ pose

    def current_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link',
                                                 rclpy.time.Time())
        except Exception:  # noqa: BLE001 - TF not available (no localization yet)
            return None
        t = tf.transform.translation
        return [round(t.x, 3), round(t.y, 3),
                round(yaw_from_quat(tf.transform.rotation), 3)]

    def save_last_pose(self):
        pose = self.current_pose()
        if pose is not None:
            self._write_json(self.last_pose_file, {'pose': pose,
                                                   'stamp': time.time()})

    def restore_pose(self):
        """If AMCL is up (localization mode) and a last pose exists, publish it
        to /initialpose so the robot resumes where it was shut down."""
        saved = self._read_json(self.last_pose_file, None)
        if not saved or 'pose' not in saved:
            return
        # AMCL-only service: its presence distinguishes localization mode
        client = self.create_client(Empty, '/reinitialize_global_localization')
        for _ in range(30):
            if client.service_is_ready():
                break
            time.sleep(1.0)
        else:
            self.get_logger().info(
                'No AMCL detected (mapping mode?): startup pose not restored')
            return
        time.sleep(2.0)  # let AMCL finish activating
        x, y, yaw = saved['pose']
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068
        self.initialpose_pub.publish(msg)
        self.get_logger().info(
            f'Startup pose restored: ({x:.2f}, {y:.2f}, yaw {yaw:.2f})')

    # ------------------------------------------------------------------ spots

    @staticmethod
    def _clean(name):
        name = ''.join(c if c.isalnum() or c in '-_ ' else '' for c in name)
        return name.strip().replace(' ', '_').lower()[:40]

    def save_spot_cb(self, msg):
        name = self._clean(msg.data)
        if not name:
            self.status(False, 'save_spot', 'empty name')
            return
        pose = self.current_pose()
        if pose is None:
            self.status(False, 'save_spot',
                        'robot pose unknown (is localization running?)')
            return
        self.spots[name] = pose
        self._write_json(self.locations_file, self.spots)
        self.publish_locations()
        self.status(True, 'save_spot', f'{name} @ {pose}')

    def delete_spot_cb(self, msg):
        name = self._clean(msg.data)
        if self.spots.pop(name, None) is None:
            self.status(False, 'delete_spot', f'unknown spot {name}')
            return
        self._write_json(self.locations_file, self.spots)
        self.publish_locations()
        self.status(True, 'delete_spot', name)

    def goto_spot_cb(self, msg):
        name = self._clean(msg.data)
        pose = self.spots.get(name)
        if pose is None:
            self.status(False, 'goto_spot', f'unknown spot {name}')
            return
        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.status(False, 'goto_spot', 'Nav2 unavailable')
            return
        x, y, yaw = pose
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.nav_client.send_goal_async(goal)
        self.status(True, 'goto_spot', f'navigating to {name}')

    # ------------------------------------------------------------------- map

    def save_map_cb(self, msg):
        name = self._clean(msg.data) or self.get_parameter('default_map_name').value
        threading.Thread(target=self._save_map, args=(name,), daemon=True).start()

    def _save_map(self, name):
        out = os.path.join(self.maps_dir, name)
        try:
            res = subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                 '-f', out, '--ros-args', '-p', 'save_map_timeout:=10.0'],
                capture_output=True, text=True, timeout=60)
            ok = res.returncode == 0 and os.path.isfile(out + '.yaml')
            detail = out + '.yaml' if ok else (res.stderr or res.stdout)[-200:]
        except Exception as exc:  # noqa: BLE001
            ok, detail = False, str(exc)
        self.status(ok, 'save_map', detail)


def main(args=None):
    rclpy.init(args=args)
    node = LocationManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

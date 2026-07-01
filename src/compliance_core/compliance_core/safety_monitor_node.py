"""Safety monitor node (runs on the Pi).

Wi-Fi / control-station watchdog: the laptop's dashboard publishes a heartbeat
on /compliance/heartbeat at 2 Hz. If the Pi misses it for heartbeat_timeout
seconds (Wi-Fi drop, laptop crash), the robot performs a LATCHED emergency
stop: zero velocity is published continuously on cmd_vel_tracker (which
outranks Nav2 in twist_mux), and the robot announces the stop once.

The latch is deliberate: the robot STAYS stopped after the connection returns
until the admin presses Resume on the dashboard (/compliance/safety_resume).
Resume is refused while the heartbeat is still missing.

The watchdog only arms after the first heartbeat is seen, so the robot can be
started before the laptop without instantly freezing.
"""

import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String


class SafetyMonitorNode(Node):

    def __init__(self):
        super().__init__('safety_monitor_node')

        self.declare_parameter('heartbeat_timeout', 3.0)
        self.declare_parameter('stop_publish_hz', 15.0)

        self.armed = False           # becomes True on the first heartbeat
        self.stopped = False         # latched safety stop
        self.stop_reason = ''
        self.stopped_since = 0.0
        self.last_heartbeat = 0.0

        self.estop_pub = self.create_publisher(Twist, 'cmd_vel_tracker', 10)
        self.status_pub = self.create_publisher(
            String, '/compliance/safety_status', 10)
        self.pa_pub = self.create_publisher(String, '/compliance/pa_warning', 10)

        self.create_subscription(Bool, '/compliance/heartbeat',
                                 self.heartbeat_cb, 10)
        self.create_subscription(Bool, '/compliance/safety_resume',
                                 self.resume_cb, 10)

        hz = self.get_parameter('stop_publish_hz').value
        self.create_timer(1.0 / max(hz, 1.0), self.tick)
        self.create_timer(1.0, self.publish_status)

        self.get_logger().info(
            'Safety monitor ready (arms on first heartbeat, '
            f'timeout {self.get_parameter("heartbeat_timeout").value}s, '
            'latched stop until admin resume)')

    def heartbeat_cb(self, _msg):
        if not self.armed:
            self.armed = True
            self.get_logger().info('First heartbeat received: watchdog armed')
        self.last_heartbeat = time.monotonic()

    def heartbeat_age(self):
        return time.monotonic() - self.last_heartbeat

    def resume_cb(self, msg):
        if not msg.data or not self.stopped:
            return
        if self.heartbeat_age() > self.get_parameter('heartbeat_timeout').value:
            self.get_logger().warn(
                'Resume refused: control-station heartbeat still missing')
            return
        self.stopped = False
        self.stop_reason = ''
        self.get_logger().warn('Safety stop RELEASED by admin')
        self.publish_status()

    def tick(self):
        timeout = self.get_parameter('heartbeat_timeout').value
        if self.armed and not self.stopped and self.heartbeat_age() > timeout:
            self.stopped = True
            self.stopped_since = time.monotonic()
            self.stop_reason = 'connection_lost'
            self.get_logger().error(
                f'HEARTBEAT LOST ({self.heartbeat_age():.1f}s): '
                'latched safety stop engaged')
            pa = String()
            pa.data = json.dumps({'message':
                                  'Connection to the control station lost. '
                                  'Robot stopped for safety.'})
            self.pa_pub.publish(pa)
            self.publish_status()
        if self.stopped:
            self.estop_pub.publish(Twist())  # zeros outrank Nav2 in twist_mux

    def publish_status(self):
        msg = String()
        msg.data = json.dumps({
            'stopped': self.stopped,
            'reason': self.stop_reason,
            'armed': self.armed,
            'heartbeat_age': round(self.heartbeat_age(), 1)
            if self.armed else None,
            'stopped_for': round(time.monotonic() - self.stopped_since, 1)
            if self.stopped else 0,
        })
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

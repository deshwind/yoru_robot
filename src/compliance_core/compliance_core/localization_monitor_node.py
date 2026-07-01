"""Localization monitor node (runs on the Pi, localization mode only).

Kidnapped-robot recovery: when the robot is picked up and put down somewhere
else, AMCL's pose covariance blows up because the laser stops matching the
map. This node watches /amcl_pose and, when the uncertainty stays above
threshold for sustain_s (or the admin presses the dashboard button ->
/compliance/relocalise_request), it:

  1. calls AMCL /reinitialize_global_localization (particles over the map)
  2. spins slowly in place (cmd_vel_tracker) so the laser sweeps the room
  3. stops when the covariance converges (or gives up after max_spin_s)

Holds off while the safety monitor has the robot stopped. In mapping mode
/amcl_pose never appears, so the node stays idle. Caveat: in symmetric rooms
global relocalisation can converge to the wrong twin spot - the manual
map-tap Relocalise on the dashboard remains the fallback.
"""

import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty


class LocalizationMonitorNode(Node):

    def __init__(self):
        super().__init__('localization_monitor_node')

        self.declare_parameter('enabled', True)
        self.declare_parameter('lost_std_m', 0.8)       # xy std dev => lost
        self.declare_parameter('converged_std_m', 0.3)  # xy std dev => found
        self.declare_parameter('sustain_s', 3.0)        # lost this long => act
        self.declare_parameter('spin_speed', 0.4)       # rad/s during recovery
        self.declare_parameter('max_spin_s', 30.0)
        self.declare_parameter('retry_cooldown_s', 60.0)

        self.state = 'ok'            # ok | lost | relocalising
        self.std_xy = None
        self.lost_since = None
        self.relocalising_since = None
        self.last_attempt = 0.0
        self.safety_stopped = False

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_tracker', 10)
        self.status_pub = self.create_publisher(
            String, '/compliance/localization_status', 10)
        self.global_loc_client = self.create_client(
            Empty, '/reinitialize_global_localization')

        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self.amcl_cb, 10)
        self.create_subscription(Bool, '/compliance/relocalise_request',
                                 self.request_cb, 10)
        self.create_subscription(String, '/compliance/safety_status',
                                 self.safety_cb, 10)
        self.create_timer(0.2, self.tick)
        self.create_timer(1.0, self.publish_status)

        self.get_logger().info(
            'Localization monitor ready (auto global relocalise when AMCL '
            'covariance blows up; manual via /compliance/relocalise_request)')

    # ---------------------------------------------------------------- inputs

    def amcl_cb(self, msg):
        cov = msg.pose.covariance
        self.std_xy = math.sqrt(max(cov[0], cov[7], 0.0))

    def safety_cb(self, msg):
        try:
            self.safety_stopped = bool(json.loads(msg.data).get('stopped'))
        except ValueError:
            pass

    def request_cb(self, msg):
        if msg.data:
            self.get_logger().warn('Manual global relocalisation requested')
            self.start_relocalise(manual=True)

    # ----------------------------------------------------------------- logic

    def start_relocalise(self, manual=False):
        if self.state == 'relocalising' or self.safety_stopped:
            return
        if not self.global_loc_client.service_is_ready():
            self.get_logger().warn(
                'AMCL global-localization service unavailable '
                '(mapping mode?) - cannot relocalise')
            return
        self.global_loc_client.call_async(Empty.Request())
        self.state = 'relocalising'
        self.relocalising_since = time.monotonic()
        self.last_attempt = time.monotonic()
        self.get_logger().warn(
            f'Global relocalisation started ({"manual" if manual else "auto"}): '
            'spinning to converge')

    def tick(self):
        if not self.get_parameter('enabled').value or self.std_xy is None:
            return
        now = time.monotonic()

        if self.state == 'relocalising':
            if self.safety_stopped:  # safety wins: abort the spin
                self.state = 'ok' if self.std_xy < self.get_parameter(
                    'lost_std_m').value else 'lost'
                self.cmd_pub.publish(Twist())
                return
            elapsed = now - self.relocalising_since
            if self.std_xy < self.get_parameter('converged_std_m').value:
                self.cmd_pub.publish(Twist())
                self.state = 'ok'
                self.lost_since = None
                self.get_logger().info(
                    f'Relocalised (xy std {self.std_xy:.2f} m after '
                    f'{elapsed:.0f}s)')
            elif elapsed > self.get_parameter('max_spin_s').value:
                self.cmd_pub.publish(Twist())
                self.state = 'lost'
                self.get_logger().error(
                    'Relocalisation gave up (still uncertain). Use the manual '
                    'map-tap Relocalise on the dashboard.')
            else:
                spin = Twist()
                spin.angular.z = float(self.get_parameter('spin_speed').value)
                self.cmd_pub.publish(spin)
            return

        lost_threshold = self.get_parameter('lost_std_m').value
        if self.std_xy > lost_threshold:
            if self.lost_since is None:
                self.lost_since = now
                self.get_logger().warn(
                    f'Localization degrading (xy std {self.std_xy:.2f} m)')
            self.state = 'lost'
            sustained = now - self.lost_since > self.get_parameter('sustain_s').value
            cooled = now - self.last_attempt > self.get_parameter('retry_cooldown_s').value
            if sustained and cooled and not self.safety_stopped:
                self.start_relocalise()
        else:
            self.lost_since = None
            if self.state == 'lost':
                self.state = 'ok'

    def publish_status(self):
        msg = String()
        msg.data = json.dumps({
            'state': self.state,
            'std_xy': round(self.std_xy, 3) if self.std_xy is not None else None,
        })
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

"""Compliance FSM node (dissertation Sections 3.9 and 4.7).

Five-stage graduated escalation:

  S0 MONITORING      confirmed event must persist before escalation
  S1 PA_WARNING      public-address audio warning
  S2 APPROACH        Nav2 navigates to a social standoff distance
  S3 DIRECT_WARNING  close-range verbal warning
  S4 LOGGING         privacy-preserving incident logging, then back to S0
     SAFE_STOP       emergency stop (obstacle), cooldown, back to S0

Safety overrides: obstacle stop (< obstacle_stop_distance during APPROACH),
target loss (track gone > target_lost_timeout), per-person cooldown,
compliance reset at any stage after compliance_clear_duration.
"""

import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from vision_msgs.msg import Detection2DArray

PA_MESSAGE = 'Smoking and vaping are prohibited in this area. Please stop immediately.'
DIRECT_MESSAGE = ('This is a final warning. Smoking and vaping are not permitted here. '
                  'This incident will be reported.')


class ComplianceFsmNode(Node):

    def __init__(self):
        super().__init__('compliance_fsm_node')

        self.declare_parameter('monitor_confirm_duration', 3.0)
        self.declare_parameter('pa_warning_duration', 15.0)
        self.declare_parameter('approach_timeout', 60.0)
        self.declare_parameter('direct_warning_duration', 15.0)
        self.declare_parameter('logging_duration', 2.0)
        self.declare_parameter('safe_stop_duration', 3.0)
        self.declare_parameter('compliance_clear_duration', 10.0)
        self.declare_parameter('cooldown_duration', 60.0)
        self.declare_parameter('obstacle_stop_distance', 0.35)
        self.declare_parameter('target_lost_timeout', 5.0)
        # One confirmed-events topic per CCTV pipeline
        self.declare_parameter('events_topics', ['/compliance/confirmed_events'])

        self.state = 'MONITORING'
        self.state_since = time.monotonic()
        self.target_track = None
        self.target_class = None
        self.track_first_seen = {}
        self.track_last_seen = {}
        self.track_metadata = {}
        self.cooldowns = {}
        self.min_scan_range = float('inf')
        self.nav_result = None
        self.stage_reached = 'S0'
        self.autonomy_paused = False

        self.status_pub = self.create_publisher(String, '/compliance/fsm_status', 10)
        self.pa_pub = self.create_publisher(String, '/compliance/pa_warning', 10)
        self.direct_pub = self.create_publisher(String, '/compliance/direct_warning', 10)
        self.incident_pub = self.create_publisher(String, '/compliance/incident_log', 10)
        # cmd_vel_tracker has higher twist_mux priority than Nav2's cmd_vel,
        # so publishing zeros here overrides navigation for an emergency stop.
        self.estop_pub = self.create_publisher(Twist, 'cmd_vel_tracker', 10)

        for topic in self.get_parameter('events_topics').value:
            self.create_subscription(Detection2DArray, topic,
                                     self.events_callback, 10)
        self.create_subscription(String, '/compliance/event_metadata',
                                 self.metadata_callback, 10)
        self.create_subscription(String, '/compliance/nav_status',
                                 self.nav_status_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback,
                                 rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(Bool, '/compliance/autonomy_paused',
                                 self.paused_callback, 10)
        self.create_timer(0.1, self.tick)

        self.get_logger().info('Compliance FSM ready (state: MONITORING)')

    # ------------------------------------------------------------------ inputs

    def events_callback(self, msg):
        now = time.monotonic()
        for det in msg.detections:
            track = det.id or 'untracked'
            if track not in self.track_first_seen:
                self.track_first_seen[track] = now
            self.track_last_seen[track] = now

    def metadata_callback(self, msg):
        try:
            meta = json.loads(msg.data)
        except ValueError:
            return
        if meta.get('status') == 'confirmed':
            self.track_metadata[meta.get('track_id')] = meta

    def nav_status_callback(self, msg):
        try:
            self.nav_result = json.loads(msg.data).get('state')
        except ValueError:
            return

    def scan_callback(self, msg):
        valid = [r for r in msg.ranges
                 if msg.range_min < r < msg.range_max]
        self.min_scan_range = min(valid) if valid else float('inf')

    def paused_callback(self, msg):
        if msg.data != self.autonomy_paused:
            self.autonomy_paused = msg.data
            self.get_logger().warn(
                'Autonomy paused by admin' if msg.data else 'Autonomy resumed')

    # ----------------------------------------------------------------- helpers

    def param(self, name):
        return self.get_parameter(name).value

    def elapsed(self):
        return time.monotonic() - self.state_since

    def transition(self, new_state):
        self.get_logger().info(
            f'FSM: {self.state} -> {new_state} '
            f'(track={self.target_track}, after {self.elapsed():.1f}s)')
        self.state = new_state
        self.state_since = time.monotonic()
        self.publish_status()

    def publish_status(self):
        meta = self.track_metadata.get(self.target_track, {})
        msg = String()
        msg.data = json.dumps({
            'state': self.state,
            'track_id': self.target_track,
            'room': meta.get('room', ''),
            'elapsed': round(self.elapsed(), 1),
            'stage_reached': self.stage_reached,
        })
        self.status_pub.publish(msg)

    def target_active(self):
        """Violation still ongoing for the current target track."""
        if self.target_track is None:
            return False
        last = self.track_last_seen.get(self.target_track)
        return last is not None and \
            time.monotonic() - last < self.param('compliance_clear_duration')

    def target_lost(self):
        if self.target_track is None:
            return True
        last = self.track_last_seen.get(self.target_track)
        return last is None or \
            time.monotonic() - last > self.param('target_lost_timeout')

    def log_incident(self, outcome):
        meta = self.track_metadata.get(self.target_track, {})
        incident = {
            'track_id': self.target_track,
            'room': meta.get('room', ''),
            'event_class': self.target_class,
            'stage_reached': self.stage_reached,
            'outcome': outcome,
            'confidence': meta.get('confidence'),
            'criteria': meta.get('criteria'),
        }
        msg = String()
        msg.data = json.dumps(incident)
        self.incident_pub.publish(msg)

    def finish_escalation(self, outcome):
        self.log_incident(outcome)
        if self.target_track is not None:
            self.cooldowns[self.target_track] = time.monotonic()
        self.target_track = None
        self.target_class = None
        self.nav_result = None
        self.stage_reached = 'S0'
        self.transition('MONITORING')

    # -------------------------------------------------------------------- tick

    def tick(self):
        now = time.monotonic()

        # Admin override: abort any active escalation and stay in MONITORING
        if self.autonomy_paused:
            if self.state != 'MONITORING':
                self.get_logger().warn('Escalation aborted: admin paused autonomy')
                self.finish_escalation('admin_override')
            return

        # Compliance reset is checked in every escalation stage
        in_escalation = self.state in ('PA_WARNING', 'APPROACH', 'DIRECT_WARNING')
        if in_escalation and not self.target_active():
            self.get_logger().info(
                f'Compliance detected at {self.state}; resetting to MONITORING')
            self.finish_escalation('complied')
            return

        if self.state == 'MONITORING':
            self.monitoring_tick(now)
        elif self.state == 'PA_WARNING':
            if self.elapsed() >= self.param('pa_warning_duration'):
                self.stage_reached = 'S2'
                self.transition('APPROACH')
        elif self.state == 'APPROACH':
            self.approach_tick()
        elif self.state == 'DIRECT_WARNING':
            if self.elapsed() >= self.param('direct_warning_duration'):
                self.stage_reached = 'S4'
                self.transition('LOGGING')
        elif self.state == 'LOGGING':
            if self.elapsed() >= self.param('logging_duration'):
                self.finish_escalation('logged_no_compliance')
        elif self.state == 'SAFE_STOP':
            self.estop_pub.publish(Twist())  # zero velocity overrides Nav2
            if self.elapsed() >= self.param('safe_stop_duration'):
                self.finish_escalation('safety_stop')

        if int(self.elapsed() * 10) % 10 == 0:  # ~1 Hz heartbeat
            self.publish_status()

    def monitoring_tick(self, now):
        cooldown = self.param('cooldown_duration')
        confirm = self.param('monitor_confirm_duration')
        for track, first in list(self.track_first_seen.items()):
            last = self.track_last_seen.get(track, 0.0)
            if now - last > self.param('compliance_clear_duration'):
                # Stale track: forget it (volatile state, privacy design)
                self.track_first_seen.pop(track, None)
                self.track_last_seen.pop(track, None)
                self.track_metadata.pop(track, None)
                continue
            if track in self.cooldowns and now - self.cooldowns[track] < cooldown:
                continue
            if now - first >= confirm:
                self.target_track = track
                meta = self.track_metadata.get(track, {})
                self.target_class = meta.get('event_class', 'cigarette')
                self.stage_reached = 'S1'
                room = meta.get('room', '')
                message = PA_MESSAGE
                if room:
                    spoken_room = room.replace('_', ' ')
                    message = (f'Attention. Smoking detected in {spoken_room}. '
                               + PA_MESSAGE)
                pa = String()
                pa.data = json.dumps({'message': message, 'track_id': track,
                                      'room': room,
                                      'event_class': self.target_class})
                self.pa_pub.publish(pa)
                self.transition('PA_WARNING')
                return

    def approach_tick(self):
        if self.min_scan_range < self.param('obstacle_stop_distance'):
            self.get_logger().warn(
                f'Obstacle at {self.min_scan_range:.2f} m: SAFE_STOP')
            self.estop_pub.publish(Twist())
            self.transition('SAFE_STOP')
            return
        if self.target_lost():
            self.get_logger().info('Target lost during approach')
            self.finish_escalation('target_lost')
            return
        if self.nav_result == 'succeeded':
            self.nav_result = None
            self.stage_reached = 'S3'
            meta = self.track_metadata.get(self.target_track, {})
            direct = String()
            direct.data = json.dumps({'message': DIRECT_MESSAGE,
                                      'track_id': self.target_track,
                                      'room': meta.get('room', ''),
                                      'event_class': self.target_class})
            self.direct_pub.publish(direct)
            self.transition('DIRECT_WARNING')
            return
        if self.nav_result in ('aborted', 'timeout', 'rejected', 'nav2_unavailable'):
            self.get_logger().warn(f'Approach failed ({self.nav_result}); logging')
            self.nav_result = None
            self.stage_reached = 'S4'
            self.transition('LOGGING')
            return
        if self.elapsed() >= self.param('approach_timeout'):
            self.stage_reached = 'S4'
            self.transition('LOGGING')


def main(args=None):
    rclpy.init(args=args)
    node = ComplianceFsmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

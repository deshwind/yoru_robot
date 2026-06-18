"""Incident emailer node (dissertation Section 4.8, keyframe capture + alerts).

Captures evidence keyframes during an escalation and emails a report when the
incident concludes:

  - CCTV keyframe (annotated detection view of the violating room), captured
    when the escalation starts (PA_WARNING)
  - robot onboard close-up, captured shortly after the robot reaches the
    person (DIRECT_WARNING)

One email per incident, sent for every outcome (complied / logged / lost).
Keyframes are stored under keyframe_dir with retention-based cleanup
(privacy: selective keyframe capture for incident verification only, no
facial recognition, no video storage; transport over TLS).

Gmail: use an app password (Google Account -> Security -> App passwords).
The password is read from the COMPLIANCE_EMAIL_PASSWORD environment variable
if set, otherwise from the 'app_password' parameter.
"""

import json
import os
import smtplib
import threading
import time
from datetime import datetime, timezone
from email.message import EmailMessage

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class IncidentEmailerNode(Node):

    def __init__(self):
        super().__init__('incident_emailer_node')

        self.declare_parameter('enabled', True)
        self.declare_parameter('smtp_host', 'smtp.gmail.com')
        self.declare_parameter('smtp_port', 587)
        self.declare_parameter('sender_email', '')
        self.declare_parameter('recipient_email', '')
        self.declare_parameter('app_password', '')
        self.declare_parameter('robot_camera_topic', '/camera/image_raw')
        # Parallel lists: one annotated CCTV stream per room
        self.declare_parameter('cctv_debug_topics',
                               ['/compliance/cctv1/debug_image'])
        self.declare_parameter('cctv_rooms', ['room_a'])
        self.declare_parameter('keyframe_dir',
                               os.path.expanduser('~/compliance_robot_logs/keyframes'))
        self.declare_parameter('retention_days', 30)
        self.declare_parameter('closeup_delay', 1.5)
        self.declare_parameter('min_email_interval', 30.0)

        self.bridge = CvBridge()
        self.keyframe_dir = self.get_parameter('keyframe_dir').value
        os.makedirs(self.keyframe_dir, exist_ok=True)
        self.cleanup_old_keyframes()

        # Latest frame caches
        self.robot_frame = None
        self.cctv_frames = {}          # room -> latest annotated frame
        self.latest_target = None      # approximate violation location

        # Per-incident evidence
        self.cctv_keyframe_path = None
        self.closeup_path = None
        self.closeup_timer = None
        self.active_room = ''
        self.prev_state = 'MONITORING'
        self.last_email_time = 0.0

        self.create_subscription(
            Image, self.get_parameter('robot_camera_topic').value,
            self.robot_image_callback, 2)
        rooms = self.get_parameter('cctv_rooms').value
        topics = self.get_parameter('cctv_debug_topics').value
        for room, topic in zip(rooms, topics):
            self.create_subscription(
                Image, topic,
                lambda msg, r=room: self.cctv_image_callback(r, msg), 2)
        self.create_subscription(String, '/compliance/fsm_status',
                                 self.fsm_callback, 10)
        self.create_subscription(String, '/compliance/incident_log',
                                 self.incident_callback, 10)
        self.create_subscription(PoseStamped, '/compliance/navigation_targets',
                                 self.target_callback, 10)

        sender = self.get_parameter('sender_email').value
        recipient = self.get_parameter('recipient_email').value
        self.get_logger().info(
            f'Incident emailer ready ({sender} -> {recipient}, '
            f'rooms: {rooms}, keyframes: {self.keyframe_dir})')

    # ------------------------------------------------------------- frame cache

    def robot_image_callback(self, msg):
        self.robot_frame = msg

    def cctv_image_callback(self, room, msg):
        self.cctv_frames[room] = msg

    def target_callback(self, msg):
        self.latest_target = (round(msg.pose.position.x, 2),
                              round(msg.pose.position.y, 2))

    # --------------------------------------------------------------- keyframes

    def save_frame(self, img_msg, name):
        try:
            frame = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Could not convert frame for {name}: {exc}')
            return None
        stamp = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')
        path = os.path.join(self.keyframe_dir, f'{stamp}_{name}.jpg')
        cv2.imwrite(path, frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        self.get_logger().info(f'Keyframe saved: {path}')
        return path

    def capture_closeup(self):
        if self.closeup_timer is not None:
            self.closeup_timer.cancel()
            self.closeup_timer = None
        if self.robot_frame is not None:
            self.closeup_path = self.save_frame(self.robot_frame, 'robot_closeup')

    def cleanup_old_keyframes(self):
        retention_s = self.get_parameter('retention_days').value * 86400
        now = time.time()
        for name in os.listdir(self.keyframe_dir):
            path = os.path.join(self.keyframe_dir, name)
            if name.endswith('.jpg') and now - os.path.getmtime(path) > retention_s:
                os.remove(path)

    # ------------------------------------------------------------- FSM events

    def fsm_callback(self, msg):
        try:
            status = json.loads(msg.data)
        except ValueError:
            return
        state = status.get('state', '')
        if state == self.prev_state:
            return
        self.prev_state = state

        if state == 'PA_WARNING':
            # New escalation: capture the CCTV evidence frame for that room
            self.active_room = status.get('room', '') or ''
            frame = self.cctv_frames.get(self.active_room)
            if frame is None and self.cctv_frames:
                frame = next(iter(self.cctv_frames.values()))
            self.cctv_keyframe_path = (
                self.save_frame(frame, f'cctv_{self.active_room or "unknown"}')
                if frame is not None else None)
            self.closeup_path = None
        elif state == 'DIRECT_WARNING':
            # Robot has arrived facing the person: close-up after a short settle
            delay = self.get_parameter('closeup_delay').value
            self.closeup_timer = self.create_timer(delay, self.capture_closeup)

    # ----------------------------------------------------------------- sending

    def incident_callback(self, msg):
        if not self.get_parameter('enabled').value:
            return
        try:
            incident = json.loads(msg.data)
        except ValueError:
            incident = {'raw': msg.data}

        now = time.monotonic()
        if now - self.last_email_time < self.get_parameter('min_email_interval').value:
            self.get_logger().warn('Email rate limit hit; incident not emailed '
                                   '(still in incidents.jsonl)')
            return
        self.last_email_time = now

        attachments = [p for p in (self.cctv_keyframe_path, self.closeup_path)
                       if p and os.path.isfile(p)]
        target = self.latest_target
        threading.Thread(target=self.send_email,
                         args=(incident, attachments, target),
                         daemon=True).start()

    def send_email(self, incident, attachments, target):
        room = incident.get('room') or self.active_room or 'unknown room'
        outcome = incident.get('outcome', 'unknown')
        stage = incident.get('stage_reached', '?')
        event_class = incident.get('event_class', 'smoking')
        spoken_room = str(room).replace('_', ' ')

        outcome_text = {
            'complied': 'The person complied after the warning.',
            'logged_no_compliance': 'The person did NOT comply after all warnings.',
            'target_lost': 'The person left before the intervention completed.',
            'safety_stop': 'The robot performed a safety stop during the approach.',
        }.get(outcome, f'Outcome: {outcome}')

        subject = (f'[Compliance Robot] {event_class} violation in {spoken_room} '
                   f'- {outcome} ({stage})')
        lines = [
            'Automated compliance incident report',
            '',
            f'Time (UTC):        {datetime.now(timezone.utc).isoformat()}',
            f'Room:              {spoken_room}',
            f'Violation:         {event_class}',
            f'Escalation stage:  {stage}',
            f'Result:            {outcome_text}',
        ]
        if target:
            lines.append(f'Approx. location:  x={target[0]} m, y={target[1]} m (map frame)')
        if incident.get('confidence') is not None:
            lines.append(f'Detection confidence: {incident["confidence"]}')
        lines += [
            '',
            'Attached: CCTV detection frame and robot close-up (if captured).',
            'Privacy: selective keyframe capture for incident verification; '
            'no facial recognition performed; keyframes auto-delete after '
            f'{self.get_parameter("retention_days").value} days.',
        ]

        email = EmailMessage()
        email['Subject'] = subject
        email['From'] = self.get_parameter('sender_email').value
        email['To'] = self.get_parameter('recipient_email').value
        email.set_content('\n'.join(lines))
        for path in attachments:
            with open(path, 'rb') as f:
                email.add_attachment(f.read(), maintype='image', subtype='jpeg',
                                     filename=os.path.basename(path))

        password = (os.environ.get('COMPLIANCE_EMAIL_PASSWORD')
                    or self.get_parameter('app_password').value)
        password = password.replace(' ', '')
        if not password:
            self.get_logger().error('No email app password configured; not sending')
            return
        try:
            with smtplib.SMTP(self.get_parameter('smtp_host').value,
                              int(self.get_parameter('smtp_port').value),
                              timeout=20) as smtp:
                smtp.starttls()
                smtp.login(self.get_parameter('sender_email').value, password)
                smtp.send_message(email)
            self.get_logger().info(
                f'Incident email sent to {email["To"]} '
                f'({len(attachments)} attachment(s))')
        except Exception as exc:  # noqa: BLE001 - report SMTP failures
            self.get_logger().error(f'Email send failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = IncidentEmailerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

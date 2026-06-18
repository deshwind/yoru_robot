"""Scenario publisher for simulation testing (dissertation Section 5.1).

Until the custom six-class YOLO model is trained, this node makes the full
pipeline testable end-to-end in Gazebo by injecting synthetic cigarette /
vape detections:

  - 'augment'   : attach a device bbox to real YOLO person detections
  - 'synthetic' : publish a synthetic person + device at configured pixels
  - 'auto'      : augment when YOLO sees a person, otherwise synthetic

Scenario types map to the test scenarios in Section 5.1:
  smoking / vaping        -> Scenario A (full escalation)
  false_positive          -> Scenario B (phone near mouth, must be rejected)
  target_loss             -> Scenario C (detections stop mid-escalation)

The simulated person "complies" (device disappears) after the FSM reaches
'comply_after_stage', which exercises the compliance-reset path.
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


def make_detection(header, class_id, score, cx, cy, w, h):
    det = Detection2D()
    det.header = header
    det.bbox.center.position.x = float(cx)
    det.bbox.center.position.y = float(cy)
    det.bbox.size_x = float(w)
    det.bbox.size_y = float(h)
    hyp = ObjectHypothesisWithPose()
    hyp.hypothesis.class_id = class_id
    hyp.hypothesis.score = float(score)
    det.results.append(hyp)
    return det


class ScenarioPublisherNode(Node):

    def __init__(self):
        super().__init__('scenario_publisher_node')

        self.declare_parameter('mode', 'auto')  # auto|augment|synthetic|off
        self.declare_parameter('scenario_type', 'smoking')
        self.declare_parameter('start_delay', 20.0)
        self.declare_parameter('publish_hz', 10.0)
        self.declare_parameter('yolo_topic', '/compliance/detections_yolo')
        self.declare_parameter('output_topic', '/compliance/detections')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('person_pixel_x', 320.0)
        self.declare_parameter('person_pixel_y', 300.0)
        self.declare_parameter('person_bbox_w', 80.0)
        self.declare_parameter('person_bbox_h', 200.0)
        self.declare_parameter('camera_frame', 'cctv_link_optical')
        self.declare_parameter('comply_after_stage', 'DIRECT_WARNING')
        self.declare_parameter('comply_delay', 5.0)
        self.declare_parameter('target_loss_after', 30.0)

        self.mode = self.get_parameter('mode').value
        self.scenario = self.get_parameter('scenario_type').value
        self.start_delay = self.get_parameter('start_delay').value
        self.comply_after_stage = self.get_parameter('comply_after_stage').value
        self.comply_delay = self.get_parameter('comply_delay').value

        self.declare_parameter('coast_duration', 3.0)

        self.start_time = time.monotonic()
        self.comply_stage_time = None
        self.complied = False
        self.latest_yolo = None
        self.latest_yolo_time = 0.0
        # Last real person bbox, kept through brief YOLO dropouts so the
        # published person position stays stable and SORT keeps the track.
        self.last_person_box = None
        self.last_person_time = 0.0

        out_topic = self.get_parameter('output_topic').value
        self.det_pub = self.create_publisher(Detection2DArray, out_topic, 10)
        self.create_subscription(
            Detection2DArray, self.get_parameter('yolo_topic').value,
            self.yolo_callback, 10)
        self.create_subscription(String, '/compliance/fsm_status', self.fsm_callback, 10)

        hz = self.get_parameter('publish_hz').value
        self.create_timer(1.0 / max(hz, 1.0), self.tick)
        self.get_logger().info(
            f'Scenario publisher: mode={self.mode}, scenario={self.scenario}, '
            f'device appears after {self.start_delay:.0f}s -> {out_topic}')

    def yolo_callback(self, msg):
        self.latest_yolo = msg
        self.latest_yolo_time = time.monotonic()

    def fsm_callback(self, msg):
        try:
            state = json.loads(msg.data).get('state', '')
        except (ValueError, AttributeError):
            return
        if state == self.comply_after_stage and self.comply_stage_time is None:
            self.comply_stage_time = time.monotonic()
            self.get_logger().info(
                f'FSM reached {state}; simulated person will comply '
                f'in {self.comply_delay:.0f}s')

    def device_class(self):
        if self.scenario == 'vaping':
            return 'vape_device'
        if self.scenario == 'false_positive':
            return 'mobile_phone'
        return 'cigarette'

    def tick(self):
        if self.mode == 'off':
            return
        elapsed = time.monotonic() - self.start_time
        scenario_active = elapsed >= self.start_delay

        if self.scenario == 'target_loss':
            loss_after = self.get_parameter('target_loss_after').value
            if elapsed >= self.start_delay + loss_after:
                return  # person vanished entirely: pipeline must handle loss

        if (self.comply_stage_time is not None and not self.complied
                and time.monotonic() - self.comply_stage_time >= self.comply_delay):
            self.complied = True
            self.get_logger().info('Simulated person complied (device removed)')

        array = Detection2DArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.header.frame_id = self.get_parameter('camera_frame').value

        now = time.monotonic()
        yolo_fresh = (self.latest_yolo is not None
                      and now - self.latest_yolo_time < 1.0)
        use_augment = self.mode == 'augment' or (self.mode == 'auto' and yolo_fresh)

        person_box = None
        if use_augment and yolo_fresh:
            array.header = self.latest_yolo.header
            for det in self.latest_yolo.detections:
                array.detections.append(det)
                if det.results and det.results[0].hypothesis.class_id == 'person':
                    if person_box is None:
                        person_box = det.bbox
        if person_box is not None:
            self.last_person_box = person_box
            self.last_person_time = now
        else:
            if use_augment and self.mode == 'augment':
                self.det_pub.publish(array)  # pass-through only
                return
            coast = self.get_parameter('coast_duration').value
            if (self.last_person_box is not None
                    and now - self.last_person_time < coast):
                # YOLO dropout: coast on the last real position so the SORT
                # track survives instead of jumping to the synthetic pixel
                b = self.last_person_box
                person = make_detection(array.header, 'person', 0.85,
                                        b.center.position.x, b.center.position.y,
                                        b.size_x, b.size_y)
            else:
                # Synthetic person at the configured pixel
                px = self.get_parameter('person_pixel_x').value
                py = self.get_parameter('person_pixel_y').value
                pw = self.get_parameter('person_bbox_w').value
                ph = self.get_parameter('person_bbox_h').value
                person = make_detection(array.header, 'person', 0.92, px, py, pw, ph)
            array.detections.append(person)
            person_box = person.bbox

        if scenario_active and not self.complied:
            # Device near the mouth region: upper third of the person bbox
            mouth_x = person_box.center.position.x
            mouth_y = (person_box.center.position.y
                       - person_box.size_y / 2.0 + 0.18 * person_box.size_y)
            array.detections.append(make_detection(
                array.header, self.device_class(), 0.78,
                mouth_x + 8.0, mouth_y, 22.0, 12.0))
            if self.scenario in ('smoking', 'vaping'):
                array.detections.append(make_detection(
                    array.header, 'hand_mouth_gesture', 0.7,
                    mouth_x, mouth_y, 50.0, 40.0))
            if self.scenario == 'smoking':
                array.detections.append(make_detection(
                    array.header, 'smoke_vapour', 0.6,
                    mouth_x + 25.0, mouth_y - 30.0, 60.0, 50.0))

        self.det_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

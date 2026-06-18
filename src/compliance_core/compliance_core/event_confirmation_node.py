"""Event confirmation node (dissertation Sections 3.6 and 4.4).

Decision gate of the perception pipeline. Confirms a smoking/vaping event
only when the multi-criteria framework is satisfied:

  C1 person present (conf > 0.7)
  C2 cigarette or vape device (conf > 0.6), OR smoke exhaled at the mouth
     (smoke_in_mouth_confirms, conf > smoke_confidence)
  C3 spatial proximity: the device/smoke overlaps the person's mouth region
  C4 temporal persistence: >= N consecutive frames
  C5 supporting evidence (optional): smoke_vapour / hand_mouth_gesture / hand_face
  C6 tracking consistency: same SORT track ID
  C7 false-positive risk: pen / mobile_phone / straw near mouth -> high risk
     (unless phone_at_mouth_is_vape: then a phone at the mouth = a vape device)

Confidence = 0.4*device + 0.3*proximity + 0.2*persistence + 0.1*support
Confirmed when Confidence >= 0.6 AND C1 AND C2 AND risk != high.
0.4-0.6 logged as 'uncertain'; below 0.4 discarded silently.
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

DEVICE_CLASSES = ('cigarette', 'vape_device')
SUPPORT_WEIGHTS = {'smoke_vapour': 0.3, 'hand_mouth_gesture': 0.2, 'hand_face': 0.1}
CONFOUNDER_CLASSES = ('pen', 'mobile_phone', 'straw')


def bbox_iou(a, b):
    ax1 = a.center.position.x - a.size_x / 2.0
    ay1 = a.center.position.y - a.size_y / 2.0
    ax2 = a.center.position.x + a.size_x / 2.0
    ay2 = a.center.position.y + a.size_y / 2.0
    bx1 = b.center.position.x - b.size_x / 2.0
    by1 = b.center.position.y - b.size_y / 2.0
    bx2 = b.center.position.x + b.size_x / 2.0
    by2 = b.center.position.y + b.size_y / 2.0
    iw = max(0.0, min(ax2, bx2) - max(ax1, bx1))
    ih = max(0.0, min(ay2, by2) - max(ay1, by1))
    inter = iw * ih
    union = a.size_x * a.size_y + b.size_x * b.size_y - inter
    return inter / union if union > 0.0 else 0.0


def in_mouth_region(person_bbox, obj_bbox, region_fraction=0.4):
    """True if the object's centre lies in the upper part of the person bbox."""
    px = person_bbox.center.position.x
    py_top = person_bbox.center.position.y - person_bbox.size_y / 2.0
    ox = obj_bbox.center.position.x
    oy = obj_bbox.center.position.y
    within_x = abs(ox - px) <= person_bbox.size_x * 0.75
    within_y = py_top <= oy <= py_top + person_bbox.size_y * region_fraction
    return within_x and within_y


class EventConfirmationNode(Node):

    def __init__(self):
        super().__init__('event_confirmation_node')

        self.declare_parameter('person_confidence', 0.7)
        self.declare_parameter('device_confidence', 0.6)
        # Smoke at the mouth can confirm on its own (a person exhaling smoke),
        # not only a visible cigarette/vape device.
        self.declare_parameter('smoke_confidence', 0.4)
        self.declare_parameter('smoke_in_mouth_confirms', True)
        # A vape held to the mouth reads as 'cell phone' to the COCO model.
        # When enabled, a phone in the mouth region counts as a vape_device
        # (and is NOT treated as a blocking confounder). Off = original C7.
        self.declare_parameter('phone_at_mouth_is_vape', False)
        self.declare_parameter('vape_phone_confidence', 0.4)
        self.declare_parameter('proximity_iou', 0.05)
        self.declare_parameter('persistence_frames', 5)
        self.declare_parameter('confirm_confidence', 0.6)
        self.declare_parameter('uncertain_confidence', 0.4)
        self.declare_parameter('input_topic', '/compliance/tracked_detections')
        self.declare_parameter('output_topic', '/compliance/confirmed_events')
        # Which room/zone this camera observes; carried into metadata and incidents
        self.declare_parameter('room_id', '')

        self.persistence_required = int(self.get_parameter('persistence_frames').value)
        self.persistence = {}  # track_id -> consecutive satisfied frames

        self.confirmed_pub = self.create_publisher(
            Detection2DArray, self.get_parameter('output_topic').value, 10)
        self.metadata_pub = self.create_publisher(
            String, '/compliance/event_metadata', 10)
        self.create_subscription(
            Detection2DArray, self.get_parameter('input_topic').value,
            self.tracked_callback, 10)

        self.get_logger().info('Event confirmation node ready (criteria C1-C7)')

    def tracked_callback(self, msg):
        person_conf_min = self.get_parameter('person_confidence').value
        device_conf_min = self.get_parameter('device_confidence').value
        smoke_conf_min = self.get_parameter('smoke_confidence').value
        smoke_confirms = self.get_parameter('smoke_in_mouth_confirms').value
        phone_is_vape = self.get_parameter('phone_at_mouth_is_vape').value
        vape_phone_conf_min = self.get_parameter('vape_phone_confidence').value
        proximity_iou_min = self.get_parameter('proximity_iou').value
        confirm_at = self.get_parameter('confirm_confidence').value
        uncertain_at = self.get_parameter('uncertain_confidence').value

        persons, devices, supports, confounders, smokes, phones = [], [], [], [], [], []
        for det in msg.detections:
            if not det.results:
                continue
            cls = det.results[0].hypothesis.class_id
            if cls == 'person':
                persons.append(det)
            elif cls in DEVICE_CLASSES:
                devices.append(det)
            elif cls in SUPPORT_WEIGHTS:
                supports.append(det)
                if cls == 'smoke_vapour':
                    smokes.append(det)
            elif cls == 'mobile_phone' and phone_is_vape:
                # Treat a phone-like object as a possible vape, not a confounder
                phones.append(det)
            elif cls in CONFOUNDER_CLASSES:
                confounders.append(det)

        confirmed = Detection2DArray()
        confirmed.header = msg.header
        seen_tracks = set()

        for person in persons:
            track_id = person.id or 'untracked'
            seen_tracks.add(track_id)
            c1 = person.results[0].hypothesis.score > person_conf_min

            # C2 + C3: best device associated with this person's mouth region
            best_device, best_prox = None, 0.0
            for dev in devices:
                if dev.results[0].hypothesis.score <= device_conf_min:
                    continue
                iou_val = bbox_iou(person.bbox, dev.bbox)
                near_mouth = in_mouth_region(person.bbox, dev.bbox)
                if iou_val > proximity_iou_min or near_mouth:
                    prox = max(min(iou_val / 0.3, 1.0), 0.8 if near_mouth else 0.0)
                    if prox > best_prox:
                        best_device, best_prox = dev, prox
            # C2 + C3 (alternative): a phone-like object at the mouth = vape.
            best_phone, best_phone_prox = None, 0.0
            for ph in phones:
                if ph.results[0].hypothesis.score <= vape_phone_conf_min:
                    continue
                iou_val = bbox_iou(person.bbox, ph.bbox)
                near_mouth = in_mouth_region(person.bbox, ph.bbox)
                if iou_val > proximity_iou_min or near_mouth:
                    prox = max(min(iou_val / 0.3, 1.0), 0.8 if near_mouth else 0.0)
                    if prox > best_phone_prox:
                        best_phone, best_phone_prox = ph, prox

            # C2 + C3 (alternative): smoke exhaled at this person's mouth.
            best_smoke, best_smoke_prox = None, 0.0
            if smoke_confirms:
                for sm in smokes:
                    if sm.results[0].hypothesis.score <= smoke_conf_min:
                        continue
                    iou_val = bbox_iou(person.bbox, sm.bbox)
                    near_mouth = in_mouth_region(person.bbox, sm.bbox,
                                                 region_fraction=0.5)
                    if iou_val > proximity_iou_min or near_mouth:
                        prox = max(min(iou_val / 0.3, 1.0), 0.8 if near_mouth else 0.0)
                        if prox > best_smoke_prox:
                            best_smoke, best_smoke_prox = sm, prox

            # Unify the violation evidence: a real device wins, then a phone-like
            # object at the mouth (vape), then smoke exhaled at the mouth.
            if best_device is not None:
                event_class = best_device.results[0].hypothesis.class_id
                evidence_score, evidence_prox, trigger = (
                    best_device.results[0].hypothesis.score, best_prox, 'device')
            elif best_phone is not None:
                event_class = 'vaping'  # phone-like object held at the mouth
                evidence_score, evidence_prox, trigger = (
                    best_phone.results[0].hypothesis.score, best_phone_prox, 'phone_vape')
            elif best_smoke is not None:
                event_class = 'smoking'  # smoke exhaled at the mouth
                evidence_score, evidence_prox, trigger = (
                    best_smoke.results[0].hypothesis.score, best_smoke_prox, 'smoke')
            else:
                event_class, evidence_score, evidence_prox, trigger = None, 0.0, 0.0, None
            c2 = event_class is not None
            c3 = evidence_prox > 0.0

            # C4 / C6: persistence on the same track ID
            if c1 and c2 and c3:
                self.persistence[track_id] = self.persistence.get(track_id, 0) + 1
            else:
                self.persistence[track_id] = 0
            frames = self.persistence[track_id]
            c4 = frames >= self.persistence_required
            persistence_score = min(frames / float(self.persistence_required), 1.0)

            # C5: supporting evidence near this person
            support_score = 0.0
            for sup in supports:
                if in_mouth_region(person.bbox, sup.bbox, region_fraction=0.6) or \
                        bbox_iou(person.bbox, sup.bbox) > 0.02:
                    support_score += SUPPORT_WEIGHTS[sup.results[0].hypothesis.class_id]
            support_score = min(support_score, 1.0)

            # C7: confounder near the mouth -> high false-positive risk
            fp_risk = 'low'
            for con in confounders:
                if in_mouth_region(person.bbox, con.bbox):
                    fp_risk = 'high'
                    break

            confidence = (0.4 * evidence_score + 0.3 * evidence_prox
                          + 0.2 * persistence_score + 0.1 * support_score)

            is_confirmed = (confidence >= confirm_at and c1 and c2 and c4
                            and fp_risk != 'high')
            status = ('confirmed' if is_confirmed
                      else 'uncertain' if confidence >= uncertain_at
                      else 'rejected')

            if is_confirmed:
                event = person  # person detection carries the track ID and bbox
                confirmed.detections.append(event)

            if status != 'rejected':
                meta = String()
                meta.data = json.dumps({
                    'track_id': track_id,
                    'room': self.get_parameter('room_id').value,
                    'status': status,
                    'confidence': round(confidence, 3),
                    'event_class': event_class,
                    'trigger': trigger,  # 'device' | 'smoke'
                    'criteria': {
                        'C1_person': c1, 'C2_device': c2, 'C3_proximity': c3,
                        'C4_persistence': c4, 'C5_support': round(support_score, 2),
                        'C6_track': track_id != 'untracked',
                        'C7_fp_risk': fp_risk,
                    },
                    'scores': {
                        'evidence': round(evidence_score, 3),
                        'proximity': round(evidence_prox, 3),
                        'persistence': round(persistence_score, 3),
                        'support': round(support_score, 3),
                    },
                })
                self.metadata_pub.publish(meta)

        # Forget tracks that disappeared (volatile, privacy-preserving)
        for stale in [t for t in self.persistence if t not in seen_tracks]:
            del self.persistence[stale]

        if confirmed.detections:
            self.confirmed_pub.publish(confirmed)


def main(args=None):
    rclpy.init(args=args)
    node = EventConfirmationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

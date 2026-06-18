"""SORT tracking node (dissertation Section 4.3).

Tracks 'person' detections across frames with SORT/Kalman so the escalation
framework can follow one individual from detection to warning delivery.
Non-person detections pass through unchanged for downstream association.

Privacy: anonymous numeric track IDs (from 1000), volatile memory only.
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

from compliance_core.sort_tracker import Sort


class TrackingNode(Node):

    def __init__(self):
        super().__init__('tracking_node')

        self.declare_parameter('iou_threshold', 0.3)
        self.declare_parameter('max_missed_frames', 5)
        self.declare_parameter('min_hits', 1)
        self.declare_parameter('input_topic', '/compliance/detections')
        self.declare_parameter('output_topic', '/compliance/tracked_detections')
        # Per-camera prefix so track IDs stay unique across pipelines (c1_, c2_)
        self.declare_parameter('track_id_prefix', '')

        self.tracker = Sort(
            iou_threshold=self.get_parameter('iou_threshold').value,
            max_missed=int(self.get_parameter('max_missed_frames').value),
            min_hits=int(self.get_parameter('min_hits').value))

        self.tracked_pub = self.create_publisher(
            Detection2DArray, self.get_parameter('output_topic').value, 10)
        self.info_pub = self.create_publisher(String, '/compliance/tracking_info', 10)
        self.create_subscription(
            Detection2DArray, self.get_parameter('input_topic').value,
            self.detections_callback, 10)

        self.get_logger().info('SORT tracking node ready')

    def detections_callback(self, msg):
        t0 = time.monotonic()

        persons = []
        others = []
        for det in msg.detections:
            if det.results and det.results[0].hypothesis.class_id == 'person':
                persons.append(det)
            else:
                others.append(det)

        boxes = [(d.bbox.center.position.x, d.bbox.center.position.y,
                  d.bbox.size_x, d.bbox.size_y) for d in persons]
        tracks = self.tracker.update(boxes)

        out = Detection2DArray()
        out.header = msg.header

        # Re-associate each track with the closest input person detection so
        # original confidence scores are preserved.
        used = set()
        for track_id, bbox, _hits in tracks:
            best_i, best_d = None, float('inf')
            for i, b in enumerate(boxes):
                if i in used:
                    continue
                d = (b[0] - bbox[0]) ** 2 + (b[1] - bbox[1]) ** 2
                if d < best_d:
                    best_i, best_d = i, d
            if best_i is None:
                continue
            used.add(best_i)
            det = persons[best_i]
            det.id = self.get_parameter('track_id_prefix').value + str(track_id)
            det.bbox.center.position.x = float(bbox[0])
            det.bbox.center.position.y = float(bbox[1])
            det.bbox.size_x = float(bbox[2])
            det.bbox.size_y = float(bbox[3])
            out.detections.append(det)

        out.detections.extend(others)
        self.tracked_pub.publish(out)

        info = String()
        info.data = json.dumps({
            'active_tracks': len(tracks),
            'persons_in': len(persons),
            'latency_ms': round((time.monotonic() - t0) * 1000.0, 2),
        })
        self.info_pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

"""YOLO detector node (dissertation Section 4.2).

Detects objects from one of four camera sources (ROS topic / USB / RTSP /
video file) and publishes vision_msgs/Detection2DArray in pixel coordinates.

Two models can run on the same frame and their detections are merged:
  - the primary model (COCO yolov8n by default) supplies 'person' and the
    confounders (C7) via COCO_CLASS_MAP;
  - an optional 'smoking_model_path' model supplies the violation classes
    (cigarette / vape_device / smoke_vapour) that the confirmation pipeline
    keys off. A ready-made cigarette detector is fetched by
    ./download_smoking_model.sh -> cigarette_yolo.pt.

The fully custom six-class model can still replace the primary model via
'model_path' once trained (set use_coco_class_map:=false).
"""

import json
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

import cv2
from cv_bridge import CvBridge

# Map COCO names to the project class vocabulary (confounders feed C7).
COCO_CLASS_MAP = {
    'person': 'person',
    'cell phone': 'mobile_phone',
    'remote': 'mobile_phone',
    'toothbrush': 'pen',
    'fork': 'pen',
    'cup': 'straw',
    'bottle': 'straw',
}

# Map a pretrained smoking model's class names to the project vocabulary.
# Lower-cased lookup so model label casing does not matter. Override per
# deployment with the 'smoking_class_map' parameter (a JSON object string).
DEFAULT_SMOKING_CLASS_MAP = {
    'cigarette': 'cigarette',
    'cig': 'cigarette',
    'cigar': 'cigarette',
    'smoking': 'cigarette',
    'smoke_cig': 'cigarette',
    'vape': 'vape_device',
    'vaping': 'vape_device',
    'vape_device': 'vape_device',
    'e-cigarette': 'vape_device',
    'ecig': 'vape_device',
    'smoke': 'smoke_vapour',
    'smoke_vapour': 'smoke_vapour',
    'vapour': 'smoke_vapour',
    'vapor': 'smoke_vapour',
}

# Classes that raise the smoking alert (devices; smoke_vapour is only
# supporting evidence in the confirmation node, so it is not an alert on its own).
ALERT_CLASSES = ('cigarette', 'vape_device')


class YoloDetectorNode(Node):

    def __init__(self):
        super().__init__('yolo_detector_node')

        self.declare_parameter('source_type', 'ros_topic')  # ros_topic|usb|rtsp|video
        self.declare_parameter('ros_topic', '/cctv/image_raw')
        self.declare_parameter('device_index', 0)
        self.declare_parameter('rtsp_url', '')
        self.declare_parameter('video_path', '')
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('input_size', 640)
        self.declare_parameter('process_hz', 5.0)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('detections_topic', '/compliance/detections')
        self.declare_parameter('debug_image_topic', '/compliance/debug_image')
        self.declare_parameter('use_coco_class_map', True)
        # Extra model(s) dedicated to smoking/vaping/smoke. Comma-separated to
        # run several (e.g. a cigarette model and a smoke model). Empty = off.
        self.declare_parameter('smoking_model_path', '')
        self.declare_parameter('smoking_confidence_threshold', 0.35)
        self.declare_parameter('smoking_class_map', '')  # JSON object string; '' = default

        self.source_type = self.get_parameter('source_type').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.smoking_conf = self.get_parameter('smoking_confidence_threshold').value
        self.input_size = int(self.get_parameter('input_size').value)
        self.use_coco_map = self.get_parameter('use_coco_class_map').value
        process_hz = self.get_parameter('process_hz').value

        self.smoking_class_map = dict(DEFAULT_SMOKING_CLASS_MAP)
        raw_map = self.get_parameter('smoking_class_map').value
        if raw_map:
            try:
                self.smoking_class_map.update(
                    {str(k).lower(): v for k, v in json.loads(raw_map).items()})
            except ValueError:
                self.get_logger().warn(
                    'smoking_class_map is not valid JSON; using defaults')

        self.bridge = CvBridge()
        self.model = None
        self.smoking_models = []
        self.capture = None
        self.latest_frame = None

        det_topic = self.get_parameter('detections_topic').value
        self.det_pub = self.create_publisher(Detection2DArray, det_topic, 10)
        self.alert_pub = self.create_publisher(String, '/compliance/smoking_detected', 10)
        self.debug_pub = None
        if self.get_parameter('publish_debug_image').value:
            self.debug_pub = self.create_publisher(
                Image, self.get_parameter('debug_image_topic').value, 2)

        self._load_models()

        if self.source_type == 'ros_topic':
            topic = self.get_parameter('ros_topic').value
            self.create_subscription(Image, topic, self.image_callback, 2)
            self.get_logger().info(f'Camera source: ROS topic {topic}')
        else:
            self._open_capture()

        self.create_timer(1.0 / max(process_hz, 0.5), self.process_frame)
        self.get_logger().info(
            f'YOLO detector ready (model={self.get_parameter("model_path").value}, '
            f'smoking_model={self.get_parameter("smoking_model_path").value or "off"}, '
            f'source={self.source_type}, publishing {det_topic})')

    def _load_one(self, path):
        try:
            from ultralytics import YOLO
            return YOLO(path)
        except Exception as exc:  # noqa: BLE001 - report and run degraded
            self.get_logger().error(
                f'Could not load YOLO model "{path}": {exc}')
            return None

    def _load_models(self):
        # Primary model: a bare name (e.g. yolov8n.pt) auto-downloads via ultralytics.
        model_path = self.get_parameter('model_path').value
        self.model = self._load_one(model_path)
        if self.model is None:
            self.get_logger().error(
                'Primary model unavailable; node stays alive but publishes '
                'no person/confounder detections.')

        smoking_spec = self.get_parameter('smoking_model_path').value
        for path in [p.strip() for p in smoking_spec.split(',') if p.strip()]:
            if not os.path.isfile(path):
                self.get_logger().warn(
                    f'smoking_model_path "{path}" not found; skipped. Run '
                    './download_smoking_model.sh on this machine, then restart.')
                continue
            model = self._load_one(path)
            if model is not None:
                self.smoking_models.append(model)
                self.get_logger().info(f'Smoking model loaded: {path}')

    def _open_capture(self):
        if self.source_type == 'usb':
            src = int(self.get_parameter('device_index').value)
        elif self.source_type == 'rtsp':
            src = self.get_parameter('rtsp_url').value
        elif self.source_type == 'video':
            src = self.get_parameter('video_path').value
        else:
            self.get_logger().error(f'Unknown source_type: {self.source_type}')
            return
        self.capture = cv2.VideoCapture(src)
        if not self.capture.isOpened():
            self.get_logger().error(f'Failed to open camera source: {src}')

    def image_callback(self, msg):
        self.latest_frame = (self.bridge.imgmsg_to_cv2(msg, 'bgr8'), msg.header)

    def _coco_class_id(self, raw_name):
        if self.use_coco_map:
            return COCO_CLASS_MAP.get(raw_name)
        return raw_name

    def _smoking_class_id(self, raw_name):
        return self.smoking_class_map.get(raw_name.lower())

    def _append_detections(self, results, array, frame, class_mapper, color):
        """Maps one model's boxes into 'array' (and onto 'frame' for the debug
        image). Returns the list of alert-worthy class ids found."""
        alert = []
        names = results[0].names
        for box in results[0].boxes:
            raw_name = names[int(box.cls[0])]
            class_id = class_mapper(raw_name)
            if class_id is None:
                continue

            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
            det = Detection2D()
            det.header = array.header
            det.bbox.center.position.x = (x1 + x2) / 2.0
            det.bbox.center.position.y = (y1 + y2) / 2.0
            det.bbox.size_x = x2 - x1
            det.bbox.size_y = y2 - y1
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = class_id
            hyp.hypothesis.score = float(box.conf[0])
            det.results.append(hyp)
            array.detections.append(det)
            if class_id in ALERT_CLASSES:
                alert.append(class_id)

            if self.debug_pub is not None:
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                cv2.putText(frame, f'{class_id} {float(box.conf[0]):.2f}',
                            (int(x1), max(int(y1) - 5, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        return alert

    def process_frame(self):
        if self.model is None and not self.smoking_models:
            return
        if self.source_type == 'ros_topic':
            if self.latest_frame is None:
                return
            frame, header = self.latest_frame
        else:
            if self.capture is None or not self.capture.isOpened():
                return
            ok, frame = self.capture.read()
            if not ok:
                if self.source_type == 'video':  # loop test videos
                    self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
                return
            header = None

        array = Detection2DArray()
        if header is not None:
            array.header = header
        else:
            array.header.stamp = self.get_clock().now().to_msg()
            array.header.frame_id = 'camera'

        alert_classes = []
        if self.model is not None:
            results = self.model.predict(
                frame, imgsz=self.input_size, conf=self.conf_threshold, verbose=False)
            alert_classes += self._append_detections(
                results, array, frame, self._coco_class_id, (0, 200, 0))
        for smoking_model in self.smoking_models:
            sresults = smoking_model.predict(
                frame, imgsz=self.input_size, conf=self.smoking_conf, verbose=False)
            alert_classes += self._append_detections(
                sresults, array, frame, self._smoking_class_id, (0, 0, 220))

        self.det_pub.publish(array)

        if alert_classes:
            alert = String()
            alert.data = json.dumps({
                'stamp': float(self.get_clock().now().nanoseconds) * 1e-9,
                'classes': alert_classes,
            })
            self.alert_pub.publish(alert)

        if self.debug_pub is not None:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

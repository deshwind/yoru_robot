"""Robot camera publisher (USB webcam via OpenCV/V4L2).

Publishes /camera/image_raw for the incident emailer's close-up shots and
any on-robot perception. For the Raspberry Pi Camera Module (ribbon cable)
on Ubuntu 22.04 use the 'camera_ros' package instead (libcamera based);
pi_hardware.launch.py selects between them with the camera:=picam|usb arg.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge


class CameraPublisherNode(Node):

    def __init__(self):
        super().__init__('camera_publisher_node')

        self.declare_parameter('device', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 5.0)
        self.declare_parameter('frame_id', 'camera_link_optical')
        self.declare_parameter('topic', '/camera/image_raw')

        self.bridge = CvBridge()
        device = int(self.get_parameter('device').value)
        self.capture = cv2.VideoCapture(device)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH,
                         int(self.get_parameter('width').value))
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT,
                         int(self.get_parameter('height').value))
        if not self.capture.isOpened():
            self.get_logger().error(
                f'Cannot open camera /dev/video{device}. '
                'Node stays alive and retries every 5 s.')

        self.pub = self.create_publisher(
            Image, self.get_parameter('topic').value, 2)
        fps = max(self.get_parameter('fps').value, 0.5)
        self.create_timer(1.0 / fps, self.tick)
        self.create_timer(5.0, self.reopen_if_needed)
        self.get_logger().info(
            f'Camera publisher: /dev/video{device} -> '
            f'{self.get_parameter("topic").value} at {fps:.0f} fps')

    def reopen_if_needed(self):
        if not self.capture.isOpened():
            self.capture.open(int(self.get_parameter('device').value))

    def tick(self):
        if not self.capture.isOpened():
            return
        ok, frame = self.capture.read()
        if not ok:
            return
        msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').value
        self.pub.publish(msg)

    def destroy_node(self):
        try:
            self.capture.release()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

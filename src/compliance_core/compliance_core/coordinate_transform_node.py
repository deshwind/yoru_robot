"""Coordinate transform node (dissertation Section 4.5, addresses RQ1).

Converts CCTV pixel coordinates of confirmed events into map-frame
coordinates for navigation. Two methods:

  'pinhole'    : intrinsics + camera pose; casts a ray through the pixel and
                 intersects the ground plane. Exact for the simulated CCTV
                 camera whose pose is known from the world file.
  'homography' : 3x3 H (image -> camera-local ground plane) estimated with
                 tools/calibrate_homography.py, then a 2D rigid transform
                 (camera yaw + translation) into the map frame.

Input pixel: bottom-centre of the person bounding box (feet position).
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


# Optical frame (z forward, x right, y down) expressed in the camera link
# frame (x forward, y left, z up) - standard ROS convention.
R_LINK_FROM_OPTICAL = np.array([[0.0, 0.0, 1.0],
                                [-1.0, 0.0, 0.0],
                                [0.0, -1.0, 0.0]])


class CoordinateTransformNode(Node):

    def __init__(self):
        super().__init__('coordinate_transform_node')

        self.declare_parameter('method', 'pinhole')  # pinhole|homography
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        # Pinhole intrinsics; fx<=0 -> derive from horizontal_fov
        self.declare_parameter('fx', -1.0)
        self.declare_parameter('fy', -1.0)
        self.declare_parameter('cx', -1.0)
        self.declare_parameter('cy', -1.0)
        self.declare_parameter('horizontal_fov', 1.3)
        # Camera link pose in the map frame
        self.declare_parameter('camera_position', [4.5, 0.0, 2.5])
        self.declare_parameter('camera_rpy', [0.0, 0.55, 3.14159])
        self.declare_parameter('floor_z', 0.0)
        # Homography method: row-major 3x3, image px -> camera-local ground (m)
        self.declare_parameter('homography', [1.0, 0.0, 0.0,
                                              0.0, 1.0, 0.0,
                                              0.0, 0.0, 1.0])
        self.declare_parameter('camera_yaw_2d', 0.0)
        self.declare_parameter('camera_xy_2d', [0.0, 0.0])
        self.declare_parameter('max_target_range', 15.0)
        self.declare_parameter('input_topic', '/compliance/confirmed_events')

        self.target_pub = self.create_publisher(
            PoseStamped, '/compliance/navigation_targets', 10)
        self.marker_pub = self.create_publisher(
            Marker, '/compliance/target_marker', 5)
        self.create_subscription(
            Detection2DArray, self.get_parameter('input_topic').value,
            self.events_callback, 10)

        self.get_logger().info(
            f'Coordinate transform ready (method='
            f'{self.get_parameter("method").value})')

    def intrinsics(self):
        w = float(self.get_parameter('image_width').value)
        h = float(self.get_parameter('image_height').value)
        fx = self.get_parameter('fx').value
        if fx <= 0.0:
            fov = self.get_parameter('horizontal_fov').value
            fx = (w / 2.0) / math.tan(fov / 2.0)
        fy = self.get_parameter('fy').value
        fy = fx if fy <= 0.0 else fy
        cx = self.get_parameter('cx').value
        cx = w / 2.0 if cx < 0.0 else cx
        cy = self.get_parameter('cy').value
        cy = h / 2.0 if cy < 0.0 else cy
        return fx, fy, cx, cy

    def pixel_to_map_pinhole(self, u, v):
        fx, fy, cx, cy = self.intrinsics()
        pos = np.array(self.get_parameter('camera_position').value, dtype=float)
        rpy = self.get_parameter('camera_rpy').value
        floor_z = self.get_parameter('floor_z').value

        d_optical = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        r_map_link = rpy_to_matrix(*rpy)
        d_map = r_map_link @ R_LINK_FROM_OPTICAL @ d_optical

        if abs(d_map[2]) < 1e-6:
            return None
        t = (floor_z - pos[2]) / d_map[2]
        if t <= 0.0:
            return None  # ray does not hit the floor in front of the camera
        point = pos + t * d_map
        return float(point[0]), float(point[1])

    def pixel_to_map_homography(self, u, v):
        h_flat = self.get_parameter('homography').value
        h_inv = np.linalg.inv(np.array(h_flat, dtype=float).reshape(3, 3))
        g = h_inv @ np.array([u, v, 1.0])
        if abs(g[2]) < 1e-9:
            return None
        gx, gy = g[0] / g[2], g[1] / g[2]
        yaw = self.get_parameter('camera_yaw_2d').value
        tx, ty = self.get_parameter('camera_xy_2d').value
        mx = math.cos(yaw) * gx - math.sin(yaw) * gy + tx
        my = math.sin(yaw) * gx + math.cos(yaw) * gy + ty
        return mx, my

    def events_callback(self, msg):
        if not msg.detections:
            return
        person = msg.detections[0]  # FSM escalates one target at a time
        # Feet position: bottom-centre of the person bounding box
        u = person.bbox.center.position.x
        v = person.bbox.center.position.y + person.bbox.size_y / 2.0

        if self.get_parameter('method').value == 'homography':
            result = self.pixel_to_map_homography(u, v)
        else:
            result = self.pixel_to_map_pinhole(u, v)
        if result is None:
            self.get_logger().warn(f'Pixel ({u:.0f},{v:.0f}) does not project to floor')
            return
        x, y = result
        if math.hypot(x, y) > self.get_parameter('max_target_range').value:
            self.get_logger().warn(f'Projected target ({x:.1f},{y:.1f}) out of range')
            return

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        self.target_pub.publish(pose)
        self.publish_marker(x, y)

    def publish_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'compliance_target'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.05
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = 0.4
        marker.scale.z = 0.1
        marker.color.r = 1.0
        marker.color.a = 0.8
        marker.lifetime.sec = 5
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = CoordinateTransformNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

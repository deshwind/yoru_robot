#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

from tf2_ros import Buffer, TransformListener


class NearestDock(Node):
    def __init__(self):
        super().__init__('nearest_dock_selector')

        # Nav2 Action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for Nav2 action server...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 ready.')

        # TF listener (to get robot pose in map frame)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 4 Dock poses (map frame)
        self.docks = [
            {"name": "Dock 1", "x": 8.735,  "y": 8.152,  "z": -0.683, "w": 0.730},
            {"name": "Dock 2", "x": 8.613,  "y": -9.232, "z": 0.662,  "w": 0.750},
            {"name": "Dock 3", "x": -8.870, "y": 9.202,  "z": -0.707, "w": 0.707},
            {"name": "Dock 4", "x": -8.707, "y": -9.315, "z": 0.721,  "w": 0.693},
        ]

        self.run_once()

    def run_once(self):
        # Ask user battery %
        try:
            battery = int(input("Enter battery percentage: "))
        except ValueError:
            self.get_logger().error("Invalid input")
            return

        if battery > 20:
            self.get_logger().info("Battery OK → No docking")
            return

        self.get_logger().info("Battery low → selecting nearest dock...")

        robot_xy = self.get_robot_xy()
        if robot_xy is None:
            self.get_logger().error("Could not read robot pose from TF (map->base_link).")
            return

        rx, ry = robot_xy
        self.get_logger().info(f"Robot pose: x={rx:.3f}, y={ry:.3f}")

        nearest = self.pick_nearest_dock(rx, ry)
        self.get_logger().info(f"Nearest: {nearest['name']} at ({nearest['x']:.3f}, {nearest['y']:.3f})")

        self.send_goal(nearest)

    def get_robot_xy(self):
        # Try a few times because TF may not be ready immediately
        for _ in range(30):
            try:
                tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                x = tf.transform.translation.x
                y = tf.transform.translation.y
                return x, y
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.1)
        return None

    def pick_nearest_dock(self, rx, ry):
        best = None
        best_d = float('inf')
        for d in self.docks:
            dist = math.hypot(d["x"] - rx, d["y"] - ry)
            if dist < best_d:
                best_d = dist
                best = d
        best["dist"] = best_d
        return best

    def send_goal(self, dock):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = dock["x"]
        goal.pose.pose.position.y = dock["y"]
        goal.pose.pose.orientation.z = dock["z"]
        goal.pose.pose.orientation.w = dock["w"]

        self.get_logger().info(f"Sending goal to {dock['name']} (distance {dock['dist']:.2f} m)...")
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response)

    def goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Dock goal rejected")
            return

        self.get_logger().info("Dock goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result)

    def goal_result(self, future):
        self.get_logger().info("Arrived at selected dock ✅")


def main():
    rclpy.init()
    node = NearestDock()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

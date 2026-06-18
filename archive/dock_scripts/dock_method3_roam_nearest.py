#!/usr/bin/env python3
"""
Method 3 (Upgraded): Random roaming + battery trigger (type 20)
→ choose nearest dock (by TRUE dock distance)
→ go to PRE-DOCK pose
→ then go to TRUE DOCK pose

ROS 2 Humble / Nav2
Run (with Gazebo + localization + Nav2 already running):
  python3 dock_method3_roam_nearest.py
Then type 20 and press Enter to start docking.
"""

import math
import random
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

from tf2_ros import Buffer, TransformListener


class RoamAndNearestDock(Node):
    def __init__(self):
        super().__init__('roam_and_nearest_dock')

        # --- Roaming limits (adjust if your map bounds differ) ---
        self.min_x, self.max_x = -9.5, 9.5
        self.min_y, self.max_y = -9.8, 9.8

        # Nav2 client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for Nav2 action server...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 ready.')

        # TF for robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Docks with PRE-DOCK and TRUE DOCK poses (map frame)
        self.docks = [
            {
                "name": "Dock 1",
                "dock": {"x": 8.735, "y": 8.152, "z": -0.683, "w": 0.730},
                "pre":  {"x": 8.702, "y": 8.651, "z": -0.683, "w": 0.730},
            },
            {
                "name": "Dock 2",
                "dock": {"x": 8.613, "y": -9.232, "z": 0.662, "w": 0.750},
                "pre":  {"x": 8.551, "y": -9.728, "z": 0.662, "w": 0.750},
            },
            {
                "name": "Dock 3",
                "dock": {"x": -8.870, "y": 9.202, "z": -0.707, "w": 0.707},
                "pre":  {"x": -8.870, "y": 9.702, "z": -0.707, "w": 0.707},
            },
            {
                "name": "Dock 4",
                "dock": {"x": -8.707, "y": -9.315, "z": 0.721, "w": 0.693},
                "pre":  {"x": -8.687, "y": -9.815, "z": 0.721, "w": 0.693},
            },
        ]

        # Goal tracking
        self.current_goal_handle = None
        self.low_battery_triggered = False

        # Docking flow state
        self.selected_dock = None          # dict for chosen dock
        self.docking_stage = None          # "pre" then "dock"

        # Background thread waits for you to type 20
        t = threading.Thread(target=self.wait_for_battery_input, daemon=True)
        t.start()

        # Start roaming
        self.get_logger().info("Roaming started. Type 20 and press ENTER to dock.")
        self.send_random_roam_goal()

    # -------------------------- INPUT THREAD --------------------------
    def wait_for_battery_input(self):
        while not self.low_battery_triggered:
            try:
                val = input("Enter battery percentage (type 20 to dock): ")
                battery = int(val.strip())
                if battery <= 20:
                    self.low_battery_triggered = True
                    self.get_logger().warn("Battery low trigger received → docking now.")
                    self.cancel_current_goal_then_dock()
                    return
            except Exception:
                pass

    # -------------------------- TF HELPERS --------------------------
    def get_robot_xy(self):
        for _ in range(30):
            try:
                tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                return tf.transform.translation.x, tf.transform.translation.y
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.1)
        return None

    # -------------------------- NEAREST DOCK --------------------------
    def pick_nearest_dock(self, rx, ry):
        best = None
        best_d = float('inf')
        for d in self.docks:
            dx = d["dock"]["x"]
            dy = d["dock"]["y"]
            dist = math.hypot(dx - rx, dy - ry)
            if dist < best_d:
                best_d = dist
                best = d
        best["dist"] = best_d
        return best

    # -------------------------- NAV2 GOALS --------------------------
    def send_goal_pose(self, x, y, z=0.0, w=1.0, label="goal"):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = float(z)
        goal.pose.pose.orientation.w = float(w)

        self.get_logger().info(f"Sending {label}: x={x:.2f}, y={y:.2f}")
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response)

    def goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            if not self.low_battery_triggered:
                self.send_random_roam_goal()
            return

        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result)

    def goal_result(self, future):
        # Docking flow
        if self.low_battery_triggered:
            if self.selected_dock is None:
                self.get_logger().info("Arrived ✅")
                return

            # Reached PRE-DOCK -> go to TRUE DOCK
            if self.docking_stage == "pre":
                self.get_logger().info("Reached PRE-DOCK ✅ -> sending TRUE DOCK goal...")
                self.docking_stage = "dock"
                dock = self.selected_dock["dock"]
                self.send_goal_pose(
                    dock["x"], dock["y"], dock["z"], dock["w"],
                    label=f"{self.selected_dock['name']} DOCK"
                )
                return

            # Reached TRUE DOCK
            if self.docking_stage == "dock":
                self.get_logger().info("Reached TRUE DOCK ✅ Docking complete 🎯")
                return

        # Roaming loop
        self.get_logger().info("Roam point reached → sending next random roam goal...")
        self.send_random_roam_goal()

    # -------------------------- ROAMING --------------------------
    def send_random_roam_goal(self):
        x = random.uniform(self.min_x, self.max_x)
        y = random.uniform(self.min_y, self.max_y)

        yaw = random.uniform(-math.pi, math.pi)
        z = math.sin(yaw / 2.0)
        w = math.cos(yaw / 2.0)

        self.send_goal_pose(x, y, z, w, label="ROAM")

    # -------------------------- DOCKING ENTRY --------------------------
    def cancel_current_goal_then_dock(self):
        if self.current_goal_handle is not None:
            self.get_logger().info("Canceling current roaming goal...")
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.after_cancel)
        else:
            self.dock_to_nearest()

    def after_cancel(self, future):
        self.get_logger().info("Roaming goal canceled.")
        self.dock_to_nearest()

    def dock_to_nearest(self):
        robot_xy = self.get_robot_xy()
        if robot_xy is None:
            self.get_logger().error("Could not read robot pose from TF (map->base_link).")
            return

        rx, ry = robot_xy
        nearest = self.pick_nearest_dock(rx, ry)

        self.selected_dock = nearest
        self.docking_stage = "pre"

        self.get_logger().warn(
            f"Nearest dock = {nearest['name']} (distance {nearest['dist']:.2f} m). Going to PRE-DOCK first..."
        )

        pre = nearest["pre"]
        self.send_goal_pose(
            pre["x"], pre["y"], pre["z"], pre["w"],
            label=f"{nearest['name']} PRE-DOCK"
        )


def main():
    rclpy.init()
    node = RoamAndNearestDock()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

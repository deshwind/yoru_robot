#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class Method2TwoStage(Node):
    def __init__(self):
        super().__init__('dock_method2_two_stage_nav2')

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("Waiting for Nav2 action server...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Nav2 ready ✅")

        self.stage = "pre"  # "pre" then "dock"
        self.run()

    def run(self):
        try:
            battery = int(input("Enter battery percentage (type 20 to dock): ").strip())
        except ValueError:
            self.get_logger().error("Invalid input.")
            return

        if battery > 20:
            self.get_logger().info("Battery OK → no docking.")
            return

        self.get_logger().warn("Battery low → docking using Method 2 (two-stage).")
        self.send_pre_dock_goal()

    def make_pose(self, x, y, z, w):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = float(z)
        pose.pose.orientation.w = float(w)
        return pose

    def send_pre_dock_goal(self):
        # --- PRE-DOCK (new) ---
        pre = self.make_pose(
            x=-8.870,
            y=9.702,
            z=-0.707,
            w=0.707
        )
        goal = NavigateToPose.Goal()
        goal.pose = pre

        self.get_logger().info("Sending PRE-DOCK goal...")
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response)

    def send_dock_goal(self):
        # --- TRUE DOCK (new) ---
        dock = self.make_pose(
            x=-8.870,
            y=9.202,
            z=-0.707,
            w=0.707
        )
        goal = NavigateToPose.Goal()
        goal.pose = dock

        self.get_logger().info("Sending TRUE DOCK goal...")
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response)

    def goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected ❌")
            return

        self.get_logger().info("Goal accepted ✅")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result)

    def goal_result(self, future):
        if self.stage == "pre":
            self.get_logger().info("Reached PRE-DOCK ✅ → sending TRUE DOCK...")
            self.stage = "dock"
            self.send_dock_goal()
        else:
            self.get_logger().info("Method 2 docking completed 🎯")

def main():
    rclpy.init()
    node = Method2TwoStage()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

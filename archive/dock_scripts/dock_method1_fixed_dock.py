#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class FixedDockMethod1(Node):
    def __init__(self):
        super().__init__('dock_method1_fixed_dock')

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("Waiting for Nav2 action server...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Nav2 ready ✅")

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

        self.get_logger().warn("Battery low → docking using Method 1 (fixed dock).")

        dock_pose = PoseStamped()
        dock_pose.header.frame_id = "map"
        dock_pose.header.stamp = self.get_clock().now().to_msg()

        dock_pose.pose.position.x = -8.870
        dock_pose.pose.position.y = 9.202
        dock_pose.pose.orientation.z = -0.707
        dock_pose.pose.orientation.w = 0.707

        goal = NavigateToPose.Goal()
        goal.pose = dock_pose

        self.get_logger().info("Sending FIXED DOCK goal...")
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response)

    def goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Dock goal rejected ❌")
            return

        self.get_logger().info("Dock goal accepted ✅")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result)

    def goal_result(self, future):
        self.get_logger().info("Method 1 docking completed ✅ (fixed dock)")


def main():
    rclpy.init()
    node = FixedDockMethod1()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

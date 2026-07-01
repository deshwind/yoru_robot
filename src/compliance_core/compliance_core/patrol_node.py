"""Patrol node.

Cycles through configured map waypoints with Nav2 while the FSM is in
MONITORING. Pauses (cancels its goal) whenever the FSM escalates or a
return-to-base request is active, and resumes afterwards.

Waypoints parameter is a flat list: [x1, y1, yaw1, x2, y2, yaw2, ...]
"""

import json
import math
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String


class PatrolNode(Node):

    def __init__(self):
        super().__init__('patrol_node')

        self.declare_parameter('enabled', True)
        self.declare_parameter('waypoints', [2.0, 0.0, 0.0, 2.0, 2.0, 1.57])
        self.declare_parameter('pause_at_waypoint', 3.0)
        self.declare_parameter('retry_delay', 5.0)
        self.declare_parameter('start_delay', 15.0)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.fsm_state = 'MONITORING'
        self.base_request = False
        self.autonomy_paused = False
        self.goal_handle = None
        self.navigating = False
        self.waypoint_index = 0
        self.next_goal_time = time.monotonic() + self.get_parameter('start_delay').value

        self.create_subscription(String, '/compliance/fsm_status',
                                 self.fsm_callback, 10)
        self.create_subscription(Bool, '/compliance/base_request',
                                 self.base_callback, 10)
        self.create_subscription(Bool, '/compliance/autonomy_paused',
                                 self.paused_callback, 10)
        # Saved spots named room*/patrol* (location_manager) override waypoints
        from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy)
        latched = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.spot_waypoints = []
        self.create_subscription(String, '/compliance/locations',
                                 self.locations_callback, latched)
        self.create_timer(1.0, self.tick)

        n = len(self.get_parameter('waypoints').value) // 3
        self.get_logger().info(f'Patrol node ready ({n} waypoints, '
                               f'enabled={self.get_parameter("enabled").value})')

    def fsm_callback(self, msg):
        try:
            state = json.loads(msg.data).get('state', self.fsm_state)
        except ValueError:
            return
        if state != self.fsm_state:
            self.fsm_state = state
            if state != 'MONITORING' and self.navigating:
                self.cancel('FSM escalation')

    def base_callback(self, msg):
        self.base_request = msg.data
        if self.base_request and self.navigating:
            self.cancel('return-to-base request')

    def paused_callback(self, msg):
        if msg.data == self.autonomy_paused:
            return
        self.autonomy_paused = msg.data
        if self.autonomy_paused and self.navigating:
            self.cancel('admin paused autonomy')

    def locations_callback(self, msg):
        try:
            spots = json.loads(msg.data).get('spots', {})
        except ValueError:
            return
        # Deterministic patrol order: sorted by name (room_1, room_2, ...)
        pts = [pose for name, pose in sorted(spots.items())
               if name.startswith(('room', 'patrol'))]
        if pts != self.spot_waypoints:
            self.spot_waypoints = pts
            self.get_logger().info(f'Patrol spots updated: {len(pts)} saved')

    def allowed(self):
        return (self.get_parameter('enabled').value
                and self.fsm_state == 'MONITORING'
                and not self.base_request
                and not self.autonomy_paused)

    def tick(self):
        if self.navigating or not self.allowed():
            return
        if time.monotonic() < self.next_goal_time:
            return

        # Saved room/patrol spots take precedence over the static parameter
        if self.spot_waypoints:
            count = len(self.spot_waypoints)
            i = self.waypoint_index % count
            x, y, yaw = self.spot_waypoints[i]
        else:
            waypoints = self.get_parameter('waypoints').value
            if len(waypoints) < 3:
                return
            count = len(waypoints) // 3
            i = self.waypoint_index % count
            x, y, yaw = waypoints[3 * i], waypoints[3 * i + 1], waypoints[3 * i + 2]

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.next_goal_time = time.monotonic() + self.get_parameter('retry_delay').value
            return

        self.get_logger().info(f'Patrol waypoint {i + 1}/{count}: ({x:.1f}, {y:.1f})')
        self.navigating = True
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle or not self.goal_handle.accepted:
            self.navigating = False
            self.next_goal_time = time.monotonic() + self.get_parameter('retry_delay').value
            return
        self.goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.navigating = False
        if future.result().status == 4:  # SUCCEEDED
            self.waypoint_index += 1
            self.next_goal_time = time.monotonic() + \
                self.get_parameter('pause_at_waypoint').value
        else:
            # Move on after a delay so one unreachable waypoint cannot stall patrol
            self.waypoint_index += 1
            self.next_goal_time = time.monotonic() + \
                self.get_parameter('retry_delay').value

    def cancel(self, reason):
        self.get_logger().info(f'Patrol paused: {reason}')
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.navigating = False


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

"""Admin joystick node - administrator controls on the wireless controller.

Driving itself is handled by joy_node + teleop_twist_joy (hold the deadman
button L2 and use the left stick; R2 = turbo). The joystick publishes to
/cmd_vel_joy which has the highest twist_mux priority, so manual driving
always overrides Nav2 and the FSM.

This node adds admin buttons on top of the same /joy stream:

  OPTIONS (button 9 on PS4) : toggle autonomy pause. While paused, the
                              patrol stops and the FSM will not escalate -
                              the robot is fully manual until resumed.
  TRIANGLE (button 2 on PS4): send the robot to its charging base
                              (return_to_base_node handles navigation).

Button indices are parameters, so other controllers just need a remap.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class AdminJoyNode(Node):

    def __init__(self):
        super().__init__('admin_joy_node')

        self.declare_parameter('pause_button', 9)   # PS4 OPTIONS
        self.declare_parameter('home_button', 2)    # PS4 TRIANGLE

        self.paused = False
        self.prev_buttons = []

        self.pause_pub = self.create_publisher(Bool, '/compliance/autonomy_paused', 10)
        self.home_pub = self.create_publisher(Bool, '/compliance/return_to_base', 10)

        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        # The pause state is shared with the web dashboard: sync from the topic
        # so a toggle here respects changes made there (and vice versa).
        self.create_subscription(Bool, '/compliance/autonomy_paused',
                                 self.paused_sync_callback, 10)
        # Re-announce the pause state so late-starting nodes pick it up
        self.create_timer(1.0, lambda: self.pause_pub.publish(Bool(data=self.paused)))

        self.get_logger().info(
            'Admin joystick ready: OPTIONS = pause/resume autonomy, '
            'TRIANGLE = return to base, L2 + left stick = drive')

    def paused_sync_callback(self, msg):
        self.paused = msg.data

    def pressed(self, buttons, index):
        """Rising edge: pressed now, was not pressed in the previous message."""
        if index >= len(buttons):
            return False
        was = self.prev_buttons[index] if index < len(self.prev_buttons) else 0
        return buttons[index] == 1 and was == 0

    def joy_callback(self, msg):
        pause_btn = int(self.get_parameter('pause_button').value)
        home_btn = int(self.get_parameter('home_button').value)

        if self.pressed(msg.buttons, pause_btn):
            self.paused = not self.paused
            self.pause_pub.publish(Bool(data=self.paused))
            self.get_logger().warn(
                'ADMIN: autonomy PAUSED - robot is manual-only'
                if self.paused else
                'ADMIN: autonomy RESUMED - patrol and escalation active')

        if self.pressed(msg.buttons, home_btn):
            self.home_pub.publish(Bool(data=True))
            self.get_logger().warn('ADMIN: return-to-base requested')

        self.prev_buttons = list(msg.buttons)


def main(args=None):
    rclpy.init(args=args)
    node = AdminJoyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

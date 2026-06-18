"""L298N motor driver node for the real robot (dissertation Section 3.3).

HARDWARE-ONLY node: requires RPi.GPIO on a Raspberry Pi. In simulation the
robot is driven by gazebo_ros2_control instead.

  - PWM speed control on ENA/ENB at 1 kHz, direction via IN1-IN4
  - quadrature encoder feedback on interrupt-driven GPIO
  - per-wheel PID velocity control at 50 Hz with anti-windup
  - differential drive kinematics + wheel odometry (odom -> base_link TF)

Subscribes the twist_mux output (default /diff_cont/cmd_vel_unstamped, so
the same twist_mux config works in sim and on hardware).
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class Pid:
    def __init__(self, kp, ki, kd, out_limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit = out_limit
        self.integral = 0.0
        self.prev_error = 0.0

    def step(self, error, dt):
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        out = self.kp * error + self.ki * self.integral + self.kd * derivative
        if -self.out_limit < out < self.out_limit:
            self.integral += error * dt  # anti-windup: freeze when saturated
        return max(-self.out_limit, min(self.out_limit, out))


class L298nDriverNode(Node):

    def __init__(self):
        super().__init__('l298n_driver_node')

        # BCM pin numbers - adjust to your wiring
        self.declare_parameter('ena_pin', 12)
        self.declare_parameter('in1_pin', 5)
        self.declare_parameter('in2_pin', 6)
        self.declare_parameter('enb_pin', 13)
        self.declare_parameter('in3_pin', 20)
        self.declare_parameter('in4_pin', 21)
        self.declare_parameter('left_encoder_pin', 17)
        self.declare_parameter('right_encoder_pin', 27)
        self.declare_parameter('pwm_frequency', 1000)
        self.declare_parameter('encoder_ticks_per_rev', 40)
        self.declare_parameter('wheel_radius', 0.033)
        self.declare_parameter('wheel_separation', 0.297)
        self.declare_parameter('max_wheel_speed', 0.3)  # m/s at 100% duty
        self.declare_parameter('kp', 2.0)
        self.declare_parameter('ki', 0.1)
        self.declare_parameter('kd', 0.5)
        self.declare_parameter('cmd_vel_topic', '/diff_cont/cmd_vel_unstamped')
        self.declare_parameter('cmd_timeout', 0.5)

        try:
            import RPi.GPIO as GPIO
        except ImportError:
            self.get_logger().fatal(
                'RPi.GPIO not available. This node only runs on the Raspberry Pi. '
                'In simulation the robot is driven by gazebo_ros2_control.')
            raise SystemExit(1)
        self.gpio = GPIO

        GPIO.setmode(GPIO.BCM)
        p = {n: int(self.get_parameter(n).value) for n in
             ('ena_pin', 'in1_pin', 'in2_pin', 'enb_pin', 'in3_pin', 'in4_pin')}
        for pin in p.values():
            GPIO.setup(pin, GPIO.OUT)
        self.pins = p
        freq = int(self.get_parameter('pwm_frequency').value)
        self.pwm_a = GPIO.PWM(p['ena_pin'], freq)
        self.pwm_b = GPIO.PWM(p['enb_pin'], freq)
        self.pwm_a.start(0.0)
        self.pwm_b.start(0.0)

        self.left_ticks = 0
        self.right_ticks = 0
        self.left_dir = 1
        self.right_dir = 1
        le = int(self.get_parameter('left_encoder_pin').value)
        re = int(self.get_parameter('right_encoder_pin').value)
        GPIO.setup(le, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(re, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(le, GPIO.BOTH, callback=self._left_tick)
        GPIO.add_event_detect(re, GPIO.BOTH, callback=self._right_tick)

        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        self.pid_left = Pid(kp, ki, kd, 100.0)
        self.pid_right = Pid(kp, ki, kd, 100.0)

        self.target_left = 0.0   # wheel surface speed, m/s
        self.target_right = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.x = self.y = self.theta = 0.0

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, self.get_parameter('cmd_vel_topic').value,
                                 self.cmd_callback, 10)
        self.create_timer(0.02, self.control_loop)  # 50 Hz (20 ms)

        self.get_logger().info('L298N driver ready (PWM 1 kHz, PID 50 Hz)')

    def _left_tick(self, _channel):
        self.left_ticks += self.left_dir

    def _right_tick(self, _channel):
        self.right_ticks += self.right_dir

    def cmd_callback(self, msg):
        half_l = self.get_parameter('wheel_separation').value / 2.0
        self.target_left = msg.linear.x - msg.angular.z * half_l
        self.target_right = msg.linear.x + msg.angular.z * half_l
        self.last_cmd_time = self.get_clock().now()

    def control_loop(self):
        dt = 0.02
        now = self.get_clock().now()
        if (now - self.last_cmd_time).nanoseconds * 1e-9 > \
                self.get_parameter('cmd_timeout').value:
            self.target_left = self.target_right = 0.0

        ticks_per_rev = self.get_parameter('encoder_ticks_per_rev').value
        radius = self.get_parameter('wheel_radius').value
        m_per_tick = 2.0 * math.pi * radius / ticks_per_rev

        d_left = (self.left_ticks - self.prev_left_ticks) * m_per_tick
        d_right = (self.right_ticks - self.prev_right_ticks) * m_per_tick
        self.prev_left_ticks = self.left_ticks
        self.prev_right_ticks = self.right_ticks
        v_left = d_left / dt
        v_right = d_right / dt

        max_speed = self.get_parameter('max_wheel_speed').value
        ff_left = 100.0 * self.target_left / max_speed
        ff_right = 100.0 * self.target_right / max_speed
        duty_left = ff_left + self.pid_left.step(self.target_left - v_left, dt)
        duty_right = ff_right + self.pid_right.step(self.target_right - v_right, dt)
        self._set_motor('a', duty_left)
        self._set_motor('b', duty_right)
        self.left_dir = 1 if duty_left >= 0 else -1
        self.right_dir = 1 if duty_right >= 0 else -1

        # Odometry integration (differential drive kinematics, Section 3.3)
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.get_parameter('wheel_separation').value
        self.x += d_center * math.cos(self.theta + d_theta / 2.0)
        self.y += d_center * math.sin(self.theta + d_theta / 2.0)
        self.theta = math.atan2(math.sin(self.theta + d_theta),
                                math.cos(self.theta + d_theta))
        self.publish_odometry(now, d_center / dt, d_theta / dt)

    def _set_motor(self, channel, duty):
        duty = max(-100.0, min(100.0, duty))
        gpio = self.gpio
        if channel == 'a':
            gpio.output(self.pins['in1_pin'], duty >= 0)
            gpio.output(self.pins['in2_pin'], duty < 0)
            self.pwm_a.ChangeDutyCycle(abs(duty))
        else:
            gpio.output(self.pins['in3_pin'], duty >= 0)
            gpio.output(self.pins['in4_pin'], duty < 0)
            self.pwm_b.ChangeDutyCycle(abs(duty))

    def publish_odometry(self, now, v, w):
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)

    def destroy_node(self):
        try:
            self.pwm_a.stop()
            self.pwm_b.stop()
            self.gpio.cleanup()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = L298nDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

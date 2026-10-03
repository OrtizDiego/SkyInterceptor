#!/usr/bin/env python3

"""
Keyboard teleoperation for the simulated quadrotor.

Publishes velocity setpoints (geometry_msgs/Twist, heading frame: x forward,
y left, z up, angular.z yaw rate) on /cmd_vel at a fixed rate and arms or
disarms the motors through /drone/arm. The flight controller in the Gazebo
plugin turns the setpoints into rotor speeds.

Each key press changes the setpoint by one step and the setpoint is held
until changed, so the drone keeps flying when the key is released. Space
stops all motion and hovers in place.
"""

import os
import select
import sys
import termios
import threading
import tty

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String

HELP = """
SkyInterceptor drone teleop
---------------------------
  Arrow up / down       forward / back        (or i / k)
  Arrow left / right    left / right          (or j / l)
  w / s                 climb / descend (thrust up / down)
  a / d                 yaw left / right
  space or h            stop: hover in place
  t                     arm motors: they idle until you climb with w
  x                     disarm motors (kills them in the air too)
  q or Ctrl-C           quit (the drone hovers)

Each press changes the speed by one step; it is held until you change it.
"""

# key -> (axis, direction)
MOTION_KEYS = {
    'UP': ('x', 1.0), 'i': ('x', 1.0),
    'DOWN': ('x', -1.0), 'k': ('x', -1.0),
    'LEFT': ('y', 1.0), 'j': ('y', 1.0),
    'RIGHT': ('y', -1.0), 'l': ('y', -1.0),
    'w': ('z', 1.0), 's': ('z', -1.0),
    'a': ('yaw', 1.0), 'd': ('yaw', -1.0),
}

ARROWS = {'A': 'UP', 'B': 'DOWN', 'C': 'RIGHT', 'D': 'LEFT'}


def read_key(fd, timeout):
    """Return one key ('UP', 'a', ' ', ...) or None if nothing arrives in time."""
    ready, _, _ = select.select([fd], [], [], timeout)
    if not ready:
        return None
    key = os.read(fd, 1).decode(errors='ignore')
    if key != '\x1b':
        return key
    # Escape sequence: arrows are ESC [ A..D (or ESC O A..D)
    seq = ''
    while len(seq) < 2 and select.select([fd], [], [], 0.02)[0]:
        seq += os.read(fd, 1).decode(errors='ignore')
    if len(seq) == 2 and seq[0] in '[O':
        return ARROWS.get(seq[1])
    return None


class DroneTeleop(Node):
    """Turns key presses into velocity setpoints and arm commands."""

    def __init__(self):
        super().__init__('drone_teleop_keyboard')
        self.declare_parameter('linear_step', 0.5)
        self.declare_parameter('vertical_step', 0.25)
        self.declare_parameter('yaw_step', 0.2)
        self.declare_parameter('max_linear_speed', 5.0)
        self.declare_parameter('max_vertical_speed', 2.0)
        self.declare_parameter('max_yaw_rate', 1.2)
        self.declare_parameter('publish_rate', 20.0)

        param = self.get_parameter
        self.steps = {
            'x': param('linear_step').value,
            'y': param('linear_step').value,
            'z': param('vertical_step').value,
            'yaw': param('yaw_step').value,
        }
        self.limits = {
            'x': param('max_linear_speed').value,
            'y': param('max_linear_speed').value,
            'z': param('max_vertical_speed').value,
            'yaw': param('max_yaw_rate').value,
        }
        self.setpoint = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
        self.lock = threading.Lock()
        self.status = 'UNKNOWN'
        self.altitude = 0.0
        self.speed = 0.0

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.arm_pub = self.create_publisher(Bool, 'drone/arm', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'drone/status', self.on_status, latched)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_timer(1.0 / param('publish_rate').value, self.publish_setpoint)

    def on_status(self, msg):
        self.status = msg.data

    def on_odom(self, msg):
        self.altitude = msg.pose.pose.position.z
        v = msg.twist.twist.linear
        self.speed = (v.x ** 2 + v.y ** 2 + v.z ** 2) ** 0.5

    def publish_setpoint(self):
        twist = Twist()
        with self.lock:
            twist.linear.x = self.setpoint['x']
            twist.linear.y = self.setpoint['y']
            twist.linear.z = self.setpoint['z']
            twist.angular.z = self.setpoint['yaw']
        self.cmd_pub.publish(twist)

    def stop(self):
        with self.lock:
            for axis in self.setpoint:
                self.setpoint[axis] = 0.0
        self.publish_setpoint()

    def arm(self, value):
        if value:
            self.stop()
        self.arm_pub.publish(Bool(data=value))

    def handle_key(self, key):
        """Apply one key press. Return False when the user wants to quit."""
        if key in MOTION_KEYS:
            axis, direction = MOTION_KEYS[key]
            with self.lock:
                value = self.setpoint[axis] + direction * self.steps[axis]
                limit = self.limits[axis]
                # round() keeps repeated steps from accumulating float noise around 0
                self.setpoint[axis] = round(max(-limit, min(limit, value)), 3)
        elif key in (' ', 'h'):
            self.stop()
        elif key == 't':
            self.arm(True)
        elif key == 'x':
            self.arm(False)
        elif key in ('q', '\x03'):
            return False
        return True

    def status_line(self):
        with self.lock:
            sp = dict(self.setpoint)
        return (
            f'[{self.status:8s}] fwd {sp["x"]:+.2f}  left {sp["y"]:+.2f}  up {sp["z"]:+.2f} m/s  '
            f'yaw {sp["yaw"]:+.2f} rad/s | alt {self.altitude:6.2f} m  '
            f'speed {self.speed:5.2f} m/s')


def main(args=None):
    if not sys.stdin.isatty():
        print('drone_teleop_keyboard needs an interactive terminal (run it with ros2 run).')
        return 1

    # Keep Ctrl-C as a KeyboardInterrupt so the final hover setpoint still goes out
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = DroneTeleop()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    print(HELP)
    try:
        tty.setcbreak(fd)
        running = True
        while running and rclpy.ok():
            key = read_key(fd, 0.1)
            if key is not None:
                running = node.handle_key(key)
            sys.stdout.write('\r' + node.status_line() + '\x1b[K')
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        # Zero the setpoint so the drone hovers rather than flying off with the last one
        node.stop()
        print('\nTeleop stopped, setpoint zeroed.')
        # Stop the spin thread before tearing down the node and the context
        executor.shutdown()
        spinner.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

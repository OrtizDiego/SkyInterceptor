#!/usr/bin/env python3
"""
Record the interceptor/target geometry during an intercept demo.

Subscribes to /odom, /target_drone/odom and /guidance/command, and prints one
line per sample: the range, the closing speed, the relative speed and the phase.
Used to check the intercept from the outside, independently of the guidance
node's own telemetry.

    ros2 run interceptor_drone record_intercept
"""

import math
import sys

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from interceptor_interfaces.msg import GuidanceCommand


class Recorder(Node):
    def __init__(self):
        super().__init__('intercept_recorder')
        self.interceptor = None
        self.target = None
        self.cmd = None
        self.t0 = None
        self.create_subscription(Odometry, '/odom', self.on_interceptor, 10)
        self.create_subscription(Odometry, '/target_drone/odom', self.on_target, 10)
        self.create_subscription(GuidanceCommand, '/guidance/command', self.on_cmd, 10)

    def on_interceptor(self, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.interceptor = (p.x, p.y, p.z, v.x, v.y, v.z)

    def on_target(self, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.target = (p.x, p.y, p.z, v.x, v.y, v.z)

    def on_cmd(self, msg):
        self.cmd = msg
        if self.interceptor is None or self.target is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0
        ip = self.interceptor[:3]
        tp = self.target[:3]
        iv = self.interceptor[3:]
        tv = self.target[3:]
        rx, ry, rz = (tp[0] - ip[0], tp[1] - ip[1], tp[2] - ip[2])
        rng = math.sqrt(rx * rx + ry * ry + rz * rz)
        rvx, rvy, rvz = (tv[0] - iv[0], tv[1] - iv[1], tv[2] - iv[2])
        rel_speed = math.sqrt(rvx * rvx + rvy * rvy + rvz * rvz)
        closing = -(rx * rvx + ry * rvy + rz * rvz) / rng if rng > 1e-6 else 0.0
        print(
            f'{t:7.2f} range={rng:7.2f} closing={closing:6.2f} '
            f'rel_speed={rel_speed:6.2f} phase={msg.status_message:8s} '
            f'zem={msg.miss_distance:6.2f} tgo={msg.time_to_go:6.2f}',
            flush=True)


def main():
    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())

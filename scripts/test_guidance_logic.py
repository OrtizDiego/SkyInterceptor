#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3, Quaternion
from nav_msgs.msg import Odometry
from interceptor_interfaces.msg import TargetState, GuidanceCommand
import math
import time

class GuidanceLogicTester(Node):
    def __init__(self):
        super().__init__('guidance_logic_tester')
        
        # Publishers
        self.target_pub = self.create_publisher(TargetState, '/target/state', 10)
        self.odom_pub = self.create_publisher(Odometry, '/ground_truth/odom', 10)
        
        # Subscriber to monitor guidance output
        self.command_sub = self.create_subscription(
            GuidanceCommand, 
            '/guidance/command', 
            self.command_callback, 
            10)
            
        self.timer = self.create_timer(0.1, self.timer_callback) # 10Hz
        
        self.start_time = time.time()
        self.test_phase = 0 # 0: Far, 1: Mid, 2: Terminal
        self.received_commands = 0
        
        print("\n" + "="*50)
        print("PHASE 4: GUIDANCE LOGIC TESTER")
        print("="*50)
        print("Testing transitions: Far (>50m) -> Mid (10-50m) -> Terminal (<10m)")
        print("-"*50)

    def timer_callback(self):
        t = time.time() - self.start_time
        
        # 1. Define Interceptor Odom (Stationary at origin)
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.pose.pose.position = Point(x=0.0, y=0.0, z=10.0)
        odom.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.odom_pub.publish(odom)
        
        # 2. Define Target State (Approaching from far)
        # We decrease distance over time to trigger phase switches
        distance = 70.0 - (t * 5.0) # Starts at 70m, closes at 5m/s
        if distance < 2.0: distance = 2.0
        
        target = TargetState()
        target.header.stamp = odom.header.stamp
        target.header.frame_id = 'map'
        target.is_valid = True
        
        # Target position (closing in on X axis)
        target.position = Point(x=distance, y=5.0, z=10.0)
        # Target velocity (moving toward origin + slight weave)
        target.velocity = Vector3(x=-5.0, y=2.0 * math.sin(t), z=0.0)
        # Target acceleration (weaving)
        target.acceleration = Vector3(x=0.0, y=2.0 * math.cos(t), z=0.0)
        
        self.target_pub.publish(target)

    def command_callback(self, msg):
        self.received_commands += 1
        
        # Calculate range for logging
        # (Assuming interceptor is at 0,0,10 and target is at dist, 5, 10)
        # Note: We can't know the exact range here without state, 
        # but we can infer it from the message data if we wanted.
        
        mode_str = "UNKNOWN"
        if msg.mode == 0: mode_str = "PN (FAR)"
        elif msg.mode == 1: mode_str = "APN (MID)"
        elif msg.mode == 3: mode_str = "TERMINAL"
        
        if self.received_commands % 10 == 0:
            print(f"Time: {time.time() - self.start_time:4.1f}s | "
                  f"N': {msg.navigation_constant:3.1f} | "
                  f"Mode: {mode_str:10} | "
                  f"Miss: {msg.miss_distance:4.2f}m | "
                  f"Acc: [{msg.acceleration_cmd.x:5.1f}, {msg.acceleration_cmd.y:5.1f}, {msg.acceleration_cmd.z:5.1f}]")

def main(args=None):
    rclpy.init(args=args)
    node = GuidanceLogicTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

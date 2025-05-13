#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import sys

class HarmoniumVelocityTest(Node):
    def __init__(self):
        super().__init__('harmonium_velocity_test')
        
        # Create publishers for Gazebo Harmonium-specific topics
        self.cmd_vel_pub = self.create_publisher(Twist, '/model/robocar/cmd_vel', 10)
        
        self.get_logger().info('Harmonium velocity test started!')
        self.get_logger().info('This will publish velocities directly to /model/robocar/cmd_vel')
        self.get_logger().info('Press Ctrl+C to stop at any time')
        
        self.speed = 0.0

    def run_test(self):
        while True:
            # Create test message
            twist = Twist()
            
            # Alternate between different movements
            if self.speed < 0.5:
                self.speed += 0.1
                twist.linear.x = self.speed
                self.get_logger().info(f'FORWARD: {self.speed:.1f} m/s')
            elif self.speed < 1.0:
                self.speed += 0.1
                twist.angular.z = 0.5
                self.get_logger().info(f'TURN LEFT at {self.speed:.1f} m/s')
            elif self.speed < 1.5:
                self.speed += 0.1
                twist.linear.x = -0.2
                self.get_logger().info(f'BACKWARD at {self.speed:.1f} m/s')
            else:
                self.speed = 0.0
                twist.angular.z = -0.5
                self.get_logger().info('TURN RIGHT')
            
            # Publish to Gazebo Harmonium specific topic
            self.cmd_vel_pub.publish(twist)
            
            # Wait
            time.sleep(1.0)
            
            # Stop
            stop_twist = Twist()
            self.cmd_vel_pub.publish(stop_twist)
            self.get_logger().info('STOP')
            time.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)
    tester = HarmoniumVelocityTest()
    
    try:
        tester.run_test()
    except KeyboardInterrupt:
        tester.get_logger().info('Test stopped by user')
    finally:
        # Make sure we stop the robot
        stop_cmd = Twist()
        tester.cmd_vel_pub.publish(stop_cmd)
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
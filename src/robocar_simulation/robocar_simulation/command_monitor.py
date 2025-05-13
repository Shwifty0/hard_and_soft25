#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CommandMonitor(Node):
    def __init__(self):
        super().__init__('command_monitor')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.get_logger().info('Command monitor started. Listening to cmd_vel topic...')
        
        # Also try with the leading slash version
        self.subscription_alt = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_alt_callback,
            10)
        
        # Monitor what happens to the command after it goes through the bridge
        # This might not work directly, but it's worth trying
        self.subscription_ignition = self.create_subscription(
            Twist,
            '/model/robocar/cmd_vel',
            self.cmd_vel_ignition_callback,
            10)
        
        # Periodically emit a diagnostic message to show we're running
        self.timer = self.create_timer(5.0, self.diagnostic_callback)
    
    def cmd_vel_callback(self, msg):
        self.get_logger().info(
            f'cmd_vel received: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}')
    
    def cmd_vel_alt_callback(self, msg):
        self.get_logger().info(
            f'/cmd_vel received: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}')
    
    def cmd_vel_ignition_callback(self, msg):
        self.get_logger().info(
            f'/model/robocar/cmd_vel received: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}')
    
    def diagnostic_callback(self):
        self.get_logger().info('Monitor is still running...')


def main(args=None):
    rclpy.init(args=args)
    node = CommandMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
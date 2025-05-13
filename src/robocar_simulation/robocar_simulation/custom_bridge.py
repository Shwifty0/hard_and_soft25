#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import subprocess
import threading
import json

class CustomBridge(Node):
    def __init__(self):
        super().__init__('custom_bridge')
        # Subscribe to ROS 2 cmd_vel topics
        self.ros_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.ros_sub_alt = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.get_logger().info('Custom bridge started')
        self.get_logger().info('Forwarding from ROS 2 cmd_vel topics to Gazebo /model/robocar/cmd_vel')
        
        # Lock for thread safety
        self.lock = threading.Lock()
        
    def cmd_vel_callback(self, msg):
        # Forward message to Gazebo using gz command
        with self.lock:
            linear_x = msg.linear.x
            linear_y = msg.linear.y
            linear_z = msg.linear.z
            angular_x = msg.angular.x
            angular_y = msg.angular.y
            angular_z = msg.angular.z
        
        self.get_logger().info(f'Forwarding: linear=({linear_x}, {linear_y}, {linear_z}), angular=({angular_x}, {angular_y}, {angular_z})')
        
        # Format the Gazebo message
        gz_msg = f"linear: {{x: {linear_x}, y: {linear_y}, z: {linear_z}}}, angular: {{x: {angular_x}, y: {angular_y}, z: {angular_z}}}"
        
        # Forward to Gazebo
        try:
            cmd = ["gz", "topic", "-t", "/model/robocar/cmd_vel", "-m", "gz.msgs.Twist", "-p", gz_msg]
            self.get_logger().info(f"Running: {' '.join(cmd)}")
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode != 0:
                self.get_logger().error(f"Error: {result.stderr}")
            else:
                self.get_logger().info("Successfully forwarded to Gazebo")
        except Exception as e:
            self.get_logger().error(f"Exception: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CustomBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
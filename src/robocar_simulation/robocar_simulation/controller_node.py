#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import Float32, Bool
import math

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        # Create publisher for robot movement
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Create subscribers for each sensor
        self.ultrasonic_sub = self.create_subscription(
            Range, 'ultrasonic_sensor', self.ultrasonic_callback, 10)
        self.magnetic_sub = self.create_subscription(
            Float32, 'magnetic_sensor', self.magnetic_callback, 10)
        self.vibration1_sub = self.create_subscription(
            Bool, 'vibration_sensor1', self.vibration1_callback, 10)
        self.vibration2_sub = self.create_subscription(
            Bool, 'vibration_sensor2', self.vibration2_callback, 10)
        self.alcohol_sub = self.create_subscription(
            Float32, 'alcohol_sensor', self.alcohol_callback, 10)
        
        # Initialize sensor values
        self.ultrasonic_distance = 999.0
        self.magnetic_value = 0.0
        self.vibration1_detected = False
        self.vibration2_detected = False
        self.alcohol_detected = 0.0
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Controller node started')
    
    def ultrasonic_callback(self, msg):
        self.ultrasonic_distance = msg.range
    
    def magnetic_callback(self, msg):
        self.magnetic_value = msg.data
    
    def vibration1_callback(self, msg):
        self.vibration1_detected = msg.data
    
    def vibration2_callback(self, msg):
        self.vibration2_detected = msg.data
    
    def alcohol_callback(self, msg):
        self.alcohol_detected = msg.data
    
    def control_loop(self):
        # Simple obstacle avoidance logic
        cmd = Twist()
        
        # Log sensor values
        self.get_logger().info(f'Sensors: US={self.ultrasonic_distance:.2f}m, ' +
                              f'Mag={self.magnetic_value:.2f}, ' +
                              f'Vib1={self.vibration1_detected}, ' +
                              f'Vib2={self.vibration2_detected}, ' +
                              f'Alcohol={self.alcohol_detected:.2f}')
        
        # Simple obstacle avoidance
        if self.ultrasonic_distance < 0.3:
            # Obstacle detected - turn
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Turn left
        else:
            # Move forward
            cmd.linear.x = 0.2
            cmd.angular.z = 0.0
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
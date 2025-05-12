#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
import time
import datetime
import json
import math

class LoggerNode(Node):
    def __init__(self):
        super().__init__('logger_node')
        
        # Subscribers for all sensor data
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
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Initialize data
        self.sensor_data = {
            'timestamp': '',
            'ultrasonic_distance': 0.0,
            'magnetic_value': 0.0,
            'vibration1': False,
            'vibration2': False,
            'alcohol': 0.0,
            'linear_velocity': 0.0,
            'angular_velocity': 0.0,
            'estimated_position_x': 0.0,
            'estimated_position_y': 0.0,
            'estimated_heading': 0.0
        }
        
        # Simple position estimation based on velocity commands
        self.position_x = 0.0
        self.position_y = 0.0
        self.heading = 0.0  # radians
        self.last_update_time = time.time()
        
        # Timer for logging
        self.timer = self.create_timer(0.5, self.log_data)
        
        self.get_logger().info('Logger node started')
    
    def ultrasonic_callback(self, msg):
        self.sensor_data['ultrasonic_distance'] = msg.range
    
    def magnetic_callback(self, msg):
        self.sensor_data['magnetic_value'] = msg.data
    
    def vibration1_callback(self, msg):
        self.sensor_data['vibration1'] = msg.data
    
    def vibration2_callback(self, msg):
        self.sensor_data['vibration2'] = msg.data
    
    def alcohol_callback(self, msg):
        self.sensor_data['alcohol'] = msg.data
    
    def cmd_vel_callback(self, msg):
        self.sensor_data['linear_velocity'] = msg.linear.x
        self.sensor_data['angular_velocity'] = msg.angular.z
        
        # Update position estimation
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # Very simple dead reckoning
        self.heading += msg.angular.z * dt
        self.position_x += msg.linear.x * dt * math.cos(self.heading)
        self.position_y += msg.linear.x * dt * math.sin(self.heading)
        
        self.sensor_data['estimated_position_x'] = self.position_x
        self.sensor_data['estimated_position_y'] = self.position_y
        self.sensor_data['estimated_heading'] = self.heading
    
    def log_data(self):
        self.sensor_data['timestamp'] = datetime.datetime.now().isoformat()
        
        # Log to console
        self.get_logger().info(f"Log: {json.dumps(self.sensor_data)}")
        
        # In the future, you can add code to stream this data to a web platform


def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
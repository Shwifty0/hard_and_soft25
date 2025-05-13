#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
import datetime
import json
import math
import os
import time
import threading

class LoggerNode(Node):
    def __init__(self):
        super().__init__('logger_node')
        
        # Create log directory if it doesn't exist
        self.log_dir = 'robocar_logs'
        os.makedirs(self.log_dir, exist_ok=True)
        self.log_file = f"{self.log_dir}/robocar_log_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
        
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
        
        # Lock for thread-safe access to sensor_data
        self.lock = threading.Lock()
        
        # Simple position estimation based on velocity commands
        self.position_x = 0.0
        self.position_y = 0.0
        self.heading = 0.0  # radians
        self.last_update_time = self.get_clock().now()
        
        # Timer for logging
        self.timer = self.create_timer(0.5, self.log_data)
        
        # Log header
        with open(self.log_file, 'w') as f:
            f.write("RoboCar Sensor Log\n")
            f.write(f"Started at: {datetime.datetime.now().isoformat()}\n")
            f.write("=" * 80 + "\n")
        
        self.get_logger().info(f'Logger node started. Logs will be saved to {self.log_file}')
    
    def ultrasonic_callback(self, msg):
        with self.lock:
            self.sensor_data['ultrasonic_distance'] = msg.range
    
    def magnetic_callback(self, msg):
        with self.lock:
            self.sensor_data['magnetic_value'] = msg.data
    
    def vibration1_callback(self, msg):
        with self.lock:
            self.sensor_data['vibration1'] = msg.data
    
    def vibration2_callback(self, msg):
        with self.lock:
            self.sensor_data['vibration2'] = msg.data
    
    def alcohol_callback(self, msg):
        with self.lock:
            self.sensor_data['alcohol'] = msg.data
    
    def cmd_vel_callback(self, msg):
        with self.lock:
            self.sensor_data['linear_velocity'] = msg.linear.x
            self.sensor_data['angular_velocity'] = msg.angular.z
            
            # Update position estimation
            current_time = self.get_clock().now()
            dt = (current_time - self.last_update_time).nanoseconds / 1e9  # Convert to seconds
            self.last_update_time = current_time
            
            # Very simple dead reckoning
            self.heading += msg.angular.z * dt
            self.position_x += msg.linear.x * dt * math.cos(self.heading)
            self.position_y += msg.linear.x * dt * math.sin(self.heading)
            
            self.sensor_data['estimated_position_x'] = self.position_x
            self.sensor_data['estimated_position_y'] = self.position_y
            self.sensor_data['estimated_heading'] = self.heading
    
    def log_data(self):
        with self.lock:
            self.sensor_data['timestamp'] = datetime.datetime.now().isoformat()
            data_copy = self.sensor_data.copy()
        
        # Format log entry
        log_entry = f"[{data_copy['timestamp']}] "
        log_entry += f"POS: ({data_copy['estimated_position_x']:.2f}, {data_copy['estimated_position_y']:.2f}, " 
        log_entry += f"{math.degrees(data_copy['estimated_heading']):.1f}°) | "
        log_entry += f"VEL: linear={data_copy['linear_velocity']:.2f}, angular={data_copy['angular_velocity']:.2f} | "
        log_entry += f"US: {data_copy['ultrasonic_distance']:.2f}m | "
        log_entry += f"MAG: {data_copy['magnetic_value']:.2f} | "
        log_entry += f"VIB: {'YES' if data_copy['vibration1'] or data_copy['vibration2'] else 'NO'} | "
        log_entry += f"ALC: {data_copy['alcohol']:.2f}"
        
        # Log to console with color-coding for easier reading
        ultrasonic_color = "\033[33m" if data_copy['ultrasonic_distance'] < 0.3 else "\033[0m"  # Yellow if close
        magnetic_color = "\033[34m" if data_copy['magnetic_value'] > 2.0 else "\033[0m"  # Blue if significant
        vibration_color = "\033[35m" if data_copy['vibration1'] or data_copy['vibration2'] else "\033[0m"  # Purple if detected
        alcohol_color = "\033[32m" if data_copy['alcohol'] > 1.0 else "\033[0m"  # Green if detected
        
        console_log = f"[{data_copy['timestamp']}] Position: ({data_copy['estimated_position_x']:.2f}, {data_copy['estimated_position_y']:.2f})\n"
        console_log += f"  Sensors:"
        console_log += f" {ultrasonic_color}US: {data_copy['ultrasonic_distance']:.2f}m\033[0m |"
        console_log += f" {magnetic_color}MAG: {data_copy['magnetic_value']:.2f}\033[0m |"
        console_log += f" {vibration_color}VIB: {'YES' if data_copy['vibration1'] or data_copy['vibration2'] else 'NO'}\033[0m |"
        console_log += f" {alcohol_color}ALC: {data_copy['alcohol']:.2f}\033[0m"
        
        self.get_logger().info(console_log)
        
        # Log to file
        with open(self.log_file, 'a') as f:
            f.write(log_entry + "\n")
        
        # In the future, you can add code to stream this data to a web platform
        # For example, using a webhook to send data to a web service
        # Or using a simple web server to provide data via REST API
        # self.send_to_web_platform(data_copy)
    
    # Placeholder for web platform integration
    def send_to_web_platform(self, data):
        # This would be implemented to send data to your web platform
        pass


def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
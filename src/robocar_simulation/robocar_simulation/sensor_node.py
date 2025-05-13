#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist  # Move import to the top
import time
import math
import random

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        
        # Create publishers for each sensor
        self.ultrasonic_pub = self.create_publisher(Range, 'ultrasonic_sensor', 10)
        self.magnetic_pub = self.create_publisher(Float32, 'magnetic_sensor', 10)
        self.vibration1_pub = self.create_publisher(Bool, 'vibration_sensor1', 10)
        self.vibration2_pub = self.create_publisher(Bool, 'vibration_sensor2', 10)
        self.alcohol_pub = self.create_publisher(Float32, 'alcohol_sensor', 10)
        
        # Initialize sensor states
        self.ultrasonic_distance = 1.0  # Initial distance in meters
        self.magnetic_field = 0.0  # Initial magnetic field
        self.vibration1_active = False
        self.vibration2_active = False
        self.alcohol_level = 0.0
        
        # Simulated environmental variables
        self.magnets_positions = [(1.8, 1.99)]  # (x, y) positions of magnets in the maze
        self.alcohol_zones = [(1.2, 1.5)]  # (x, y) positions of alcohol containers
        self.bump_positions = [(0.9, 1.3)]  # (x, y) positions of speed bumps for vibration
        
        # Subscribe to robot position (for simulating sensor readings based on position)
        # In a real implementation, this would come from odometry or localization
        # For now, we'll use a simple timer to simulate movement and sensor readings
        
        # Timer for sensor simulation
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Current simulated position (starts at the beginning of maze)
        self.current_pos_x = 1.0
        self.current_pos_y = 0.1
        self.current_heading = 0.0  # Radians, 0 is pointing along positive x-axis
        
        # Subscribe to cmd_vel for updating position based on movement commands
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        self.last_update_time = self.get_clock().now()
        
        self.get_logger().info('Sensor node started')
    
    def cmd_vel_callback(self, msg):
        # Update simulated position based on velocity commands
        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds / 1e9  # Convert to seconds
        self.last_update_time = current_time
        
        # Simple dead reckoning
        self.current_heading += msg.angular.z * dt
        self.current_pos_x += msg.linear.x * dt * math.cos(self.current_heading)
        self.current_pos_y += msg.linear.x * dt * math.sin(self.current_heading)
        
        # Log position for debugging
        if dt > 0.01:  # Only log significant movements
            self.get_logger().info(f'Position: x={self.current_pos_x:.2f}, y={self.current_pos_y:.2f}, ' +
                                  f'heading={self.current_heading:.2f}')
    
    def timer_callback(self):
        # Simulate ultrasonic sensor based on position in the maze
        # In a real implementation, we would use raycasting on the maze geometry
        # For this simulation, we'll use random noise for simplicity
        ultrasonic_msg = Range()
        ultrasonic_msg.header.stamp = self.get_clock().now().to_msg()
        ultrasonic_msg.header.frame_id = 'ultrasonic_sensor'
        ultrasonic_msg.radiation_type = Range.ULTRASOUND
        ultrasonic_msg.field_of_view = 0.3
        ultrasonic_msg.min_range = 0.02
        ultrasonic_msg.max_range = 4.0
        
        # Simple simulation - random noise around a baseline with occasional obstacles
        baseline_distance = 1.0
        noise = random.uniform(-0.05, 0.05)
        obstacle_chance = random.random()
        if obstacle_chance < 0.2:  # 20% chance of detecting an obstacle
            obstacle_distance = random.uniform(0.2, 0.5)
            self.ultrasonic_distance = obstacle_distance
        else:
            self.ultrasonic_distance = baseline_distance + noise
        
        ultrasonic_msg.range = self.ultrasonic_distance
        self.ultrasonic_pub.publish(ultrasonic_msg)
        
        # Simulate magnetic sensor based on proximity to magnets
        magnetic_msg = Float32()
        min_distance_to_magnet = float('inf')
        for mx, my in self.magnets_positions:
            distance = math.sqrt((self.current_pos_x - mx)**2 + (self.current_pos_y - my)**2)
            min_distance_to_magnet = min(min_distance_to_magnet, distance)
        
        # Magnetic field strength is inversely proportional to square of distance
        if min_distance_to_magnet < 0.5:  # Only detect magnets within 0.5m
            field_strength = 10.0 / (min_distance_to_magnet**2 + 0.1)  # Avoid division by zero
            self.magnetic_field = field_strength + random.uniform(-0.2, 0.2)  # Add noise
        else:
            self.magnetic_field = random.uniform(-0.2, 0.2)  # Background noise
        
        magnetic_msg.data = self.magnetic_field
        self.magnetic_pub.publish(magnetic_msg)
        
        # Simulate vibration sensors based on proximity to speed bumps
        for bx, by in self.bump_positions:
            distance = math.sqrt((self.current_pos_x - bx)**2 + (self.current_pos_y - by)**2)
            if distance < 0.1:  # Within 10cm of a speed bump
                # Simulate vibration based on robot speed (from cmd_vel)
                self.vibration1_active = random.random() < 0.8  # 80% chance of detecting vibration
                self.vibration2_active = random.random() < 0.7  # 70% chance, slightly different from sensor 1
            else:
                # Random chance of false positive
                self.vibration1_active = random.random() < 0.05  # 5% chance of false positive
                self.vibration2_active = random.random() < 0.05
        
        vibration1_msg = Bool()
        vibration1_msg.data = self.vibration1_active
        self.vibration1_pub.publish(vibration1_msg)
        
        vibration2_msg = Bool()
        vibration2_msg.data = self.vibration2_active
        self.vibration2_pub.publish(vibration2_msg)
        
        # Simulate alcohol sensor based on proximity to alcohol containers
        alcohol_msg = Float32()
        min_distance_to_alcohol = float('inf')
        for ax, ay in self.alcohol_zones:
            distance = math.sqrt((self.current_pos_x - ax)**2 + (self.current_pos_y - ay)**2)
            min_distance_to_alcohol = min(min_distance_to_alcohol, distance)
        
        # Alcohol reading is inversely proportional to distance
        if min_distance_to_alcohol < 0.3:  # Only detect alcohol within 30cm
            alcohol_reading = 5.0 / (min_distance_to_alcohol + 0.05)  # Higher when closer
            self.alcohol_level = alcohol_reading + random.uniform(-0.2, 0.2)  # Add noise
        else:
            self.alcohol_level = random.uniform(0.0, 0.2)  # Background noise
        
        alcohol_msg.data = self.alcohol_level
        self.alcohol_pub.publish(alcohol_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
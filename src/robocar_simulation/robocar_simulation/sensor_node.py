#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32, Bool
import time

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        
        # Create publishers for each sensor
        self.ultrasonic_pub = self.create_publisher(Range, 'ultrasonic_sensor', 10)
        self.magnetic_pub = self.create_publisher(Float32, 'magnetic_sensor', 10)
        self.vibration1_pub = self.create_publisher(Bool, 'vibration_sensor1', 10)
        self.vibration2_pub = self.create_publisher(Bool, 'vibration_sensor2', 10)
        self.alcohol_pub = self.create_publisher(Float32, 'alcohol_sensor', 10)
        
        # Timer for sensor simulation
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('Sensor node started')
    
    def timer_callback(self):
        # Simulate ultrasonic sensor
        ultrasonic_msg = Range()
        ultrasonic_msg.header.stamp = self.get_clock().now().to_msg()
        ultrasonic_msg.header.frame_id = 'ultrasonic_sensor'
        ultrasonic_msg.radiation_type = Range.ULTRASOUND
        ultrasonic_msg.field_of_view = 0.3
        ultrasonic_msg.min_range = 0.02
        ultrasonic_msg.max_range = 4.0
        ultrasonic_msg.range = 0.5  # Sample value
        self.ultrasonic_pub.publish(ultrasonic_msg)
        
        # Simulate magnetic sensor
        magnetic_msg = Float32()
        magnetic_msg.data = 0.0  # Sample value
        self.magnetic_pub.publish(magnetic_msg)
        
        # Simulate vibration sensors
        vibration1_msg = Bool()
        vibration1_msg.data = False  # Sample value
        self.vibration1_pub.publish(vibration1_msg)
        
        vibration2_msg = Bool()
        vibration2_msg.data = False  # Sample value
        self.vibration2_pub.publish(vibration2_msg)
        
        # Simulate alcohol sensor
        alcohol_msg = Float32()
        alcohol_msg.data = 0.0  # Sample value
        self.alcohol_pub.publish(alcohol_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
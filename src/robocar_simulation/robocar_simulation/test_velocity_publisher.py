#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.publish_velocity)
        self.get_logger().info('Velocity publisher started. Publishing forward movement every second...')
        self.counter = 0

    def publish_velocity(self):
        twist = Twist()
        
        # Alternate between different movements to test
        if self.counter % 4 == 0:
            twist.linear.x = 0.5  # Forward
            movement = "FORWARD"
        elif self.counter % 4 == 1:
            twist.linear.x = -0.5  # Backward
            movement = "BACKWARD"
        elif self.counter % 4 == 2:
            twist.angular.z = 0.5  # Left turn
            movement = "TURN LEFT"
        else:
            twist.angular.z = -0.5  # Right turn
            movement = "TURN RIGHT"
        
        self.publisher.publish(twist)
        self.get_logger().info(f'Published: {movement}')
        self.counter += 1
        
        # Send stop command after each movement
        time.sleep(0.5)
        stop_twist = Twist()
        self.publisher.publish(stop_twist)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
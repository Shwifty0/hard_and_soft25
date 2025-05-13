#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import select

class SimpleKeyboardController(Node):
    def __init__(self):
        super().__init__('simple_keyboard_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info("""
Simple Keyboard Control:
------------------------
Press keys to control the robot:
  w - move forward (0.2 m/s)
  s - move backward (0.2 m/s)
  a - turn left (0.5 rad/s)
  d - turn right (0.5 rad/s)
  space - stop
  q - quit
""")

    def getKey(self):
        # Save terminal settings
        old_attr = termios.tcgetattr(sys.stdin)
        try:
            # Set terminal to raw mode
            tty.setraw(sys.stdin.fileno())
            
            # Wait for keypress
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
            else:
                key = ''
                
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
            
        return key

    def run(self):
        # Main control loop
        while True:
            key = self.getKey()
            
            # Create a new Twist message
            twist = Twist()
            
            # Process key
            if key == 'w':
                twist.linear.x = 0.2
                self.get_logger().info('Forward')
            elif key == 's':
                twist.linear.x = -0.2
                self.get_logger().info('Backward')
            elif key == 'a':
                twist.angular.z = 0.5
                self.get_logger().info('Left')
            elif key == 'd':
                twist.angular.z = -0.5
                self.get_logger().info('Right')
            elif key == ' ':
                # Stop - all values default to 0
                self.get_logger().info('Stop')
            elif key == 'q':
                self.get_logger().info('Quitting')
                break
            elif key == '\x03':  # CTRL-C
                self.get_logger().info('CTRL-C detected, quitting')
                break
            
            # Publish command
            self.publisher.publish(twist)
            
            # Small delay to avoid flooding
            rclpy.spin_once(self, timeout_sec=0.1)

def main(args=None):
    rclpy.init(args=args)
    controller = SimpleKeyboardController()
    try:
        controller.run()
    except Exception as e:
        controller.get_logger().error(f'Error: {e}')
    finally:
        # Ensure we stop the robot before quitting
        stop_cmd = Twist()
        controller.publisher.publish(stop_cmd)
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
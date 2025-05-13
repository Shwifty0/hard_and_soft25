#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios
import subprocess
import threading

msg = """
RoboCar Keyboard Control
---------------------------
Moving around:
   w
a  s  d

w/s : increase/decrease linear velocity
a/d : increase/decrease angular velocity

CTRL-C to quit
"""

moveBindings = {
    'w': (0.1, 0.0),
    's': (-0.1, 0.0),
    'a': (0.0, 0.1),
    'd': (0.0, -0.1),
}

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.x = 0.0
        self.th = 0.0
        self.status = 0
        self.get_logger().info('Keyboard control node started')
        print(msg)
        
        # Add lock for thread safety when calling Gazebo
        self.lock = threading.Lock()

    def timer_callback(self):
        twist = Twist()
        twist.linear.x = self.x
        twist.angular.z = self.th
        self.publisher.publish(twist)
        
        # ADDITION: Also publish directly to Gazebo
        with self.lock:
            # Only forward if there's actual movement
            if self.x != 0.0 or self.th != 0.0:
                self.send_to_gazebo(self.x, self.th)

    def send_to_gazebo(self, linear_x, angular_z):
        # Format the Gazebo message
        gz_msg = f"linear: {{x: {linear_x}, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {angular_z}}}"
        
        # Forward to Gazebo
        try:
            cmd = ["gz", "topic", "-t", "/model/robocar/cmd_vel", "-m", "gz.msgs.Twist", "-p", gz_msg]
            subprocess.run(cmd, capture_output=True, text=True)
        except Exception as e:
            self.get_logger().error(f"Error forwarding to Gazebo: {e}")

    def getKey(self):
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        try:
            while True:
                key = self.getKey()
                if key in moveBindings.keys():
                    self.x += moveBindings[key][0]
                    self.th += moveBindings[key][1]
                    self.get_logger().info(f'Linear: {self.x}, Angular: {self.th}')
                elif key == '\x03':  # CTRL-C
                    break
        except Exception as e:
            self.get_logger().error(f'Exception: {e}')
        finally:
            # Stop the robot
            twist = Twist()
            self.publisher.publish(twist)
            # Also stop in Gazebo
            self.send_to_gazebo(0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
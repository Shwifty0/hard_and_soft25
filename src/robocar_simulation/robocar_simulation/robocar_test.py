#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class RoboCarTest(Node):
    def __init__(self):
        super().__init__('robocar_test')
        
        # Create publisher for cmd_vel (which is bridged to Gazebo)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Also create a direct publisher to Gazebo topic
        self.direct_pub = self.create_publisher(Twist, '/model/robocar/cmd_vel', 10)
        
        self.get_logger().info('RoboCar test started!')
        self.get_logger().info('This will publish test movement commands to RoboCar')
        
    def run_test(self):
        # First attempt through the bridge
        self.get_logger().info('Testing movement through ROS→Gazebo bridge...')
        self.test_sequence(self.cmd_vel_pub)
        
        time.sleep(1.0)
        
        # Then try direct publishing to Gazebo topic
        self.get_logger().info('Testing direct publishing to Gazebo topic...')
        self.test_sequence(self.direct_pub)
        
        self.get_logger().info('Test complete!')
    
    def test_sequence(self, publisher):
        # Test forward movement
        self.get_logger().info('Testing FORWARD movement...')
        self.publish_cmd(publisher, 0.5, 0.0, 2.0)
        
        # Test backward movement
        self.get_logger().info('Testing BACKWARD movement...')
        self.publish_cmd(publisher, -0.5, 0.0, 2.0)
        
        # Test left turn
        self.get_logger().info('Testing LEFT TURN...')
        self.publish_cmd(publisher, 0.0, 0.5, 2.0)
        
        # Test right turn
        self.get_logger().info('Testing RIGHT TURN...')
        self.publish_cmd(publisher, 0.0, -0.5, 2.0)
        
        # Test forward+left
        self.get_logger().info('Testing FORWARD+LEFT...')
        self.publish_cmd(publisher, 0.5, 0.5, 2.0)
        
        # Test forward+right
        self.get_logger().info('Testing FORWARD+RIGHT...')
        self.publish_cmd(publisher, 0.5, -0.5, 2.0)
        
        # Stop
        self.get_logger().info('STOPPING...')
        self.publish_cmd(publisher, 0.0, 0.0, 1.0)
    
    def publish_cmd(self, publisher, linear_x, angular_z, duration):
        # Create and publish the command
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        
        start_time = time.time()
        rate = self.create_rate(10)  # 10 Hz
        
        # Publish continuously for the specified duration
        while time.time() - start_time < duration:
            publisher.publish(twist)
            self.get_logger().info(f'Published: linear={linear_x}, angular={angular_z}')
            rate.sleep()
        
        # Stop after duration
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        publisher.publish(twist)
        self.get_logger().info('Published STOP command')

def main(args=None):
    rclpy.init(args=args)
    tester = RoboCarTest()
    
    try:
        tester.run_test()
    except KeyboardInterrupt:
        tester.get_logger().info('Test stopped by user')
    finally:
        # Make sure we stop the robot
        stop_cmd = Twist()
        tester.cmd_vel_pub.publish(stop_cmd)
        tester.direct_pub.publish(stop_cmd)
        tester.destroy_node()
        rclpy.shutdown()
    
    return 0

if __name__ == '__main__':
    main()
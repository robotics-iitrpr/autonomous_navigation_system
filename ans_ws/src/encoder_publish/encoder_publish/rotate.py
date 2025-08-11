#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class MotorCommandPublisher(Node):
    def __init__(self):
        super().__init__('motor_command_publisher')
        
        # Publishers for each motor
        self.publisher_A = self.create_publisher(String, '/motor_command_A', 10)
        self.publisher_B = self.create_publisher(String, '/motor_command_B', 10)
        self.publisher_C = self.create_publisher(String, '/motor_command_C', 10)

        # Timer to run at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_command)

        self.start_time = self.get_clock().now()
        self.duration = 3.0  # seconds

    def publish_command(self):
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9

        msg = String()
        if elapsed_time < self.duration:
            msg.data = 'F25'
            self.get_logger().info('Publishing F20 to all motors')
        else:
            msg.data = 'F0'
            self.get_logger().info('Publishing F0 to all motors (once)')

            # Publish F0 once, then shut down the node
            self.publisher_A.publish(msg)
            self.publisher_B.publish(msg)
            self.publisher_C.publish(msg)

            # Stop the timer to prevent further publishing
            self.timer.cancel()
            self.destroy_node()
            return

        # Publish F20 during first 5 seconds
        self.publisher_A.publish(msg)
        self.publisher_B.publish(msg)
        self.publisher_C.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotorCommandPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

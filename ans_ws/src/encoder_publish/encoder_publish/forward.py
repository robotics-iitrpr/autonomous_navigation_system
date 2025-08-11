#!/usr/bin/env python3

import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ForwardNode(Node):
    def __init__(self):
        super().__init__('forward_node')

        # Serial port setup
        self.serA = serial.Serial('/dev/arduino_A', 9600, timeout=0.5)
        self.serB = serial.Serial('/dev/arduino_B', 9600, timeout=0.5)
        self.serC = serial.Serial('/dev/arduino_C', 9600, timeout=0.5)        

        # Publishers only for motor A and motor B
        self.publisher_A = self.create_publisher(String, '/motor_command_A', 10)
        self.publisher_B = self.create_publisher(String, '/motor_command_B', 10)

        # Publish commands at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_command)

        self.start_time = self.get_clock().now()
        self.duration = 3.0  # seconds to move forward

    def publish_command(self):
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9

        # msg = String()
        if elapsed_time < self.duration:
            # msg.data = 'F25'  # Forward command at speed 25
            self.get_logger().info('Publishing F25 to motors A and B')
        else:
            # msg.data = 'F0'  # Stop command
            self.get_logger().info('Publishing F0 to motors A and B (once)')

            # Publish stop once
            # self.publisher_A.publish(msg)
            # self.publisher_B.publish(msg)
            self.serA.write(b'F0\n')
            self.serB.write(b'F0\n')


            # Stop the timer and shut down
            self.timer.cancel()
            self.destroy_node()
            return

        # Publish forward command repeatedly
        self.serA.write(b'B25\n')
        self.serB.write(b'F25\n')

def main(args=None):
    rclpy.init(args=args)
    node = ForwardNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

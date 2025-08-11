#!/usr/bin/env python3

import rclpy                                                                   ######################## LAUNCH FILE NEEDED #########################
from rclpy.node import Node                                                    
from std_msgs.msg import Int64
from std_msgs.msg import String
import serial
import re

class EncoderReaderNode(Node):
    def __init__(self):
        super().__init__('encoder_reader_node')

        # Serial port setup
        self.serA = serial.Serial('/dev/arduino_A', 9600, timeout=0.5)
        self.serB = serial.Serial('/dev/arduino_B', 9600, timeout=0.5)
        self.serC = serial.Serial('/dev/arduino_C', 9600, timeout=0.5)

        # Publisher
        self.pubA = self.create_publisher(Int64, 'raw_encoder_count_A', 10)
        self.pubB = self.create_publisher(Int64, 'raw_encoder_count_B', 10)
        self.pubC = self.create_publisher(Int64, 'raw_encoder_count_C', 10)

        self.motorA = self.create_subscription(String,'/motor_command_A',self.motor_command_callback_A,10)
        self.motorB = self.create_subscription(String,'/motor_command_B',self.motor_command_callback_B,10)
        self.motorC = self.create_subscription(String,'/motor_command_C',self.motor_command_callback_C,10)

                                                                                 ################## Timer to request encoder count every 100 ms
        self.timerA = self.create_timer(0.1, self.read_encoderA)
        self.timerB = self.create_timer(0.1, self.read_encoderB)
        self.timerC = self.create_timer(0.1, self.read_encoderC)
        # self.timer = self.create_timer(0.05, self.read_encoder)
        

    def read_encoderA(self):
        try:
            # Send the 'P' command to request encoder value
            self.serA.write(b'P\n')
            self.serA.flush()  # Ensure command is sent

            lineA = self.serA.readline().decode('utf-8').strip()

            # Parse line like: "Encoder Count: 1234"
            match = re.search(r'P:(-?\d+)', lineA)
            if match:
                count = int(match.group(1))
                msg = Int64()
                msg.data = count
                self.pubA.publish(msg)
                self.get_logger().info(f"Encoder A: {count}")

        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")

    def read_encoderB(self):
        try:
            # Send the 'P' command to request encoder value
            self.serB.write(b'P\n')
            self.serB.flush()  # Ensure command is sent

            lineB = self.serB.readline().decode('utf-8').strip()

            match = re.search(r'P:(-?\d+)', lineB)
            if match:
                count = int(match.group(1))
                msg = Int64()
                msg.data = count
                self.pubB.publish(msg)
                self.get_logger().info(f"Encoder B: {count}")

        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")

    def read_encoderC(self):
        try:
            # Send the 'P' command to request encoder value
            self.serC.write(b'P\n')
            self.serC.flush()  # Ensure command is sent

            lineC = self.serC.readline().decode('utf-8').strip()

            match = re.search(r'P:(-?\d+)', lineC)
            if match:
                count = int(match.group(1))
                msg = Int64()
                msg.data = count
                self.pubC.publish(msg)
                self.get_logger().info(f"Encoder C: {count}")

        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")
    
    def motor_command_callback_A(self,msg):
        self.serA.write(f"{msg.data}\n".encode('utf-8'))
        self.serA.flush()  # Ensure command is sent

    def motor_command_callback_B(self,msg):
        self.serB.write(f"{msg.data}\n".encode('utf-8'))
        self.serB.flush()  # Ensure command is sent

    def motor_command_callback_C(self,msg):
        self.serC.write(f"{msg.data}\n".encode('utf-8'))
        self.serC.flush()  # Ensure command is sent

def main(args=None):
    rclpy.init(args=args)
    node = EncoderReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        ()
    finally:
        node.serA.close()
        node.serB.close()
        node.serC.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

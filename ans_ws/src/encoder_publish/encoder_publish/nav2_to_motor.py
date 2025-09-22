#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math
from geometry_msgs.msg import Twist
import sys
import signal

# Distance from robot center to wheel (cm)
L = 13.6
# Speed scaling
SPEED = 150  # base motor command speed
ROT_SCALE = 0.1

class Nav2_to_motors(Node):
    def __init__(self):
        super().__init__('Nav2_to_motors')

        self.last_motor_commands = (0,0,0)

        self.pubA = self.create_publisher(String, '/motor_command_A', 10)
        self.pubB = self.create_publisher(String, '/motor_command_B', 10)
        self.pubC = self.create_publisher(String, '/motor_command_C', 10)

        self.sub = self.create_subscription(Twist,'/cmd_vel',self.cmd_vel_callback,10)

    # self.stopped = True
        self.get_logger().info("Nav2_to_motor started")

    def cmd_vel_callback(self,msg):
        vx = msg.linear.x
        vy = msg.linear.y

        omega = msg.angular.z

        wA = vx - (vy / math.sqrt(3)) - (L * omega * ROT_SCALE)
        wB = -vx - (vy / math.sqrt(3)) - (L * omega * ROT_SCALE)
        wC = (2 * vy / math.sqrt(3)) - (L * omega * ROT_SCALE)

        mA = int(wA * SPEED)
        mB = int(wB * SPEED)
        mC = int(wC * SPEED)

        if(mA > 30 or mB > 30 or mC > 30):
            return


        if(self.last_motor_commands != (mA,mB,mC)):
            self.send_motor_cmds(mA, mB, mC)
            self.last_motor_commands = (mA,mB,mC)


    def send_motor_cmds(self, mA, mB, mC):
        def cmd_str(val):
            if val > 0:
                return f"F{abs(val)}"
            elif val < 0:
                return f"B{abs(val)}"
            else:
                return "S"

        msgA = String(); msgA.data = cmd_str(mA)
        msgB = String(); msgB.data = cmd_str(mB)
        msgC = String(); msgC.data = cmd_str(mC)

        self.pubA.publish(msgA)
        self.pubB.publish(msgB)
        self.pubC.publish(msgC)

def main(args=None):
    rclpy.init(args=args)
    node = Nav2_to_motors()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("CTRL+C detected. Sending stop command.")
        node.send_motor_cmds(0, 0, 0)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

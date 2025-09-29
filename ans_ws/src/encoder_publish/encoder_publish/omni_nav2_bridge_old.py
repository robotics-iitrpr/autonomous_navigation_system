#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math

# Distance from robot center to wheel (meters)
SCALE = (30/math.pi)*1.275  # base motor command speed
SPEED = 1.5

def inverse_kinematics(Vx, Vy, omega, r, L):
    # Common terms
    sqrt3 = math.sqrt(3)

    # Inverse kinematics equations
    w1 = (1 / r) * (L * omega + Vy)
    w2 = (1 / r) * (L * omega - 0.5 * Vy - (sqrt3 / 2) * Vx)
    w3 = (1 / r) * (L * omega - 0.5 * Vy + (sqrt3 / 2) * Vx)

    return w1, w2, w3

class OmniNav2Bridge(Node):
    def __init__(self):
        super().__init__('omni_nav2_bridge')

        self.pubA = self.create_publisher(String, '/motor_command_A', 10)
        self.pubB = self.create_publisher(String, '/motor_command_B', 10)
        self.pubC = self.create_publisher(String, '/motor_command_C', 10)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.get_logger().info("Omni Nav2 Bridge started — listening to /cmd_vel")

        self.r = 0.052  # Wheel radius = 5.2 cm
        self.L = 0.136   # Distance from center to wheel = 13.6 cm

    def cmd_vel_callback(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z

        (wA,wB,wC) = inverse_kinematics(vx,vy,omega,self.r,self.L)

        mA = int(wA * SCALE)
        mB = int(wB * SCALE)
        mC = int(wC * SCALE)

        self.get_logger().info(f"mA : {mA}. mB : {mB}, mC : {mC}")

        # self.send_motor_cmds(mA, mB, mC)

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
    node = OmniNav2Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

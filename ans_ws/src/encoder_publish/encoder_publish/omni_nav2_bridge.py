#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math

# Distance from robot center to wheel (meters)
L = 0.136  # was cm, now in meters
# Speed scaling factor from m/s to your motor command units
SPEED = 20
ROT_SCALE = 0.1

class OmniNav2Bridge(Node):
    def __init__(self):
        super().__init__('omni_nav2_bridge')

        self.pubA = self.create_publisher(String, '/motor_command_A', 10)
        self.pubB = self.create_publisher(String, '/motor_command_B', 10)
        self.pubC = self.create_publisher(String, '/motor_command_C', 10)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.get_logger().info("Omni Nav2 Bridge started — listening to /cmd_vel")

    def cmd_vel_callback(self, msg: Twist):
        vx = msg.linear.x      # forward/back (m/s)
        vy = msg.linear.y      # left/right (m/s)
        omega = msg.angular.z  # rotation (rad/s)

        # Kinematic transformation for 3-wheel omni
        wA = vx - (vy / math.sqrt(3)) - (L * omega * ROT_SCALE)
        wB = -vx - (vy / math.sqrt(3)) - (L * omega * ROT_SCALE)
        wC = (2 * vy / math.sqrt(3)) - (L * omega * ROT_SCALE)

        mA = int(wA * SPEED)
        mB = int(wB * SPEED)
        mC = int(wC * SPEED)

        self.send_motor_cmds(mA, mB, mC)

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

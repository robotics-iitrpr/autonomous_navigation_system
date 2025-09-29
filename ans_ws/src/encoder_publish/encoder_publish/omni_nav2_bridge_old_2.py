#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math

# Distance from robot center to wheel (meters)
L = 13.6  # was cm, now in meters
# Speed scaling factor from m/s to your motor command units
SPEED = 10
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
        vx = -msg.linear.x    # forward/back (m/s)
        vy = -msg.linear.y    # left/right (m/s)
        omega = -msg.angular.z # rotation (rad/s)

        [mB,mA,mC] = self.cmd_vel_to_motors(vx,vy,omega)

        # self.get_logger().info(f"ma:{mA} , mb:{mB}, mc:{mC}")

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
    
    def cmd_vel_to_motors(self,vx, vy, omega, L=0.136, R=0.052):
        # Angles of wheels in radians (A=0°, B=120°, C=240°)
        theta = [0, 2*math.pi/3, 4*math.pi/3]  # 0, 120, 240 degrees

        # Calculate wheel speeds (rad/s)
        wheel_speeds = []
        for angle in theta:
            v_i = vx * math.sin(angle) - vy * math.cos(angle) + omega * L
            omega_i = v_i / R  # rad/s
            wheel_speeds.append(omega_i)

        # Normalize wheel speeds to -255 to 255 (motor command range)
        max_speed = max(abs(speed) for speed in wheel_speeds)
        if max_speed > 0:
            scale = 25 / max_speed
            wheel_cmds = [int(speed * scale) for speed in wheel_speeds]
        else:
            wheel_cmds = [0, 0, 0]

        return wheel_cmds

def main(args=None):
    rclpy.init(args=args)
    node = OmniNav2Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

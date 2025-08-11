#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys, termios, tty, select
import math
import time

# Distance from robot center to wheel (cm)
L = 13.6
# Speed scaling
SPEED = 20  # base motor command speed
ROT_SCALE = 0.1

def get_key(timeout=0.05):
    """Non-blocking key read."""
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1)
    return None

class OmniTeleopNode(Node):
    def __init__(self):
        super().__init__('omni_teleop_continuous')

        self.pubA = self.create_publisher(String, '/motor_command_A', 10)
        self.pubB = self.create_publisher(String, '/motor_command_B', 10)
        self.pubC = self.create_publisher(String, '/motor_command_C', 10)

        # Save terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        self.get_logger().info(
            "Omni teleop started:\n"
            "W/S forward/back, A/D strafe left/right, Q/E rotate left/right\n"
            "Hold keys to move, release to stop.\n"
            "Press Ctrl+C to exit."
        )

        vx = vy = omega = 0.0

        try:
            while rclpy.ok():
                key = get_key()

                if key:
                    # Reset velocities
                    vx, vy, omega = 0.0, 0.0, 0.0

                    if key == 'w':
                        vx = 1.0
                    elif key == 's':
                        vx = -1.0
                    elif key == 'a':
                        vy = 1.0
                    elif key == 'd':
                        vy = -1.0
                    elif key == 'q':
                        omega = 1.0
                    elif key == 'e':
                        omega = -1.0
                    elif key == '\x03':  # Ctrl+C
                        break

                    # Calculate wheel speeds
                    # Calculate wheel speeds with rotation scaled
                    wA = vx - (vy / math.sqrt(3)) - (L * omega * ROT_SCALE)
                    wB = -vx - (vy / math.sqrt(3)) - (L * omega * ROT_SCALE)
                    wC = (2 * vy / math.sqrt(3)) - (L * omega * ROT_SCALE)

                    mA = int(wA * SPEED)
                    mB = int(wB * SPEED)
                    mC = int(wC * SPEED)

                    self.send_motor_cmds(mA, mB, mC)

                else:
                    # No key pressed → stop motors
                    self.send_motor_cmds(0, 0, 0)

                time.sleep(0.001)

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            self.send_motor_cmds(0, 0, 0)

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
    OmniTeleopNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

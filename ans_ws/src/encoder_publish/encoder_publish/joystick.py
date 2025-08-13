#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import math
import time

# Distance from robot center to wheel (cm)
L = 13.6
# Speed scaling
SPEED = 20  # base motor command speed
ROT_SCALE = 0.1

class OmniJoystick(Node):
    def __init__(self):
        super().__init__('omni_teleop_joy')

        self.last_motor_commands = (0, 0, 0)

        self.pubA = self.create_publisher(String, '/motor_command_A', 10)
        self.pubB = self.create_publisher(String, '/motor_command_B', 10)
        self.pubC = self.create_publisher(String, '/motor_command_C', 10)

        # Subscribe to joystick topic
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.get_logger().info(
            "Omni teleop started with /joy input.\n"
            "Left stick Y → forward/back, Left stick X → strafe left/right, Right stick X → rotate.\n"
        )

    def joy_callback(self, msg: Joy):
        # Axes mapping (may need adjustment depending on your controller)
        # Common: left stick Y = axes[1], left stick X = axes[0], right stick X = axes[3]
        vx = -msg.axes[1]   # Forward/backward (invert if needed)
        vy = msg.axes[0]    # Strafe left/right
        omega = msg.axes[3] # Rotation

        # Calculate wheel speeds
        wA = vx - (vy / math.sqrt(3)) - (L * omega * ROT_SCALE)
        wB = -vx - (vy / math.sqrt(3)) - (L * omega * ROT_SCALE)
        wC = (2 * vy / math.sqrt(3)) - (L * omega * ROT_SCALE)

        mA = int(wA * SPEED)
        mB = int(wB * SPEED)
        mC = int(wC * SPEED)

        # Only send if changed
        if self.last_motor_commands != (mA, mB, mC):
            self.send_motor_cmds(mA, mB, mC)
            self.last_motor_commands = (mA, mB, mC)

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
    node = OmniJoystick()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_motor_cmds(0, 0, 0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
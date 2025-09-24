import rclpy                             ######################## LAUNCH FILE NEEDED #########################
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import Int64
import time

class EncoderListenerNode(Node):
    def __init__(self):
        super().__init__('encoder_listener_node')
        self.subscriptionA = self.create_subscription(
            Int64,
            '/raw_encoder_count_A',
            self.listener_callbackA,
            10
        )
        self.subscriptionB = self.create_subscription(
            Int64,
            '/raw_encoder_count_B',
            self.listener_callbackB,
            10
        )
        self.subscriptionC = self.create_subscription(
            Int64,
            '/raw_encoder_count_C',
            self.listener_callbackC,
            10
        )
        self.publisherA = self.create_publisher(Float64, '/rpmA', 10)
        self.publisherB = self.create_publisher(Float64, '/rpmB', 10)
        self.publisherC = self.create_publisher(Float64, '/rpmC', 10)
        
        # Initialize variables for RPM calculation
        self.prev_countA = 0
        self.prev_timeA = time.time()
        self.first_readingA = True

        self.prev_countB = 0
        self.prev_timeB = time.time()
        self.first_readingB = True

        self.prev_countC = 0
        self.prev_timeC = time.time()
        self.first_readingC = True
        
        # Motor/encoder specifications (adjust these values for your setup)
        self.encoder_ppr = 280  # Pulses per revolution (change to your encoder's PPR)

    def listener_callbackA(self, msg):
        current_time = time.time()
        current_count = msg.data
        
        if self.first_readingA:
            # Skip first reading to establish baseline
            self.prev_countA = current_count
            self.prev_timeA = current_time
            self.first_readingA = False
            return
        
        # Calculate time difference
        dt = current_time - self.prev_timeA
        
        if dt > 0:  # Avoid division by zero
            # Calculate count difference
            count_diff = current_count - self.prev_countA
            
            # Calculate RPM
            # RPM = (count_diff / encoder_ppr) * (60 / dt)
            rpm = (count_diff / self.encoder_ppr) * (60.0 / dt)
            
            # Publish RPM
            rpm_msg = Float64()
            rpm_msg.data = rpm
            self.publisherA.publish(rpm_msg)
            
            self.get_logger().info(f"Encoder A: {current_count}, RPM: {rpm:.2f}")
        
        # Update previous values
        self.prev_countA = current_count
        self.prev_timeA = current_time

    def listener_callbackB(self, msg):
        current_time = time.time()
        current_count = msg.data
        
        if self.first_readingB:
            # Skip first reading to establish baseline
            self.prev_countB = current_count
            self.prev_timeB = current_time
            self.first_readingB = False
            return
        
        # Calculate time difference
        dt = current_time - self.prev_timeB
        
        if dt > 0:  # Avoid division by zero
            # Calculate count difference
            count_diff = current_count - self.prev_countB
            
            # Calculate RPM
            # RPM = (count_diff / encoder_ppr) * (60 / dt)
            rpm = (count_diff / self.encoder_ppr) * (60.0 / dt)
            
            # Publish RPM
            rpm_msg = Float64()
            rpm_msg.data = rpm
            self.publisherB.publish(rpm_msg)
            
            self.get_logger().info(f"Encoder B: {current_count}, RPM: {rpm:.2f}")
        
        # Update previous values
        self.prev_countB = current_count
        self.prev_timeB = current_time

    def listener_callbackC(self, msg):
        current_time = time.time()
        current_count = msg.data
        
        if self.first_readingC:
            # Skip first reading to establish baseline
            self.prev_countC = current_count
            self.prev_timeC = current_time
            self.first_readingC = False
            return
        
        # Calculate time difference
        dt = current_time - self.prev_timeC
        
        if dt > 0:  # Avoid division by zero
            # Calculate count difference
            count_diff = current_count - self.prev_countC
            
            # Calculate RPM
            # RPM = (count_diff / encoder_ppr) * (60 / dt)
            rpm = (count_diff / self.encoder_ppr) * (60.0 / dt)
            
            # Publish RPM
            rpm_msg = Float64()
            rpm_msg.data = rpm
            self.publisherC.publish(rpm_msg)
            
            self.get_logger().info(f"Encoder C: {current_count}, RPM: {rpm:.2f}")
        
        # Update previous values
        self.prev_countC = current_count
        self.prev_timeC = current_time


def main(args=None):
    rclpy.init(args=args)
    node = EncoderListenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
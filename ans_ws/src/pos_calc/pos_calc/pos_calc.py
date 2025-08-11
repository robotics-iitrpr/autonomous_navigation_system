import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import math
from std_msgs.msg import Float64
import tf_transformations
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_node')

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_timer(0.05, self.update_odom)  # 20Hz

        self.r = 0.052  # Wheel radius = 5.2 cm
        self.L = 0.136   # Distance from center to wheel = 13.6 cm

        self.last_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0  # from IMU

        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(Float64, '/rpmA', self.get_encoder_speedA, 10)
        self.create_subscription(Float64, '/rpmB', self.get_encoder_speedB, 10)
        self.create_subscription(Float64, '/rpmC', self.get_encoder_speedC, 10)

        # Replace with real encoder data
        self.w1 = 0.0
        self.w2 = 0.0
        self.w3 = 0.0

        self.odom_broadcaster = TransformBroadcaster(self)
        self.tf_broadcaster = StaticTransformBroadcaster(self)


    def imu_callback(self, msg):
        quat = msg.orientation
        _, _, self.yaw = tf_transformations.euler_from_quaternion([
            quat.x, quat.y, quat.z, quat.w
        ])

    def get_encoder_speedA(self,msg):
        self.w1 = msg.data * (2*math.pi / 60)
    
    def get_encoder_speedB(self,msg):
        self.w2 = msg.data * (2*math.pi / 60)
    
    def get_encoder_speedC(self,msg):
        self.w3 = msg.data * (2*math.pi / 60)

    def update_odom(self):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        dt = current_time - self.last_time
        self.last_time = current_time

        w1 = self.w1
        w2 = self.w2
        w3 = self.w3

        # Forward kinematics (robot frame)
        Vx = (2/3) * self.r * (w1 - 0.5 * w2 - 0.5 * w3)
        Vy = (1/math.sqrt(3)) * self.r * (w2 - w3)

        # Transform to world frame
        dx = (Vx * math.cos(self.yaw) - Vy * math.sin(self.yaw)) * dt
        dy = (Vx * math.sin(self.yaw) + Vy * math.cos(self.yaw)) * dt

        self.x += dx
        self.y += dy

        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        

        q = tf_transformations.quaternion_from_euler(0, 0, self.yaw)
        orientation = Quaternion()
        orientation.x = q[0]
        orientation.y = q[1]
        orientation.z = q[2]
        orientation.w = q[3]
        odom_msg.pose.pose.orientation = orientation


        odom_msg.twist.twist.linear.x = Vx
        odom_msg.twist.twist.linear.y = Vy
        odom_msg.twist.twist.angular.z = 0.0  # optional

        # Broadcast the TF transform
        odom_tf = TransformStamped()
        odom_tf.header.stamp = self.get_clock().now().to_msg()
        odom_tf.header.frame_id = "odom"
        odom_tf.child_frame_id = "base_link"
        odom_tf.transform.translation.x = self.x
        odom_tf.transform.translation.y = self.y
        odom_tf.transform.translation.z = 0.0
        odom_tf.transform.rotation = orientation
        self.odom_broadcaster.sendTransform(odom_tf)




        self.odom_pub.publish(odom_msg)


        # Create and send the transform
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = 'base_link'
        static_tf.child_frame_id = 'laser'

        # Translation (adjust for your robot)
        static_tf.transform.translation.x = 0.0
        static_tf.transform.translation.y = 0.0
        static_tf.transform.translation.z = 0.1  # laser 10 cm above base

        # No rotation
        static_tf.transform.rotation.x = 0.0
        static_tf.transform.rotation.y = 0.0
        static_tf.transform.rotation.z = 0.0
        static_tf.transform.rotation.w = 1.0

        # Send it
        self.tf_broadcaster.sendTransform(static_tf)

        static_transformStamped = TransformStamped()

        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = 'base_link'      # parent frame
        static_transformStamped.child_frame_id = 'imu_link'        # child frame

        # Set your imu_link pose relative to base_link here:
        static_transformStamped.transform.translation.x = 0.0  # meters
        static_transformStamped.transform.translation.y = 0.0
        static_transformStamped.transform.translation.z = 0.0

        # Set rotation as quaternion (roll, pitch, yaw)
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)  # no rotation
        static_transformStamped.transform.rotation.x = q[0]
        static_transformStamped.transform.rotation.y = q[1]
        static_transformStamped.transform.rotation.z = q[2]
        static_transformStamped.transform.rotation.w = q[3]

        # Publish the static transform once
        self.tf_broadcaster.sendTransform(static_transformStamped)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        ()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
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

        self.last_time = self.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0  # from IMU
        self.imu_angular_velocity_z = 0.0  # initialize variable

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
        self.imu_angular_velocity_z = msg.angular_velocity.z

    def get_encoder_speedA(self,msg):
        rpm = max(min(msg.data, 500), -500)  # clamp RPM between -500 and 500
        self.w1 = rpm * (2 * math.pi / 60)
    
    def get_encoder_speedB(self,msg):
        rpm = max(min(msg.data, 500), -500)  # clamp RPM between -500 and 500
        self.w2 = rpm * (2 * math.pi / 60)
    
    def get_encoder_speedC(self,msg):
        rpm = max(min(msg.data, 500), -500)  # clamp RPM between -500 and 500
        self.w3 = rpm * (2 * math.pi / 60)

    def update_odom(self):
        now = self.get_clock().now()
        dt = (now.nanoseconds - self.last_time.nanoseconds) * 1e-9  # seconds

        if dt <= 0.0 or dt > 1.0:
            self.get_logger().warn(f"Skipping update due to invalid dt: {dt}")
            self.last_time = now
            return

        self.last_time = now

        w1 = self.w1
        w2 = self.w2
        w3 = self.w3

        # Robot frame velocities (assuming wheels at 0°, 120°, 240°)
        Vx = (2/3) * self.r * (w1 - 0.5 * w2 - 0.5 * w3)
        Vy = (1/math.sqrt(3)) * self.r * (w2 - w3)
        omega = self.imu_angular_velocity_z

        # If omega is significant, you might want to zero out Vx, Vy
        if abs(omega) > 0.1:
            Vx = 0.0
            Vy = 0.0

        # Transform to world frame
        dx = (Vx * math.cos(self.yaw) - Vy * math.sin(self.yaw)) * dt
        dy = (Vx * math.sin(self.yaw) + Vy * math.cos(self.yaw)) * dt

        # max_allowed_delta = 0.05  # meters per update (tune this!)
        # if abs(dx) > max_allowed_delta or abs(dy) > max_allowed_delta:
        #     self.get_logger().warn(f"Ignoring sudden jump in position: dx={dx:.3f}, dy={dy:.3f}")
        #     return

        self.x += dx
        self.y += dy

        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"

        odom_msg.pose.pose.position.x = -self.x
        odom_msg.pose.pose.position.y = -self.y
        

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

        self.odom_pub.publish(odom_msg)


        # Broadcast the TF transform
        odom_tf = TransformStamped()
        odom_tf.header.stamp = self.get_clock().now().to_msg()
        odom_tf.header.frame_id = "odom"
        odom_tf.child_frame_id = "base_footprint"
        odom_tf.transform.translation.x = -self.x
        odom_tf.transform.translation.y = -self.y
        odom_tf.transform.translation.z = 0.0
        odom_tf.transform.rotation = orientation
        self.odom_broadcaster.sendTransform(odom_tf)
        

        odom_tf = TransformStamped()
        odom_tf.header.stamp = self.get_clock().now().to_msg()
        odom_tf.header.frame_id = "base_footprint"
        odom_tf.child_frame_id = "base_link"
        odom_tf.transform.translation.x = 0.0
        odom_tf.transform.translation.y = 0.0
        odom_tf.transform.translation.z = 0.0
        odom_tf.transform.rotation.x = 0.0
        odom_tf.transform.rotation.y = 0.0
        odom_tf.transform.rotation.z = 0.0
        odom_tf.transform.rotation.w = 1.0
        self.odom_broadcaster.sendTransform(odom_tf)


        # Create and send the transform
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = 'base_footprint'
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
        static_transformStamped.header.frame_id = 'base_footprint'      # parent frame
        static_transformStamped.child_frame_id = 'imu_link'        # child frame

        # Set your imu_link pose relative to base_footprint here:
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
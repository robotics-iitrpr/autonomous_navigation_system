#!/usr/bin/env python3
############################################## INCLUDED IN LAUNCH FILE ##########################################
import rclpy                        
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped, QuaternionStamped
from std_msgs.msg import Header
import smbus
import time
import struct
import math

class BNO055Node(Node):
    def __init__(self):
        super().__init__('bno055_node')
        
        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x28)
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('frame_id', 'imu_link')
        
        # Get parameters
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.i2c_address = self.get_parameter('i2c_address').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Initialize I2C
        try:
            self.bus = smbus.SMBus(self.i2c_bus)
            self.get_logger().info(f'Initialized I2C bus {self.i2c_bus}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize I2C: {e}')
            return
        
        # BNO055 Register addresses
        self.BNO055_CHIP_ID = 0x00
        self.BNO055_OPR_MODE = 0x3D
        self.BNO055_PWR_MODE = 0x3E
        self.BNO055_SYS_TRIGGER = 0x3F
        
        # Data registers
        self.BNO055_EULER_H_LSB = 0x1A
        self.BNO055_QUATERNION_DATA_W_LSB = 0x20
        self.BNO055_LINEAR_ACCEL_DATA_X_LSB = 0x28
        self.BNO055_GRAVITY_DATA_X_LSB = 0x2E
        self.BNO055_GYRO_DATA_X_LSB = 0x14
        self.BNO055_ACCEL_DATA_X_LSB = 0x08
        
        # Operation modes
        self.NDOF_MODE = 0x0C
        
        # Initialize sensor
        if not self.initialize_sensor():
            self.get_logger().error('Failed to initialize BNO055 sensor')
            return
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)                                       ##################### USEFUL IN CODE #######################
        self.euler_pub = self.create_publisher(Vector3Stamped, 'imu/euler', 10)                           ##################### FOR DEBUGGING ########################
        self.quaternion_pub = self.create_publisher(QuaternionStamped, 'imu/quaternion', 10)
        
        # Timer for publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_data)
        
        self.get_logger().info(f'BNO055 node started, publishing at {self.publish_rate} Hz')
    
    def initialize_sensor(self):
        """Initialize the BNO055 sensor"""
        try:
            # Check chip ID
            chip_id = self.bus.read_byte_data(self.i2c_address, self.BNO055_CHIP_ID)
            if chip_id != 0xA0:
                self.get_logger().warn(f'Unexpected chip ID: 0x{chip_id:02X} (expected 0xA0)')
            
            # Set operation mode to NDOF
            self.bus.write_byte_data(self.i2c_address, self.BNO055_OPR_MODE, self.NDOF_MODE)
            time.sleep(0.1)
            
            self.get_logger().info('BNO055 sensor initialized successfully')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error initializing BNO055: {e}')
            return False
    
    def read_euler_angles(self):
        """Read Euler angles in radians"""
        try:
            data = self.bus.read_i2c_block_data(self.i2c_address, self.BNO055_EULER_H_LSB, 6)
            
            # Convert to degrees first, then to radians
            heading = struct.unpack('<h', bytes(data[0:2]))[0] / 16.0 * math.pi / 180.0
            roll = struct.unpack('<h', bytes(data[2:4]))[0] / 16.0 * math.pi / 180.0
            pitch = struct.unpack('<h', bytes(data[4:6]))[0] / 16.0 * math.pi / 180.0
            
            return heading, roll, pitch
            
        except Exception as e:
            self.get_logger().warn(f'Error reading Euler angles: {e}')
            return None, None, None
    
    def read_quaternion(self):
        """Read quaternion data"""
        try:
            data = self.bus.read_i2c_block_data(self.i2c_address, self.BNO055_QUATERNION_DATA_W_LSB, 8)
            
            w = struct.unpack('<h', bytes(data[0:2]))[0] / 16384.0
            x = struct.unpack('<h', bytes(data[2:4]))[0] / 16384.0
            y = struct.unpack('<h', bytes(data[4:6]))[0] / 16384.0
            z = struct.unpack('<h', bytes(data[6:8]))[0] / 16384.0
            
            return w, x, y, z
            
        except Exception as e:
            self.get_logger().warn(f'Error reading quaternion: {e}')
            return None, None, None, None
    
    def read_gyroscope(self):
        """Read gyroscope data in rad/s"""
        try:
            data = self.bus.read_i2c_block_data(self.i2c_address, self.BNO055_GYRO_DATA_X_LSB, 6)
            
            # Convert to rad/s (sensor gives dps, divide by 16 then convert to rad/s)
            x = struct.unpack('<h', bytes(data[0:2]))[0] / 16.0 * math.pi / 180.0
            y = struct.unpack('<h', bytes(data[2:4]))[0] / 16.0 * math.pi / 180.0
            z = struct.unpack('<h', bytes(data[4:6]))[0] / 16.0 * math.pi / 180.0
            
            return x, y, z
            
        except Exception as e:
            self.get_logger().warn(f'Error reading gyroscope: {e}')
            return None, None, None
    
    def read_linear_acceleration(self):
        """Read linear acceleration in m/s²"""
        try:
            data = self.bus.read_i2c_block_data(self.i2c_address, self.BNO055_LINEAR_ACCEL_DATA_X_LSB, 6)
            
            x = struct.unpack('<h', bytes(data[0:2]))[0] / 100.0
            y = struct.unpack('<h', bytes(data[2:4]))[0] / 100.0
            z = struct.unpack('<h', bytes(data[4:6]))[0] / 100.0
            
            return x, y, z
            
        except Exception as e:
            self.get_logger().warn(f'Error reading linear acceleration: {e}')
            return None, None, None
    
    def publish_data(self):
        """Publish sensor data to ROS2 topics"""
        current_time = self.get_clock().now()
        
        # Create header
        header = Header()
        header.stamp = current_time.to_msg()
        header.frame_id = self.frame_id
        
        # Read sensor data
        w, x, y, z = self.read_quaternion()
        gx, gy, gz = self.read_gyroscope()
        ax, ay, az = self.read_linear_acceleration()
        heading, roll, pitch = self.read_euler_angles()
        
        # Publish IMU message (combines quaternion, gyroscope, and acceleration)
        if all(v is not None for v in [w, x, y, z, gx, gy, gz, ax, ay, az]):
            imu_msg = Imu()
            imu_msg.header = header
            
            # Orientation (quaternion)
            imu_msg.orientation.w = w
            imu_msg.orientation.x = x
            imu_msg.orientation.y = y
            imu_msg.orientation.z = z
            
            # Angular velocity
            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz
            
            # Linear acceleration
            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az
            
            # # Covariance matrices (unknown, so set to -1)
            # imu_msg.orientation_covariance[0] = -1
            # imu_msg.angular_velocity_covariance[0] = -1
            # imu_msg.linear_acceleration_covariance[0] = -1

            # Set full covariance arrays to zeros:
            imu_msg.orientation_covariance = [0.0] * 9
            imu_msg.angular_velocity_covariance = [0.0] * 9
            imu_msg.linear_acceleration_covariance = [0.0] * 9
            
            self.imu_pub.publish(imu_msg)
        
        # Publish Euler angles
        if all(v is not None for v in [heading, roll, pitch]):
            euler_msg = Vector3Stamped()
            euler_msg.header = header
            euler_msg.vector.x = roll    # Roll around X-axis
            euler_msg.vector.y = pitch   # Pitch around Y-axis  
            euler_msg.vector.z = heading # Yaw around Z-axis
            
            self.euler_pub.publish(euler_msg)
        
        # Publish quaternion separately
        if all(v is not None for v in [w, x, y, z]):
            quat_msg = QuaternionStamped()
            quat_msg.header = header
            quat_msg.quaternion.w = w
            quat_msg.quaternion.x = x
            quat_msg.quaternion.y = y
            quat_msg.quaternion.z = z
            
            self.quaternion_pub.publish(quat_msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = BNO055Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
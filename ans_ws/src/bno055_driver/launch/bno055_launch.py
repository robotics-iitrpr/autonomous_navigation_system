from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bno055_driver',  # Replace with your package name
            executable='bno055_node',
            name='bno055_node',
            parameters=[
                {'i2c_bus': 1},
                {'i2c_address': 0x28},  # or 0x29 if ADR pin is high
                {'publish_rate': 50.0},  # Hz
                {'frame_id': 'imu_link'}
            ],
            output='screen'
        )
    ])
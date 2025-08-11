from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'configuration_directory': '/home/ans/ans_ws/src/cartographer/config',
                'configuration_basename': 'my_robot.lua',
            }],
             remappings=[
                ('/imu', '/imu/data'),
            ],
            arguments=['-configuration_directory', '/home/ans/ans_ws/src/cartographer/config',
                       '-configuration_basename', 'my_robot.lua']
        )
    ])

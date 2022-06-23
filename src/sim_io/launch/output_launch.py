from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([Node(
        package='sim_io',
        executable='sim_output',
        output='screen'),
    ])
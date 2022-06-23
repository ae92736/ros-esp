from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    joy = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("sim_io"), 'joy_node_launch.py')]))
    input_node = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("sim_io"), 'input_launch.py')]))
    output_node = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("sim_io"), 'output_launch.py')]))
    starter_node = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("sim_io"), 'starter_launch.py')]))
    mqtt = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("sim_io"), 'mqtt_launch.py')]))
    out = LaunchDescription([joy, input_node, output_node, starter_node, mqtt])
    return out
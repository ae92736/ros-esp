from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    all_io_node = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("sim_io"), 'all_io_launch.py')]))
    logger = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("logger"), 'log_launch.py')]))
    nesne = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("nesne"), 'nesne_launch.py')]))
    mapping = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("haritalama"), 'map_launch.py')]))
    serit = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("serit"), 'serit_launch.py')]))
    gui = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("gui"), 'gui_launch.py')]))
    out = LaunchDescription([all_io_node, logger, nesne, mapping, serit, gui])
    return out
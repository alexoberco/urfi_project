import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    urfi_starter_node = Node(
        package='starter',
        executable='urfi_starter',
        output='screen'
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('urfi_gz_ignition'),
                        'launch', 'spawn_urfi.launch.py'))),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('urfi_navigation'),
                        'launch', 'slam.launch.py'))),
            urfi_starter_node
            
        ]
    )

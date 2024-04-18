import random
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node

def generate_launch_description():
    # Argumentos
    urdf_file = os.path.join(
        get_package_share_directory('urfi-description'),
        'urdf',
        'robot_basico.urdf'
    )
    
    spawn_robot = Node(
        package = 'gazebo_ros',
        executable = 'spawn_entity.py',
        name = 'spawn_entity',
        output = 'screen',
        arguments=['-entity', 'robot_basico', '-file', urdf_file] 
    )
    

    return LaunchDescription([
        spawn_robot,
    ])

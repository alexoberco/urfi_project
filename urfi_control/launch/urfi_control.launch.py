import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_diff_controller_1 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_controller_1'],
        output='screen'
    )

    load_diff_controller_2 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_controller_2'],
        output='screen'
    )

    load_diff_controller_3 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_controller_3'],
        output='screen'
    )
    
      # Lista todos los nodos y manejadores de eventos que se lanzarán
    nodes = [
        load_joint_state_controller,
        load_diff_controller_1,
        load_diff_controller_2,
        load_diff_controller_3,
    ]

    # Retorna la descripción del lanzamiento con todos los nodos y manejadores de eventos
    return LaunchDescription(nodes)


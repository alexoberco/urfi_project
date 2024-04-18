import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    controller_params_file = os.path.join(get_package_share_directory("urfi_control"),'config','urfi_controllers.yaml')

    robot_description = Command(['ros2 param get -- hide-type /robot_state_publuisher robot_description'])

    control_node = None(
        package = "controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)}, controller_params_file],
        remapping=[
            ('/diff_controller_1/cmd_vel', '/cmd_vel'),
            ('/diff_controller_1/cmd_vel_unstamped', '/cmd_vel'),
            ('/diff_controller_1/cmd_vel_out', '/cmd_vel_out'),
            ('/diff_controller_1/odom', '/odom'),
            ('/diff_controller_2/cmd_vel', '/cmd_vel'),
            ('/diff_controller_2/cmd_vel_unstamped', '/cmd_vel'),
            ('/diff_controller_2/cmd_vel_out', '/cmd_vel_out'),
            ('/diff_controller_2/odom', '/odom'),
            ('/diff_controller_3/cmd_vel', '/cmd_vel'),
            ('/diff_controller_3/cmd_vel_unstamped', '/cmd_vel'),
            ('/diff_controller_3/cmd_vel_out', '/cmd_vel_out'),
            ('/diff_controller_3/odom', '/odom'),
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    diff_drive_controller_1_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller_1", "--controller-manager", "/controller_manager"],
    )

    # Nodo para lanzar el controlador del segundo par de ruedas
    diff_drive_controller_2_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller_2", "--controller-manager", "/controller_manager"],
    )

    # Nodo para lanzar el controlador del tercer par de ruedas
    diff_drive_controller_3_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller_3", "--controller-manager", "/controller_manager"],
    )

     # Delay start of diff_drive_controller_spawner after `joint_state_broadcaster`
    delay_diff_drive_controller_1_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_1_spawner],
        )
    )

    delay_diff_drive_controller_2_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=diff_drive_controller_1_spawner,
            on_exit=[diff_drive_controller_2_spawner],
        )
    )

    delay_diff_drive_controller_3_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=diff_drive_controller_2_spawner,
            on_exit=[diff_drive_controller_3_spawner],
        )
    )
    
      # Lista todos los nodos y manejadores de eventos que se lanzarán
    nodes = [
        joint_state_broadcaster_spawner,
        delay_diff_drive_controller_1_spawner,
        delay_diff_drive_controller_2_spawner,
        delay_diff_drive_controller_3_spawner,
    ]

    # Retorna la descripción del lanzamiento con todos los nodos y manejadores de eventos
    return LaunchDescription(nodes)


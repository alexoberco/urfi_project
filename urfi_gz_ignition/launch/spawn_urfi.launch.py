import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    urfi_path = os.path.join(
        get_package_share_directory('urfi_description'))
    rviz_config_path = os.path.join(urfi_path,'rviz', 'urfi.rviz')
    xacro_file = os.path.join(urfi_path, 'urdf', 'robot_model.xacro.urdf')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml(), 'use_sim_time': use_sim_time}



    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    cmd_vel_replicator_node = Node(
        package='starter',
        executable='cmd_vel_replicator',
        output='screen'
    )

    odom_replicator_node = Node(
        package='starter',
        executable='odom_replicator',
        output='screen'
    )


    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', doc.toxml(),
                   '-name', 'urfi',
                   '-allow_renaming', 'true',
                   '-x','0.0',
                   '-y','0.0',
                   '-z','0.1'],
    )
    # Define the bridge for LIDAR data
    lidar_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '--ros-args', '--remap', '/lidar:=/scan'
        ],
        output='screen'
    )


    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )


    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )


    return LaunchDescription([
        bridge,
        lidar_bridge,
        IncludeLaunchDescription(
           PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('urfi_control'),
                'launch', 'urfi_control.launch.py'))),
    
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_ign_gazebo'),
                              'launch', 'ign_gazebo.launch.py')]),
            launch_arguments=[('gz_args', [' -r -v 4 src/urfi_project/urfi_description/worlds/urfi_world.sdf'])]),

        node_robot_state_publisher,
        ignition_spawn_entity,
        node_rviz,
        cmd_vel_replicator_node,
        odom_replicator_node,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])
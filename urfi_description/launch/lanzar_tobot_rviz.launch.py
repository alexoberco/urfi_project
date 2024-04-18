import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'robot_basico.urdf'
    
    urdf = os.path.join(
        get_package_share_directory('urfi-description'), "sdf",
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
        
    rviz_config_path = os.path.join(
        get_package_share_directory('urfi-description'),
        'rvizConfigs',
        'default.rviz'
    )
    gazebo_world_path = os.path.join(
        get_package_share_directory('urfi-description'),
        'worlds','building_robot.sdf'
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', gazebo_world_path],
            output='screen'
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            arguments=[
                '/world/car_world/model/robot_basico/joint_state@sensor_msgs/msg/JointState@ignition.msgs.Model',
            ],
            output='screen',
        ),
        
    ])
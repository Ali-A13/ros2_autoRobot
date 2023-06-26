import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('my_robot_package'), 'tinykart.urdf')
    config_file = os.path.join(get_package_share_directory('my_robot_package'), 'ackermann_steering_controller.yaml')

    return LaunchDescription([
        # Publishes TF for links of the robot without joints
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf]
        ),

        # Publish TF for joints only
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # Launch Gazebo with the robot model
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Spawn the robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=[
                '-topic', '/robot_description',
                '-entity', 'tinykart',
                '-file', urdf,
                '-x', '0',
                '-y', '0',
                '-z', '0',
                '-R', '0',
                '-P', '0',
                '-Y', '0'
            ]
        ),

        # Start the controller
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[config_file],
            output='screen'
        )
    ])

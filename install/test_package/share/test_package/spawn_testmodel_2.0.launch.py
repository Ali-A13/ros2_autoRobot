import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess


def generate_launch_description():
    # Get the path to the SDF model
    sdf_path = os.path.join(get_package_share_directory('test_package'), 'sdf', 'AC_TestModel_2.0', 'model.sdf')

    # Declare the launch arguments
    sdf_arg = DeclareLaunchArgument('sdf', default_value=sdf_path, description='Comment: Path to the SDF model file for spawning in Gazebo')

    # Execute the Gazebo spawn_entity command to spawn the model
    spawn_entity_cmd = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/srv/SpawnEntity',
             '{model_name: ac_test_model_2_0, entity_name: ac_test_model_2_0, '
             'initial_pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, '
             'reference_frame: world, robot_namespace: ""}'],
        output='screen'
    )
    # Create the launch description
    ld = LaunchDescription()

    # Add the launch actions
    ld.add_action(sdf_arg)
    ld.add_action(spawn_entity_cmd)

    return ld

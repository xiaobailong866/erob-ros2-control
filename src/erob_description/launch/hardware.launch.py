import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory('erob_description')
    controller_config_file = os.path.join(pkg_path, 'config', 'controllers.yaml')

    urdf_path = os.path.join(pkg_path, 'urdf', 'erob_hardware.urdf.xacro')
    robot_description = {'robot_description': xacro.process_file(urdf_path).toxml()}

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config_file],
        output='screen',
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    left_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_arm_controller', '--controller-manager', '/controller_manager'],
    )

    return LaunchDescription([
        controller_manager_node,
        rsp_node,
        joint_state_broadcaster_spawner,
        left_arm_controller_spawner,
    ])

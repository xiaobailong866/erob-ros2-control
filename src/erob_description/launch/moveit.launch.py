import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_path = get_package_share_directory('erob_description')

    moveit_config = MoveItConfigsBuilder(
        "shu_pr03", package_name="erob_moveit_config"
    ).to_moveit_configs()

    # 用真实URDF覆盖MoveItConfigsBuilder生成的仿真xacro URDF
    urdf_path = os.path.join(pkg_path, 'urdf', 'erob.urdf')
    robot_description = {'robot_description': xacro.process_file(urdf_path).toxml()}
    moveit_config.robot_description = robot_description

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(
            get_package_share_directory('erob_moveit_config'), 'config', 'moveit.rviz'
        )],
        parameters=[
            robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([
        move_group_node,
        rviz_node,
    ])

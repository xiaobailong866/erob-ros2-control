from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 自动去 erob_moveit_config 包里寻找并打包所有需要的 URDF/SRDF 参数
    moveit_config = MoveItConfigsBuilder("shu_pr03", package_name="erob_moveit_config").to_moveit_configs()
    # 启动我们的桥接节点，并把参数喂给它
    moveit_bridge_node = Node(
        package="moveit_bridge",
        executable="moveit_bridge_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([moveit_bridge_node])

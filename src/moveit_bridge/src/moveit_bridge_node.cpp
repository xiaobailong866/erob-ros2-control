#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    //创建专门给MoveIt用的节点 
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto moveit_node = rclcpp::Node::make_shared("moveit_core_node", node_options);

    //将MoveIt节点放进独立的执行器和线程
    rclcpp::executors::SingleThreadedExecutor moveit_executor;
    moveit_executor.add_node(moveit_node);
    std::thread moveit_thread([&moveit_executor]() { moveit_executor.spin(); });

    //初始化MoveGroup，绑定到后台节点
    static const std::string PLANNING_GROUP = "left_arm"; 
    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(moveit_node, PLANNING_GROUP);

    //创建专门负责监听UI的节点
    auto ui_listen_node = rclcpp::Node::make_shared("ui_listener_node");

    //订阅UI发送的相对增量位姿
    auto pose_sub = ui_listen_node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/left_arm_node/teleop_pose", 10,
        [move_group, ui_listen_node](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            
            RCLCPP_INFO(ui_listen_node->get_logger(), "收到UI增量指令，开始处理...");
            
            //获取当前绝对位姿
            geometry_msgs::msg::PoseStamped current_pose = move_group->getCurrentPose();
            geometry_msgs::msg::Pose target_pose = current_pose.pose;

            //加上直线增量
            target_pose.position.x += msg->pose.position.x;
            target_pose.position.y += msg->pose.position.y;
            target_pose.position.z += msg->pose.position.z;

            //处理旋转增量
            tf2::Quaternion q_current(
                current_pose.pose.orientation.x,
                current_pose.pose.orientation.y,
                current_pose.pose.orientation.z,
                current_pose.pose.orientation.w);
                
            tf2::Quaternion q_delta(
                msg->pose.orientation.x,
                msg->pose.orientation.y,
                msg->pose.orientation.z,
                msg->pose.orientation.w);

            tf2::Quaternion q_new = q_current * q_delta;
            q_new.normalize();

            target_pose.orientation.x = q_new.x();
            target_pose.orientation.y = q_new.y();
            target_pose.orientation.z = q_new.z();
            target_pose.orientation.w = q_new.w();

            //下发规划
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose); //将目标点加入路点集合

            moveit_msgs::msg::RobotTrajectory trajectory;
            const double eef_step = 0.005;      
            const double jump_threshold = 1.5; 

            //计算直线轨迹，返回值为成功计算的轨迹比例 (0.0 到 1.0)
            double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

            // 如果成功率大于90%，我们就认为规划成功并执行
            if (fraction > 0.9) {
                RCLCPP_INFO(ui_listen_node->get_logger(), "笛卡尔直线规划成功 (%.1f%%)，开始执行！", fraction * 100.0);
                move_group->execute(trajectory);
            } else {
                RCLCPP_ERROR(ui_listen_node->get_logger(), "点动失败！末端走直线会碰到奇异点或死角 (仅能走出 %.1f%%)。", fraction * 100.0);
            }
        });

    RCLCPP_INFO(ui_listen_node->get_logger(), "MoveIt 桥接节点已就绪！");
    rclcpp::spin(ui_listen_node);
    rclcpp::shutdown();
    moveit_thread.join();
    return 0;
}

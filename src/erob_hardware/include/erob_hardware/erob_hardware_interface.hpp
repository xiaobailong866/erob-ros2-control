#ifndef EROB_HARDWARE__EROB_HARDWARE_INTERFACE_HPP_
#define EROB_HARDWARE__EROB_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

// 引入 IgH 相关的头文件
#include <ecrt.h> 

namespace erob_hardware
{

class ErobHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ErobHardwareInterface)

  // 1. 生命周期管理函数（必须重写）
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // 2. 状态与命令接口导出（告诉 ROS 2 我们可以读写什么）
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // 3. 实时读写循环（将被 1000Hz 频率调用）
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ROS 2 Control 用的双精度数组 (标准格式)
  std::vector<double> hw_position_commands_;
  std::vector<double> hw_position_states_;

  // 用于与 EtherCAT 线程安全通信的原子数组 (你原有的灵魂逻辑)
  std::array<std::atomic<int32_t>, 7> hw_target_position_;
  std::array<std::atomic<int32_t>, 7> hw_actual_position_;
  
  // EtherCAT 线程控制
  std::atomic<bool> ec_running_;
  std::thread ec_thread_;
  std::atomic<bool> is_operational_;  //用来记录是否到达 OP

  // 你原有的 EtherCAT 线程函数，现在作为类的私有成员函数
  void ethercat_thread_func();
};

}  // namespace erob_hardware

#endif  // EROB_HARDWARE__EROB_HARDWARE_INTERFACE_HPP_
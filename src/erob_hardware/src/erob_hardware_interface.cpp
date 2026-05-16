#include "erob_hardware/erob_hardware_interface.hpp"
#include <chrono>
#include <cmath>
#include <cstring>
#include <inttypes.h>
#include <limits>
#include <memory>
#include <sched.h>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <unistd.h>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#define MAX_SAFE_STACK (8 * 1024)
#define NSEC_PER_SEC (1000000000L)
#define FREQUENCY 1000
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)
#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)
#define SHIFT0 (PERIOD_NS/2)

#define STATE_FAULT              0x0008
#define STATE_SWITCH_ON_DISABLED 0x0040
#define STATE_READY_TO_SWITCH_ON 0x0021
#define STATE_SWITCHED_ON        0x0023
#define STATE_OPERATION_ENABLED  0x0027

#define CONTROL_WORD_SHUTDOWN           0x0006
#define CONTROL_WORD_SWITCH_ON         0x0007
#define CONTROL_WORD_ENABLE_OPERATION  0x000F
#define CONTROL_WORD_FAULT_RESET       0x0080

#define NUM_MOTORS 7

const double JOINT_OFFSETS[7] = {130900, 266906, 342004, 216841, 109234, 49978, 66759};//J1:130900, J2:266906, J3:342004, J4:216841, J5:109234, J6:49978, J7:66759

const double JOINT_DIRS[7] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

const double ENCODER_RES[7] = {524288, 524288, 524288, 524288, 262144, 262144, 262144};

const int32_t MAX_RAMP_STEP[7] = {50, 50, 50, 50, 25, 25, 25};

// 电机上下文结构体
struct MotorContext {
    uint16_t alias;
    uint16_t position;       
    uint32_t vendor_id;
    uint32_t product_code;
    ec_slave_config_t* slave_config;

    uint controlword;  
    uint statusword;                 
    uint target_position; 
    uint actual_position;
    uint error_code; // 新增

    uint16_t last_state = 0xFFFF;

    int32_t current_cmd_pos;
    bool is_first_enable;
};

// ZeroErr 与 Ti5 的 PDO 定义
static ec_pdo_entry_info_t zeroerr_pdo_entries[] = {
    {0x6040, 0x00, 16}, {0x607a, 0x00, 32},
    {0x6041, 0x00, 16}, {0x6064, 0x00, 32}, {0x603f, 0x00, 16}
};
static ec_pdo_info_t zeroerr_pdos[] = {
    {0x1600, 2, zeroerr_pdo_entries + 0}, 
    {0x1a00, 3, zeroerr_pdo_entries + 2}, 
};
static ec_sync_info_t zeroerr_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT , 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, zeroerr_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT , 1, zeroerr_pdos + 1, EC_WD_DISABLE},
    {0xFF}
};

static ec_pdo_entry_info_t ti5_pdo_entries[] = {
    {0x6040, 0x00, 16}, {0x607a, 0x00, 32}, 
    {0x6041, 0x00, 16}, {0x6064, 0x00, 32}, {0x603f, 0x00, 16}
};
static ec_pdo_info_t ti5_pdos[] = {
    {0x1600, 2, ti5_pdo_entries + 0}, 
    {0x1a00, 3, ti5_pdo_entries + 2}, 
};
static ec_sync_info_t ti5_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT , 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, ti5_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT , 1, ti5_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

// 辅助函数
inline void timespec_add(struct timespec* result, struct timespec* time1, struct timespec* time2) {
    if ((time1->tv_nsec + time2->tv_nsec) >= NSEC_PER_SEC) {
        result->tv_sec  = time1->tv_sec + time2->tv_sec + 1;
        result->tv_nsec = time1->tv_nsec + time2->tv_nsec - NSEC_PER_SEC;
    } else {
        result->tv_sec  = time1->tv_sec + time2->tv_sec;
        result->tv_nsec = time1->tv_nsec + time2->tv_nsec;
    }
}

void stack_prefault(void) {
    unsigned char dummy[MAX_SAFE_STACK];
    memset(dummy, 0, MAX_SAFE_STACK);
}

uint16_t getDriveState(uint16_t statusWord) {
    return statusWord & 0x6f; 
}

namespace erob_hardware
{

// 1. 初始化
hardware_interface::CallbackReturn ErobHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 7 个电机，调整数组大小
  hw_position_states_.resize(7, 0.0);
  hw_position_commands_.resize(7, 0.0);

  for (int i = 0; i < 7; ++i) {
    hw_target_position_[i].store(0);
    hw_actual_position_[i].store(0);
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

//把类里的变量地址告诉ROS2 Control框架，以便在实时读写循环中访问
// 2. 声明并导出从硬件读取的状态接口
std::vector<hardware_interface::StateInterface> ErobHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_states_[i]));
  }
  return state_interfaces;
}
// 3. 声明并导出写入硬件的命令接口
std::vector<hardware_interface::CommandInterface> ErobHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_commands_[i]));
  }
  return command_interfaces;
}

//配置硬件，建立通信
hardware_interface::CallbackReturn ErobHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}


// 4. 激活系统 (启动 EtherCAT线程)
hardware_interface::CallbackReturn ErobHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ErobHardwareInterface"), "激活中... 启动 EtherCAT 通信线程，并等待所有电机进入OP状态。");
  
  is_operational_.store(false); // 重置状态
  ec_running_ = true;
  ec_thread_ = std::thread(&ErobHardwareInterface::ethercat_thread_func, this);

  int timeout_counter = 0;
  while (!is_operational_.load() && rclcpp::ok() && timeout_counter < 300) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      timeout_counter++;
  }

  if (timeout_counter >= 300) {
      RCLCPP_ERROR(rclcpp::get_logger("ErobHardwareInterface"),
          "EtherCAT 初始化超时（30秒），电机未进入OP状态！");
      ec_running_ = false;
      if (ec_thread_.joinable()) { ec_thread_.join(); }
      return hardware_interface::CallbackReturn::ERROR;
  }
  
  for (int i = 0; i < 7; i++) {
    double pulses = static_cast<double>(hw_actual_position_[i].load()); 
    double encoder_res = ENCODER_RES[i]; 

    hw_position_states_[i] = ((pulses - JOINT_OFFSETS[i]) / encoder_res) * 2.0 * M_PI * JOINT_DIRS[i];
    hw_position_commands_[i] = hw_position_states_[i];
  }
  RCLCPP_INFO(rclcpp::get_logger("ErobHardwareInterface"), "硬件激活完成，已将控制权传递给管理器。");
  return hardware_interface::CallbackReturn::SUCCESS;
}


// 5. 关闭系统 (安全退出线程)
hardware_interface::CallbackReturn ErobHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ErobHardwareInterface"), "关闭中... 停止EtherCAT通信线程。");
  
  ec_running_ = false;
  if (ec_thread_.joinable()) {
    ec_thread_.join();
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// 在控制循环中，负责从物理硬件读取最新的状态数据，并更新到对应的状态接口
hardware_interface::return_type ErobHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  double current_raw_pulses[7];

  for (int i = 0; i < 7; ++i) {
    double pulses = static_cast<double>(hw_actual_position_[i].load());
    current_raw_pulses[i] = pulses; // 存下最原始的脉冲
    double encoder_res = ENCODER_RES[i]; 
    hw_position_states_[i] = ((pulses - JOINT_OFFSETS[i]) / encoder_res) * 2.0 * M_PI * JOINT_DIRS[i]; 
  }

  //每 5000 次循环（即 5 秒钟），打印一次当前 7 个电机的原始脉冲
  static int print_count = 0;
  if (print_count++ % 5000 == 0) {
    RCLCPP_INFO(rclcpp::get_logger("Calibration"),
      "各个关节的脉冲值：J1:%.0f, J2:%.0f, J3:%.0f, J4:%.0f, J5:%.0f, J6:%.0f, J7:%.0f",
      current_raw_pulses[0], current_raw_pulses[1], current_raw_pulses[2],
      current_raw_pulses[3], current_raw_pulses[4], current_raw_pulses[5], current_raw_pulses[6]);
  }

  // 任一电机掉出 OP 状态，CM 将停止所有控制器
  if (!is_operational_.load()) {
      return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

//在控制循环中，从命令接口获取控制指令，并发送给物理硬件
hardware_interface::return_type ErobHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  //如果电机掉线
  if (!is_operational_.load()) {
      for (int i = 0; i < 7; ++i) {
          //强制让命令状态跟随当前真实的物理状态
          hw_position_commands_[i] = hw_position_states_[i]; 
      }
  }

  for (int i = 0; i < 7; ++i) {
    double radians = hw_position_commands_[i];
    double encoder_res = ENCODER_RES[i]; 

    int32_t target_pulses = static_cast<int32_t>(((radians * JOINT_DIRS[i]) / (2.0 * M_PI)) * encoder_res + JOINT_OFFSETS[i]);
    hw_target_position_[i].store(target_pulses);
  }
  return hardware_interface::return_type::OK;
}

//6.EtherCAT实时底层代码
void ErobHardwareInterface::ethercat_thread_func()
{
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(4, &set);
    if (sched_setaffinity(0, sizeof(set), &set)) {
        RCLCPP_ERROR(rclcpp::get_logger("ErobHardwareInterface"), "设置CPU亲和性失败！");
        return;
    }

    struct sched_param param = {};
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("ErobHardwareInterface"), "设置实时调度策略失败！");
    }

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("ErobHardwareInterface"), "锁定内存失败！");
        return;
    }

    stack_prefault();

    ec_master_t* master = ecrt_request_master(0);
    if (!master) {
        RCLCPP_ERROR(rclcpp::get_logger("ErobHardwareInterface"), "请求EtherCAT主站失败！");
        return;
    }

    MotorContext motors[NUM_MOTORS];
    std::vector<ec_pdo_entry_reg_t> domain_regs;
    ec_domain_t* domain1 = nullptr;
    uint8_t* domain1_pd = nullptr;
    struct timespec wakeupTime, time;
    struct timespec cycleTime = {0, PERIOD_NS};
    bool all_op_done = false;
    bool init_ok = false;

    for (int i = 0; i < NUM_MOTORS; ++i) {
        motors[i].alias = 0;
        motors[i].position = i;
        motors[i].is_first_enable = true;
        motors[i].current_cmd_pos = 0;

        if (i < 4) {
            motors[i].vendor_id = 0x5a65726f;
            motors[i].product_code = 0x00029252;
            motors[i].slave_config = ecrt_master_slave_config(master, motors[i].alias, motors[i].position, motors[i].vendor_id, motors[i].product_code);
            if (!motors[i].slave_config) goto cleanup;
            if (ecrt_slave_config_pdos(motors[i].slave_config, 4, zeroerr_syncs)) goto cleanup;
        } else {
            motors[i].vendor_id = 0x00522227;
            motors[i].product_code = 0x00009253;
            motors[i].slave_config = ecrt_master_slave_config(master, motors[i].alias, motors[i].position, motors[i].vendor_id, motors[i].product_code);
            if (!motors[i].slave_config) goto cleanup;
            if (ecrt_slave_config_pdos(motors[i].slave_config, 4, ti5_syncs)) goto cleanup;
        }

        ecrt_slave_config_sdo8(motors[i].slave_config, 0x6060, 0, 0x08);
        ecrt_slave_config_sdo16(motors[i].slave_config, 0x6040, 0, 0x0080);

        domain_regs.push_back({0, (uint16_t)i, motors[i].vendor_id, motors[i].product_code, 0x6040, 0x00, &motors[i].controlword});
        domain_regs.push_back({0, (uint16_t)i, motors[i].vendor_id, motors[i].product_code, 0x607a, 0x00, &motors[i].target_position});
        domain_regs.push_back({0, (uint16_t)i, motors[i].vendor_id, motors[i].product_code, 0x6041, 0x00, &motors[i].statusword});
        domain_regs.push_back({0, (uint16_t)i, motors[i].vendor_id, motors[i].product_code, 0x6064, 0x00, &motors[i].actual_position});
        domain_regs.push_back({0, (uint16_t)i, motors[i].vendor_id, motors[i].product_code, 0x603f, 0x00, &motors[i].error_code});
        ecrt_slave_config_dc(motors[i].slave_config, 0x0300, PERIOD_NS, SHIFT0, 0, 0);
    }

    domain_regs.push_back({});

    domain1 = ecrt_master_create_domain(master);
    if (ecrt_domain_reg_pdo_entry_list(domain1, domain_regs.data())) goto cleanup;

    struct timespec masterInitTime;
    clock_gettime(CLOCK_MONOTONIC, &masterInitTime);
    ecrt_master_application_time(master, TIMESPEC2NS(masterInitTime));

    RCLCPP_INFO(rclcpp::get_logger("ErobHardwareInterface"), "激活EtherCAT主站...");
    if (ecrt_master_activate(master)) goto cleanup;

    domain1_pd = ecrt_domain_data(domain1);
    if (!domain1_pd) goto cleanup;

    init_ok = true;

    clock_gettime(CLOCK_MONOTONIC, &wakeupTime);

    //1. 等待OP状态
    while (ec_running_) {
        timespec_add(&wakeupTime, &wakeupTime, &cycleTime);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeupTime, NULL);

        ecrt_master_receive(master);
        
        bool all_operational = true;
        for (int i = 0; i < NUM_MOTORS; ++i) {
            ec_slave_config_state_t slave_state;
            ecrt_slave_config_state(motors[i].slave_config, &slave_state);
            if (!slave_state.operational) {
                all_operational = false;
                break;
            }
        }

        if (all_operational) {
            RCLCPP_INFO(rclcpp::get_logger("ErobHardwareInterface"), "所有电机已进入OP状态！");
            is_operational_.store(true);
            break; 
        }

        ecrt_domain_queue(domain1);
        clock_gettime(CLOCK_MONOTONIC, &time);
        ecrt_master_application_time(master, TIMESPEC2NS(time));
        ecrt_master_sync_reference_clock(master);
        ecrt_master_sync_slave_clocks(master);
        ecrt_master_send(master);
    }


    //2. 1000Hz核心通讯循环
    while (ec_running_) {
        timespec_add(&wakeupTime, &wakeupTime, &cycleTime);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeupTime, NULL);
        
        ecrt_master_receive(master);
        ecrt_domain_process(domain1);
        
        //每一轮循环重置OP状态计数器
        uint8_t op_cnt = 0;

        for (int i = 0; i < NUM_MOTORS; ++i) {
            
            //获取从站真实的AL状态
            ec_slave_config_state_t slave_al_state;
            ecrt_slave_config_state(motors[i].slave_config, &slave_al_state);
            if (slave_al_state.operational) {
                op_cnt++; 
            }

            int32_t actPos = EC_READ_S32(domain1_pd + motors[i].actual_position);
            hw_actual_position_[i].store(actPos);
            uint16_t statusWord = EC_READ_U16(domain1_pd + motors[i].statusword);
            uint16_t state = getDriveState(statusWord);
            uint16_t cw = 0; 

            //状态发生改变时才打印
            if (state != motors[i].last_state) {
                if (state == STATE_FAULT) {
                    RCLCPP_WARN(rclcpp::get_logger("ErobHardwareInterface"), "电机%d进入 FAULT 状态！", i+1);
                } else if (state == STATE_SWITCH_ON_DISABLED) {
                    RCLCPP_INFO(rclcpp::get_logger("ErobHardwareInterface"), "电机%d进入 DISABLED 状态", i+1);
                }
                // 把当前状态存下来
                motors[i].last_state = state;
            }
            
            //CiA402状态机逻辑
            switch(state) {
                case STATE_FAULT:
                    cw = CONTROL_WORD_FAULT_RESET;
                    motors[i].current_cmd_pos = actPos;
                    EC_WRITE_S32(domain1_pd + motors[i].target_position, motors[i].current_cmd_pos);
                    motors[i].is_first_enable = true;
                    break;
                case STATE_SWITCH_ON_DISABLED:
                    cw = CONTROL_WORD_SHUTDOWN;
                    motors[i].current_cmd_pos = actPos;
                    EC_WRITE_S32(domain1_pd + motors[i].target_position, motors[i].current_cmd_pos);
                    motors[i].is_first_enable = true;
                    break;
                case STATE_READY_TO_SWITCH_ON:
                    cw = CONTROL_WORD_SWITCH_ON;
                    motors[i].current_cmd_pos = actPos;
                    EC_WRITE_S32(domain1_pd + motors[i].target_position, motors[i].current_cmd_pos);
                    motors[i].is_first_enable = true;
                    break;
                case STATE_SWITCHED_ON:
                    cw = CONTROL_WORD_ENABLE_OPERATION;
                    motors[i].current_cmd_pos = actPos;
                    EC_WRITE_S32(domain1_pd + motors[i].target_position, motors[i].current_cmd_pos);
                    motors[i].is_first_enable = true;
                    break;
                case STATE_OPERATION_ENABLED:
                    if (motors[i].is_first_enable) {
                        hw_target_position_[i].store(motors[i].current_cmd_pos); 
                        motors[i].is_first_enable = false;
                    } else {
                        int32_t final_target_pos = hw_target_position_[i].load(); 
                        int32_t max_step_per_cycle = MAX_RAMP_STEP[i];
                        int32_t diff = final_target_pos - motors[i].current_cmd_pos;
                        if (diff > max_step_per_cycle) {
                            motors[i].current_cmd_pos += max_step_per_cycle;
                        } else if (diff < -max_step_per_cycle) {
                            motors[i].current_cmd_pos -= max_step_per_cycle;
                        } else {
                            motors[i].current_cmd_pos = final_target_pos;
                        }
                    }
                    EC_WRITE_S32(domain1_pd + motors[i].target_position, motors[i].current_cmd_pos);
                    cw = CONTROL_WORD_ENABLE_OPERATION;
                    break;
                default:
                    cw = CONTROL_WORD_SHUTDOWN;
                    break;
            }
            EC_WRITE_U16(domain1_pd + motors[i].controlword, cw);
        } 

        if (!all_op_done) {
            // 启动阶段：等待7个电机全部进入OP状态
            if (op_cnt == NUM_MOTORS) {
                all_op_done = true;
                RCLCPP_INFO(rclcpp::get_logger("ErobHardwareInterface"), 
                            "所有电机已首次进入OP状态！");
            }
        } else if (op_cnt != NUM_MOTORS) {
            is_operational_.store(false);
            for (int j = 0; j < NUM_MOTORS; ++j) {
                ec_slave_config_state_t check_state;
                ecrt_slave_config_state(motors[j].slave_config, &check_state);
                if (!check_state.operational) {
                    uint16_t err = EC_READ_U16(domain1_pd + motors[j].error_code);
                    RCLCPP_FATAL(rclcpp::get_logger("ErobHardwareInterface"),
                        "电机 %d 掉出OP状态，错误代码: 0x%04X，机械臂已停止！", j+1, err);
                }
            }
            break;
        }

        ecrt_domain_queue(domain1);
        clock_gettime(CLOCK_MONOTONIC, &time);
        ecrt_master_application_time(master, TIMESPEC2NS(time));
        ecrt_master_sync_reference_clock(master);
        ecrt_master_sync_slave_clocks(master);
        ecrt_master_send(master);
    }

cleanup:
    if (!init_ok) {
        RCLCPP_ERROR(rclcpp::get_logger("ErobHardwareInterface"), "EtherCAT 初始化失败！");
    }
    if (master) {
        ecrt_release_master(master);
    }
}
}  // namespace erob_hardware

//注册插件，将其注册为插件，以便ros2_control可以动态加载
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  erob_hardware::ErobHardwareInterface, hardware_interface::SystemInterface)
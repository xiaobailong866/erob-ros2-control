cmake_minimum_required(VERSION 3.8)
project(erob_hardware)

# 添加标准编译警告选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 寻找 ROS 2 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# 2. EtherLab 配置 
set(ETHERLAB_DIR /usr/local/etherlab)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# 寻找 EtherCAT 库
find_library(ETHERCAT_LIB ethercat HINTS ${ETHERLAB_DIR}/lib)
if(NOT ETHERCAT_LIB)
  message(FATAL_ERROR "🚨 找不到 EtherCAT 库！请检查 ${ETHERLAB_DIR}/lib 路径")
endif()

# 3.  声明要编译的动态库 
add_library(${PROJECT_NAME} SHARED
  src/erob_hardware_interface.cpp
)

# 4. 包含头文件目录 (包含 ROS 2 的 include 和 EtherCAT 的 include)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ETHERLAB_DIR}/include
)

# 5. 链接 ROS 2 库
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

# 6. 链接 IgH EtherCAT 库
target_link_libraries(${PROJECT_NAME} ${ETHERCAT_LIB})

# 7. 导出插件 XML (让 ROS 2 能够找到并加载这个硬件接口)
pluginlib_export_plugin_description_file(hardware_interface erob_hardware.xml)

# 8. 安装规则
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include
)

# 9. 导出给其他包使用
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)

ament_package()
# Setup

## System Dependencies

Recommended environment:

- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2
- ros2_control and ros2_controllers
- IgH EtherCAT master

Install common ROS dependencies:

```bash
sudo apt update
sudo apt install ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install ros-humble-xacro ros-humble-robot-state-publisher
sudo apt install python3-pyqt5 python3-numpy python3-matplotlib
```

Install and configure IgH EtherCAT separately. The current CMake configuration expects:

```text
/usr/local/etherlab/include/ecrt.h
/usr/local/etherlab/lib/libethercat.so
```

## Build

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

To build only the hardware package:

```bash
colcon build --packages-select erob_hardware --symlink-install
```

## Real Hardware Launch

Use this only after checking the robot workspace, emergency stop, drive power, and EtherCAT network:

```bash
source install/setup.bash
ros2 launch erob_description hardware.launch.py
```

This starts:

- `ros2_control_node`
- `robot_state_publisher`
- `joint_state_broadcaster`
- `left_arm_controller`

The real hardware launch loads `erob_hardware.urdf.xacro`, which adds the EtherCAT-backed `erob_hardware/ErobHardwareInterface` plugin to the base robot model.

## MoveIt 2 Launch

In another terminal:

```bash
source install/setup.bash
ros2 launch erob_description moveit.launch.py
```

For visualization-only testing, `ros2 launch erob_moveit_config demo.launch.py` uses the fake `mock_components/GenericSystem` ros2_control configuration from the MoveIt config package and does not load the real EtherCAT hardware plugin.

## MoveIt Bridge Launch

Start the bridge node before using Cartesian jogging or trajectory error analysis:

```bash
source install/setup.bash
ros2 launch moveit_bridge bridge.launch.py
```

## GUI Tools

Joint and Cartesian jogging:

```bash
source install/setup.bash
python3 src/control_gui/joint_pose_control.py
```

Trajectory error analysis:

```bash
source install/setup.bash
python3 src/control_gui/error_analysis.py
```

The analysis tool writes `trajectory_error_analysis.csv` in the current working directory. This file is ignored by Git.

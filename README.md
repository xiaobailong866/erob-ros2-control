# EROB ROS 2 Control System

A ROS 2 based real-time control and trajectory analysis system for a 7-DOF robotic arm. The project integrates MoveIt 2, ros2_control, a custom IgH EtherCAT hardware interface, and PyQt-based operator tools for jogging and Cartesian trajectory error analysis.

> Current scope: the left 7-DOF arm is connected to real EtherCAT hardware. The right arm and head are modeled in the robot description and can be kept as mock components for system-level planning and visualization.

## Highlights

- Custom `ros2_control` hardware interface for 7 EtherCAT servo drives.
- 1000 Hz low-level communication loop using IgH EtherCAT.
- CiA402 drive state handling, PDO registration, distributed clock configuration, and OP-state monitoring.
- Encoder pulse to joint-position conversion with joint offsets, encoder resolutions, and command ramp limiting.
- MoveIt 2 configuration for planning groups, joint limits, kinematics, controller mapping, and RViz visualization.
- Bridge node that converts GUI Cartesian jogging commands into MoveIt 2 Cartesian paths.
- PyQt5 tools for joint jogging, Cartesian jogging, TF2 end-effector feedback, trajectory recording, CSV export, and millimeter-level error plotting.

## Repository Layout

```text
src/
  erob_description/       Robot URDF, meshes, launch files, and real ros2_control xacro
  erob_hardware/          Custom ros2_control SystemInterface for EtherCAT hardware
  erob_moveit_config/     MoveIt 2 configuration generated and adapted for shu_pr03
  moveit_bridge/          MoveIt 2 bridge for Cartesian teleoperation commands
  control_gui/            PyQt5 operator and trajectory-analysis tools
docs/
  architecture.md         System architecture and data flow
  setup.md                Build and launch guide
  safety.md               Hardware safety notes
```

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2
- ros2_control and ros2_controllers
- IgH EtherCAT master installed under `/usr/local/etherlab`
- Python packages: `PyQt5`, `numpy`, `matplotlib`

This project controls real hardware. Review [docs/safety.md](docs/safety.md) before running the hardware launch files.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If `erob_hardware` fails to build, check that the IgH EtherCAT headers and library are available at `/usr/local/etherlab/include` and `/usr/local/etherlab/lib`.

## Launch

Start the real left-arm hardware interface and controllers:

```bash
ros2 launch erob_description hardware.launch.py
```

Start MoveIt 2 and RViz:

```bash
ros2 launch erob_description moveit.launch.py
```

Start the bridge that converts GUI Cartesian commands into MoveIt Cartesian path execution:

```bash
ros2 launch moveit_bridge bridge.launch.py
```

Run the joint / Cartesian jogging GUI:

```bash
python3 src/control_gui/joint_pose_control.py
```

Run Cartesian trajectory error analysis:

```bash
python3 src/control_gui/error_analysis.py
```

## Key Packages

### `erob_hardware`

Implements `erob_hardware/ErobHardwareInterface`, a `hardware_interface::SystemInterface` plugin. It creates an EtherCAT master, configures PDO entries for multiple drive types, runs a 1000 Hz communication loop, maps encoder pulses to ROS joint positions, and stops control when any motor leaves OP state.

### `erob_description`

Contains the robot model, meshes, launch files, and the real hardware ros2_control xacro. The base `erob.urdf` stays hardware-agnostic; `erob_hardware.urdf.xacro` adds the real left-arm EtherCAT interface for hardware launches.

### `erob_moveit_config`

Contains MoveIt 2 planning groups, kinematics, joint limits, controller mappings, RViz configuration, and the fake ros2_control xacro used by `demo.launch.py`.

### `moveit_bridge`

Subscribes to `/left_arm_node/teleop_pose`, reads the current end-effector pose from MoveIt, applies the requested Cartesian translation and rotation increment, computes a Cartesian path for the `left_arm` planning group, and executes the trajectory when the planned fraction is high enough.

### `control_gui`

Contains PyQt5 tools built on `rclpy`:

- `joint_pose_control.py`: joint jogging, Cartesian jogging, `/joint_states` feedback, and TF2 end-effector pose display.
- `error_analysis.py`: Cartesian displacement command, TF2 path recording, 3D path visualization, XYZ error curves, total spatial error, and CSV export.

## Documentation

- [Architecture](docs/architecture.md)
- [Setup](docs/setup.md)
- [Safety](docs/safety.md)

## License

This project is released under the BSD-3-Clause License. See [LICENSE](LICENSE).

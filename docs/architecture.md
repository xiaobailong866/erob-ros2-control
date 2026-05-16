# Architecture

## Overview

The project is organized as a ROS 2 control stack for a 7-DOF manipulator:

```text
MoveIt 2 joint commands / GUI Cartesian commands
        |
        v
joint_trajectory_controller / moveit_bridge
        |
        v
ros2_control controller_manager
        |
        v
erob_hardware/ErobHardwareInterface
        |
        v
IgH EtherCAT master
        |
        v
7 EtherCAT servo drives
```

The hardware loop runs separately from the ROS 2 controller loop. Shared atomic variables pass target and actual encoder positions between the `ros2_control` interface and the EtherCAT thread.

## ROS 2 Packages

- `erob_description`: URDF, meshes, real hardware ros2_control xacro, controller configuration, and launch files.
- `erob_hardware`: custom `hardware_interface::SystemInterface` plugin backed by IgH EtherCAT.
- `erob_moveit_config`: MoveIt 2 planning configuration for the `shu_pr03` robot model.
- `moveit_bridge`: subscribes to GUI Cartesian increments and executes MoveIt Cartesian paths for the left arm.
- `control_gui`: PyQt5 operator tools using `rclpy`, `/joint_states`, TF2, and command topics.

## Hardware Interface

`ErobHardwareInterface` exports position state and command interfaces for the 7 left-arm joints. The real hardware launch uses `erob_hardware.urdf.xacro`, which includes the base robot model and adds the real `erob_hardware/ErobHardwareInterface` plugin. The MoveIt demo path uses the fake ros2_control xacro in `erob_moveit_config`, so demo launches do not load the EtherCAT hardware plugin.

The EtherCAT thread handles:

- master creation and activation,
- PDO registration,
- distributed clock configuration,
- CiA402 state transitions,
- actual-position reads,
- target-position writes,
- command ramp limiting,
- OP-state monitoring and fail-stop behavior.

## Control and Feedback

The main controller is `joint_trajectory_controller/JointTrajectoryController`, configured for position control. TF2 feedback is produced by `robot_state_publisher` and consumed by the GUI tools to display the current end-effector pose and record Cartesian paths.

The GUI publishes Cartesian relative pose commands on `/left_arm_node/teleop_pose`. The `moveit_bridge` node receives these commands, queries MoveIt for the current pose, computes a Cartesian path for the `left_arm` group, and executes the resulting trajectory through the MoveIt controller pipeline.

## Current Hardware Scope

The left 7-DOF arm is mapped to the real EtherCAT hardware interface. The right arm and head are included in the model and MoveIt configuration, and can be represented by mock components until hardware support is added.

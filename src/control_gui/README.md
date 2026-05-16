# Control GUI Tools

This directory contains PyQt5 tools for operating and evaluating the left arm.

## `joint_pose_control.py`

Features:

- subscribes to `/joint_states`,
- publishes joint jogging commands to `/left_arm_controller/joint_trajectory`,
- publishes Cartesian relative pose commands to `/left_arm_node/teleop_pose`,
- listens to TF2 from `base_link` to `L_wrist_3`,
- displays joint and end-effector feedback in the UI.

Run:

Terminal 1:

```bash
source install/setup.bash
ros2 launch moveit_bridge bridge.launch.py
```

Terminal 2:

```bash
source install/setup.bash
python3 src/control_gui/joint_pose_control.py
```

## `error_analysis.py`

Features:

- publishes Cartesian displacement commands to `/left_arm_node/teleop_pose`,
- records TF2 end-effector positions at 30 Hz,
- plots the target line, measured path, XYZ deviation, and total spatial error,
- exports `trajectory_error_analysis.csv`.

Run:

Terminal 1:

```bash
source install/setup.bash
ros2 launch moveit_bridge bridge.launch.py
```

Terminal 2:

```bash
source install/setup.bash
python3 src/control_gui/error_analysis.py
```

These scripts are currently standalone tools rather than a formal ROS 2 Python package.

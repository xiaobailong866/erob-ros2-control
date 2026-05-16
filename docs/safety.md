# Safety

This repository contains software for real robotic hardware. Treat every launch command as potentially capable of moving the robot.

## Before Running

- Verify that the emergency stop is reachable and functional.
- Keep the robot workspace clear.
- Start with low drive power and conservative motion limits when testing changes.
- Confirm joint zero offsets and encoder directions before enabling motion.
- Make sure the EtherCAT slave order matches the order expected by the hardware interface.
- Do not run the GUI jogging tools until `/joint_states` and TF2 feedback are stable.

## Runtime Behavior

The hardware interface monitors whether all motors remain in EtherCAT OP state. If a motor leaves OP state after startup, the interface reports an error to `ros2_control`, logs the motor error code, and stops the control loop.

The interface also limits target-position changes per cycle to reduce abrupt command jumps. These limits are not a substitute for mechanical hard stops, drive-level safety settings, or supervised testing.

## Development Notes

- Test configuration changes in visualization or with drives disabled before enabling real motion.
- Keep MoveIt joint limits, URDF limits, and drive-level limits consistent.
- Record any calibration changes to joint offsets, encoder resolution, or motor order.
- Do not commit local build artifacts, logs, or generated trajectory CSV files.

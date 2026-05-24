# Safety Guide

JoyCapture-UR5 controls a real UR5 robotic arm, a Robotiq gripper, and RGB-D cameras during live teleoperation and data collection. Read this guide before running the system on hardware.

This project is intended for controlled laboratory use by trained users. The maintainers are not responsible for damage, injury, data loss, or unsafe operation caused by incorrect configuration, hardware failure, network issues, or misuse.

## 1. Before Powering the Robot

Make sure that:

- The UR5 workcell is clear of people, loose objects, cables, and fragile equipment.
- The emergency stop button is reachable by the operator.
- The robot speed and force limits are appropriate for manual teleoperation.
- The teach pendant is available and the operator can stop the robot immediately.
- The gripper is securely mounted and does not collide with the table, cameras, or fixtures.
- The tool center point (TCP), payload, and mounting orientation are configured correctly.
- The robot controller and Ubuntu PC are on the same trusted local network.
- The configured `robot_host` points to the correct robot controller.
- The configured camera serial numbers correspond to the physical cameras in the scene.

## 2. Before Starting Teleoperation

Before running:

```bash
./run_ur5_xbox_ubuntu.sh
```

check that:

- The UR teach pendant is in a safe state.
- The External Control program is loaded and ready.
- The Xbox/gamepad device is connected and responding.
- The motion direction mapping has been tested at low speed.
- The gripper open/close command has been tested away from objects and fingers.
- The robot starts from a known, collision-free pose.
- No person is inside the robot workspace.

Start with low speed and small joystick motions. Do not test a new configuration near obstacles.

## 3. Safe Teleoperation Rules

During teleoperation:

- Keep one hand close to the emergency stop.
- Move slowly when testing a new tool orientation or workspace.
- Do not stand inside the reachable workspace of the robot.
- Do not place hands near the gripper while the program is active.
- Do not rely on the gamepad as a safety device.
- Do not use wireless controllers if connection loss has not been tested.
- Stop the program immediately if the robot moves in an unexpected direction.
- Avoid large joystick deflections until the direction mapping is verified.
- Avoid replaying trajectories recorded under a different TCP, payload, table height, or object layout.

The `Start` button exits the program, but the emergency stop and teach pendant safety functions should always be considered the primary safety mechanisms.

## 4. Recording Safety

When recording demonstrations:

- Press the recording button only after the cameras and robot are stable.
- Keep the workspace unchanged during one recording session unless the task requires it.
- Confirm that camera stands are outside the robot collision envelope.
- Do not move cameras during recording unless the dataset is intended to include camera motion.
- Check that generated `.bag`, timestamp, metadata, and robot trajectory files are saved correctly before collecting large datasets.

## 5. Replay Safety

Trajectory replay is potentially more dangerous than live teleoperation because the robot may move automatically.

Before replaying:

- Inspect the recorded trajectory.
- Confirm that the replay folder corresponds to the current task and workspace.
- Confirm that the TCP, payload, gripper, object positions, and table layout match the original recording.
- Start with low speed if configurable.
- Keep the emergency stop reachable.
- Do not replay trajectories near people or unverified obstacles.

Do not replay a trajectory recorded with a different robot setup unless it has been manually checked.

## 6. Configuration Checklist

Before using a new robot, camera, or gripper setup, verify:

- `robot_host`
- Robotiq gripper IP/port
- camera names and serial numbers
- recording output directory
- robot FPS and camera FPS
- TCP orientation compensation
- joystick direction mapping
- gripper open/close logic
- replay folder
- generated data paths

Machine-specific configuration files should not be committed to Git.

## 7. Network and Data Safety

- Use a trusted local network for robot control.
- Do not expose the robot controller directly to the public internet.
- Do not commit local IP addresses, private lab paths, raw recordings, or large camera dumps.
- Do not commit generated `.bag`, `.mp4`, `.avi`, `.pkl`, `.hdf5`, `.h5`, or dataset folders unless intentionally publishing sample data.

## 8. Emergency Procedure

If anything unexpected happens:

1. Release the gamepad controls.
2. Press the emergency stop if the robot continues moving or approaches a collision.
3. Stop the UR program from the teach pendant.
4. Terminate the JoyCapture-UR5 process.
5. Inspect the robot, gripper, cameras, and workspace.
6. Check the configuration before restarting.

Do not restart the system until the cause of the unsafe behavior is understood.

## 9. Recommended First Test

For a new setup, test in this order:

1. Start the program without recording.
2. Move the TCP slowly in each direction.
3. Test gripper open/close away from objects.
4. Set a Home pose.
5. Move away and return to Home.
6. Start a short recording.
7. Stop recording and inspect the saved files.
8. Replay only after confirming the path is safe.

## 10. Disclaimer

This project is provided as research and engineering software. Users are responsible for safe deployment, local risk assessment, hardware configuration, and compliance with their institution's robot safety procedures.

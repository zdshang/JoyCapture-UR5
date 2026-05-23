# JoyCapture-UR5

**JoyCapture-UR5: Gamepad-Guided Multimodal Demonstration Capture for UR5 Manipulation**

![JoyCapture-UR5 social preview](joycapture-ur5-social-preview.png)

A lightweight Xbox/gamepad teleoperation and multimodal data collection system for UR5 manipulation, recording robot trajectories, gripper events, controller actions, and synchronized RGB-D camera streams.

The live recorder uses a source-data-first workflow: robot/action/gripper CSV files, session metadata, camera sync tables, and per-camera RealSense `.bag` files are written during teleoperation. Videos, exported image/depth frames, HDF5 episodes, and RLDS-style datasets are generated later by offline postprocessing.

![JoyCapture-UR5 system overview](joycapture-ur5-system-overview.png)

For full configuration, output layout, dataset format, and troubleshooting details, see [docs/DETAILS.md](docs/DETAILS.md).

## Features

- Xbox controller teleoperation for UR5 TCP motion.
- Robotiq gripper activation and open/close toggle.
- Home pose set and return.
- Raw trajectory recording and replay.
- Dual-camera recording with Intel RealSense D455 and L515.
- Robot and camera sampling FPS can be configured and checked for consistency.
- Live recording writes lightweight raw source data first.
- Offline export to review videos, frames, depth CSV, HDF5, and RLDS-style data.

## Hardware

- UR5 / UR controller with External Control available on the teach pendant.
- Robotiq gripper using socket control, commonly on port `63352`.
- Xbox-compatible controller readable through Linux input devices.
- Intel RealSense D455.
- Intel RealSense L515.
- Ubuntu PC on the same network as the robot controller.

## Quick Start

From a fresh clone on Ubuntu:

```bash
cd JoyCapture-UR5
sudo apt update
grep -vE '^\s*(#|$)' env/ubuntu_system_packages.txt | sudo xargs apt install -y
conda env create -f env/environment.yml
conda activate UR_xbox
./scripts/setup_urxbox_env.sh all
./scripts/check_urxbox_env.sh all
cp config/teleop_launcher_config.json config/teleop_launcher_config.local.json
```

Edit `config/teleop_launcher_config.local.json` for the local robot and cameras. At minimum set `robot_host`, confirm the camera entries, and adjust `motion.xy_rotate_deg` / `motion.rot_axes_rotate_deg` for the tool mounting.

Before running:

- Connect the Xbox controller, D455, L515, and robot network cable.
- Make sure the robot controller and Ubuntu PC are on the same network.
- On the UR teach pendant, make sure the External Control program is available and ready to run.

Start teleoperation:

```bash
./run_ur5_xbox_ubuntu.sh
```

If `conda` is not available on `PATH`, pass it explicitly:

```bash
./run_ur5_xbox_ubuntu.sh --conda-bin /path/to/conda --env-name UR_xbox
```

If shell scripts are not executable after downloading the repository as a zip file:

```bash
chmod +x run_ur5_xbox_ubuntu.sh scripts/*.sh
```

## Controller Mapping

- Left stick: TCP left / right / up / down.
- `LB` / `RB`: TCP backward / forward.
- `LT` / `RT`: tool pitch.
- Right stick left / right: tool self rotation.
- Right stick up / down: wrist / end rotation.
- `X`: gripper open / close.
- `Y`: start / stop recording.
- `A`: set home pose.
- `B`: move to home pose.
- `Back`: replay the latest in-memory path or load a raw trajectory from the configured playback folder.
- `Start`: exit.

On Xbox controllers, `Back` is usually labeled `View`; it is the small center-left button with two overlapping squares. `Start` is usually labeled `Menu`; it is the small center-right button with three horizontal lines.

## Run

Default run:

```bash
./run_ur5_xbox_ubuntu.sh
```

Run and record into a task-specific raw folder:

```bash
./run_ur5_xbox_ubuntu.sh \
  --env-name UR_xbox \
  --output-dir /home/user/Desktop/JoyCapture-UR5/paths/task_name
```

During teleoperation, press `Y` to start/stop one recording session. With `recording.session_subdirs=true`, repeated recordings are saved as:

```text
paths/task_name/1/
paths/task_name/2/
paths/task_name/3/
```

Replay the latest raw trajectory from a task folder after restarting the program:

```bash
./run_ur5_xbox_ubuntu.sh \
  --env-name UR_xbox \
  --output-dir /home/user/Desktop/JoyCapture-UR5/paths/new_task \
  --playback-dir /home/user/Desktop/JoyCapture-UR5/paths/task_name \
  --playback-session latest
```

Then press `Back` / `View` on the controller. To replay one concrete session, replace `latest` with a session id such as `20260523_230941`.

`--playback-dir` can point to `paths/`, a task folder such as `paths/task_name`, a numbered recording folder such as `paths/task_name/1`, or a concrete `session_manifest_*.json` file. The program searches saved raw sessions recursively.

## Check A Recording

List saved sessions:

```bash
find paths -path '*/session_metadata/session_manifest_*.json' | sort
```

Inspect the latest manifest:

```bash
python -m json.tool "$(find paths -path '*/session_metadata/session_manifest_*.json' | sort | tail -n 1)"
```

A normal recording should have:

- `point_count` greater than `0`.
- `action_row_count` greater than `0`.
- Two camera entries, normally `d455` and `l515`.
- A positive `frame_count` for each camera.
- Existing `.bag`, timestamp CSV, metadata JSON, and intrinsics JSON paths for each camera.
- `robot_record_fps` matching the configured camera FPS when `recording.require_fps_match` is `true`.

Quickly list generated files for one task:

```bash
find paths/task_name -maxdepth 4 -type f | sort
```

## Offline Postprocessing

Create all common outputs from the latest raw recording:

```bash
./scripts/postprocess_recording.sh --input paths --session latest --outputs video,frames,depth_csv,hdf5,rlds
```

Common choices:

```bash
# Quick visual inspection.
./scripts/postprocess_recording.sh --input paths --session latest --outputs video

# Export image/depth files and HDF5 for training.
./scripts/postprocess_recording.sh --input paths --session latest --outputs frames,depth_csv,hdf5

# Convert one concrete session id.
./scripts/postprocess_recording.sh --input paths --session 20260523_221812 --outputs video,frames,depth_csv,hdf5,rlds
```

## Project Layout

- `run_ur5_xbox_ubuntu.sh`: root launcher.
- `src/`: teleoperation, camera services, robot initialization, conversion, and postprocessing code.
- `scripts/`: setup, checks, conversion, postprocessing, and L515 service entry points.
- `config/`: public launcher config template and ignored local config override.
- `requirements/`: pip dependency lists split by robot, camera, dataset, and all modes.
- `env/`: Conda environment file, Ubuntu system package list, and `.env` template.
- `docs/`: detailed documentation.
- `paths/`: generated recordings and postprocessed datasets; ignored by git.

## Contributing

Issues and pull requests are welcome. Before opening a pull request, make sure local robot addresses, private lab paths, generated recordings, camera dumps, and machine-specific configs are not committed.

Recommended workflow:

```bash
git checkout -b feature/short-description
# make changes
git status
git diff
```

Then push the branch and open a pull request for review.

## License

This project is released under the MIT License. See [LICENSE](LICENSE).

## GitHub Notes

Do not commit generated or machine-local content:

- `.vendor/`
- `.tmp/`
- `.pip-cache/`
- `paths/`
- `config/teleop_launcher_config.local.json`
- recorded media/datasets such as `.bag`, `.mp4`, `.avi`, `.pkl`, `.hdf5`, `.h5`

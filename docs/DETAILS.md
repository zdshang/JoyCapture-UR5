# Detailed Documentation

This document contains the longer setup, configuration, recording, output layout, and dataset conversion notes for the UR5 Xbox teleoperation project.

## Tested Environment

- Ubuntu 24.04 LTS.
- Conda environment: `UR_xbox`.
- Python: 3.12.
- RealSense SDK / librealsense: tested locally with system `realsense2` 2.50.0.
- FFmpeg available on `PATH`.
- Cameras: Intel RealSense D455 and L515.
- Robot: UR5 / UR controller with External Control available on the teach pendant.
- Gripper: Robotiq socket control, commonly on port `63352`.

## Full Environment Setup

This repository publishes the runtime environment in several small files:

- `env/environment.yml`: base Conda environment, default name `UR_xbox`.
- `requirements/base.txt`: standard pip entry point for all Python dependencies.
- `requirements/robot.txt`: robot/control Python dependencies.
- `requirements/camera.txt`: camera Python dependencies.
- `requirements/dataset.txt`: HDF5/RLDS-style conversion dependencies.
- `requirements/all.txt`: combined robot + camera dependency list.
- `env/ubuntu_system_packages.txt`: Ubuntu packages needed for compiling/running the L515 system recorder.
- `env/env.example`: optional runtime environment variable template.

Install Ubuntu packages first:

```bash
sudo apt update
grep -vE '^\s*(#|$)' env/ubuntu_system_packages.txt | sudo xargs apt install -y
```

If `librealsense2-*` packages are not found, add Intel RealSense's official apt source for your Ubuntu version, then run the command again.

After installing RealSense udev rules, unplug and reconnect the cameras. A reboot is also acceptable.

Create the Conda environment:

```bash
conda env create -f env/environment.yml
conda activate UR_xbox
```

Install project Python dependencies into the local project vendor directory:

```bash
./scripts/setup_urxbox_env.sh all
./scripts/check_urxbox_env.sh all
```

For robot-only development:

```bash
./scripts/setup_urxbox_env.sh robot
./scripts/check_urxbox_env.sh robot
```

For camera-only development:

```bash
./scripts/setup_urxbox_env.sh camera
./scripts/check_urxbox_env.sh camera
```

For dataset conversion only:

```bash
./scripts/setup_urxbox_env.sh dataset
./scripts/check_urxbox_env.sh dataset
```

If `conda` is not on `PATH`, pass it explicitly:

```bash
./scripts/setup_urxbox_env.sh all --conda-bin /path/to/conda --env-name UR_xbox
./scripts/check_urxbox_env.sh all --conda-bin /path/to/conda --env-name UR_xbox
```

Optional runtime defaults can be copied from `env/env.example`:

```bash
cp env/env.example .env
```

`.env` is ignored by git. The run/setup/check scripts load it automatically if it exists.

If the Xbox controller cannot be read because of Linux input permissions, add the current user to the `input` group, then log out and log back in:

```bash
sudo usermod -aG input "$USER"
```

## External Control Notes

The launcher can release robot brakes, activate the Robotiq gripper, and start local camera service processes. Motion still requires the UR teach pendant External Control program to be available and runnable.

Before motion:

- The robot controller and Ubuntu PC must be on the same network.
- `robot_host` must match the robot controller IP.
- External Control should be ready to play on the pendant.
- The workspace should be clear.
- Motion speed should be reduced before the first live run on a new setup.

If RTDE connects but motion does not start, check the pendant program state, robot network settings, firewall rules, and conflicting URCaps such as EtherNet/IP, PROFINET, or MODBUS.

## Configuration Reference

`config/teleop_launcher_config.json` is a public template. For a real robot, create a local override:

```bash
cp config/teleop_launcher_config.json config/teleop_launcher_config.local.json
```

Then edit `config/teleop_launcher_config.local.json`.

Required parameters for a new machine:

- `robot_host`
  - UR controller IP address, for example `192.168.x.x`.
- `output_dir`
  - Task output root. Set to `paths/task_name`; each recording then creates a numbered raw folder below it when `recording.session_subdirs` is enabled. Command-line `--output-dir` overrides this value.
- `motion.xy_rotate_deg`
  - Translation direction compensation for the local tool/camera mounting.
- `motion.rot_axes_rotate_deg`
  - Rotation-axis compensation for the local tool/camera mounting.
- `gripper.mode`
  - Current supported production mode is `robotiq_socket`.
- `gripper.socket_port`
  - Robotiq URCap socket port, commonly `63352`.
- `gripper.activate_in_teleop`
  - Keep this `false` when `gripper_activate_mode` is enabled at launcher startup. The launcher activates the gripper once before teleoperation starts; teleop then skips a second activation to avoid resetting an already active gripper.
- `cameras[].device_name`
  - Device keyword used to select the RealSense camera, for example `D455` or `L515`.
- `cameras[].control_port`
  - Local TCP control port for each camera service. Keep ports unique.
- `cameras[].service_backend`
  - Use `system_cpp` for L515 on machines where Python `pyrealsense2` cannot see L515.
- `cameras[].fps`
  - Camera capture/BAG frame rate.
- `recording.robot_fps`
  - Robot pose/action recording target FPS. Set this to the same value as the camera FPS when synchronized training data is needed.
- `recording.require_fps_match`
  - Default is `true`; the launcher checks that `recording.robot_fps` matches all enabled camera stream FPS fields before running.
- `recording.record_interval_s`
  - Compatibility field. When `recording.robot_fps` is present, the launcher derives the interval from `robot_fps`.
- `recording.session_subdirs`
  - Default is `true`; each recording creates the next numbered raw folder under `output_dir`, such as `paths/task_name/1`, `paths/task_name/2`, and `paths/task_name/3`.
- `cameras[].deferred_postprocess`
  - Default is `true`; live recording stores lightweight `.bag` and timestamp data only.
  - Keep this enabled when robot motion stability and camera frame completeness are more important than immediate MP4/PNG files.
- `cameras[].export_fps`
  - Offline exported image/depth CSV frame rate. For example, with `fps: 30` and `export_fps: 10`, postprocessing exports one frame every three captured frames.
- `cameras[].export_max_frames`
  - Maximum number of offline exported image/depth CSV frames. `0` means unlimited.
- `recording.convert_on_stop`
  - Default is `false`; HDF5/RLDS conversion is not run during robot recording.
  - Set to `true` only if conversion should run immediately after `Y` stops recording.
- `recording.output_formats`
  - Dataset formats used only when `recording.convert_on_stop` is `true`.
  - Offline postprocessing can choose formats independently with `scripts/postprocess_recording.sh --outputs ...`.
- `recording.rlds_copy_media`
  - If `false`, RLDS-style output references raw image/depth paths to avoid duplicating data.
  - If `true`, image/depth files are copied into the RLDS-style directory.
- `recording.hdf5_embed_binary`
  - If `false`, HDF5 stores training arrays and metadata.
  - If `true`, `.mp4`, `.bag`, `.pkl`, and playback script files are embedded too, making output much larger.
- `playback.input_dir`
  - Raw recording directory or `session_manifest_*.json` used when `Back` loads a saved trajectory. Empty means use the current output directory.
- `playback.session`
  - Session id to replay from `playback.input_dir`, or `latest`.

Runtime parameters:

- `--conda-bin`
  - Optional path to `conda` if it is not on `PATH`. If `conda activate UR_xbox` works in the terminal, this is normally not needed.
- `--env-name`
  - Conda environment name. Default: `UR_xbox`.
- `--config`
  - Config JSON path. Default: `config/teleop_launcher_config.local.json` if present, else `config/teleop_launcher_config.json`.
- `--output-dir`
  - Output directory. Default: config `output_dir`, then `paths`.
- `--playback-dir`
  - Raw recording directory or `session_manifest_*.json` used by `Back` playback. Default: current output directory.
- `--playback-session`
  - Session id used by `Back` playback, or `latest`.

Environment variable equivalents:

- `UR5_CONDA_BIN`
- `UR5_ENV_NAME`
- `UR5_CONFIG_PATH`
- `UR5_OUTPUT_DIR`
- `UR5_PLAYBACK_DIR`
- `UR5_PLAYBACK_SESSION`

## Camera Behavior

- Recording starts when `Y` is pressed, not at program launch.
- During live teleoperation, D455 and L515 both record only lightweight source data: `.bag`, frame timestamps, metadata, and intrinsics.
- The default D455 config records color, depth, and infrared streams.
- The default L515 config records color and depth; infrared is disabled because the L515 setup used here is normally more stable without it.
- Robot pose/action rows are sampled by a dedicated recording thread at `recording.robot_fps`.
- MP4 video, exported PNG frames, depth PNG, depth visualization PNG, depth CSV, HDF5, and RLDS-style outputs are generated offline after recording.
- By default `export_fps` is `30`, so offline postprocessing exports every captured frame when the camera runs at 30 FPS. Lower it to reduce exported PNG/CSV volume.
- Start synchronization and stop synchronization are aligned in software using a shared host-side timestamp.

## Data Processing Model

Live recording prioritizes robot responsiveness and camera frame completeness. Pressing `Y` starts or stops one synchronized recording session. Robot pose/action sampling is performed by a dedicated recording thread at `recording.robot_fps`; camera capture uses each camera's `fps` / `depth_fps` / `infra_fps` fields. The default configuration sets all enabled streams to `30` FPS and requires them to match.

Each recording is saved as a numbered raw session directory under the task output root, for example:

```text
paths/task_name/1/
paths/task_name/2/
paths/task_name/3/
```

The live session writes raw source data only:

- robot pose/path CSV
- action CSV
- gripper event CSV
- dataset sample CSV
- session manifest JSON
- camera-robot sync CSV
- D455 `.bag`, timestamp CSV, metadata JSON, and intrinsics JSON
- L515 `.bag`, timestamp CSV, metadata JSON, and intrinsics JSON

The session manifest is the index for the whole recording. Offline tools read the manifest and raw files, then generate only the formats requested by `--outputs`.

FPS configuration:

- `recording.robot_fps`: target robot CSV/action sampling FPS.
- `recording.require_fps_match`: when `true`, the launcher refuses to run if enabled camera FPS fields do not match `recording.robot_fps`.
- `recording.record_interval_s`: legacy interval field kept for compatibility; `recording.robot_fps` takes priority when both are present.
- `recording.session_subdirs`: when `true`, each `Y` recording creates a numbered raw subdirectory under `output_dir`.
- `cameras[].fps`: RealSense color/BAG FPS.
- `cameras[].depth_fps`: RealSense depth FPS when depth recording is enabled.
- `cameras[].infra_fps`: RealSense infrared FPS when infrared recording is enabled.

To record at 15 FPS, set `recording.robot_fps`, `cameras[].fps`, and enabled depth/infra FPS fields to `15`. To record at 30 FPS, keep them all at `30`.

## Default Recording Output

The default live recording writes only the raw source data needed for reliable synchronization and later conversion:

```json
"robot_fps": 30,
"require_fps_match": true,
"convert_on_stop": false,
"output_formats": ["raw"]
```

Live outputs:

- robot path/action/gripper CSV files
- session manifest and sync CSV files
- per-camera `.bag`, timestamp CSV, metadata JSON, and intrinsics JSON

This keeps the recording loop light and prioritizes complete camera frame capture. MP4, PNG/CSV frame exports, HDF5, and RLDS-style outputs are selected during offline postprocessing.

## Output Layout

The default runtime output directory is `paths/`. It can be changed with `--output-dir` or `UR5_OUTPUT_DIR`.

The output directory is a task root when `recording.session_subdirs` is `true`. If `--output-dir paths/task_name` is used, the first recording is written under `paths/task_name/1/`, the second under `paths/task_name/2/`, and so on.

If `paths/` has been deleted, the program recreates the required subdirectories automatically on the next run.

Robot/session outputs inside one numbered raw folder:

- `paths/task_name/1/csv/`
- `paths/task_name/1/actions/`
- `paths/task_name/1/dataset_samples/`
- `paths/task_name/1/gripper_events/`
- `paths/task_name/1/script/`
- `paths/task_name/1/pkl/`
- `paths/task_name/1/sync/`
- `paths/task_name/1/session_metadata/`

Camera outputs:

- `paths/task_name/1/camera_bag/<label>/`
- `paths/task_name/1/camera_timestamps/<label>/`
- `paths/task_name/1/camera_metadata/<label>/`
- `paths/task_name/1/camera_intrinsics/<label>/`
- `paths/task_name/1/camera_video/<label>/` created by offline postprocessing
- `paths/task_name/1/camera_frames/<label>/` created by offline postprocessing
- `paths/task_name/1/camera_depth_csv/<label>/` created by offline postprocessing

Converted dataset outputs:

- `paths/task_name/1/hdf5/` created by offline postprocessing or manual conversion
- `paths/task_name/1/rlds/` created by offline postprocessing or manual conversion

## Dataset Formats

The raw multi-file format is always produced first because it is the easiest format to inspect and debug. The HDF5 and RLDS-style outputs are normally converted from that raw session during offline postprocessing.

Live recording defaults:

```json
"recording": {
  "robot_fps": 30,
  "require_fps_match": true,
  "session_subdirs": true,
  "record_interval_s": 0.0333333,
  "convert_on_stop": false,
  "output_formats": ["raw"],
  "rlds_copy_media": false,
  "hdf5_embed_binary": false
}
```

Supported formats:

- `raw`
  - Live source output: robot CSV, action CSV, gripper CSV, PKL, session JSON, sync CSV, camera `.bag`, camera timestamp CSV, camera metadata JSON, and camera intrinsics JSON.
  - Offline media output can add MP4, PNG frames, depth PNG, depth visualization PNG, and depth CSV.
  - Stored directly under one numbered raw folder, such as `paths/task_name/1/`.
  - Always produced first.
- `hdf5`
  - Single `.hdf5` episode file under the raw session directory, for example `paths/task_name/1/hdf5/ur5_episode_<session>.hdf5`.
  - Stores robot observations, actions, RL-style step fields, camera frame arrays when exported, raw tables, sync rows, and metadata.
  - Set `recording.hdf5_embed_binary` to `true` only if the HDF5 file must also contain large raw media such as `.bag` and `.mp4`.
- `rlds`
  - RLDS-style episode directory under the raw session directory, for example `paths/task_name/1/rlds/ur5_episode_<session>/`.
  - Uses RLDS field semantics: `observation`, `action`, `reward`, `discount`, `is_first`, `is_last`, and `is_terminal`.
  - This is a lightweight directory format and is not a TensorFlow Datasets package.

Common choices:

- Live recording default: only raw source data is written.
- Offline quick check: `--outputs video`.
- Offline training export: `--outputs frames,depth_csv,hdf5`.
- Offline compatibility export: `--outputs video,frames,depth_csv,hdf5,rlds`.

## Offline Postprocessing

Postprocess the latest recording with all common outputs:

```bash
./scripts/postprocess_recording.sh --input paths --session latest --outputs video,frames,depth_csv,hdf5,rlds
```

Examples:

```bash
# Only make quick review videos.
./scripts/postprocess_recording.sh --input paths --session latest --outputs video

# Export image/depth files for training, but skip HDF5/RLDS.
./scripts/postprocess_recording.sh --input paths --session latest --outputs frames,depth_csv

# Convert an already-postprocessed raw session into HDF5 only.
./scripts/postprocess_recording.sh --input paths --session latest --outputs hdf5

# Convert one concrete session id.
./scripts/postprocess_recording.sh --input paths --session 20260523_221812 --outputs video,frames,depth_csv,hdf5,rlds
```

Manual dataset-only conversion, when media has already been exported:

```bash
./scripts/convert_dataset_format.sh --from raw --to hdf5,rlds --input paths --session latest
./scripts/convert_dataset_format.sh --from hdf5 --to raw,rlds --input paths/task_name/1/hdf5/ur5_episode_<session>.hdf5
./scripts/convert_dataset_format.sh --from rlds --to raw,hdf5 --input paths/task_name/1/rlds/ur5_episode_<session>
```

Manual conversion accepts either `latest` or a concrete session id such as `20260523_213634`.

## Safety Notes

- Keep the workspace clear before enabling teleoperation.
- Verify robot IP and External Control state before motion.
- Verify the Xbox controller and both RealSense cameras are connected before recording.
- Reduce motion speed in the config before the first live run on a new setup.

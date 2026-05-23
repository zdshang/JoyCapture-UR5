#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"
CONFIG_DIR = REPO_ROOT / "config"
SCRIPTS_DIR = REPO_ROOT / "scripts"
DEFAULT_CONFIG_PATH = CONFIG_DIR / "teleop_launcher_config.json"
DEFAULT_LOCAL_CONFIG_PATH = CONFIG_DIR / "teleop_launcher_config.local.json"
TELEOP_SCRIPT = SRC_DIR / "ur5_xbox_rtde_clean.py"
INIT_SCRIPT = SRC_DIR / "ur5_init_only.py"
CAMERA_SERVICE_SCRIPT = SRC_DIR / "d455_recorder_service.py"
SYSTEM_L515_SERVICE_SCRIPT = SCRIPTS_DIR / "l515_system_recorder_service.sh"
OUTPUT_ROOT = REPO_ROOT / "paths"
PLAYBACK_INPUT_DIR = ""
PLAYBACK_SESSION = ""


def resolve_runtime_path(value: str | Path, *, base: Path = REPO_ROOT) -> Path:
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = base / path
    return path.resolve()


def resolve_config_path(explicit_config: str | None) -> Path:
    if explicit_config:
        return resolve_runtime_path(explicit_config)
    env_config = os.environ.get("UR5_CONFIG_PATH", "").strip()
    if env_config:
        return resolve_runtime_path(env_config)
    if DEFAULT_LOCAL_CONFIG_PATH.exists():
        return DEFAULT_LOCAL_CONFIG_PATH
    return DEFAULT_CONFIG_PATH


def load_config(config_path: Path) -> dict[str, object]:
    with config_path.open("r", encoding="utf-8") as f:
        return json.load(f)


def require_modules(modules: list[str]) -> None:
    missing: list[str] = []
    for module_name in modules:
        try:
            __import__(module_name)
        except Exception:
            missing.append(module_name)
    if missing:
        joined = ", ".join(missing)
        raise SystemExit(
            "UR_xbox environment is missing required Python modules: "
            f"{joined}\n"
            "Install them into the UR_xbox conda environment before launching."
        )


def wait_port(host: str, port: int, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            with socket.create_connection((host, port), timeout=0.5):
                return True
        except OSError:
            time.sleep(0.2)
    return False


def request_json(host: str, port: int, payload: dict[str, object], timeout_s: float = 2.0) -> dict[str, object]:
    with socket.create_connection((host, port), timeout=timeout_s) as sock:
        sock.settimeout(timeout_s)
        sock.sendall((json.dumps(payload, ensure_ascii=True) + "\n").encode("utf-8"))
        data = b""
        while not data.endswith(b"\n"):
            chunk = sock.recv(4096)
            if not chunk:
                break
            data += chunk
    if not data:
        return {}
    try:
        return json.loads(data.decode("utf-8", errors="replace").strip())
    except Exception:
        return {}


def stop_existing_service(host: str, port: int) -> None:
    try:
        _ = request_json(host, port, {"cmd": "shutdown"}, timeout_s=1.2)
    except Exception:
        pass
    time.sleep(0.3)


def build_camera_dirs(camera_label: str) -> dict[str, Path]:
    return {
        "video": OUTPUT_ROOT / "camera_video" / camera_label,
        "bag": OUTPUT_ROOT / "camera_bag" / camera_label,
        "timestamps": OUTPUT_ROOT / "camera_timestamps" / camera_label,
        "metadata": OUTPUT_ROOT / "camera_metadata" / camera_label,
        "intrinsics": OUTPUT_ROOT / "camera_intrinsics" / camera_label,
        "frames": OUTPUT_ROOT / "camera_frames" / camera_label,
        "depth_csv": OUTPUT_ROOT / "camera_depth_csv" / camera_label,
    }


def camera_export_fps(camera: dict[str, object]) -> float:
    capture_fps = max(1, int(camera.get("fps", 30) or 30))
    try:
        export_fps = float(camera.get("export_fps", capture_fps) or capture_fps)
    except (TypeError, ValueError):
        export_fps = float(capture_fps)
    return max(1.0, min(float(capture_fps), export_fps))


def camera_export_frame_every_n(camera: dict[str, object]) -> int:
    capture_fps = max(1, int(camera.get("fps", 30) or 30))
    if "export_fps" in camera:
        export_fps = camera_export_fps(camera)
        return max(1, int(round(capture_fps / export_fps)))
    return max(1, int(camera.get("export_frame_every_n", 1) or 1))


def robot_record_fps(recording: dict[str, object], cameras: list[object], camera_enabled: bool) -> float:
    for key in ("robot_fps", "fps"):
        try:
            fps = float(recording.get(key, 0) or 0)
        except (TypeError, ValueError):
            fps = 0.0
        if fps > 0:
            return fps
    try:
        interval = float(recording.get("record_interval_s", 0) or 0)
    except (TypeError, ValueError):
        interval = 0.0
    if interval > 0:
        return 1.0 / interval
    if camera_enabled:
        for camera in cameras:
            if isinstance(camera, dict):
                try:
                    fps = float(camera.get("fps", 0) or 0)
                except (TypeError, ValueError):
                    fps = 0.0
                if fps > 0:
                    return fps
    return 30.0


def require_recording_fps_match(
    recording: dict[str, object],
    cameras: list[object],
    camera_enabled: bool,
    target_robot_fps: float,
) -> None:
    if not camera_enabled or not bool(recording.get("require_fps_match", True)):
        return
    mismatches: list[str] = []
    for camera in cameras:
        if not isinstance(camera, dict):
            continue
        label = str(camera.get("label", camera.get("device_name", "camera")))
        for key in ("fps", "depth_fps", "infra_fps"):
            if key == "depth_fps" and not bool(camera.get("record_depth", True)):
                continue
            if key == "infra_fps" and not bool(camera.get("record_infra", False)):
                continue
            try:
                value = float(camera.get(key, camera.get("fps", target_robot_fps)) or target_robot_fps)
            except (TypeError, ValueError):
                value = target_robot_fps
            if abs(value - target_robot_fps) > 1e-6:
                mismatches.append(f"{label}.{key}={value:g}")
    if mismatches:
        joined = ", ".join(mismatches)
        raise SystemExit(
            "Recording FPS mismatch: robot_fps="
            f"{target_robot_fps:g}, but {joined}. "
            "Set recording.robot_fps and all enabled camera fps fields to the same value, "
            "or set recording.require_fps_match=false."
        )


def output_formats_arg(recording: dict[str, object]) -> str:
    raw_value = recording.get("output_formats", ["raw"])
    if isinstance(raw_value, str):
        values = [item.strip().lower() for item in raw_value.split(",") if item.strip()]
    elif isinstance(raw_value, list):
        values = [str(item).strip().lower() for item in raw_value if str(item).strip()]
    else:
        values = ["raw"]
    allowed = {"raw", "hdf5", "rlds"}
    selected = [item for item in values if item in allowed]
    return ",".join(selected or ["raw"])


def start_camera_services(config: dict[str, object]) -> list[subprocess.Popen[str]]:
    cameras = list(config.get("cameras", []))
    if not cameras:
        return []

    camera_host = str(config.get("camera_control_host", "127.0.0.1"))
    processes: list[subprocess.Popen[str]] = []
    for camera in cameras:
        if not isinstance(camera, dict):
            continue
        port = int(camera.get("control_port", 0) or 0)
        if port <= 0:
            raise SystemExit(f"Invalid camera control port in config: {camera}")
        label = str(camera.get("label", "camera")).strip() or "camera"
        backend = str(camera.get("service_backend", "python")).strip().lower() or "python"
        stop_existing_service(camera_host, port)
        if backend == "system_cpp":
            cmd = [str(SYSTEM_L515_SERVICE_SCRIPT), "--host", camera_host, "--port", str(port)]
        else:
            cmd = [sys.executable, "-u", str(CAMERA_SERVICE_SCRIPT), "--host", camera_host, "--port", str(port)]
        proc = subprocess.Popen(cmd, cwd=str(REPO_ROOT), text=True)
        if not wait_port(camera_host, port, timeout_s=8.0):
            try:
                proc.terminate()
            except Exception:
                pass
            raise SystemExit(f"Camera service for {label} failed to start on {camera_host}:{port}")
        ping = request_json(camera_host, port, {"cmd": "ping"}, timeout_s=2.0)
        if not ping.get("ok", False):
            raise SystemExit(f"Camera service for {label} is not responding correctly on port {port}")
        print(f"[launcher] camera service ready: {label} -> {camera_host}:{port}", flush=True)
        processes.append(proc)
    return processes


def stop_camera_services(config: dict[str, object], processes: list[subprocess.Popen[str]]) -> None:
    camera_host = str(config.get("camera_control_host", "127.0.0.1"))
    cameras = list(config.get("cameras", []))
    for camera in cameras:
        if not isinstance(camera, dict):
            continue
        port = int(camera.get("control_port", 0) or 0)
        if port > 0:
            stop_existing_service(camera_host, port)
    for proc in processes:
        try:
            proc.wait(timeout=3.0)
        except Exception:
            try:
                proc.terminate()
            except Exception:
                pass
    for proc in processes:
        if proc.poll() is None:
            try:
                proc.kill()
            except Exception:
                pass


def run_init_if_enabled(config: dict[str, object]) -> None:
    if not bool(config.get("auto_init", True)):
        return
    robot_host = str(config.get("robot_host", "")).strip()
    if not robot_host:
        raise SystemExit("launcher config must define robot_host")
    cmd = [
        sys.executable,
        "-u",
        str(INIT_SCRIPT),
        "--host",
        robot_host,
        "--gripper-activate-mode",
        str(config.get("gripper_activate_mode", "auto")),
    ]
    print("[launcher] running robot initialization...", flush=True)
    result = subprocess.run(cmd, cwd=str(REPO_ROOT), check=False)
    if result.returncode != 0:
        raise SystemExit(f"Robot initialization failed with exit code {result.returncode}")


def run_gripper_activate_if_enabled(config: dict[str, object]) -> None:
    mode = str(config.get("gripper_activate_mode", "auto")).strip().lower()
    if mode in {"", "none"}:
        return
    robot_host = str(config.get("robot_host", "")).strip()
    if not robot_host:
        raise SystemExit("launcher config must define robot_host")
    gripper = dict(config.get("gripper", {}))
    cmd = [
        sys.executable,
        "-u",
        str(INIT_SCRIPT),
        "--host",
        robot_host,
        "--mode",
        "gripper_activate_only",
        "--gripper-activate-mode",
        mode,
        "--robotiq-socket-port",
        str(gripper.get("socket_port", 63352)),
        "--robotiq-speed",
        str(gripper.get("speed", 180)),
        "--robotiq-force",
        str(gripper.get("force", 120)),
        "--robotiq-init-pos",
        str(gripper.get("open_pos", 0)),
    ]
    print("[launcher] activating gripper...", flush=True)
    result = subprocess.run(cmd, cwd=str(REPO_ROOT), check=False)
    if result.returncode != 0:
        raise SystemExit(f"Gripper activate failed with exit code {result.returncode}")


def run_brake_release_if_enabled(config: dict[str, object]) -> None:
    if not bool(config.get("brake_release_on_start", True)):
        return
    robot_host = str(config.get("robot_host", "")).strip()
    if not robot_host:
        raise SystemExit("launcher config must define robot_host")
    cmd = [
        sys.executable,
        "-u",
        str(INIT_SCRIPT),
        "--host",
        robot_host,
        "--mode",
        "brake_release_only",
        "--gripper-activate-mode",
        "none",
    ]
    print("[launcher] sending brake release...", flush=True)
    result = subprocess.run(cmd, cwd=str(REPO_ROOT), check=False)
    if result.returncode != 0:
        raise SystemExit(f"Brake release failed with exit code {result.returncode}")


def build_teleop_command(config: dict[str, object]) -> list[str]:
    robot_host = str(config.get("robot_host", "")).strip()
    if not robot_host or robot_host in {"ROBOT_IP_HERE", "192.168.0.10"}:
        raise SystemExit("launcher config must define robot_host")

    motion = dict(config.get("motion", {}))
    gripper = dict(config.get("gripper", {}))
    recording = dict(config.get("recording", {}))
    playback = dict(config.get("playback", {}))
    cameras = list(config.get("cameras", []))
    camera_enabled = bool(config.get("camera_enable", True)) and len(cameras) > 0
    target_robot_fps = robot_record_fps(recording, cameras, camera_enabled)
    require_recording_fps_match(recording, cameras, camera_enabled, target_robot_fps)

    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)

    teleop_args = [
        sys.executable,
        "-u",
        str(TELEOP_SCRIPT),
        "--host",
        robot_host,
        "--no-auto-init",
        "--gripper-mode",
        str(gripper.get("mode", "robotiq_socket")),
        "--translation-speed-mmps",
        str(motion.get("translation_speed_mmps", 90)),
        "--rotation-speed-degps",
        str(motion.get("rotation_speed_degps", 55)),
        "--xy-rotate-deg",
        str(motion.get("xy_rotate_deg", 0)),
        "--rot-axes-rotate-deg",
        str(motion.get("rot_axes_rotate_deg", 0)),
        "--acceleration",
        str(motion.get("acceleration", 0.45)),
        "--period-s",
        str(motion.get("period_s", 0.01)),
        "--deadzone",
        str(motion.get("deadzone", 0.14)),
        "--path-output-dir",
        str(OUTPUT_ROOT),
        "--record-session-subdirs" if bool(recording.get("session_subdirs", True)) else "--no-record-session-subdirs",
        "--playback-input-dir",
        str(resolve_runtime_path(PLAYBACK_INPUT_DIR or str(playback.get("input_dir", "") or OUTPUT_ROOT))),
        "--playback-session",
        str(PLAYBACK_SESSION or playback.get("session", "latest") or "latest"),
        "--record-interval-s",
        f"{1.0 / target_robot_fps:.9f}",
        "--robot-record-fps",
        f"{target_robot_fps:g}",
        "--dataset-output-formats",
        output_formats_arg(recording),
        "--home-velocity",
        str(motion.get("home_velocity", 0.12)),
        "--home-acceleration",
        str(motion.get("home_acceleration", 0.20)),
        "--playback-entry-velocity",
        str(motion.get("playback_entry_velocity", 0.08)),
        "--playback-entry-acceleration",
        str(motion.get("playback_entry_acceleration", 0.12)),
        "--path-playback-velocity",
        str(motion.get("playback_velocity", 0.10)),
        "--path-playback-acceleration",
        str(motion.get("playback_acceleration", 0.18)),
        "--robotiq-socket-port",
        str(gripper.get("socket_port", 63352)),
        "--robotiq-open-pos",
        str(gripper.get("open_pos", 0)),
        "--robotiq-close-pos",
        str(gripper.get("close_pos", 255)),
        "--robotiq-speed",
        str(gripper.get("speed", 180)),
        "--robotiq-force",
        str(gripper.get("force", 120)),
    ]

    activate_in_teleop = gripper.get("activate_in_teleop", None)
    if activate_in_teleop is None:
        skip_gripper_activate_in_teleop = bool(gripper.get("skip_activate", True))
    else:
        skip_gripper_activate_in_teleop = not bool(activate_in_teleop)

    if skip_gripper_activate_in_teleop:
        teleop_args.append("--robotiq-skip-activate")
    else:
        teleop_args.append("--no-robotiq-skip-activate")

    if bool(gripper.get("start_open", True)):
        teleop_args.append("--gripper-start-open")
    else:
        teleop_args.append("--no-gripper-start-open")

    if bool(gripper.get("open_on_home", True)):
        teleop_args.append("--open-gripper-on-home")
    else:
        teleop_args.append("--no-open-gripper-on-home")

    if bool(recording.get("rlds_copy_media", False)):
        teleop_args.append("--rlds-copy-media")
    else:
        teleop_args.append("--no-rlds-copy-media")

    if bool(recording.get("hdf5_embed_binary", False)):
        teleop_args.append("--hdf5-embed-binary")
    else:
        teleop_args.append("--no-hdf5-embed-binary")

    if bool(recording.get("convert_on_stop", False)):
        teleop_args.append("--convert-datasets-on-stop")
    else:
        teleop_args.append("--no-convert-datasets-on-stop")

    if camera_enabled:
        camera_names: list[str] = []
        camera_labels: list[str] = []
        camera_ports: list[str] = []
        camera_specs: list[dict[str, object]] = []
        first_camera = dict(cameras[0]) if cameras and isinstance(cameras[0], dict) else {}

        for camera in cameras:
            if not isinstance(camera, dict):
                continue
            name = str(camera.get("device_name", "")).strip()
            label = str(camera.get("label", "")).strip()
            port = int(camera.get("control_port", 0) or 0)
            if not name or not label or port <= 0:
                raise SystemExit(f"Invalid camera config entry: {camera}")
            camera_names.append(name)
            camera_labels.append(label)
            camera_ports.append(str(port))
            export_every_n = camera_export_frame_every_n(camera)
            camera_specs.append(
                {
                    "label": label,
                    "device_name": name,
                    "control_port": port,
                    "width": int(camera.get("width", 640)),
                    "height": int(camera.get("height", 480)),
                    "fps": int(camera.get("fps", 30)),
                    "depth_width": int(camera.get("depth_width", camera.get("width", 640))),
                    "depth_height": int(camera.get("depth_height", camera.get("height", 480))),
                    "depth_fps": int(camera.get("depth_fps", camera.get("fps", 30))),
                    "infra_width": int(camera.get("infra_width", camera.get("depth_width", 640))),
                    "infra_height": int(camera.get("infra_height", camera.get("depth_height", 480))),
                    "infra_fps": int(camera.get("infra_fps", camera.get("depth_fps", 30))),
                    "record_depth": bool(camera.get("record_depth", True)),
                    "record_infra": bool(camera.get("record_infra", False)),
                    "save_bag": bool(camera.get("save_bag", True)),
                    "deferred_postprocess": bool(camera.get("deferred_postprocess", True)),
                    "export_fps": camera_export_fps(camera),
                    "export_frame_every_n": export_every_n,
                    "export_max_frames": int(camera.get("export_max_frames", 0) or 0),
                }
            )

        first_dirs = build_camera_dirs(camera_labels[0])
        teleop_args.extend(
            [
                "--camera-enable",
                "--camera-process-mode",
                "--camera-specs-json",
                json.dumps(camera_specs, ensure_ascii=True),
                "--camera-device-names",
                ",".join(camera_names),
                "--camera-labels",
                ",".join(camera_labels),
                "--camera-control-ports",
                ",".join(camera_ports),
                "--camera-control-host",
                str(config.get("camera_control_host", "127.0.0.1")),
                "--camera-output-dir",
                str(first_dirs["video"].parent),
                "--camera-bag-output-dir",
                str(first_dirs["bag"].parent),
                "--camera-frame-ts-output-dir",
                str(first_dirs["timestamps"].parent),
                "--camera-metadata-output-dir",
                str(first_dirs["metadata"].parent),
                "--camera-intrinsics-output-dir",
                str(first_dirs["intrinsics"].parent),
                "--camera-frames-output-dir",
                str(first_dirs["frames"].parent),
                "--camera-depth-csv-output-dir",
                str(first_dirs["depth_csv"].parent),
                "--camera-width",
                str(first_camera.get("width", 640)),
                "--camera-height",
                str(first_camera.get("height", 480)),
                "--camera-fps",
                str(first_camera.get("fps", 30)),
                "--camera-depth-width",
                str(first_camera.get("depth_width", first_camera.get("width", 640))),
                "--camera-depth-height",
                str(first_camera.get("depth_height", first_camera.get("height", 480))),
                "--camera-depth-fps",
                str(first_camera.get("depth_fps", first_camera.get("fps", 30))),
                "--camera-infra-width",
                str(first_camera.get("infra_width", first_camera.get("depth_width", 640))),
                "--camera-infra-height",
                str(first_camera.get("infra_height", first_camera.get("depth_height", 480))),
                "--camera-infra-fps",
                str(first_camera.get("infra_fps", first_camera.get("depth_fps", 30))),
                "--camera-start-timeout-s",
                str(config.get("camera_start_timeout_s", 90)),
                "--camera-stop-timeout-s",
                str(config.get("camera_stop_timeout_s", 150)),
                "--camera-request-timeout-s",
                str(config.get("camera_request_timeout_s", 12)),
                "--camera-inter-start-delay-s",
                str(config.get("camera_inter_start_delay_s", 2.0)),
                "--camera-inter-stop-delay-s",
                str(config.get("camera_inter_stop_delay_s", 0.5)),
            ]
        )
        if bool(config.get("camera_auto_record_on_start", False)):
            teleop_args.append("--camera-auto-record-on-start")
        else:
            teleop_args.append("--no-camera-auto-record-on-start")
        if bool(first_camera.get("save_bag", True)):
            teleop_args.append("--camera-save-bag")
        else:
            teleop_args.append("--no-camera-save-bag")
        if bool(first_camera.get("deferred_postprocess", True)):
            teleop_args.append("--camera-deferred-postprocess")
        else:
            teleop_args.append("--no-camera-deferred-postprocess")
        if bool(first_camera.get("record_depth", True)):
            teleop_args.append("--camera-record-depth")
        else:
            teleop_args.append("--no-camera-record-depth")
        if bool(first_camera.get("record_infra", False)):
            teleop_args.append("--camera-record-infra")
        else:
            teleop_args.append("--no-camera-record-infra")
        teleop_args.extend(
            [
                "--camera-export-frame-every-n",
                str(camera_export_frame_every_n(first_camera)),
                "--camera-export-max-frames",
                str(int(first_camera.get("export_max_frames", 0) or 0)),
            ]
        )

    return teleop_args


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="UR5 Xbox Ubuntu launcher.")
    parser.add_argument(
        "--config",
        default="",
        help="Path to launcher config JSON. Default: UR5_CONFIG_PATH, then config/teleop_launcher_config.local.json, then config/teleop_launcher_config.json.",
    )
    parser.add_argument(
        "--output-dir",
        default=os.environ.get("UR5_OUTPUT_DIR", ""),
        help="Path for generated recordings. Default: UR5_OUTPUT_DIR, config output_dir, or ./paths.",
    )
    parser.add_argument(
        "--playback-dir",
        default=os.environ.get("UR5_PLAYBACK_DIR", ""),
        help="Raw paths directory or session_manifest_*.json used by Back playback. Default: playback.input_dir or --output-dir.",
    )
    parser.add_argument(
        "--playback-session",
        default=os.environ.get("UR5_PLAYBACK_SESSION", ""),
        help="Session id used by Back playback, or latest. Default: playback.session or latest.",
    )
    return parser.parse_args()


def main() -> int:
    global OUTPUT_ROOT, PLAYBACK_INPUT_DIR, PLAYBACK_SESSION

    args = parse_args()
    config_path = resolve_config_path(str(args.config).strip() or None)
    if not config_path.exists():
        raise SystemExit(f"Missing launcher config: {config_path}")

    config = load_config(config_path)
    config_output_dir = str(config.get("output_dir", "") or "").strip()
    if args.output_dir:
        OUTPUT_ROOT = resolve_runtime_path(str(args.output_dir))
    elif config_output_dir:
        OUTPUT_ROOT = resolve_runtime_path(config_output_dir)
    else:
        OUTPUT_ROOT = resolve_runtime_path(OUTPUT_ROOT)
    if args.playback_dir:
        PLAYBACK_INPUT_DIR = str(resolve_runtime_path(str(args.playback_dir)))
    if args.playback_session:
        PLAYBACK_SESSION = str(args.playback_session)

    required_modules = ["rtde_control", "rtde_receive", "inputs"]
    if bool(config.get("camera_enable", True)):
        required_modules.extend(["pyrealsense2", "cv2"])
    require_modules(required_modules)
    camera_processes: list[subprocess.Popen[str]] = []

    def _signal_handler(signum: int, _frame: object) -> None:
        print(f"[launcher] signal {signum} received, shutting down...", flush=True)
        stop_camera_services(config, camera_processes)
        raise SystemExit(130)

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    try:
        run_brake_release_if_enabled(config)
        run_gripper_activate_if_enabled(config)
        run_init_if_enabled(config)
        if bool(config.get("camera_enable", True)):
            camera_processes = start_camera_services(config)
        teleop_cmd = build_teleop_command(config)
        print("[launcher] starting teleop...", flush=True)
        result = subprocess.run(teleop_cmd, cwd=str(REPO_ROOT), check=False)
        return int(result.returncode)
    finally:
        stop_camera_services(config, camera_processes)


if __name__ == "__main__":
    raise SystemExit(main())

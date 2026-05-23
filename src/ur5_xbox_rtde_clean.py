#!/usr/bin/env python3
"""
Clean RTDE Xbox teleop for UR5 + Robotiq.

Single-file rewrite with one clear behavior:
- Use RTDE speedL for arm motion (low latency)
- Use dashboard runner for Robotiq open/close

Controller mapping:
- Left stick L/R : TCP left/right
- Left stick U/D : TCP up/down
- LB / RB        : TCP backward/forward
- LT / RT        : tool pitch
- Right stick L/R: tool self rotation
- Right stick U/D: wrist/end rotation
- X              : gripper toggle (open/close)
- Y              : start/stop path recording (save on stop)
- A              : set home pose (current TCP pose)
- B              : move to home pose
- Back           : playback latest recorded path
- Start          : exit
"""

from __future__ import annotations

import argparse
import atexit
import bisect
import copy
import csv
import ctypes
from datetime import datetime
import json
import math
import os
import pickle
import select
import re
import subprocess
import signal
import socket
import struct
import sys
import threading
import time
from pathlib import Path

from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
try:
    import cv2
    import numpy as np
    import pyrealsense2 as rs
except Exception:
    cv2 = None
    np = None
    rs = None

try:
    from inputs import UnpluggedError, get_gamepad
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "Missing dependency: inputs\n"
        "Install with: python -m pip install inputs"
    ) from exc

try:
    import evdev
except Exception:
    evdev = None


SCRIPT_VERSION = "rtde-clean-v18-ubuntu-dual-camera-robot-fps-thread"
REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "paths"
DEFAULT_CAMERA_VIDEO_DIR = DEFAULT_OUTPUT_ROOT / "camera_video"
DEFAULT_CAMERA_TS_DIR = DEFAULT_OUTPUT_ROOT / "camera_timestamps"
DEFAULT_CAMERA_BAG_DIR = DEFAULT_OUTPUT_ROOT / "camera_bag"
URSCRIPT_PORT = 30002
DASHBOARD_PORT = 29999


def split_csv_arg(value: str | None) -> list[str]:
    if value is None:
        return []
    return [part.strip() for part in value.split(",") if part.strip()]


def parse_camera_specs_json(value: str | None) -> list[dict[str, object]]:
    if not value:
        return []
    try:
        payload = json.loads(value)
    except Exception:
        return []
    if isinstance(payload, list):
        return [item for item in payload if isinstance(item, dict)]
    return []


def sanitize_camera_label(value: str, fallback: str) -> str:
    raw = value.strip().lower() or fallback
    label = re.sub(r"[^a-z0-9_-]+", "_", raw).strip("_")
    return label or fallback


def indent(text: str) -> str:
    return "\n".join(f"  {line}" if line.strip() else line for line in text.splitlines())


def wrap_program(body: str) -> str:
    return f"def pc_fallback_program():\n{indent(body)}\nend\npc_fallback_program()\n"


def relative_movel_script(
    dx: float,
    dy: float,
    dz: float,
    drx: float,
    dry: float,
    drz: float,
    acceleration: float,
    velocity: float,
) -> str:
    body = "\n".join(
        [
            "target = pose_trans(get_actual_tcp_pose(), "
            f"p[{dx:.6f}, {dy:.6f}, {dz:.6f}, {drx:.6f}, {dry:.6f}, {drz:.6f}])",
            f"movel(target, a={acceleration:.3f}, v={velocity:.3f})",
        ]
    )
    return wrap_program(body + "\n")


class URScriptFallbackClient:
    def __init__(self, host: str, timeout: float) -> None:
        self.host = host
        self.timeout = timeout
        self.sock: socket.socket | None = None

    def connect(self) -> None:
        if self.sock is not None:
            return
        self.sock = socket.create_connection((self.host, URSCRIPT_PORT), timeout=self.timeout)
        self.sock.settimeout(self.timeout)

    def close(self) -> None:
        if self.sock is not None:
            try:
                self.sock.close()
            finally:
                self.sock = None

    def send_script(self, script: str) -> None:
        payload = script.encode("utf-8")
        try:
            self.connect()
            assert self.sock is not None
            self.sock.sendall(payload)
        except OSError:
            self.close()
            self.connect()
            assert self.sock is not None
            self.sock.sendall(payload)


class XboxController:
    MAX_TRIG_VAL = float(2**8)
    MAX_JOY_VAL = float(2**15)

    def __init__(self) -> None:
        self.left_x = 0.0
        self.left_y = 0.0
        self.right_x = 0.0
        self.right_y = 0.0
        self.left_trigger = 0.0
        self.right_trigger = 0.0
        self.lb = 0
        self.rb = 0
        self.a = 0
        self.b = 0
        self.x = 0
        self.y = 0
        self.start = 0
        self.back = 0
        self.connected = False
        self.backend = "inputs"
        self._evdev_abs_ranges: dict[str, tuple[int, int]] = {}
        self._evdev_uses_gas_brake_triggers = False
        self._thread = threading.Thread(target=self._monitor, daemon=True)
        self._thread.start()

    def _canonical_event_code(self, code: object) -> str:
        if isinstance(code, (tuple, list)):
            parts = [str(item) for item in code]
            preferred = (
                "BTN_A",
                "BTN_B",
                "BTN_X",
                "BTN_Y",
                "BTN_SOUTH",
                "BTN_EAST",
                "BTN_WEST",
                "BTN_NORTH",
                "BTN_TL",
                "BTN_TR",
                "BTN_SELECT",
                "BTN_START",
                "BTN_MODE",
                "BTN_THUMBL",
                "BTN_THUMBR",
                "ABS_X",
                "ABS_Y",
                "ABS_RX",
                "ABS_RY",
                "ABS_Z",
                "ABS_RZ",
                "ABS_GAS",
                "ABS_BRAKE",
            )
            for item in preferred:
                if item in parts:
                    return item
            return parts[0] if parts else ""
        return str(code)

    def _apply_event(self, code: object, state: int) -> None:
        code = self._canonical_event_code(code)
        if code == "ABS_X":
            self.left_x = self._normalize_stick(code, state)
        elif code == "ABS_Y":
            self.left_y = self._normalize_stick(code, state)
        elif code == "ABS_RX":
            self.right_x = self._normalize_stick(code, state)
        elif code == "ABS_RY":
            self.right_y = self._normalize_stick(code, state)
        elif code == "ABS_Z":
            if self._evdev_uses_gas_brake_triggers:
                self.right_x = self._normalize_stick(code, state)
            else:
                self.left_trigger = self._normalize_trigger(code, state)
        elif code == "ABS_RZ":
            if self._evdev_uses_gas_brake_triggers:
                self.right_y = self._normalize_stick(code, state)
            else:
                self.right_trigger = self._normalize_trigger(code, state)
        elif code == "ABS_GAS":
            self.left_trigger = self._normalize_trigger(code, state)
        elif code == "ABS_BRAKE":
            self.right_trigger = self._normalize_trigger(code, state)
        elif code == "BTN_TL":
            self.lb = state
        elif code == "BTN_TR":
            self.rb = state
        elif code in ("BTN_A", "BTN_SOUTH"):
            self.a = state
        elif code in ("BTN_B", "BTN_EAST"):
            self.b = state
        elif code in ("BTN_X", "BTN_WEST"):
            self.x = state
        elif code in ("BTN_Y", "BTN_NORTH"):
            self.y = state
        elif code == "BTN_START":
            self.start = state
        elif code == "BTN_SELECT":
            self.back = state

    def _normalize_stick(self, code: str, state: int) -> float:
        range_info = self._evdev_abs_ranges.get(code)
        if range_info is not None:
            min_v, max_v = range_info
            if max_v > min_v:
                center = (min_v + max_v) / 2.0
                span = max(max_v - center, center - min_v)
                if span > 0:
                    return max(-1.0, min(1.0, (state - center) / span))
        return max(-1.0, min(1.0, state / self.MAX_JOY_VAL))

    def _normalize_trigger(self, code: str, state: int) -> float:
        range_info = self._evdev_abs_ranges.get(code)
        if range_info is not None:
            min_v, max_v = range_info
            if max_v > min_v:
                return max(0.0, min(1.0, (state - min_v) / float(max_v - min_v)))
        divisor = self.MAX_TRIG_VAL if state <= int(self.MAX_TRIG_VAL) else 1023.0
        return max(0.0, min(1.0, state / divisor))

    def _configure_evdev_axis_ranges(self, dev: object) -> None:
        if evdev is None:
            self._evdev_abs_ranges = {}
            self._evdev_uses_gas_brake_triggers = False
            return
        ranges: dict[str, tuple[int, int]] = {}
        try:
            caps = dev.capabilities()
            abs_caps = caps.get(evdev.ecodes.EV_ABS, [])
        except Exception:
            abs_caps = []
        for item in abs_caps:
            if not isinstance(item, tuple) or len(item) != 2:
                continue
            code_num, abs_info = item
            code_name = evdev.ecodes.bytype[evdev.ecodes.EV_ABS].get(code_num)
            if not code_name or abs_info is None:
                continue
            try:
                ranges[str(code_name)] = (int(abs_info.min), int(abs_info.max))
            except Exception:
                continue
        self._evdev_abs_ranges = ranges
        self._evdev_uses_gas_brake_triggers = "ABS_GAS" in ranges or "ABS_BRAKE" in ranges

    def _find_evdev_device(self) -> object | None:
        if evdev is None:
            return None
        try:
            devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        except Exception:
            return None
        for dev in devices:
            try:
                name = (dev.name or "").lower()
                caps = dev.capabilities(verbose=True)
            except Exception:
                continue
            has_abs = any("ABS_" in str(item) for values in caps.values() for item in values)
            has_btn = any("BTN_" in str(item) for values in caps.values() for item in values)
            if has_abs and has_btn and any(key in name for key in ("xbox", "gamepad", "controller", "joystick")):
                return dev
        return None

    def _find_event_device_from_procfs(self) -> str | None:
        proc_path = Path("/proc/bus/input/devices")
        if not proc_path.exists():
            return None
        try:
            text = proc_path.read_text(encoding="utf-8", errors="replace")
        except Exception:
            return None
        for block in text.split("\n\n"):
            lower = block.lower()
            if not any(key in lower for key in ("xbox", "gamepad", "controller", "joystick")):
                continue
            event_match = re.search(r"\bevent(\d+)\b", block)
            if event_match:
                return f"/dev/input/event{event_match.group(1)}"
        return None

    def _monitor_linux_event_device(self) -> None:
        device_path = self._find_event_device_from_procfs()
        if not device_path:
            raise RuntimeError("linux event backend could not find controller event device")
        EVENT_SIZE = 24
        EV_KEY = 0x01
        EV_ABS = 0x03
        while True:
            try:
                with open(device_path, "rb", buffering=0) as f:
                    self.backend = "linux-event"
                    self.connected = True
                    while True:
                        ready, _, _ = select.select([f], [], [], 1.0)
                        if not ready:
                            continue
                        data = f.read(EVENT_SIZE)
                        if len(data) != EVENT_SIZE:
                            raise OSError("short read from event device")
                        _, _, ev_type, code, value = struct.unpack("llHHI", data)
                        if ev_type == EV_KEY:
                            self._apply_linux_event_key(code, int(value))
                        elif ev_type == EV_ABS:
                            self._apply_linux_event_abs(code, int(value))
            except OSError:
                self.connected = False
                time.sleep(0.25)
            except Exception:
                self.connected = False
                time.sleep(0.25)

    def _apply_linux_event_key(self, code: int, value: int) -> None:
        key_map = {
            304: "BTN_SOUTH",
            305: "BTN_EAST",
            307: "BTN_NORTH",
            308: "BTN_WEST",
            310: "BTN_TL",
            311: "BTN_TR",
            314: "BTN_SELECT",
            315: "BTN_START",
        }
        mapped = key_map.get(code)
        if mapped is not None:
            self._apply_event(mapped, value)

    def _apply_linux_event_abs(self, code: int, value: int) -> None:
        abs_map = {
            0: "ABS_X",
            1: "ABS_Y",
            2: "ABS_Z",
            3: "ABS_RX",
            4: "ABS_RY",
            5: "ABS_RZ",
            9: "ABS_GAS",
            10: "ABS_BRAKE",
        }
        mapped = abs_map.get(code)
        if mapped is not None:
            self._apply_event(mapped, value)

    def _monitor_evdev(self) -> None:
        if evdev is None:
            raise RuntimeError("evdev backend unavailable")
        while True:
            dev = self._find_evdev_device()
            if dev is None:
                self.connected = False
                time.sleep(0.5)
                continue
            try:
                self.backend = "evdev"
                self._configure_evdev_axis_ranges(dev)
                self.connected = True
                for event in dev.read_loop():
                    if event.type not in (evdev.ecodes.EV_ABS, evdev.ecodes.EV_KEY):
                        continue
                    code = evdev.ecodes.bytype[event.type].get(event.code)
                    if code is None:
                        continue
                    self._apply_event(code, int(event.value))
            except OSError:
                self.connected = False
                time.sleep(0.25)
            except Exception:
                self.connected = False
                time.sleep(0.25)

    def _monitor(self) -> None:
        while True:
            try:
                events = get_gamepad()
                self.backend = "inputs"
                self.connected = True
                for event in events:
                    self._apply_event(event.code, int(event.state))
            except UnpluggedError:
                self.connected = False
                if evdev is not None:
                    try:
                        self._monitor_evdev()
                        continue
                    except Exception:
                        self.connected = False
                try:
                    self._monitor_linux_event_device()
                    continue
                except Exception:
                    self.connected = False
                time.sleep(0.25)

    def snapshot(self) -> dict[str, float | int | bool]:
        return {
            "left_x": self.left_x,
            "left_y": self.left_y,
            "right_x": self.right_x,
            "right_y": self.right_y,
            "left_trigger": self.left_trigger,
            "right_trigger": self.right_trigger,
            "lb": self.lb,
            "rb": self.rb,
            "a": self.a,
            "b": self.b,
            "x": self.x,
            "y": self.y,
            "start": self.start,
            "back": self.back,
            "connected": self.connected,
            "backend": self.backend,
        }


class RealSenseRecorder:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.enabled = bool(args.camera_enable) and cv2 is not None and np is not None and rs is not None
        self.pipeline: object | None = None
        self.writer: object | None = None
        self.video_path: Path | None = None
        self.bag_path: Path | None = None
        self.stop_event = threading.Event()
        self.capture_thread: threading.Thread | None = None
        self.active = False
        if args.camera_enable and not self.enabled:
            print("[camera] camera requested but pyrealsense2/cv2/numpy is unavailable.", flush=True)

    def start_session(self, session_id: str) -> Path | None:
        if not self.enabled or self.active:
            return None
        if os.name == "nt" and self.args.camera_close_viewer_on_record:
            self._close_realsense_viewer_if_running()
        out_dir = Path(self.args.camera_output_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        self.video_path = out_dir / f"realsense_{session_id}.mp4"
        self.bag_path = None
        last_exc: Exception | None = None
        for attempt in range(1, max(1, self.args.camera_start_retries) + 1):
            try:
                self.pipeline = rs.pipeline()  # type: ignore[union-attr]
                cfg = rs.config()  # type: ignore[union-attr]

                serial = self._select_device_serial()
                if serial:
                    cfg.enable_device(serial)  # type: ignore[union-attr]
                    print(f"[camera] selected device serial: {serial}", flush=True)

                if self.args.camera_save_bag:
                    bag_dir = Path(self.args.camera_bag_output_dir)
                    bag_dir.mkdir(parents=True, exist_ok=True)
                    self.bag_path = bag_dir / f"realsense_{session_id}.bag"
                    cfg.enable_record_to_file(str(self.bag_path))  # type: ignore[union-attr]

                cfg.enable_stream(
                    rs.stream.color,  # type: ignore[union-attr]
                    self.args.camera_width,
                    self.args.camera_height,
                    rs.format.bgr8,  # type: ignore[union-attr]
                    self.args.camera_fps,
                )
                if self.args.camera_record_depth:
                    cfg.enable_stream(
                        rs.stream.depth,  # type: ignore[union-attr]
                        self.args.camera_depth_width,
                        self.args.camera_depth_height,
                        rs.format.z16,  # type: ignore[union-attr]
                        self.args.camera_depth_fps,
                    )
                self.pipeline.start(cfg)  # type: ignore[union-attr]

                fourcc = cv2.VideoWriter_fourcc(*self.args.camera_codec)
                self.writer = cv2.VideoWriter(
                    str(self.video_path),
                    fourcc,
                    float(self.args.camera_fps),
                    (self.args.camera_width, self.args.camera_height),
                )
                self.stop_event.clear()
                self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
                self.active = True
                self.capture_thread.start()
                print(f"[camera] recording started: {self.video_path}", flush=True)
                if self.bag_path is not None:
                    print(f"[camera] bag recording started: {self.bag_path}", flush=True)
                if self.args.camera_record_depth:
                    print(
                        "[camera] depth stream recording enabled "
                        f"({self.args.camera_depth_width}x{self.args.camera_depth_height}@{self.args.camera_depth_fps})",
                        flush=True,
                    )
                return self.video_path
            except Exception as exc:
                last_exc = exc
                exc_text = str(exc)
                print(
                    f"[camera] start failed (attempt {attempt}/{self.args.camera_start_retries}): {exc}",
                    flush=True,
                )
                # On Windows, RealSense Viewer/Depth Quality Tool may hold the device.
                # If device appears missing while viewer is visibly streaming, force-release and retry.
                if os.name == "nt" and "No device connected" in exc_text:
                    self._close_realsense_viewer_if_running()
                self._close_resources()
                self.bag_path = None
                time.sleep(self.args.camera_retry_delay_s)
                continue

        if last_exc is not None:
            print(f"[camera] failed to start recording after retries: {last_exc}", flush=True)
        return None

    def _select_device_serial(self) -> str | None:
        try:
            ctx = rs.context()  # type: ignore[union-attr]
            devices = list(ctx.query_devices())  # type: ignore[union-attr]
        except Exception:
            return None
        if not devices:
            return None

        preferred = self.args.camera_device_name.strip().lower()
        first_serial: str | None = None
        for dev in devices:
            try:
                name = str(dev.get_info(rs.camera_info.name)).lower()  # type: ignore[union-attr]
            except Exception:
                name = ""
            try:
                serial = str(dev.get_info(rs.camera_info.serial_number))  # type: ignore[union-attr]
            except Exception:
                serial = ""
            if not serial:
                continue
            if first_serial is None:
                first_serial = serial
            if preferred and preferred in name:
                return serial
        return first_serial

    def _close_realsense_viewer_if_running(self) -> None:
        for proc in ("realsense-viewer.exe", "depth-quality-tool.exe"):
            try:
                result = subprocess.run(
                    ["taskkill", "/IM", proc, "/F"],
                    capture_output=True,
                    text=True,
                    check=False,
                )
                if result.returncode == 0:
                    print(f"[camera] {proc} was closed to release camera device.", flush=True)
            except Exception:
                pass
        time.sleep(0.6)

    def _capture_loop(self) -> None:
        assert self.pipeline is not None
        assert self.writer is not None
        while not self.stop_event.is_set():
            try:
                frames = self.pipeline.wait_for_frames(1000)  # type: ignore[union-attr]
                color = frames.get_color_frame()
                if not color:
                    continue
                frame = np.asanyarray(color.get_data())  # type: ignore[union-attr]
                self.writer.write(frame)  # type: ignore[union-attr]
            except Exception:
                time.sleep(0.01)

    def stop_session(self) -> Path | None:
        if not self.active:
            return self.video_path
        self.stop_event.set()
        if self.capture_thread is not None and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=1.0)
        out = self.video_path
        self._close_resources()
        self.active = False
        if out is not None:
            print(f"[camera] recording saved: {out}", flush=True)
        if self.bag_path is not None:
            print(f"[camera] bag saved: {self.bag_path}", flush=True)
        return out

    def _close_resources(self) -> None:
        try:
            if self.writer is not None:
                self.writer.release()  # type: ignore[union-attr]
        except Exception:
            pass
        try:
            if self.pipeline is not None:
                self.pipeline.stop()  # type: ignore[union-attr]
        except Exception:
            pass
        self.writer = None
        self.pipeline = None


class RemoteCameraRecorder:
    def __init__(self, args: argparse.Namespace, label: str | None = None) -> None:
        self.args = args
        self.enabled = bool(args.camera_enable)
        self.label = sanitize_camera_label(
            label or str(getattr(args, "camera_label", "") or getattr(args, "camera_device_name", "camera")),
            "camera",
        )
        self.last_video_path: Path | None = None
        self.last_bag_path: Path | None = None
        self.last_frame_ts_path: Path | None = None
        self.last_metadata_path: Path | None = None
        self.last_intrinsics_path: Path | None = None
        self.last_frame_export_dir: Path | None = None
        self.last_depth_csv_dir: Path | None = None
        self.last_frame_count = 0
        self.last_started_fps = 0
        self.last_response: dict[str, object] = {}
        self.start_in_progress = False
        if self.enabled:
            print(
                f"[camera:{self.label}] remote camera process mode enabled "
                f"({self.args.camera_control_host}:{self.args.camera_control_port})",
                flush=True,
            )

    def _request(self, payload: dict[str, object], timeout_s: float | None = None) -> dict[str, object]:
        req_timeout = float(timeout_s if timeout_s is not None else self.args.camera_request_timeout_s)
        try:
            with socket.create_connection(
                (self.args.camera_control_host, self.args.camera_control_port),
                timeout=req_timeout,
            ) as sock:
                sock.settimeout(req_timeout)
                sock.sendall((json.dumps(payload, ensure_ascii=True) + "\n").encode("utf-8"))
                data = b""
                while not data.endswith(b"\n"):
                    chunk = sock.recv(4096)
                    if not chunk:
                        break
                    data += chunk
        except TimeoutError:
            return {"ok": False, "error": "camera service request timed out"}
        except OSError as exc:
            return {"ok": False, "error": f"camera service socket error: {exc}"}
        if not data:
            return {"ok": False, "error": "empty response from camera service"}
        try:
            return json.loads(data.decode("utf-8", errors="replace").strip())
        except Exception as exc:
            return {"ok": False, "error": f"invalid response from camera service: {exc}"}

    def start_session(self, session_id: str, record_start_host_ns: int) -> Path | None:
        if not self.enabled:
            return None
        self.start_in_progress = True
        payload: dict[str, object] = {
            "cmd": "start",
            "session_id": session_id,
            "record_start_host_ns": int(record_start_host_ns),
            "camera_deferred_postprocess": bool(self.args.camera_deferred_postprocess),
            "camera_output_dir": self.args.camera_output_dir,
            "camera_bag_output_dir": self.args.camera_bag_output_dir,
            "camera_frame_ts_output_dir": self.args.camera_frame_ts_output_dir,
            "camera_metadata_output_dir": self.args.camera_metadata_output_dir,
            "camera_intrinsics_output_dir": self.args.camera_intrinsics_output_dir,
            "camera_frames_output_dir": self.args.camera_frames_output_dir,
            "camera_depth_csv_output_dir": self.args.camera_depth_csv_output_dir,
            "camera_save_bag": bool(self.args.camera_save_bag),
            "camera_width": int(self.args.camera_width),
            "camera_height": int(self.args.camera_height),
            "camera_fps": int(self.args.camera_fps),
            "camera_record_depth": bool(self.args.camera_record_depth),
            "camera_depth_width": int(self.args.camera_depth_width),
            "camera_depth_height": int(self.args.camera_depth_height),
            "camera_depth_fps": int(self.args.camera_depth_fps),
            "camera_record_infra": bool(self.args.camera_record_infra),
            "camera_infra_width": int(self.args.camera_infra_width),
            "camera_infra_height": int(self.args.camera_infra_height),
            "camera_infra_fps": int(self.args.camera_infra_fps),
            "camera_codec": str(self.args.camera_codec),
            "camera_device_name": str(self.args.camera_device_name),
            "camera_export_frame_every_n": int(self.args.camera_export_frame_every_n),
            "camera_export_max_frames": int(self.args.camera_export_max_frames),
            "camera_label": self.label,
        }
        resp = self._request(payload, timeout_s=self.args.camera_start_timeout_s)
        if not bool(resp.get("ok", False)):
            err = str(resp.get("error", ""))
            if "timed out" in err.lower():
                print(f"[camera:{self.label}] start timed out once, retrying after USB settle...", flush=True)
                time.sleep(2.0)
                resp = self._request(payload, timeout_s=max(self.args.camera_start_timeout_s, 120.0))
        if not bool(resp.get("ok", False)):
            print(f"[camera:{self.label}] start failed: {resp.get('error', 'unknown error')}", flush=True)
            self.start_in_progress = False
            return None
        self.last_response = dict(resp)
        video_path = str(resp.get("video_path", "")).strip()
        bag_path = str(resp.get("bag_path", "")).strip()
        frame_ts_path = str(resp.get("frame_ts_path", "")).strip()
        metadata_path = str(resp.get("metadata_path", "")).strip()
        intrinsics_path = str(resp.get("intrinsics_path", "")).strip()
        frame_export_dir = str(resp.get("frame_export_dir", "")).strip()
        depth_csv_dir = str(resp.get("depth_csv_dir", "")).strip()
        if video_path:
            self.last_video_path = Path(video_path)
            print(f"[camera:{self.label}] recording started: {self.last_video_path}", flush=True)
        if bag_path:
            self.last_bag_path = Path(bag_path)
            print(f"[camera:{self.label}] bag recording started: {self.last_bag_path}", flush=True)
        if frame_ts_path:
            self.last_frame_ts_path = Path(frame_ts_path)
            print(f"[camera:{self.label}] frame timestamps target: {self.last_frame_ts_path}", flush=True)
        if metadata_path:
            self.last_metadata_path = Path(metadata_path)
            print(f"[camera:{self.label}] metadata target: {self.last_metadata_path}", flush=True)
        if intrinsics_path:
            self.last_intrinsics_path = Path(intrinsics_path)
            print(f"[camera:{self.label}] intrinsics target: {self.last_intrinsics_path}", flush=True)
        if frame_export_dir:
            self.last_frame_export_dir = Path(frame_export_dir)
            print(f"[camera:{self.label}] frame export target: {self.last_frame_export_dir}", flush=True)
        if depth_csv_dir:
            self.last_depth_csv_dir = Path(depth_csv_dir)
            print(f"[camera:{self.label}] depth csv target: {self.last_depth_csv_dir}", flush=True)
        serial = str(resp.get("serial", "")).strip()
        if serial:
            print(f"[camera:{self.label}] selected device serial: {serial}", flush=True)
        started_depth = bool(resp.get("started_depth", False))
        started_infra = bool(resp.get("started_infra", False))
        started_bag = bool(resp.get("started_bag", False))
        print(
            f"[camera:{self.label}] stream mode: color=on, depth={'on' if started_depth else 'off'}, "
            f"infra={'on' if started_infra else 'off'}",
            flush=True,
        )
        print(f"[camera:{self.label}] bag mode: {'on' if started_bag else 'off'}", flush=True)
        codec = str(resp.get("video_codec", "")).strip()
        if codec:
            print(f"[camera:{self.label}] video codec: {codec}", flush=True)
        if bool(resp.get("postprocess_required", False)):
            print(f"[camera:{self.label}] lightweight live recording; export media offline after stop.", flush=True)
        self.last_started_fps = int(resp.get("camera_fps", 0) or 0)
        if self.last_started_fps > 0:
            print(f"[camera:{self.label}] started fps: {self.last_started_fps}", flush=True)
        self.start_in_progress = False
        return self.last_bag_path or self.last_video_path

    def status(self) -> dict[str, object]:
        if not self.enabled:
            return {"ok": False, "error": "camera recorder disabled"}
        resp = self._request({"cmd": "status"}, timeout_s=self.args.camera_request_timeout_s)
        if not bool(resp.get("ok", False)):
            return resp
        self.last_response = dict(resp)
        video_path = str(resp.get("video_path", "")).strip()
        bag_path = str(resp.get("bag_path", "")).strip()
        frame_ts_path = str(resp.get("frame_ts_path", "")).strip()
        metadata_path = str(resp.get("metadata_path", "")).strip()
        intrinsics_path = str(resp.get("intrinsics_path", "")).strip()
        frame_export_dir = str(resp.get("frame_export_dir", "")).strip()
        depth_csv_dir = str(resp.get("depth_csv_dir", "")).strip()
        if video_path:
            self.last_video_path = Path(video_path)
        if bag_path:
            self.last_bag_path = Path(bag_path)
        if frame_ts_path:
            self.last_frame_ts_path = Path(frame_ts_path)
        if metadata_path:
            self.last_metadata_path = Path(metadata_path)
        if intrinsics_path:
            self.last_intrinsics_path = Path(intrinsics_path)
        if frame_export_dir:
            self.last_frame_export_dir = Path(frame_export_dir)
        if depth_csv_dir:
            self.last_depth_csv_dir = Path(depth_csv_dir)
        self.last_frame_count = int(resp.get("frame_count", 0) or 0)
        self.last_started_fps = int(resp.get("camera_fps", self.last_started_fps) or self.last_started_fps or 0)
        return resp

    def mark_start(self, record_start_host_ns: int) -> dict[str, object]:
        if not self.enabled:
            return {"ok": False, "error": "camera recorder disabled"}
        resp = self._request(
            {"cmd": "mark_start", "record_start_host_ns": int(record_start_host_ns)},
            timeout_s=self.args.camera_request_timeout_s,
        )
        if bool(resp.get("ok", False)):
            self.last_frame_count = int(resp.get("frame_count", 0) or 0)
        return resp

    def stop_session(self) -> Path | None:
        if not self.enabled:
            return None
        wait_timeout = self.args.camera_stop_timeout_s
        if self.start_in_progress:
            wait_timeout = max(wait_timeout, self.args.camera_start_timeout_s + 30.0)
        resp = self._request({"cmd": "stop"}, timeout_s=wait_timeout)
        if not bool(resp.get("ok", False)):
            print(f"[camera:{self.label}] stop failed: {resp.get('error', 'unknown error')}", flush=True)
            vp = str(resp.get("video_path", "")).strip()
            if vp:
                print(f"[camera:{self.label}] reported video path: {vp}", flush=True)
            return None
        self.last_response = dict(resp)
        video_path = str(resp.get("video_path", "")).strip()
        bag_path = str(resp.get("bag_path", "")).strip()
        frame_ts_path = str(resp.get("frame_ts_path", "")).strip()
        metadata_path = str(resp.get("metadata_path", "")).strip()
        intrinsics_path = str(resp.get("intrinsics_path", "")).strip()
        frame_export_dir = str(resp.get("frame_export_dir", "")).strip()
        depth_csv_dir = str(resp.get("depth_csv_dir", "")).strip()
        if video_path:
            self.last_video_path = Path(video_path)
            print(f"[camera:{self.label}] recording saved: {self.last_video_path}", flush=True)
            size = int(resp.get("video_size", 0) or 0)
            if size > 0:
                print(f"[camera:{self.label}] video size: {size} bytes", flush=True)
        if bag_path:
            self.last_bag_path = Path(bag_path)
            print(f"[camera:{self.label}] bag saved: {self.last_bag_path}", flush=True)
        if frame_ts_path:
            self.last_frame_ts_path = Path(frame_ts_path)
            print(f"[camera:{self.label}] frame timestamps saved: {self.last_frame_ts_path}", flush=True)
        if metadata_path:
            self.last_metadata_path = Path(metadata_path)
            print(f"[camera:{self.label}] metadata saved: {self.last_metadata_path}", flush=True)
        if intrinsics_path:
            self.last_intrinsics_path = Path(intrinsics_path)
            print(f"[camera:{self.label}] intrinsics saved: {self.last_intrinsics_path}", flush=True)
        if frame_export_dir:
            self.last_frame_export_dir = Path(frame_export_dir)
        if depth_csv_dir:
            self.last_depth_csv_dir = Path(depth_csv_dir)
        self.last_frame_count = int(resp.get("frame_count", 0) or 0)
        if self.last_frame_count > 0:
            print(f"[camera:{self.label}] frame count: {self.last_frame_count}", flush=True)
        if bool(resp.get("postprocess_required", False)):
            return None
        return self.last_video_path

    def shutdown(self) -> None:
        if not self.enabled:
            return
        try:
            _ = self._request({"cmd": "shutdown"}, timeout_s=1.5)
        except Exception:
            pass

    def as_record(self) -> dict[str, object]:
        return {
            "label": self.label,
            "device_name": str(getattr(self.args, "camera_device_name", "")),
            "control_host": str(getattr(self.args, "camera_control_host", "")),
            "control_port": int(getattr(self.args, "camera_control_port", 0) or 0),
            "video_path": str(self.last_video_path or ""),
            "bag_path": str(self.last_bag_path or ""),
            "frame_ts_path": str(self.last_frame_ts_path or ""),
            "metadata_path": str(self.last_metadata_path or ""),
            "intrinsics_path": str(self.last_intrinsics_path or ""),
            "frame_export_dir": str(self.last_frame_export_dir or ""),
            "depth_csv_dir": str(self.last_depth_csv_dir or ""),
            "frame_count": int(self.last_frame_count),
            "camera_fps": int(self.last_started_fps),
            "serial": str(self.last_response.get("serial", "")),
            "pipeline_mode": str(self.last_response.get("pipeline_mode", "")),
            "started_depth": bool(self.last_response.get("started_depth", False)),
            "started_infra": bool(self.last_response.get("started_infra", False)),
            "started_bag": bool(self.last_response.get("started_bag", False)),
            "video_codec": str(self.last_response.get("video_codec", "")),
            "postprocess_required": bool(self.last_response.get("postprocess_required", False)),
            "postprocess_done": bool(self.last_response.get("postprocess_done", False)),
            "export_frame_every_n": int(
                self.last_response.get("export_frame_every_n", getattr(self.args, "camera_export_frame_every_n", 1)) or 1
            ),
            "export_max_frames": int(
                self.last_response.get("export_max_frames", getattr(self.args, "camera_export_max_frames", 0)) or 0
            ),
        }


class MultiRemoteCameraRecorder:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.enabled = bool(args.camera_enable)
        self.recorders: list[RemoteCameraRecorder] = []
        self.last_camera_records: list[dict[str, object]] = []
        self.last_video_path: Path | None = None
        self.last_bag_path: Path | None = None
        self.last_frame_ts_path: Path | None = None
        self.last_frame_count = 0
        self.last_started_fps = 0
        if not self.enabled:
            return

        spec_records = parse_camera_specs_json(getattr(args, "camera_specs_json", ""))
        spec_map: dict[str, dict[str, object]] = {}
        for spec in spec_records:
            label = sanitize_camera_label(str(spec.get("label", "") or ""), "")
            device_name = str(spec.get("device_name", "") or "").strip()
            if label:
                spec_map[label] = spec
            if device_name:
                spec_map[device_name.lower()] = spec

        names = split_csv_arg(getattr(args, "camera_device_names", "")) or [str(args.camera_device_name)]
        labels = split_csv_arg(getattr(args, "camera_labels", ""))
        ports = [int(p) for p in split_csv_arg(getattr(args, "camera_control_ports", ""))]
        base_port = int(args.camera_control_port)

        for idx, name in enumerate(names):
            cam_args = copy.copy(args)
            label = sanitize_camera_label(labels[idx] if idx < len(labels) else name, f"camera{idx + 1}")
            port = ports[idx] if idx < len(ports) else base_port + idx
            spec = spec_map.get(label) or spec_map.get(name.lower()) or {}
            cam_args.camera_device_name = name
            cam_args.camera_label = label
            cam_args.camera_control_port = port
            cam_args.camera_width = int(spec.get("width", getattr(args, "camera_width", 640)))
            cam_args.camera_height = int(spec.get("height", getattr(args, "camera_height", 480)))
            cam_args.camera_fps = int(spec.get("fps", getattr(args, "camera_fps", 30)))
            cam_args.camera_depth_width = int(spec.get("depth_width", getattr(args, "camera_depth_width", cam_args.camera_width)))
            cam_args.camera_depth_height = int(spec.get("depth_height", getattr(args, "camera_depth_height", cam_args.camera_height)))
            cam_args.camera_depth_fps = int(spec.get("depth_fps", getattr(args, "camera_depth_fps", cam_args.camera_fps)))
            cam_args.camera_infra_width = int(spec.get("infra_width", getattr(args, "camera_infra_width", cam_args.camera_depth_width)))
            cam_args.camera_infra_height = int(spec.get("infra_height", getattr(args, "camera_infra_height", cam_args.camera_depth_height)))
            cam_args.camera_infra_fps = int(spec.get("infra_fps", getattr(args, "camera_infra_fps", cam_args.camera_depth_fps)))
            cam_args.camera_record_depth = bool(spec.get("record_depth", getattr(args, "camera_record_depth", True)))
            cam_args.camera_record_infra = bool(spec.get("record_infra", getattr(args, "camera_record_infra", False)))
            cam_args.camera_save_bag = bool(spec.get("save_bag", getattr(args, "camera_save_bag", True)))
            cam_args.camera_deferred_postprocess = bool(
                spec.get("deferred_postprocess", getattr(args, "camera_deferred_postprocess", True))
            )
            cam_args.camera_export_frame_every_n = int(
                spec.get("export_frame_every_n", getattr(args, "camera_export_frame_every_n", 15))
            )
            cam_args.camera_export_max_frames = int(
                spec.get("export_max_frames", getattr(args, "camera_export_max_frames", 0))
            )
            if bool(getattr(args, "camera_separate_output_dirs", True)):
                cam_args.camera_output_dir = str(Path(args.camera_output_dir) / label)
                cam_args.camera_bag_output_dir = str(Path(args.camera_bag_output_dir) / label)
                cam_args.camera_frame_ts_output_dir = str(Path(args.camera_frame_ts_output_dir) / label)
                cam_args.camera_metadata_output_dir = str(Path(args.camera_metadata_output_dir) / label)
                cam_args.camera_intrinsics_output_dir = str(Path(args.camera_intrinsics_output_dir) / label)
                cam_args.camera_frames_output_dir = str(Path(args.camera_frames_output_dir) / label)
                cam_args.camera_depth_csv_output_dir = str(Path(args.camera_depth_csv_output_dir) / label)
            self.recorders.append(RemoteCameraRecorder(cam_args, label=label))

    def _refresh_last_records(self) -> None:
        self.last_camera_records = [rec.as_record() for rec in self.recorders]
        self.last_video_path = None
        self.last_bag_path = None
        self.last_frame_ts_path = None
        self.last_frame_count = 0
        self.last_started_fps = 0
        for rec in self.recorders:
            if self.last_video_path is None and rec.last_video_path is not None:
                self.last_video_path = rec.last_video_path
            if self.last_bag_path is None and rec.last_bag_path is not None:
                self.last_bag_path = rec.last_bag_path
            if self.last_frame_ts_path is None and rec.last_frame_ts_path is not None:
                self.last_frame_ts_path = rec.last_frame_ts_path
            self.last_frame_count += int(rec.last_frame_count)
            if rec.last_started_fps > 0 and self.last_started_fps == 0:
                self.last_started_fps = rec.last_started_fps

    def start_session(self, session_id: str, record_start_host_ns: int) -> Path | None:
        if not self.enabled:
            return None
        first_video: Path | None = None
        results: list[Path | None] = [None] * len(self.recorders)
        threads: list[threading.Thread] = []

        def _worker(idx: int, recorder: RemoteCameraRecorder) -> None:
            results[idx] = recorder.start_session(session_id, record_start_host_ns)

        for idx, recorder in enumerate(self.recorders):
            thread = threading.Thread(target=_worker, args=(idx, recorder), daemon=True)
            threads.append(thread)
            thread.start()
            if idx < len(self.recorders) - 1:
                time.sleep(max(0.0, float(self.args.camera_inter_start_delay_s)))
        for thread in threads:
            thread.join()
        for video in results:
            if first_video is None and video is not None:
                first_video = video
        self._refresh_last_records()
        return first_video

    def status(self) -> dict[str, object]:
        if not self.enabled:
            return {"ok": False, "error": "camera recorder disabled"}
        all_ok = True
        records: list[dict[str, object]] = []
        first_error = ""
        for recorder in self.recorders:
            resp = recorder.status()
            if not bool(resp.get("ok", False)):
                all_ok = False
                if not first_error:
                    first_error = str(resp.get("error", "unknown error"))
            records.append(recorder.as_record())
        self._refresh_last_records()
        result: dict[str, object] = {"ok": all_ok, "records": records}
        if first_error:
            result["error"] = first_error
        return result

    def mark_start(self, record_start_host_ns: int) -> dict[str, object]:
        if not self.enabled:
            return {"ok": False, "error": "camera recorder disabled"}
        all_ok = True
        first_error = ""
        for recorder in self.recorders:
            resp = recorder.mark_start(record_start_host_ns)
            if not bool(resp.get("ok", False)):
                all_ok = False
                if not first_error:
                    first_error = str(resp.get("error", "unknown error"))
        self._refresh_last_records()
        result: dict[str, object] = {"ok": all_ok, "record_start_host_ns": int(record_start_host_ns)}
        if first_error:
            result["error"] = first_error
        return result

    def stop_session(self) -> Path | None:
        if not self.enabled:
            return None
        first_video: Path | None = None
        results: list[Path | None] = [None] * len(self.recorders)
        threads: list[threading.Thread] = []

        def _worker(idx: int, recorder: RemoteCameraRecorder) -> None:
            results[idx] = recorder.stop_session()

        for idx, recorder in enumerate(self.recorders):
            thread = threading.Thread(target=_worker, args=(idx, recorder), daemon=True)
            threads.append(thread)
            thread.start()
        for thread in threads:
            thread.join()
        for video in results:
            if first_video is None and video is not None:
                first_video = video
        self._refresh_last_records()
        return first_video

    def shutdown(self) -> None:
        for recorder in self.recorders:
            recorder.shutdown()


class Teleop:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.gamepad = XboxController()
        self.enabled = bool(self.args.start_enabled)
        self.running = True
        self.was_moving = False
        # Start in RTDE mode; only switch to fallback when RTDE becomes unavailable.
        self.use_fallback = False
        self.last_buttons = {name: 0 for name in ("a", "b", "x", "y", "start", "back")}
        self.last_button_time = {name: 0.0 for name in ("a", "b", "x", "y", "start", "back")}
        self.last_gripper_time = 0.0
        self.last_rtde_restart_attempt = 0.0
        self.rtde_not_running_since = 0.0
        self.last_rtde_warn = 0.0
        self.last_fallback_error = 0.0
        self.rtde_reupload_attempts = 0
        self.rtde_needs_reconnect = False
        self.rtde_auto_reconnect_attempts = 0
        self.last_rtde_auto_reconnect_try = 0.0
        self.buttons_armed = False
        self.buttons_armed_since = 0.0
        self.require_center_on_resume = True
        self.waiting_center_msg_ts = 0.0
        self.gripper_is_open = bool(self.args.gripper_start_open)
        self.home_pose: list[float] | None = None
        self.recording = False
        self.recording_pending = False
        self.recorded_points: list[tuple[float, list[float]]] = []
        self.recorded_gripper_events: list[tuple[float, str]] = []
        self.recorded_action_rows: list[tuple[float, list[float], dict[str, float | int | bool]]] = []
        self.base_output_dir = Path(self.args.path_output_dir)
        self.current_record_output_dir = self.base_output_dir
        self.record_start_t = 0.0
        self.record_start_host_ns = 0
        self.last_record_t = 0.0
        self._record_lock = threading.RLock()
        self._pose_lock = threading.Lock()
        self._robot_record_stop = threading.Event()
        self._robot_record_thread: threading.Thread | None = None
        self._latest_action_speed = [0.0] * 6
        self._latest_action_snap = self.record_snapshot(self.gamepad.snapshot())
        self.last_saved_points: list[list[float]] = []
        self.last_saved_gripper_events: list[tuple[float, str]] = []
        self.last_saved_action_rows: list[tuple[float, list[float], dict[str, float | int | bool]]] = []
        self.record_session_id = ""
        self.playback_running = False
        self.playback_stop_requested = False
        self.playback_stop_armed = True
        self._playback_thread: threading.Thread | None = None
        self.fallback_client = URScriptFallbackClient(self.args.host, self.args.timeout)
        self.rtde_c: RTDEControl | None = None
        self.rtde_r: RTDEReceive | None = None
        self.last_rtde_receive_retry = 0.0
        self.last_pose_error_log_t = 0.0
        self._gripper_lock = threading.Lock()
        self._pending_gripper: str | None = None
        self._gripper_busy = False
        self._robotiq_socket_initialized = bool(self.args.robotiq_skip_activate)
        self._gripper_thread = threading.Thread(target=self._gripper_worker, daemon=True)
        self._gripper_thread.start()
        if self.args.camera_enable and self.args.camera_process_mode:
            if split_csv_arg(getattr(self.args, "camera_device_names", "")):
                self.camera_recorder = MultiRemoteCameraRecorder(args)
            else:
                self.camera_recorder = RemoteCameraRecorder(args)
        else:
            self.camera_recorder = RealSenseRecorder(args)
        self._camera_start_thread: threading.Thread | None = None
        self.camera_auto_recording = False
        self.camera_auto_session_id = ""
        self.camera_auto_start_host_ns = 0
        self._cleanup_done = False
        self._install_exit_hooks()

        print(f"[teleop] version: {SCRIPT_VERSION}", flush=True)
        if self.args.auto_init:
            self.run_startup_sequence()
        print(f"[teleop] connecting RTDE control to {self.args.host} ...", flush=True)
        self.init_rtde()
        self.start_camera_auto_record_if_enabled()

    def _install_exit_hooks(self) -> None:
        atexit.register(self._cleanup_on_exit)
        try:
            signal.signal(signal.SIGINT, self._signal_handler)
            signal.signal(signal.SIGTERM, self._signal_handler)
        except Exception:
            pass
        if os.name == "nt":
            self._install_windows_close_handler()

    def _signal_handler(self, signum: int, _frame: object) -> None:
        print(f"Signal {signum} received, stopping teleop safely.", flush=True)
        self.running = False
        self.enabled = False
        self.emergency_stop_best_effort()
        # Do not raise here; let normal shutdown/atexit flow finish cleanly.
        return

    def _install_windows_close_handler(self) -> None:
        if os.name != "nt":
            return
        kernel32 = ctypes.windll.kernel32
        handler_func_type = ctypes.WINFUNCTYPE(ctypes.c_bool, ctypes.c_uint)

        @handler_func_type
        def _console_ctrl_handler(ctrl_type: int) -> bool:
            # 0=CTRL_C,1=CTRL_BREAK,2=CTRL_CLOSE,5=LOGOFF,6=SHUTDOWN
            if ctrl_type in (0, 1, 2, 5, 6):
                try:
                    print("Console close signal received, sending emergency stop.", flush=True)
                    self.running = False
                    self.enabled = False
                    self.emergency_stop_best_effort()
                except Exception:
                    pass
            return False

        self._console_ctrl_handler_ref = _console_ctrl_handler
        kernel32.SetConsoleCtrlHandler(self._console_ctrl_handler_ref, True)

    def run_startup_sequence(self) -> None:
        print("[teleop] running startup initialization...", flush=True)
        startup_cmds = (
            "close safety popup",
            "unlock protective stop",
            "power on",
            "brake release",
        )
        for command in startup_cmds:
            try:
                reply = self._dashboard_command(command, timeout=self.args.dashboard_timeout_s)
                print(f"[init] {command} -> {reply}", flush=True)
            except Exception as exc:
                print(f"[init] {command} failed: {exc}", flush=True)
            time.sleep(0.15)

        if self.args.gripper_activate:
            try:
                print(f"[init] activating gripper via {self.args.gripper_activate} ...", flush=True)
                self.run_gripper_program(self.args.gripper_activate)
            except Exception as exc:
                print(f"[init] gripper activate failed: {exc}", flush=True)
        self.wait_dashboard_program_stopped(self.args.startup_settle_timeout_s)
        try:
            reply = self._dashboard_command("stop", timeout=self.args.dashboard_timeout_s)
            print(f"[init] stop -> {reply}", flush=True)
        except Exception as exc:
            print(f"[init] stop failed: {exc}", flush=True)
        self.wait_dashboard_program_stopped(self.args.startup_settle_timeout_s)

    def wait_dashboard_program_stopped(self, timeout_s: float) -> None:
        deadline = time.monotonic() + max(0.1, timeout_s)
        while time.monotonic() < deadline:
            try:
                state = self._dashboard_command("programState", timeout=self.args.dashboard_timeout_s)
            except Exception:
                time.sleep(0.2)
                continue
            state_low = state.lower()
            if "stopped" in state_low:
                return
            time.sleep(0.2)
        print("[init] warning: programState did not become STOPPED before timeout.", flush=True)

    def emergency_stop_best_effort(self) -> None:
        # Triple-layer stop: RTDE speedStop, Dashboard stop, URScript stopl.
        try:
            self.stop_robot()
        except Exception:
            pass
        try:
            if self.rtde_c is not None:
                self.rtde_c.speedStop(self.args.stop_acceleration)
        except Exception:
            pass
        try:
            self._dashboard_command("stop", timeout=self.args.dashboard_timeout_s)
        except Exception:
            pass
        try:
            self.fallback_client.send_script("stopl(2.0)\n")
        except Exception:
            pass

    def _cleanup_on_exit(self) -> None:
        if self._cleanup_done:
            return
        self._cleanup_done = True
        try:
            self.emergency_stop_best_effort()
        except Exception:
            pass

    def try_create_rtde(self, flags: int) -> RTDEControl:
        return RTDEControl(
            self.args.host,
            self.args.rtde_frequency,
            flags,
        )

    def init_rtde(self) -> None:
        verbose_flags = RTDEControl.FLAG_VERBOSE
        upload_flags = verbose_flags
        if self.args.rtde_upload_script:
            upload_flags |= RTDEControl.FLAG_UPLOAD_SCRIPT

        attempts: list[tuple[str, int]] = [("upload", upload_flags)]
        if self.args.rtde_upload_script:
            attempts.append(("attach", verbose_flags))

        last_error: Exception | None = None
        for label, flags in attempts:
            for attempt in range(1, self.args.rtde_init_retries + 1):
                try:
                    self.rtde_c = self.try_create_rtde(flags)
                except RuntimeError as exc:
                    last_error = exc
                    if "already in use" in str(exc).lower() and attempt < self.args.rtde_init_retries:
                        print(
                            f"[teleop] RTDE {label} attempt {attempt}/{self.args.rtde_init_retries} hit register conflict; retrying...",
                            flush=True,
                        )
                        time.sleep(self.args.rtde_init_retry_delay_s)
                        continue
                    print(f"[teleop] RTDE {label} failed: {exc}", flush=True)
                    break
                except Exception as exc:
                    last_error = exc
                    print(f"[teleop] RTDE {label} failed with unexpected error: {exc}", flush=True)
                    break
                else:
                    print(f"[teleop] RTDE connected via {label} mode.", flush=True)
                    try:
                        self.rtde_r = RTDEReceive(self.args.host)
                        print("[teleop] RTDE receive connected.", flush=True)
                    except Exception as exc:
                        self.rtde_r = None
                        print(f"[teleop] RTDE receive unavailable: {exc}", flush=True)
                    return

        self.use_fallback = bool(self.args.allow_fallback)
        if last_error is not None:
            if self.use_fallback:
                print(
                    "[teleop] Falling back to URScript socket motion. "
                    "Disable EtherNet/IP, PROFINET, MODBUS, or conflicting URCaps to restore RTDE.",
                    flush=True,
                )
            else:
                print(
                    "[teleop] RTDE unavailable and fallback disabled for safety. "
                    "Teleop motion will stay paused until RTDE is healthy.",
                    flush=True,
                )
                self.enabled = False
        else:
            if self.use_fallback:
                print("[teleop] Falling back to URScript socket motion.", flush=True)
            else:
                print("[teleop] RTDE unavailable and fallback disabled for safety.", flush=True)
                self.enabled = False

    def edge_pressed(self, snap: dict[str, float | int | bool], name: str) -> bool:
        current = int(snap[name])  # type: ignore[arg-type]
        previous = self.last_buttons[name]
        self.last_buttons[name] = current
        if current == 1 and previous == 0:
            now = time.monotonic()
            if now - self.last_button_time[name] >= self.args.button_cooldown_s:
                self.last_button_time[name] = now
                return True
        return False

    def button_released(self, snap: dict[str, float | int | bool], name: str) -> bool:
        current = int(snap[name])  # type: ignore[arg-type]
        self.last_buttons[name] = current
        return current == 0

    def clamp_deadzone(self, value: float) -> float:
        if abs(value) < self.args.deadzone:
            return 0.0
        return max(-1.0, min(1.0, value))

    def stop_robot(self) -> None:
        if self.was_moving and not self.use_fallback and self.rtde_c is not None:
            try:
                self.rtde_c.speedStop(self.args.stop_acceleration)
            except Exception:
                pass
            self.was_moving = False
        elif self.was_moving:
            self.was_moving = False

    def _send_secondary_urscript(self, lines: list[str]) -> None:
        script = wrap_program("\n".join(lines) + "\n")
        with socket.create_connection((self.args.host, URSCRIPT_PORT), timeout=self.args.timeout) as sock:
            sock.sendall(script.encode("utf-8"))

    def _send_robotiq_socket_commands(self, commands: list[str]) -> None:
        with socket.create_connection(
            (self.args.host, self.args.robotiq_socket_port),
            timeout=self.args.timeout,
        ) as sock:
            sock.settimeout(self.args.dashboard_timeout_s)
            for cmd in commands:
                sock.sendall((cmd.strip() + "\n").encode("utf-8"))
                try:
                    _ = sock.recv(128)
                except Exception:
                    # Some firmware/URCap combos do not always reply for each line.
                    pass

    def run_gripper_program(self, filepath: str) -> None:
        self._dashboard_command(f"load {filepath}", timeout=self.args.dashboard_timeout_s)
        # play occasionally does not reply in time when gripper program is busy.
        # We keep teleop responsive by treating play timeout as non-fatal.
        try:
            self._dashboard_command("play", timeout=self.args.dashboard_timeout_s)
        except TimeoutError:
            print("Dashboard play reply timed out; command was sent, teleop will continue.", flush=True)
        # Let gripper program run to completion whenever possible.
        # Only force stop if it appears stuck for too long.
        start = time.monotonic()
        while time.monotonic() - start < self.args.gripper_wait_done_timeout_s:
            try:
                running_reply = self._dashboard_command("running", timeout=self.args.dashboard_timeout_s)
            except Exception:
                time.sleep(0.10)
                continue
            if "false" in running_reply.lower():
                return
            time.sleep(0.08)
        print("Gripper program still running after timeout, sending stop.", flush=True)
        try:
            self._dashboard_command("stop", timeout=self.args.dashboard_timeout_s)
        except Exception:
            pass

    def run_gripper_action(self, action: str) -> None:
        if self.args.gripper_mode == "urscript":
            cmd = "rq_open()" if action == "open" else "rq_close()"
            self._send_secondary_urscript([cmd, f"sleep({self.args.gripper_urscript_sleep_s:.2f})"])
            return
        if self.args.gripper_mode == "robotiq_socket":
            pos = self.args.robotiq_open_pos if action == "open" else self.args.robotiq_close_pos
            cmds: list[str] = []
            if not self._robotiq_socket_initialized:
                # Activate once only; repeating ACT on every command can cause
                # visible reset behavior (open-close-open).
                if not self.args.robotiq_skip_activate:
                    cmds.extend(
                        [
                            "SET ACT 1",
                            "SET GTO 1",
                        ]
                    )
                self._robotiq_socket_initialized = True
            cmds.extend(
                [
                    "SET GTO 1",
                    f"SET SPE {self.args.robotiq_speed}",
                    f"SET FOR {self.args.robotiq_force}",
                    f"SET POS {pos}",
                ]
            )
            self._send_robotiq_socket_commands(cmds)
            return
        if action == "open":
            self.run_gripper_program(self.args.gripper_open)
        else:
            self.run_gripper_program(self.args.gripper_close)

    def _recv_dashboard_line(self, sock: socket.socket, timeout: float) -> str:
        sock.settimeout(timeout)
        data = sock.recv(4096)
        return data.decode("utf-8", errors="replace").strip()

    def _dashboard_command(self, command: str, timeout: float) -> str:
        with socket.create_connection((self.args.host, DASHBOARD_PORT), timeout=timeout) as sock:
            _ = self._recv_dashboard_line(sock, timeout)
            sock.sendall((command.strip() + "\n").encode("utf-8"))
            return self._recv_dashboard_line(sock, timeout)

    def request_gripper_action(self, action: str) -> bool:
        if action not in ("open", "close"):
            return False
        with self._gripper_lock:
            if self._gripper_busy:
                return False
            if self._pending_gripper is not None:
                return False
            self._pending_gripper = action
            return True

    def _gripper_worker(self) -> None:
        while self.running:
            action: str | None = None
            with self._gripper_lock:
                if self._pending_gripper is not None and not self._gripper_busy:
                    action = self._pending_gripper
                    self._pending_gripper = None
                    self._gripper_busy = True
            if action is None:
                time.sleep(0.01)
                continue
            try:
                self.run_gripper_action(action)
            except Exception as exc:
                print(f"Gripper command failed: {exc}", flush=True)
            finally:
                with self._gripper_lock:
                    self._gripper_busy = False
                if self.rtde_c is not None and not self.use_fallback:
                    try:
                        if not self.rtde_c.isProgramRunning():
                            self.rtde_needs_reconnect = True
                            self.stop_robot()
                            self.require_center_on_resume = True
                            if self.args.auto_reconnect_after_gripper:
                                print("RTDE dropped after gripper action. Auto reconnecting...", flush=True)
                            else:
                                self.enabled = False
                                print("RTDE dropped after gripper action. Restart teleop to reconnect.", flush=True)
                    except Exception:
                        self.rtde_needs_reconnect = True
                        self.stop_robot()
                        self.require_center_on_resume = True
                        if self.args.auto_reconnect_after_gripper:
                            print("RTDE state unknown after gripper action. Auto reconnecting...", flush=True)
                        else:
                            self.enabled = False
                            print("RTDE state unknown after gripper action. Restart teleop to reconnect.", flush=True)

    def is_gripper_busy(self) -> bool:
        with self._gripper_lock:
            return self._gripper_busy

    def get_actual_tcp_pose(self) -> list[float] | None:
        with self._pose_lock:
            now = time.monotonic()

            # Lazy/retry connect for RTDE receive channel.
            if self.rtde_r is None and now - self.last_rtde_receive_retry >= 1.0:
                self.last_rtde_receive_retry = now
                try:
                    self.rtde_r = RTDEReceive(self.args.host)
                    print("[teleop] RTDE receive reconnected.", flush=True)
                except Exception as exc:
                    if now - self.last_pose_error_log_t >= 2.0:
                        print(f"[teleop] pose read unavailable (RTDE receive connect failed): {exc}", flush=True)
                        self.last_pose_error_log_t = now

            if self.rtde_r is not None:
                try:
                    pose = self.rtde_r.getActualTCPPose()
                    if pose and len(pose) == 6:
                        return [float(v) for v in pose]
                    if now - self.last_pose_error_log_t >= 2.0:
                        print("[teleop] pose read failed: RTDE receive returned empty pose.", flush=True)
                        self.last_pose_error_log_t = now
                except Exception as exc:
                    # Drop and retry on next call.
                    self.rtde_r = None
                    if now - self.last_pose_error_log_t >= 2.0:
                        print(f"[teleop] pose read failed: {exc}", flush=True)
                        self.last_pose_error_log_t = now
        return None

    def set_home_pose(self) -> None:
        pose = self.get_actual_tcp_pose()
        if pose is None:
            print("Set home failed: cannot read current TCP pose.", flush=True)
            return
        self.home_pose = pose
        xyz_mm = [pose[0] * 1000.0, pose[1] * 1000.0, pose[2] * 1000.0]
        print(
            f"Home pose set: x={xyz_mm[0]:.1f}mm y={xyz_mm[1]:.1f}mm z={xyz_mm[2]:.1f}mm",
            flush=True,
        )

    def move_to_home_pose(self) -> None:
        if self.home_pose is None:
            print("Go home failed: home pose is not set. Press A first.", flush=True)
            return
        if self.use_fallback or self.rtde_c is None:
            print("Go home failed: RTDE motion is unavailable.", flush=True)
            return
        try:
            self.stop_robot()
            self.rtde_c.moveL(
                self.home_pose,
                self.args.home_velocity,
                self.args.home_acceleration,
                True,
            )
            if self.args.open_gripper_on_home:
                try:
                    self.run_gripper_action("open")
                    self.gripper_is_open = True
                    if self.recording:
                        t_rel = max(0.0, time.monotonic() - self.record_start_t)
                        self.recorded_gripper_events.append((t_rel, "open"))
                    print("Gripper opened at home pose.", flush=True)
                except Exception as exc:
                    print(f"Open gripper at home failed: {exc}", flush=True)
            print("Moved to home pose.", flush=True)
        except Exception as exc:
            print(f"Go home failed: {exc}", flush=True)

    def get_camera_records(self) -> list[dict[str, object]]:
        if getattr(self, "camera_auto_recording", False) and hasattr(self.camera_recorder, "status"):
            try:
                _ = self.camera_recorder.status()
            except Exception as exc:
                print(f"[camera] status refresh failed: {exc}", flush=True)
        records = getattr(self.camera_recorder, "last_camera_records", None)
        if records:
            return [dict(item) for item in records]
        if not self.args.camera_enable:
            return []
        return [
            {
                "label": "camera",
                "device_name": str(getattr(self.args, "camera_device_name", "")),
                "control_host": str(getattr(self.args, "camera_control_host", "")),
                "control_port": int(getattr(self.args, "camera_control_port", 0) or 0),
                "video_path": str(getattr(self.camera_recorder, "last_video_path", "") or ""),
                "bag_path": str(getattr(self.camera_recorder, "last_bag_path", "") or ""),
                "frame_ts_path": str(getattr(self.camera_recorder, "last_frame_ts_path", "") or ""),
                "metadata_path": str(getattr(self.camera_recorder, "last_metadata_path", "") or ""),
                "intrinsics_path": str(getattr(self.camera_recorder, "last_intrinsics_path", "") or ""),
                "frame_count": int(getattr(self.camera_recorder, "last_frame_count", 0)),
                "camera_fps": int(getattr(self.camera_recorder, "last_started_fps", self.args.camera_fps)),
            }
        ]

    def start_camera_auto_record_if_enabled(self) -> None:
        if not self.args.camera_enable or not getattr(self.args, "camera_auto_record_on_start", False):
            return
        if self.camera_auto_recording:
            return
        self.camera_auto_session_id = datetime.now().strftime("startup_%Y%m%d_%H%M%S")
        self.camera_auto_start_host_ns = time.monotonic_ns()
        print("[camera] auto-record on startup enabled, starting all cameras...", flush=True)
        try:
            try:
                camera_video = self.camera_recorder.start_session(
                    self.camera_auto_session_id,
                    self.camera_auto_start_host_ns,
                )
            except TypeError:
                camera_video = self.camera_recorder.start_session(self.camera_auto_session_id)
        except Exception as exc:
            print(f"[camera] auto-record start exception: {exc}", flush=True)
            return
        if camera_video is None:
            print("[camera] auto-record start failed; teleop will continue without startup camera recording.", flush=True)
            return
        self.camera_auto_recording = True
        time.sleep(0.08)
        print("[camera] startup recording is active.", flush=True)

    def stop_camera_auto_record_if_needed(self) -> None:
        if not self.args.camera_enable or not self.camera_auto_recording:
            return
        try:
            self.camera_recorder.stop_session()
        except Exception as exc:
            print(f"[camera] stop on exit failed: {exc}", flush=True)
        finally:
            self.camera_auto_recording = False

    def _start_camera_session_worker(self, session_id: str, record_start_host_ns: int) -> None:
        camera_started = False
        try:
            try:
                camera_video = self.camera_recorder.start_session(
                    session_id,
                    record_start_host_ns,
                )
            except TypeError:
                camera_video = self.camera_recorder.start_session(session_id)
            if camera_video is None:
                print(
                    "[camera] recording did not start. Path/gripper recording continues. "
                    "See the previous [camera] start failed reason for details.",
                    flush=True,
                )
            else:
                camera_started = True
        except Exception as exc:
            print(f"[camera] start exception: {exc}", flush=True)
        if camera_started:
            sync_host_ns = time.monotonic_ns()
            if hasattr(self.camera_recorder, "mark_start"):
                try:
                    mark_resp = self.camera_recorder.mark_start(sync_host_ns)
                    if not bool(mark_resp.get("ok", False)):
                        print(
                            f"[camera] mark_start failed: {mark_resp.get('error', 'unknown error')}",
                            flush=True,
                        )
                except Exception as exc:
                    print(f"[camera] mark_start exception: {exc}", flush=True)
            self.record_start_t = sync_host_ns / 1_000_000_000.0
            self.record_start_host_ns = sync_host_ns
            self.last_record_t = 0.0
            self.recording_pending = False
            if self.recording:
                self.start_robot_record_thread()
            print("[camera] camera session is active.", flush=True)
            print("Path recording is active.", flush=True)
        else:
            self.record_start_t = record_start_host_ns / 1_000_000_000.0
            self.record_start_host_ns = record_start_host_ns
            self.last_record_t = 0.0
            self.recording_pending = False
            if self.recording:
                self.start_robot_record_thread()
            print("Path recording is active without camera sync.", flush=True)

    def export_camera_sync(self, label: str, camera_ts_path: Path, sync_path: Path) -> None:
        robot_t = [t for t, _ in self.recorded_points]
        robot_pose = [p for _, p in self.recorded_points]

        with camera_ts_path.open("r", encoding="utf-8", newline="") as f_in, sync_path.open(
            "w", encoding="utf-8", newline=""
        ) as f_out:
            reader = csv.DictReader(f_in)
            writer = csv.writer(f_out)
            writer.writerow(
                [
                    "camera_label",
                    "frame_idx",
                    "camera_t_rel_s",
                    "camera_host_monotonic_ns",
                    "camera_system_time_ns",
                    "rs_frame_number",
                    "rs_timestamp_ms",
                    "robot_t_s_interp",
                    "robot_idx_left",
                    "robot_idx_right",
                    "interp_alpha",
                    "x_interp",
                    "y_interp",
                    "z_interp",
                    "rx_interp",
                    "ry_interp",
                    "rz_interp",
                    "dt_left_s",
                    "dt_right_s",
                ]
            )
            for row in reader:
                try:
                    cam_t = float(row.get("t_rel_s", "0"))
                except Exception:
                    continue
                if not robot_t:
                    continue
                right_idx = bisect.bisect_left(robot_t, cam_t)
                if right_idx <= 0:
                    left_idx = right_idx = 0
                    alpha = 0.0
                    interp_t = robot_t[0]
                    rp = robot_pose[0]
                elif right_idx >= len(robot_t):
                    left_idx = right_idx = len(robot_t) - 1
                    alpha = 0.0
                    interp_t = robot_t[left_idx]
                    rp = robot_pose[left_idx]
                else:
                    left_idx = right_idx - 1
                    t0 = robot_t[left_idx]
                    t1 = robot_t[right_idx]
                    denom = t1 - t0
                    if denom <= 1e-9:
                        alpha = 0.0
                        interp_t = t0
                        rp = robot_pose[left_idx]
                    else:
                        alpha = max(0.0, min(1.0, (cam_t - t0) / denom))
                        interp_t = t0 + alpha * denom
                        p0 = robot_pose[left_idx]
                        p1 = robot_pose[right_idx]
                        rp = [p0[i] * (1.0 - alpha) + p1[i] * alpha for i in range(6)]
                dt_left = abs(cam_t - robot_t[left_idx])
                dt_right = abs(robot_t[right_idx] - cam_t)
                writer.writerow(
                    [
                        label,
                        row.get("frame_idx", ""),
                        f"{cam_t:.6f}",
                        row.get("host_monotonic_ns", ""),
                        row.get("system_time_ns", ""),
                        row.get("rs_frame_number", ""),
                        row.get("rs_timestamp_ms", ""),
                        f"{interp_t:.6f}",
                        left_idx,
                        right_idx,
                        f"{alpha:.6f}",
                        f"{rp[0]:.6f}",
                        f"{rp[1]:.6f}",
                        f"{rp[2]:.6f}",
                        f"{rp[3]:.6f}",
                        f"{rp[4]:.6f}",
                        f"{rp[5]:.6f}",
                        f"{dt_left:.6f}",
                        f"{dt_right:.6f}",
                    ]
                )

    def allocate_record_output_dir(self) -> Path:
        base = self.base_output_dir.expanduser().resolve()
        if not bool(getattr(self.args, "record_session_subdirs", True)):
            base.mkdir(parents=True, exist_ok=True)
            return base
        base.mkdir(parents=True, exist_ok=True)
        used = [
            int(child.name)
            for child in base.iterdir()
            if child.is_dir() and child.name.isdigit()
        ]
        flat_session_count = len(list((base / "session_metadata").glob("session_manifest_*.json")))
        next_idx = max([*used, flat_session_count], default=0) + 1
        out_dir = base / str(next_idx)
        out_dir.mkdir(parents=True, exist_ok=False)
        return out_dir

    def camera_dirs_for_output(self, output_dir: Path, label: str) -> dict[str, Path]:
        return {
            "video": output_dir / "camera_video" / label,
            "bag": output_dir / "camera_bag" / label,
            "timestamps": output_dir / "camera_timestamps" / label,
            "metadata": output_dir / "camera_metadata" / label,
            "intrinsics": output_dir / "camera_intrinsics" / label,
            "frames": output_dir / "camera_frames" / label,
            "depth_csv": output_dir / "camera_depth_csv" / label,
        }

    def apply_camera_output_dirs(self, output_dir: Path) -> None:
        def _apply_to_args(target_args: argparse.Namespace, label: str) -> None:
            dirs = self.camera_dirs_for_output(output_dir, sanitize_camera_label(label, "camera"))
            target_args.camera_output_dir = str(dirs["video"])
            target_args.camera_bag_output_dir = str(dirs["bag"])
            target_args.camera_frame_ts_output_dir = str(dirs["timestamps"])
            target_args.camera_metadata_output_dir = str(dirs["metadata"])
            target_args.camera_intrinsics_output_dir = str(dirs["intrinsics"])
            target_args.camera_frames_output_dir = str(dirs["frames"])
            target_args.camera_depth_csv_output_dir = str(dirs["depth_csv"])

        label = sanitize_camera_label(str(getattr(self.args, "camera_label", "") or "camera"), "camera")
        _apply_to_args(self.args, label)
        recorder = getattr(self, "camera_recorder", None)
        if isinstance(recorder, MultiRemoteCameraRecorder):
            for remote in recorder.recorders:
                _apply_to_args(remote.args, remote.label)
        elif isinstance(recorder, RemoteCameraRecorder):
            _apply_to_args(recorder.args, recorder.label)
        elif isinstance(recorder, RealSenseRecorder):
            _apply_to_args(recorder.args, label)

    def prepare_record_output_dir(self) -> Path:
        out_dir = self.allocate_record_output_dir()
        self.current_record_output_dir = out_dir
        self.args.path_output_dir = str(out_dir)
        self.apply_camera_output_dirs(out_dir)
        print(f"Recording output root: {out_dir}", flush=True)
        return out_dir

    def _resolve_existing_raw_path(self, path_value: str | Path | None, raw_root: Path, fallback: Path) -> Path:
        if path_value:
            path = Path(str(path_value)).expanduser()
            if path.exists():
                return path
            if not path.is_absolute():
                candidate = raw_root / path
                if candidate.exists():
                    return candidate
        return fallback

    def find_session_manifests(self, raw_root: Path) -> list[Path]:
        return sorted(
            set(raw_root.rglob("session_metadata/session_manifest_*.json")),
            key=lambda path: path.stat().st_mtime,
        )

    def resolve_playback_manifest(self) -> Path:
        input_dir = str(getattr(self.args, "playback_input_dir", "") or "").strip()
        root = Path(input_dir).expanduser() if input_dir else Path(self.args.path_output_dir).expanduser()
        if root.is_file() and root.suffix.lower() == ".json":
            return root.resolve()
        raw_root = root.resolve()
        session = str(getattr(self.args, "playback_session", "latest") or "latest").strip()
        if session == "latest":
            manifests = self.find_session_manifests(raw_root)
            if not manifests:
                raise FileNotFoundError(f"No session manifests found under {raw_root}")
            return manifests[-1].resolve()
        matches = sorted(
            set(raw_root.rglob(f"session_metadata/session_manifest_{session}.json")),
            key=lambda path: path.stat().st_mtime,
        )
        if matches:
            return matches[-1].resolve()
        raise FileNotFoundError(f"Session manifest not found under {raw_root}: {session}")

    def load_playback_path_from_raw(self) -> bool:
        try:
            manifest_path = self.resolve_playback_manifest()
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            raw_root = manifest_path.parents[1]
            session_id = str(manifest.get("created_at", "") or manifest_path.stem.removeprefix("session_manifest_"))
            robot_csv = self._resolve_existing_raw_path(
                manifest.get("robot_path_csv", ""),
                raw_root,
                raw_root / "csv" / f"ur5_path_{session_id}.csv",
            )
            gripper_csv = self._resolve_existing_raw_path(
                manifest.get("gripper_events_csv", ""),
                raw_root,
                raw_root / "gripper_events" / f"ur5_gripper_events_{session_id}.csv",
            )
            points: list[tuple[float, list[float]]] = []
            with robot_csv.open("r", encoding="utf-8", newline="") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    try:
                        t_s = float(row["t_s"])
                        pose = [float(row[name]) for name in ("x", "y", "z", "rx", "ry", "rz")]
                    except Exception:
                        continue
                    points.append((t_s, pose))
            if not points:
                raise RuntimeError(f"No robot path rows loaded from {robot_csv}")

            gripper_events: list[tuple[float, str]] = []
            if gripper_csv.exists():
                with gripper_csv.open("r", encoding="utf-8", newline="") as f:
                    reader = csv.DictReader(f)
                    for row in reader:
                        try:
                            t_s = float(row.get("t_s", ""))
                        except Exception:
                            continue
                        action = str(row.get("action", "")).strip().lower()
                        if action in {"open", "close"}:
                            gripper_events.append((t_s, action))

            with self._record_lock:
                self.recorded_points = points
                self.last_saved_points = [pose[:] for _, pose in points]
                self.last_saved_gripper_events = gripper_events
                self.record_session_id = session_id
            print(
                f"Playback path loaded: session={session_id}, points={len(points)}, "
                f"gripper_events={len(gripper_events)}, source={manifest_path}",
                flush=True,
            )
            return True
        except Exception as exc:
            print(f"Playback load failed: {exc}", flush=True)
            return False

    def interpolate_pose_at(self, t_s: float) -> list[float] | None:
        if not self.recorded_points:
            return None
        robot_t = [t for t, _ in self.recorded_points]
        robot_pose = [p for _, p in self.recorded_points]
        right_idx = bisect.bisect_left(robot_t, t_s)
        if right_idx <= 0:
            return robot_pose[0][:]
        if right_idx >= len(robot_t):
            return robot_pose[-1][:]
        left_idx = right_idx - 1
        t0 = robot_t[left_idx]
        t1 = robot_t[right_idx]
        if t1 - t0 <= 1e-9:
            return robot_pose[left_idx][:]
        alpha = max(0.0, min(1.0, (t_s - t0) / (t1 - t0)))
        p0 = robot_pose[left_idx]
        p1 = robot_pose[right_idx]
        return [p0[i] * (1.0 - alpha) + p1[i] * alpha for i in range(6)]

    def record_snapshot(self, snap: dict[str, float | int | bool]) -> dict[str, float | int | bool]:
        return {
            "left_x": float(snap.get("left_x", 0.0)),
            "left_y": float(snap.get("left_y", 0.0)),
            "right_x": float(snap.get("right_x", 0.0)),
            "right_y": float(snap.get("right_y", 0.0)),
            "left_trigger": float(snap.get("left_trigger", 0.0)),
            "right_trigger": float(snap.get("right_trigger", 0.0)),
            "lb": int(snap.get("lb", 0)),
            "rb": int(snap.get("rb", 0)),
            "a": int(snap.get("a", 0)),
            "b": int(snap.get("b", 0)),
            "x": int(snap.get("x", 0)),
            "y": int(snap.get("y", 0)),
            "back": int(snap.get("back", 0)),
            "start": int(snap.get("start", 0)),
            "connected": bool(snap.get("connected", False)),
            "gripper_open": bool(self.gripper_is_open),
        }

    def update_latest_action(self, snap: dict[str, float | int | bool], speed: list[float]) -> None:
        with self._record_lock:
            self._latest_action_speed = [float(v) for v in speed]
            self._latest_action_snap = self.record_snapshot(snap)

    def start_robot_record_thread(self) -> None:
        if not self.recording or self.recording_pending:
            return
        if self._robot_record_thread is not None and self._robot_record_thread.is_alive():
            return
        self._robot_record_stop.clear()
        self._robot_record_thread = threading.Thread(target=self.robot_record_worker, daemon=True)
        self._robot_record_thread.start()

    def stop_robot_record_thread(self) -> None:
        self._robot_record_stop.set()
        thread = self._robot_record_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=max(2.0, float(getattr(self.args, "timeout", 1.0)) + 1.0))
        self._robot_record_thread = None

    def robot_record_worker(self) -> None:
        interval = max(0.001, float(self.args.record_interval_s))
        next_t = self.record_start_t
        while self.recording and not self._robot_record_stop.is_set():
            now = time.monotonic()
            delay = next_t - now
            if delay > 0:
                time.sleep(min(delay, 0.005))
                continue

            pose = self.get_actual_tcp_pose()
            sample_t = time.monotonic()
            t_rel = sample_t - self.record_start_t
            with self._record_lock:
                speed = self._latest_action_speed[:]
                snap = dict(self._latest_action_snap)
                if pose is not None:
                    self.recorded_points.append((t_rel, pose))
                self.recorded_action_rows.append((t_rel, speed, snap))
                self.last_record_t = sample_t

            next_t += interval
            late_by = time.monotonic() - next_t
            if late_by > interval:
                skipped = int(late_by // interval) + 1
                next_t += skipped * interval

    def start_recording(self) -> None:
        self.stop_robot()
        self.stop_robot_record_thread()
        with self._record_lock:
            self.recorded_points = []
            self.recorded_gripper_events = []
            self.recorded_action_rows = []
            self._latest_action_speed = [0.0] * 6
            self._latest_action_snap = self.record_snapshot(self.gamepad.snapshot())
        self.prepare_record_output_dir()
        self.record_session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.last_record_t = 0.0
        self._camera_start_thread = None
        self.recording = True
        self.recording_pending = True
        pending_host_ns = time.monotonic_ns()
        if self.args.camera_enable and not self.camera_auto_recording:
            print("[camera] preparing camera session before path recording...", flush=True)
            self._camera_start_thread = threading.Thread(
                target=self._start_camera_session_worker,
                args=(self.record_session_id, pending_host_ns),
                daemon=True,
            )
            self._camera_start_thread.start()
            print("Path recording pending: waiting for cameras to become active...", flush=True)
        else:
            self.record_start_t = pending_host_ns / 1_000_000_000.0
            self.record_start_host_ns = pending_host_ns
            self.last_record_t = 0.0
            self.recording_pending = False
            self.start_robot_record_thread()
            print("Path recording is active.", flush=True)

    def stop_recording_and_save(self) -> None:
        self.stop_robot()
        self.recording = False
        self.recording_pending = False
        self.stop_robot_record_thread()
        if self._camera_start_thread is not None and self._camera_start_thread.is_alive():
            wait_s = max(2.0, self.args.camera_start_timeout_s + 5.0)
            print(f"[camera] waiting for camera start thread to settle ({wait_s:.0f}s max)...", flush=True)
            self._camera_start_thread.join(timeout=wait_s)
        camera_video = None
        if self.args.camera_enable and not self.camera_auto_recording:
            try:
                camera_video = self.camera_recorder.stop_session()
            except Exception as exc:
                print(f"[camera] stop session exception: {exc}", flush=True)
        if not self.recorded_points and not self.recorded_action_rows:
            print("Path recording stopped: no robot points or action samples captured.", flush=True)
            return
        out_dir = Path(self.args.path_output_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        csv_dir = out_dir / "csv"
        action_dir = out_dir / "actions"
        dataset_dir = out_dir / "dataset_samples"
        gripper_events_dir = out_dir / "gripper_events"
        script_dir = out_dir / "script"
        pkl_dir = out_dir / "pkl"
        csv_dir.mkdir(parents=True, exist_ok=True)
        action_dir.mkdir(parents=True, exist_ok=True)
        dataset_dir.mkdir(parents=True, exist_ok=True)
        gripper_events_dir.mkdir(parents=True, exist_ok=True)
        script_dir.mkdir(parents=True, exist_ok=True)
        pkl_dir.mkdir(parents=True, exist_ok=True)
        ts = self.record_session_id or datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = csv_dir / f"ur5_path_{ts}.csv"
        action_csv_path = action_dir / f"ur5_actions_{ts}.csv"
        dataset_csv_path = dataset_dir / f"ur5_dataset_samples_{ts}.csv"
        script_path = script_dir / f"ur5_path_{ts}.script"
        pkl_path = pkl_dir / f"ur5_path_{ts}.pkl"

        with csv_path.open("w", encoding="utf-8") as f:
            f.write("t_s,x,y,z,rx,ry,rz\n")
            for t_s, p in self.recorded_points:
                f.write(
                    f"{t_s:.4f},{p[0]:.6f},{p[1]:.6f},{p[2]:.6f},{p[3]:.6f},{p[4]:.6f},{p[5]:.6f}\n"
                )
        self.last_saved_points = [p[:] for _, p in self.recorded_points]
        self.last_saved_gripper_events = list(self.recorded_gripper_events)
        self.last_saved_action_rows = [
            (
                t_s,
                speed[:],
                dict(snap),
            )
            for t_s, speed, snap in self.recorded_action_rows
        ]

        with action_csv_path.open("w", encoding="utf-8", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "t_s",
                    "cmd_vx",
                    "cmd_vy",
                    "cmd_vz",
                    "cmd_wx",
                    "cmd_wy",
                    "cmd_wz",
                    "left_x",
                    "left_y",
                    "right_x",
                    "right_y",
                    "left_trigger",
                    "right_trigger",
                    "lb",
                    "rb",
                    "a",
                    "b",
                    "x",
                    "y",
                    "back",
                    "start",
                    "controller_connected",
                    "gripper_open",
                ]
            )
            for t_s, speed, snap in self.recorded_action_rows:
                writer.writerow(
                    [
                        f"{t_s:.6f}",
                        f"{speed[0]:.6f}",
                        f"{speed[1]:.6f}",
                        f"{speed[2]:.6f}",
                        f"{speed[3]:.6f}",
                        f"{speed[4]:.6f}",
                        f"{speed[5]:.6f}",
                        f"{float(snap['left_x']):.6f}",
                        f"{float(snap['left_y']):.6f}",
                        f"{float(snap['right_x']):.6f}",
                        f"{float(snap['right_y']):.6f}",
                        f"{float(snap['left_trigger']):.6f}",
                        f"{float(snap['right_trigger']):.6f}",
                        int(snap["lb"]),
                        int(snap["rb"]),
                        int(snap["a"]),
                        int(snap["b"]),
                        int(snap["x"]),
                        int(snap["y"]),
                        int(snap["back"]),
                        int(snap["start"]),
                        int(bool(snap["connected"])),
                        int(bool(snap["gripper_open"])),
                    ]
                )

        with dataset_csv_path.open("w", encoding="utf-8", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "t_s",
                    "pose_x",
                    "pose_y",
                    "pose_z",
                    "pose_rx",
                    "pose_ry",
                    "pose_rz",
                    "cmd_vx",
                    "cmd_vy",
                    "cmd_vz",
                    "cmd_wx",
                    "cmd_wy",
                    "cmd_wz",
                    "left_x",
                    "left_y",
                    "right_x",
                    "right_y",
                    "left_trigger",
                    "right_trigger",
                    "lb",
                    "rb",
                    "gripper_open",
                ]
            )
            for t_s, speed, snap in self.recorded_action_rows:
                pose = self.interpolate_pose_at(t_s)
                pose_cols = [f"{v:.6f}" for v in pose] if pose is not None else [""] * 6
                writer.writerow(
                    [
                        f"{t_s:.6f}",
                        *pose_cols,
                        f"{speed[0]:.6f}",
                        f"{speed[1]:.6f}",
                        f"{speed[2]:.6f}",
                        f"{speed[3]:.6f}",
                        f"{speed[4]:.6f}",
                        f"{speed[5]:.6f}",
                        f"{float(snap['left_x']):.6f}",
                        f"{float(snap['left_y']):.6f}",
                        f"{float(snap['right_x']):.6f}",
                        f"{float(snap['right_y']):.6f}",
                        f"{float(snap['left_trigger']):.6f}",
                        f"{float(snap['right_trigger']):.6f}",
                        int(snap["lb"]),
                        int(snap["rb"]),
                        int(bool(snap["gripper_open"])),
                    ]
                )

        gripper_csv_path = gripper_events_dir / f"ur5_gripper_events_{ts}.csv"
        with gripper_csv_path.open("w", encoding="utf-8") as f:
            f.write("t_s,action\n")
            for t_s, action in self.recorded_gripper_events:
                f.write(f"{t_s:.4f},{action}\n")

        camera_records = self.get_camera_records()
        first_camera = camera_records[0] if camera_records else {}
        payload = {
            "created_at": ts,
            "host": self.args.host,
            "points": self.recorded_points,
            "gripper_events": self.recorded_gripper_events,
            "action_rows": self.recorded_action_rows,
            "meta": {
                "record_interval_s": self.args.record_interval_s,
                "robot_record_fps": self.args.robot_record_fps,
                "xy_rotate_deg": self.args.xy_rotate_deg,
                "gripper_mode": self.args.gripper_mode,
                "camera_enabled": bool(self.args.camera_enable),
                "camera_count": len(camera_records),
                "action_row_count": len(self.recorded_action_rows),
                "camera_fps": int(first_camera.get("camera_fps", self.args.camera_fps) or self.args.camera_fps),
                "camera_frame_count": sum(int(item.get("frame_count", 0) or 0) for item in camera_records),
                "camera_video_path": str(first_camera.get("video_path", "")),
                "camera_bag_path": str(first_camera.get("bag_path", "")),
                "camera_frame_ts_path": str(first_camera.get("frame_ts_path", "")),
                "cameras": camera_records,
            },
        }
        with pkl_path.open("wb") as f:
            pickle.dump(payload, f)

        sync_paths: list[str] = []
        if camera_records:
            sync_dir = out_dir / "sync"
            sync_dir.mkdir(parents=True, exist_ok=True)
            for record in camera_records:
                camera_ts = str(record.get("frame_ts_path", "") or "")
                if not camera_ts:
                    continue
                label = sanitize_camera_label(str(record.get("label", "") or "camera"), "camera")
                sync_path = sync_dir / f"ur5_camera_sync_{label}_{ts}.csv"
                try:
                    self.export_camera_sync(label, Path(camera_ts), sync_path)
                    sync_paths.append(str(sync_path))
                    record["sync_path"] = str(sync_path)
                    print(f"Camera-robot sync saved ({label}): {sync_path}", flush=True)
                except Exception as exc:
                    print(f"Camera-robot sync export failed ({label}): {exc}", flush=True)

        lines = ["def recorded_path()"]
        event_idx = 0
        for t_s, p in self.recorded_points:
            while event_idx < len(self.recorded_gripper_events) and self.recorded_gripper_events[event_idx][0] <= t_s:
                _, action = self.recorded_gripper_events[event_idx]
                lines.append(f"  # gripper event at {t_s:.3f}s: {action}")
                if self.args.gripper_mode == "urscript":
                    lines.append(f"  {'rq_open()' if action == 'open' else 'rq_close()'}")
                event_idx += 1
            lines.append(
                f"  movel(p[{p[0]:.6f},{p[1]:.6f},{p[2]:.6f},{p[3]:.6f},{p[4]:.6f},{p[5]:.6f}], "
                f"a={self.args.path_playback_acceleration:.3f}, v={self.args.path_playback_velocity:.3f})"
            )
        lines.append("end")
        lines.append("recorded_path()")
        script_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

        metadata_dir = out_dir / "session_metadata"
        metadata_dir.mkdir(parents=True, exist_ok=True)
        manifest_path = metadata_dir / f"session_manifest_{ts}.json"
        manifest = {
            "created_at": ts,
            "host": self.args.host,
            "script_version": SCRIPT_VERSION,
            "robot_path_csv": str(csv_path),
            "actions_csv": str(action_csv_path),
            "dataset_samples_csv": str(dataset_csv_path),
            "gripper_events_csv": str(gripper_csv_path),
            "playback_script": str(script_path),
            "pickle": str(pkl_path),
            "sync_csv": sync_paths,
            "point_count": len(self.recorded_points),
            "gripper_event_count": len(self.recorded_gripper_events),
            "action_row_count": len(self.recorded_action_rows),
            "record_interval_s": self.args.record_interval_s,
            "robot_record_fps": self.args.robot_record_fps,
            "controller_mapping": {
                "left_stick": "tcp_up_down_left_right",
                "lb_rb": "tcp_backward_forward",
                "lt_rt": "tool_pitch",
                "right_stick_lr": "tool_self_rotation",
                "right_stick_ud": "wrist_end_rotation",
                "x": "gripper_toggle",
                "y": "record_toggle",
                "a": "set_home",
                "b": "go_home",
                "back": "playback_toggle",
                "start": "exit",
            },
            "cameras": camera_records,
        }
        manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

        dataset_formats = {item.strip().lower() for item in split_csv_arg(self.args.dataset_output_formats)}
        if bool(self.args.convert_datasets_on_stop) and dataset_formats & {"hdf5", "rlds"}:
            try:
                from dataset_format_converter import export_session

                dataset_outputs = export_session(
                    manifest_path,
                    sorted(dataset_formats),
                    output_root=out_dir,
                    rlds_copy_media=bool(self.args.rlds_copy_media),
                    hdf5_embed_binary=bool(self.args.hdf5_embed_binary),
                )
                for fmt, path in sorted(dataset_outputs.items()):
                    if fmt != "raw":
                        print(f"Dataset export saved ({fmt}): {path}", flush=True)
            except Exception as exc:
                print(f"Dataset format export failed: {exc}", flush=True)
        elif dataset_formats & {"hdf5", "rlds"}:
            print("Dataset conversion skipped during recording; run scripts/postprocess_recording.sh when needed.", flush=True)

        print(f"Path saved: {csv_path}", flush=True)
        print(f"Action samples saved: {action_csv_path} ({len(self.recorded_action_rows)} rows)", flush=True)
        print(f"Dataset samples saved: {dataset_csv_path}", flush=True)
        print(f"Gripper events saved: {gripper_csv_path} ({len(self.recorded_gripper_events)} events)", flush=True)
        print(f"URScript saved: {script_path}", flush=True)
        print(f"PKL saved: {pkl_path}", flush=True)
        print(f"Session manifest saved: {manifest_path}", flush=True)
        if camera_video is not None:
            print(f"First camera video saved: {camera_video}", flush=True)
        for record in camera_records:
            label = record.get("label", "camera")
            postprocess_required = bool(record.get("postprocess_required", False))
            if record.get("video_path") and not postprocess_required:
                print(f"Camera video saved ({label}): {record['video_path']}", flush=True)
            elif record.get("video_path") and postprocess_required:
                print(f"Camera video target ({label}, offline export): {record['video_path']}", flush=True)
            if record.get("bag_path"):
                print(f"Camera bag saved ({label}): {record['bag_path']}", flush=True)
            if record.get("frame_ts_path"):
                print(f"Camera frame timestamps saved ({label}): {record['frame_ts_path']}", flush=True)
            if record.get("metadata_path"):
                print(f"Camera metadata saved ({label}): {record['metadata_path']}", flush=True)
            if record.get("intrinsics_path"):
                print(f"Camera intrinsics saved ({label}): {record['intrinsics_path']}", flush=True)

    def playback_latest_path(self) -> None:
        if self.playback_running:
            print("Playback is already running.", flush=True)
            return
        self.playback_running = True
        self.playback_stop_requested = False
        if self.recording:
            print("Playback ignored: stop recording first.", flush=True)
            self.playback_running = False
            return
        if not self.last_saved_points:
            if not self.load_playback_path_from_raw():
                print("Playback failed: no recorded path in memory or configured raw playback folder.", flush=True)
                self.playback_running = False
                return
        if self.use_fallback or self.rtde_c is None:
            print("Playback failed: RTDE motion is unavailable.", flush=True)
            self.playback_running = False
            return
        try:
            self.stop_robot()
            if not self.ensure_rtde_ready_for_playback():
                print(
                    "Playback failed: RTDE control script is not running. "
                    "Press Play on the teach pendant External Control program and try again.",
                    flush=True,
                )
                return
            # Remove near-duplicate points so playback is smoother and faster.
            filtered: list[list[float]] = []
            for pose in self.last_saved_points:
                if not filtered:
                    filtered.append(pose)
                    continue
                prev = filtered[-1]
                dp = abs(pose[0] - prev[0]) + abs(pose[1] - prev[1]) + abs(pose[2] - prev[2])
                dr = abs(pose[3] - prev[3]) + abs(pose[4] - prev[4]) + abs(pose[5] - prev[5])
                if dp >= 0.0008 or dr >= 0.01:
                    filtered.append(pose)

            print(f"Playback started: {len(filtered)} points.", flush=True)
            # Build timeline by reusing recorded timestamps aligned to filtered poses.
            timeline: list[tuple[float, list[float]]] = []
            f_idx = 0
            for t_s, p in self.recorded_points:
                if f_idx >= len(filtered):
                    break
                fp = filtered[f_idx]
                if all(abs(fp[i] - p[i]) < 1e-9 for i in range(6)):
                    timeline.append((t_s, fp))
                    f_idx += 1
            if len(timeline) < len(filtered):
                # Fallback to uniform timing when exact mapping misses.
                dt = max(self.args.period_s, self.args.record_interval_s)
                timeline = [(i * dt, p) for i, p in enumerate(filtered)]

            use_servo = hasattr(self.rtde_c, "servoL")
            # First move to configured home pose (if available), then to first path point.
            if self.home_pose is not None:
                if self.playback_stop_requested:
                    print("Playback stopped.", flush=True)
                    return
                self.rtde_c.moveL(
                    self.home_pose,
                    self.args.playback_entry_velocity,
                    self.args.playback_entry_acceleration,
                    True,
                )
                if self.args.open_gripper_on_home:
                    try:
                        self.run_gripper_action("open")
                        self.gripper_is_open = True
                    except Exception as exc:
                        print(f"Playback home gripper open failed: {exc}", flush=True)

            first_pose = timeline[0][1]
            if self.playback_stop_requested:
                print("Playback stopped.", flush=True)
                return
            self.rtde_c.moveL(
                first_pose,
                self.args.playback_entry_velocity,
                self.args.playback_entry_acceleration,
                False,
            )
            start_t = time.monotonic()
            event_idx = 0
            for t_s, pose in timeline:
                if self.playback_stop_requested:
                    print("Playback stopped.", flush=True)
                    return
                while True:
                    elapsed = time.monotonic() - start_t
                    while event_idx < len(self.last_saved_gripper_events) and self.last_saved_gripper_events[event_idx][0] <= elapsed:
                        _, action = self.last_saved_gripper_events[event_idx]
                        try:
                            self.run_gripper_action(action)
                        except Exception as exc:
                            print(f"Playback gripper event failed ({action}): {exc}", flush=True)
                        event_idx += 1
                    if elapsed >= t_s:
                        break
                    time.sleep(0.002)
                    if self.playback_stop_requested:
                        print("Playback stopped.", flush=True)
                        return

                if use_servo:
                    self.rtde_c.servoL(
                        pose,
                        self.args.path_playback_velocity,
                        self.args.path_playback_acceleration,
                        self.args.period_s,
                        self.args.servo_lookahead_time,
                        self.args.servo_gain,
                    )
                else:
                    self.rtde_c.moveL(
                        pose,
                        self.args.path_playback_velocity,
                        self.args.path_playback_acceleration,
                        False,
                    )
            # Ensure trailing gripper events (after last pose timestamp) are not lost.
            while event_idx < len(self.last_saved_gripper_events):
                if self.playback_stop_requested:
                    print("Playback stopped.", flush=True)
                    return
                _, action = self.last_saved_gripper_events[event_idx]
                try:
                    self.run_gripper_action(action)
                except Exception as exc:
                    print(f"Playback trailing gripper event failed ({action}): {exc}", flush=True)
                event_idx += 1
            try:
                self.rtde_c.speedStop(self.args.stop_acceleration)
            except Exception:
                pass
            print("Playback finished.", flush=True)
        except Exception as exc:
            print(f"Playback failed: {exc}", flush=True)
        finally:
            self.playback_running = False
            self.playback_stop_requested = False

    def start_playback_async(self) -> None:
        if self.playback_running:
            print("Playback is already running.", flush=True)
            return
        self._playback_thread = threading.Thread(target=self.playback_latest_path, daemon=True)
        self._playback_thread.start()

    def stop_playback(self) -> None:
        if not self.playback_running:
            print("Playback is not running.", flush=True)
            return
        self.playback_stop_requested = True
        try:
            if self.rtde_c is not None:
                self.rtde_c.speedStop(self.args.stop_acceleration)
                self.rtde_c.stopL(self.args.stop_acceleration)
        except Exception:
            pass
        print("Playback stop requested.", flush=True)

    def motion_input_is_centered(self, snap: dict[str, float | int | bool]) -> bool:
        axis_ok = (
            abs(float(snap["left_x"])) < self.args.center_threshold
            and abs(float(snap["left_y"])) < self.args.center_threshold
            and abs(float(snap["right_x"])) < self.args.center_threshold
            and abs(float(snap["right_y"])) < self.args.center_threshold
            and float(snap["left_trigger"]) < self.args.center_threshold
            and float(snap["right_trigger"]) < self.args.center_threshold
        )
        shoulder_ok = int(snap["lb"]) == 0 and int(snap["rb"]) == 0
        return axis_ok and shoulder_ok

    def ensure_rtde_program_running(self) -> bool:
        if self.use_fallback or self.rtde_c is None:
            return False

        try:
            if self.rtde_c.isProgramRunning():
                self.rtde_not_running_since = 0.0
                self.rtde_reupload_attempts = 0
                self.rtde_needs_reconnect = False
                return True
        except Exception:
            pass

        now = time.monotonic()
        if self.rtde_not_running_since == 0.0:
            self.rtde_not_running_since = now

        if now - self.last_rtde_warn >= 1.0:
            print(
                "RTDE control script is not running. Teleop waiting for reconnect. "
                "请先在示教器按下 External Control 程序 Play。",
                flush=True,
            )
            self.last_rtde_warn = now
        self.rtde_needs_reconnect = True

        if self.rtde_reupload_attempts >= self.args.rtde_max_reupload_attempts:
            self.enabled = False
            self.stop_robot()
            self.running = False
            print(
                "RTDE control script restart failed repeatedly. Exiting teleop to avoid stuck state.",
                flush=True,
            )
            return False

        if now - self.rtde_not_running_since >= self.args.rtde_restart_timeout_s:
            if self.args.allow_fallback:
                self.use_fallback = True
                print("Switching to URScript fallback motion mode.")
            else:
                self.enabled = False
                self.stop_robot()
                print("RTDE control script not running. Motion paused for safety (fallback disabled).")
        return False

    def manual_reconnect_rtde(self) -> bool:
        if self.use_fallback or self.rtde_c is None:
            return False
        try:
            if self.rtde_c.isProgramRunning():
                self.rtde_needs_reconnect = False
                return True
        except Exception:
            pass
        for _ in range(max(1, self.args.manual_rtde_reconnect_attempts)):
            try:
                self.rtde_c.reuploadScript()
                time.sleep(0.10)
                if self.rtde_c.isProgramRunning():
                    self.rtde_needs_reconnect = False
                    self.rtde_reupload_attempts = 0
                    print("RTDE reconnect successful.", flush=True)
                    return True
            except Exception:
                time.sleep(0.10)
        print("RTDE reconnect failed.", flush=True)
        return False

    def ensure_rtde_ready_for_playback(self) -> bool:
        if self.use_fallback or self.rtde_c is None:
            return False
        try:
            if self.rtde_c.isProgramRunning():
                return True
        except Exception:
            pass
        print("Playback waiting for RTDE control script...", flush=True)
        if self.manual_reconnect_rtde():
            return True
        try:
            self.rtde_c.reuploadScript()
            time.sleep(0.20)
            return bool(self.rtde_c.isProgramRunning())
        except Exception as exc:
            print(f"Playback RTDE reconnect failed: {exc}", flush=True)
            return False

    def auto_reconnect_rtde_if_needed(self) -> bool:
        if self.use_fallback or self.rtde_c is None:
            return False
        if not self.rtde_needs_reconnect:
            self.rtde_auto_reconnect_attempts = 0
            return True
        if not self.args.auto_reconnect_after_gripper:
            return False

        now = time.monotonic()
        if now - self.last_rtde_auto_reconnect_try < self.args.auto_reconnect_interval_s:
            return False
        self.last_rtde_auto_reconnect_try = now

        if self.rtde_auto_reconnect_attempts >= self.args.auto_reconnect_max_attempts:
            self.enabled = False
            print("Auto reconnect failed after multiple attempts. Restart teleop.", flush=True)
            return False

        self.rtde_auto_reconnect_attempts += 1
        print(
            f"Auto reconnect attempt {self.rtde_auto_reconnect_attempts}/{self.args.auto_reconnect_max_attempts}...",
            flush=True,
        )
        if self.manual_reconnect_rtde():
            self.rtde_auto_reconnect_attempts = 0
            self.require_center_on_resume = True
            print("Auto reconnect succeeded.", flush=True)
            return True
        return False

    def send_fallback_motion(self, speed: list[float]) -> None:
        dx = speed[0] * self.args.period_s
        dy = speed[1] * self.args.period_s
        dz = speed[2] * self.args.period_s
        drx = speed[3] * self.args.period_s
        dry = speed[4] * self.args.period_s
        drz = speed[5] * self.args.period_s
        script = relative_movel_script(
            dx,
            dy,
            dz,
            drx,
            dry,
            drz,
            self.args.fallback_acceleration,
            self.args.fallback_velocity,
        )
        self.fallback_client.send_script(script)

    def compute_speed(self, snap: dict[str, float | int | bool]) -> list[float]:
        left_x = self.clamp_deadzone(float(snap["left_x"]))
        left_y = self.clamp_deadzone(float(snap["left_y"]))
        right_x = self.clamp_deadzone(float(snap["right_x"]))
        right_y = self.clamp_deadzone(float(snap["right_y"]))
        lt = 0.0 if float(snap["left_trigger"]) < self.args.deadzone else float(snap["left_trigger"])
        rt = 0.0 if float(snap["right_trigger"]) < self.args.deadzone else float(snap["right_trigger"])

        forward = 0.0
        if int(snap["lb"]) == 1:
            forward -= 1.0
        if int(snap["rb"]) == 1:
            forward += 1.0

        # Horizontal translation compensation:
        # The calibrated plane is already correct, but on this setup the controller
        # sources for left/right and forward/back are swapped relative to the
        # physical robot motion. Keep the plane compensation and swap only these
        # two sources:
        # - left stick L/R -> physical TCP left/right
        # - LB/RB -> physical TCP forward/back
        c = math.cos(self.args.xy_rotate_rad)
        s = math.sin(self.args.xy_rotate_rad)
        horiz_lr = forward * c + left_x * s
        horiz_fb = -forward * s + left_x * c

        vx = horiz_lr * self.args.translation_speed_mps
        vy = horiz_fb * self.args.translation_speed_mps
        vz = -left_y * self.args.translation_speed_mps
        raw_wx = right_x * self.args.rotation_speed_rps
        raw_wy = (rt - lt) * self.args.rotation_speed_rps
        raw_wz = right_y * self.args.rotation_speed_rps
        # Orientation compensation for self-rotation(right stick L/R) + pitch(LT/RT).
        # Keep right stick U/D wrist/end rotation independent so that axis stays predictable.
        rc = math.cos(self.args.rot_axes_rotate_rad)
        rs = math.sin(self.args.rot_axes_rotate_rad)
        wx = raw_wx * rc - raw_wy * rs
        wy = raw_wx * rs + raw_wy * rc
        wz = raw_wz
        return [vx, vy, vz, wx, wy, wz]

    def handle_buttons(self, snap: dict[str, float | int | bool]) -> None:
        if self.edge_pressed(snap, "start"):
            self.running = False
            print("Exit requested from controller.")
            return

        now = time.monotonic()
        if self.edge_pressed(snap, "a"):
            self.set_home_pose()

        if self.edge_pressed(snap, "b"):
            self.move_to_home_pose()

        if self.edge_pressed(snap, "y"):
            if self.recording:
                self.stop_recording_and_save()
            else:
                self.start_recording()

        if self.edge_pressed(snap, "back"):
            if self.playback_running:
                if self.playback_stop_armed:
                    self.stop_playback()
                    self.playback_stop_armed = False
                else:
                    print("Playback stop ignored until Back/View is released.", flush=True)
            else:
                self.start_playback_async()
                self.playback_stop_armed = False

        if not self.playback_stop_armed and int(snap["back"]) == 0:
            self.playback_stop_armed = True

        if self.edge_pressed(snap, "x"):
            if now - self.last_gripper_time >= self.args.gripper_cooldown_s:
                self.last_gripper_time = now
                desired_action = "close" if self.gripper_is_open else "open"
                if self.request_gripper_action(desired_action):
                    if self.recording:
                        t_rel = max(0.0, now - self.record_start_t)
                        with self._record_lock:
                            self.recorded_gripper_events.append((t_rel, desired_action))
                    self.gripper_is_open = not self.gripper_is_open
                    print(f"Gripper {desired_action} requested.")
                else:
                    print("Gripper busy, toggle request ignored.")

    def print_controls(self) -> None:
        print("UR5 RTDE teleop clean ready.")
        if self.use_fallback:
            print("Motion mode: URScript fallback")
        else:
            print("Motion mode: RTDE speedL")
        print("Teleop starts enabled.")
        print("Left stick: end-effector up/down/left/right")
        print("LB/RB: backward/forward")
        print("LT/RT: tool pitch")
        print("Right stick L/R: tool self rotation")
        print("Right stick U/D: wrist/end rotation")
        print("X: gripper toggle (open/close)")
        print("Y: arm synchronized path recording start/stop (save to csv+script)")
        print("Back: playback latest path (press again to stop)")
        playback_dir = str(getattr(self.args, "playback_input_dir", "") or self.args.path_output_dir)
        playback_session = str(getattr(self.args, "playback_session", "latest") or "latest")
        print(f"Playback source: {playback_dir} session={playback_session}")
        print("A: set home pose | B: move to home pose")
        print("Start: exit")
        print(f"Gripper mode: {self.args.gripper_mode}")
        if self.args.camera_enable:
            print(f"Camera record sync: enabled -> {self.args.camera_output_dir}")
            if self.args.camera_process_mode:
                camera_names = split_csv_arg(getattr(self.args, "camera_device_names", ""))
                if camera_names:
                    ports = split_csv_arg(getattr(self.args, "camera_control_ports", ""))
                    print(f"Camera process mode: enabled multi-camera {camera_names} ports={ports or 'auto'}")
                else:
                    print(
                        f"Camera process mode: enabled ({self.args.camera_control_host}:{self.args.camera_control_port})"
                    )
            if self.args.camera_save_bag:
                print(f"Camera bag sync: enabled -> {self.args.camera_bag_output_dir}")
            print(
                f"Camera streams: color({self.args.camera_width}x{self.args.camera_height}@{self.args.camera_fps})"
                + (
                    f", depth({self.args.camera_depth_width}x{self.args.camera_depth_height}@{self.args.camera_depth_fps})"
                    if self.args.camera_record_depth
                    else ", depth(disabled)"
                )
            )
            if getattr(self.args, "camera_auto_record_on_start", False):
                print("Camera startup mode: auto-record on launch.")
            else:
                print("Camera startup mode: Y starts/stops camera recording with path recording.")
        print(
            f"Speed: {self.args.translation_speed_mps*1000:.0f} mm/s, "
            f"rot: {math.degrees(self.args.rotation_speed_rps):.0f} deg/s, "
            f"period: {self.args.period_s:.3f}s, "
            f"robot-record: {self.args.robot_record_fps:.1f} fps, "
            f"xy-rotate: {self.args.xy_rotate_deg:.1f} deg, "
            f"rot-axes-rotate: {self.args.rot_axes_rotate_deg:.1f} deg"
        )

    def run(self) -> int:
        self.print_controls()
        print("Waiting for controller events...")

        while self.running and not self.gamepad.connected:
            time.sleep(0.1)

        if not self.gamepad.connected:
            print("No controller detected.")
            return 1

        # sync startup button states to avoid stale edges
        startup = self.gamepad.snapshot()
        for name in self.last_buttons:
            self.last_buttons[name] = int(startup[name])  # type: ignore[arg-type]

        print(f"Controller connected. Teleop is live. backend={self.gamepad.backend}")
        print("If you see register-conflict error, disable EtherNet/IP/PROFINET/MODBUS or conflicting URCap.")

        try:
            while self.running:
                if os.name == "nt":
                    try:
                        import msvcrt  # type: ignore

                        if msvcrt.kbhit():
                            key = msvcrt.getwch().lower()
                            if key == "q":
                                print("Exit requested from keyboard (q).")
                                self.running = False
                                break
                    except Exception:
                        pass
                snap = self.gamepad.snapshot()
                if not self.buttons_armed:
                    all_released = all(int(snap[name]) == 0 for name in ("a", "b", "x", "y", "back", "start"))
                    now = time.monotonic()
                    if all_released:
                        if self.buttons_armed_since == 0.0:
                            self.buttons_armed_since = now
                        elif now - self.buttons_armed_since >= 0.25:
                            self.buttons_armed = True
                    else:
                        self.buttons_armed_since = 0.0
                    time.sleep(self.args.period_s)
                    continue
                self.handle_buttons(snap)
                if not self.running:
                    break

                if self.recording_pending:
                    self.stop_robot()
                    time.sleep(self.args.period_s)
                    continue

                if self.enabled:
                    if self.playback_running:
                        self.update_latest_action(snap, [0.0] * 6)
                        time.sleep(0.01)
                        continue
                    if self.require_center_on_resume:
                        if self.motion_input_is_centered(snap):
                            self.require_center_on_resume = False
                        else:
                            now = time.monotonic()
                            if now - self.waiting_center_msg_ts >= 1.0:
                                print(
                                    "Waiting for sticks/triggers to return to center... "
                                    "请先把摇杆、扳机、肩键全部回中后再动。",
                                    flush=True,
                                )
                                self.waiting_center_msg_ts = now
                            self.update_latest_action(snap, [0.0] * 6)
                            self.stop_robot()
                            time.sleep(0.02)
                            continue
                    if self.is_gripper_busy():
                        # Avoid fighting RTDE motion with dashboard-run URP execution.
                        self.update_latest_action(snap, [0.0] * 6)
                        self.stop_robot()
                        time.sleep(0.02)
                        continue
                    if self.rtde_needs_reconnect and not self.use_fallback:
                        if not self.auto_reconnect_rtde_if_needed():
                            self.update_latest_action(snap, [0.0] * 6)
                            self.stop_robot()
                            time.sleep(0.03)
                            continue
                    speed = self.compute_speed(snap)
                    self.update_latest_action(snap, speed)
                    if any(abs(v) > 1e-6 for v in speed):
                        if self.use_fallback:
                            try:
                                self.send_fallback_motion(speed)
                            except OSError as exc:
                                now = time.monotonic()
                                if now - self.last_fallback_error >= 1.0:
                                    print(f"Fallback motion send failed: {exc}", flush=True)
                                    print(
                                        "Check that port 30002 is enabled and no remote-control restriction is blocking URScript.",
                                        flush=True,
                                    )
                                    self.last_fallback_error = now
                                self.stop_robot()
                                time.sleep(0.1)
                                continue
                            self.was_moving = True
                        else:
                            if not self.ensure_rtde_program_running():
                                time.sleep(0.05)
                                continue
                            assert self.rtde_c is not None
                            self.rtde_c.speedL(speed, self.args.acceleration, self.args.period_s)
                            self.was_moving = True
                    else:
                        self.update_latest_action(snap, [0.0] * 6)
                        self.stop_robot()
                else:
                    self.update_latest_action(snap, [0.0] * 6)
                    self.stop_robot()

                time.sleep(self.args.period_s)
        except KeyboardInterrupt:
            print("Stopping teleop from keyboard interrupt.")
        except Exception as exc:
            print(f"Teleop fatal error: {exc}", flush=True)
        finally:
            if self.recording:
                self.stop_recording_and_save()
            elif self.args.camera_enable:
                self.stop_camera_auto_record_if_needed()
            if self.args.camera_enable and self.args.camera_process_mode:
                try:
                    self.camera_recorder.shutdown()
                except Exception as exc:
                    print(f"[camera] shutdown on exit failed: {exc}", flush=True)
            if self.playback_running:
                self.stop_playback()
            if self._playback_thread is not None and self._playback_thread.is_alive():
                self._playback_thread.join(timeout=0.5)
            self.stop_robot()
            try:
                if self.rtde_c is not None:
                    self.rtde_c.stopScript()
            except Exception:
                pass
            self.running = False
            if self._gripper_thread.is_alive():
                self._gripper_thread.join(timeout=0.3)
            self.fallback_client.close()

        print("Teleop stopped.")
        return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Clean RTDE Xbox teleop for UR5 + Robotiq.")
    parser.add_argument("--host", required=True, help="Robot controller IP")
    parser.add_argument("--timeout", type=float, default=3.0, help="Dashboard subprocess timeout")
    parser.add_argument("--dashboard-timeout-s", type=float, default=1.2, help="Dashboard command timeout")
    parser.add_argument(
        "--startup-settle-timeout-s",
        type=float,
        default=6.0,
        help="Max wait time for startup UR programs to reach STOPPED before RTDE init",
    )
    parser.add_argument(
        "--auto-init",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Auto initialize robot via dashboard (power on + brake release + protective unlock)",
    )
    parser.add_argument(
        "--start-enabled",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Start teleop motion enabled",
    )
    parser.add_argument("--rtde-frequency", type=float, default=125.0, help="RTDE frequency for control interface")
    parser.add_argument(
        "--rtde-upload-script",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Upload and run RTDE control script automatically",
    )
    parser.add_argument("--translation-speed-mmps", type=float, default=120.0, help="TCP speed in mm/s")
    parser.add_argument("--rotation-speed-degps", type=float, default=70.0, help="Angular speed in deg/s")
    parser.add_argument(
        "--xy-rotate-deg",
        type=float,
        default=0.0,
        help="Rotate left-stick XY direction mapping (degrees); 0 keeps direct left/right + forward/back mapping",
    )
    parser.add_argument(
        "--rot-axes-rotate-deg",
        type=float,
        default=0.0,
        help="Rotate orientation mapping for LT/RT and right-stick U/D (degrees)",
    )
    parser.add_argument("--acceleration", type=float, default=0.70, help="speedL acceleration")
    parser.add_argument("--stop-acceleration", type=float, default=2.0, help="speedStop deceleration")
    parser.add_argument("--period-s", type=float, default=0.008, help="Control loop period")
    parser.add_argument("--deadzone", type=float, default=0.14, help="Controller deadzone")
    parser.add_argument(
        "--center-threshold",
        type=float,
        default=0.12,
        help="Threshold to treat sticks/triggers as centered before motion arm",
    )
    parser.add_argument("--button-cooldown-s", type=float, default=0.18, help="General button debounce")
    parser.add_argument("--gripper-cooldown-s", type=float, default=0.70, help="Gripper command cooldown")
    parser.add_argument(
        "--gripper-wait-done-timeout-s",
        type=float,
        default=2.2,
        help="Max wait for gripper URP to finish before force stop",
    )
    parser.add_argument(
        "--auto-reconnect-after-gripper",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Automatically reconnect RTDE after gripper URP interrupts control script",
    )
    parser.add_argument(
        "--auto-reconnect-max-attempts",
        type=int,
        default=3,
        help="Max automatic RTDE reconnect attempts after gripper action",
    )
    parser.add_argument(
        "--auto-reconnect-interval-s",
        type=float,
        default=0.35,
        help="Minimum gap between automatic RTDE reconnect attempts",
    )
    parser.add_argument("--rtde-restart-timeout-s", type=float, default=2.5, help="Timeout before fallback mode")
    parser.add_argument(
        "--rtde-max-reupload-attempts",
        type=int,
        default=6,
        help="Max RTDE reupload attempts before teleop exits",
    )
    parser.add_argument(
        "--manual-rtde-reconnect-attempts",
        type=int,
        default=2,
        help="How many reupload tries when pressing B to manually recover RTDE",
    )
    parser.add_argument("--rtde-init-retries", type=int, default=3, help="RTDE init attempts before fallback")
    parser.add_argument("--rtde-init-retry-delay-s", type=float, default=0.75, help="Delay between RTDE init retries")
    parser.add_argument(
        "--allow-fallback",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Allow URScript fallback motion when RTDE is unhealthy (default: disabled for safety)",
    )
    parser.add_argument("--fallback-velocity", type=float, default=0.18, help="Fallback movel velocity")
    parser.add_argument("--fallback-acceleration", type=float, default=0.35, help="Fallback movel acceleration")
    parser.add_argument(
        "--gripper-mode",
        choices=("robotiq_socket", "urscript", "urp"),
        default="robotiq_socket",
        help="Gripper backend: robotiq_socket (recommended), urscript, or urp load/play",
    )
    parser.add_argument(
        "--gripper-urscript-sleep-s",
        type=float,
        default=0.35,
        help="Sleep time after rq_open/rq_close in secondary URScript mode",
    )
    parser.add_argument(
        "--robotiq-socket-port",
        type=int,
        default=63352,
        help="Robotiq URCap TCP command port on robot controller",
    )
    parser.add_argument("--robotiq-open-pos", type=int, default=0, help="Robotiq open target [0..255]")
    parser.add_argument("--robotiq-close-pos", type=int, default=255, help="Robotiq close target [0..255]")
    parser.add_argument("--robotiq-speed", type=int, default=220, help="Robotiq speed [0..255]")
    parser.add_argument("--robotiq-force", type=int, default=220, help="Robotiq force [0..255]")
    parser.add_argument(
        "--robotiq-skip-activate",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Do not send SET ACT from teleop (avoid unintended deactivate/reset)",
    )
    parser.add_argument(
        "--gripper-start-open",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Initial gripper toggle state assumption",
    )
    parser.add_argument("--gripper-open", default="/programs/gripper_open.urp", help="URP path for open")
    parser.add_argument("--gripper-close", default="/programs/gripper_close.urp", help="URP path for close")
    parser.add_argument(
        "--gripper-activate",
        default="/programs/gripper_activate.urp",
        help="URP path for gripper activation at startup (empty to skip)",
    )
    parser.add_argument(
        "--home-velocity",
        type=float,
        default=0.20,
        help="Velocity for move-to-home movel (m/s)",
    )
    parser.add_argument(
        "--home-acceleration",
        type=float,
        default=0.35,
        help="Acceleration for move-to-home movel (m/s^2)",
    )
    parser.add_argument(
        "--open-gripper-on-home",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Open gripper automatically when moving to home pose and before playback",
    )
    parser.add_argument(
        "--record-interval-s",
        type=float,
        default=0.02,
        help="Record sampling interval in seconds",
    )
    parser.add_argument(
        "--robot-record-fps",
        type=float,
        default=0.0,
        help="Robot recording target FPS. If >0, overrides --record-interval-s.",
    )
    parser.add_argument(
        "--path-output-dir",
        default=str(DEFAULT_OUTPUT_ROOT),
        help="Output directory for recorded path files",
    )
    parser.add_argument(
        "--record-session-subdirs",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Create numbered raw subdirectories (1,2,3,...) under --path-output-dir for each recording.",
    )
    parser.add_argument(
        "--playback-input-dir",
        default="",
        help="Raw paths directory or session_manifest_*.json used when Back replays a saved trajectory. Default: --path-output-dir.",
    )
    parser.add_argument(
        "--playback-session",
        default="latest",
        help="Session id to load from --playback-input-dir when Back is pressed, or latest.",
    )
    parser.add_argument(
        "--dataset-output-formats",
        default="raw",
        help="Comma-separated dataset formats for optional conversion: raw,hdf5,rlds. Raw files are always written first.",
    )
    parser.add_argument(
        "--convert-datasets-on-stop",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Convert HDF5/RLDS immediately after recording stops. Default is off; use offline postprocessing instead.",
    )
    parser.add_argument(
        "--rlds-copy-media",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Copy image/depth media into RLDS-style output instead of referencing raw paths",
    )
    parser.add_argument(
        "--hdf5-embed-binary",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Embed small binary raw files into HDF5 in addition to training arrays",
    )
    parser.add_argument(
        "--path-playback-velocity",
        type=float,
        default=0.12,
        help="Velocity used in exported playback URScript movel commands",
    )
    parser.add_argument(
        "--path-playback-acceleration",
        type=float,
        default=0.25,
        help="Acceleration used in exported playback URScript movel commands",
    )
    parser.add_argument(
        "--playback-entry-velocity",
        type=float,
        default=0.08,
        help="Velocity for moving to first playback point",
    )
    parser.add_argument(
        "--playback-entry-acceleration",
        type=float,
        default=0.15,
        help="Acceleration for moving to first playback point",
    )
    parser.add_argument(
        "--servo-lookahead-time",
        type=float,
        default=0.10,
        help="servoL lookahead_time for smooth playback",
    )
    parser.add_argument(
        "--servo-gain",
        type=int,
        default=300,
        help="servoL gain for smooth playback",
    )
    parser.add_argument(
        "--camera-enable",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Enable RealSense camera recording support",
    )
    parser.add_argument(
        "--camera-auto-record-on-start",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Start camera recording immediately when teleop launches and stop only on program exit",
    )
    parser.add_argument(
        "--camera-process-mode",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Use dedicated camera recorder process (recommended)",
    )
    parser.add_argument("--camera-control-host", default="127.0.0.1", help="Camera service host")
    parser.add_argument("--camera-control-port", type=int, default=61337, help="Camera service TCP port")
    parser.add_argument(
        "--camera-specs-json",
        default="",
        help="JSON list of per-camera specs passed by the launcher for multi-camera recording",
    )
    parser.add_argument(
        "--camera-device-names",
        default="",
        help="Comma-separated RealSense device keywords for multi-camera recording, e.g. D455,L515",
    )
    parser.add_argument(
        "--camera-labels",
        default="",
        help="Comma-separated output labels matching --camera-device-names, e.g. d455,l515",
    )
    parser.add_argument(
        "--camera-control-ports",
        default="",
        help="Comma-separated camera service ports matching --camera-device-names, e.g. 61337,61338",
    )
    parser.add_argument(
        "--camera-separate-output-dirs",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Put each camera's outputs under camera output subdirectories named by camera label",
    )
    parser.add_argument(
        "--camera-inter-start-delay-s",
        type=float,
        default=1.0,
        help="Delay between starting each camera service recording to reduce USB/resource contention",
    )
    parser.add_argument(
        "--camera-inter-stop-delay-s",
        type=float,
        default=0.2,
        help="Delay between stopping each camera service recording",
    )
    parser.add_argument(
        "--camera-request-timeout-s",
        type=float,
        default=12.0,
        help="Default timeout for camera service requests",
    )
    parser.add_argument(
        "--camera-start-timeout-s",
        type=float,
        default=90.0,
        help="Timeout for camera start request (can be longer due to stream fallback)",
    )
    parser.add_argument(
        "--camera-stop-timeout-s",
        type=float,
        default=120.0,
        help="Timeout for camera stop request",
    )
    parser.add_argument("--camera-width", type=int, default=640, help="RealSense color width")
    parser.add_argument("--camera-height", type=int, default=480, help="RealSense color height")
    parser.add_argument("--camera-fps", type=int, default=30, help="RealSense color fps")
    parser.add_argument(
        "--camera-device-name",
        default="D455",
        help="Preferred RealSense device name keyword (e.g., D455, L515); first available is used if not found",
    )
    parser.add_argument(
        "--camera-start-retries",
        type=int,
        default=5,
        help="How many attempts to open camera streams when recording starts",
    )
    parser.add_argument(
        "--camera-retry-delay-s",
        type=float,
        default=0.7,
        help="Delay between camera open retries",
    )
    parser.add_argument(
        "--camera-record-depth",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable RealSense depth stream recording (stored in .bag)",
    )
    parser.add_argument("--camera-depth-width", type=int, default=640, help="RealSense depth width")
    parser.add_argument("--camera-depth-height", type=int, default=480, help="RealSense depth height")
    parser.add_argument("--camera-depth-fps", type=int, default=30, help="RealSense depth fps")
    parser.add_argument(
        "--camera-record-infra",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable infrared stereo streams recording (saved in .bag)",
    )
    parser.add_argument("--camera-infra-width", type=int, default=640, help="RealSense infrared width")
    parser.add_argument("--camera-infra-height", type=int, default=480, help="RealSense infrared height")
    parser.add_argument("--camera-infra-fps", type=int, default=30, help="RealSense infrared fps")
    parser.add_argument("--camera-codec", default="mp4v", help="OpenCV fourcc codec for saved video")
    parser.add_argument(
        "--camera-close-viewer-on-record",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Legacy option, ignored in pure code-only recording mode",
    )
    parser.add_argument(
        "--camera-output-dir",
        default=str(DEFAULT_CAMERA_VIDEO_DIR),
        help="Directory for camera recording videos",
    )
    parser.add_argument(
        "--camera-frame-ts-output-dir",
        default=str(DEFAULT_CAMERA_TS_DIR),
        help="Directory for camera frame timestamp CSV files",
    )
    parser.add_argument(
        "--camera-metadata-output-dir",
        default=str(DEFAULT_OUTPUT_ROOT / "camera_metadata"),
        help="Directory for camera metadata JSON files",
    )
    parser.add_argument(
        "--camera-intrinsics-output-dir",
        default=str(DEFAULT_OUTPUT_ROOT / "camera_intrinsics"),
        help="Directory for camera intrinsics JSON files",
    )
    parser.add_argument(
        "--camera-frames-output-dir",
        default=str(DEFAULT_OUTPUT_ROOT / "camera_frames"),
        help="Directory for exported per-frame color/depth images",
    )
    parser.add_argument(
        "--camera-depth-csv-output-dir",
        default=str(DEFAULT_OUTPUT_ROOT / "camera_depth_csv"),
        help="Directory for exported per-frame depth CSV files",
    )
    parser.add_argument(
        "--camera-save-bag",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Save RealSense .bag alongside MP4 during recording sessions",
    )
    parser.add_argument(
        "--camera-deferred-postprocess",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Record only lightweight camera BAG/timestamps live; export MP4/images/depth CSV offline.",
    )
    parser.add_argument(
        "--camera-bag-output-dir",
        default=str(DEFAULT_CAMERA_BAG_DIR),
        help="Directory for RealSense .bag recordings",
    )
    parser.add_argument(
        "--camera-export-frame-every-n",
        type=int,
        default=15,
        help="Export one color/depth frame artifact every N recorded frames",
    )
    parser.add_argument(
        "--camera-export-max-frames",
        type=int,
        default=0,
        help="Maximum exported frame artifacts per session per camera (0 means no limit)",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    args.translation_speed_mps = args.translation_speed_mmps / 1000.0
    args.rotation_speed_rps = math.radians(args.rotation_speed_degps)
    args.xy_rotate_rad = math.radians(args.xy_rotate_deg)
    args.rot_axes_rotate_rad = math.radians(args.rot_axes_rotate_deg)
    if args.robot_record_fps > 0:
        args.record_interval_s = 1.0 / float(args.robot_record_fps)
    else:
        args.record_interval_s = max(0.001, float(args.record_interval_s))
        args.robot_record_fps = 1.0 / args.record_interval_s
    teleop = Teleop(args)
    return teleop.run()


if __name__ == "__main__":
    sys.exit(main())

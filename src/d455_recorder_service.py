#!/usr/bin/env python3
"""
RealSense recorder service (dedicated process).

Outputs per recording session:
- MP4/AVI color video
- BAG file (color/depth/infra streams if enabled)
- Camera frame timestamp CSV for robot-camera alignment
- Metadata JSON
- Camera intrinsics JSON
- Per-frame exported color/depth images and depth CSV
"""

from __future__ import annotations

import argparse
import csv
import json
import socket
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "paths"
DEFAULT_CAMERA_VIDEO_DIR = DEFAULT_OUTPUT_ROOT / "camera_video"
DEFAULT_CAMERA_TS_DIR = DEFAULT_OUTPUT_ROOT / "camera_timestamps"
DEFAULT_CAMERA_BAG_DIR = DEFAULT_OUTPUT_ROOT / "camera_bag"
DEFAULT_CAMERA_META_DIR = DEFAULT_OUTPUT_ROOT / "camera_metadata"
DEFAULT_CAMERA_INTRINSICS_DIR = DEFAULT_OUTPUT_ROOT / "camera_intrinsics"
DEFAULT_CAMERA_FRAMES_DIR = DEFAULT_OUTPUT_ROOT / "camera_frames"
DEFAULT_CAMERA_DEPTH_CSV_DIR = DEFAULT_OUTPUT_ROOT / "camera_depth_csv"


class RealSenseRecorderService:
    """Small line-oriented JSON service that owns one RealSense pipeline.

    The teleop process talks to this service over localhost so camera capture can
    keep running even when robot control is busy. Commands are intentionally
    simple (`ping`, `start`, `status`, `mark_start`, `stop`, `shutdown`) because
    they are used from both the launcher and the main teleoperation loop.
    """

    def __init__(self) -> None:
        self.pipeline: rs.pipeline | None = None
        self.pipeline_profile: rs.pipeline_profile | None = None
        self.capture_thread: threading.Thread | None = None
        self.stop_event = threading.Event()
        self.running = True

        self.writer: cv2.VideoWriter | None = None
        self.recording = False

        self.video_path: Path | None = None
        self.bag_path: Path | None = None
        self.frame_ts_path: Path | None = None
        self.metadata_path: Path | None = None
        self.intrinsics_path: Path | None = None
        self.frame_export_dir: Path | None = None
        self.depth_csv_dir: Path | None = None
        self.video_codec = ""
        self.video_fps = 60.0
        self.video_size = (640, 480)
        self.deferred_postprocess = True
        self.device_serial = ""
        self.device_name = ""
        self.pipeline_mode = "stopped"
        self.camera_label = "camera"

        self.record_start_host_ns = 0
        self.frame_rows: list[tuple[int, int, int, int, float, float, int]] = []
        self.frame_rows_lock = threading.Lock()
        self.frame_idx = 0
        self.export_frame_every_n = 1
        self.max_export_frames = 0
        self.depth_scale = 0.0
        self.depth_enabled = False
        self.infra_enabled = False

    def _frame_rows_snapshot(self) -> list[tuple[int, int, int, int, float, float, int]]:
        with self.frame_rows_lock:
            return list(self.frame_rows)

    def _frame_rows_count(self) -> int:
        with self.frame_rows_lock:
            return len(self.frame_rows)

    def _list_available_devices(self) -> list[tuple[str, str]]:
        items: list[tuple[str, str]] = []
        try:
            devices = list(rs.context().query_devices())
        except Exception:
            return items
        for dev in devices:
            name = ""
            serial = ""
            try:
                if dev.supports(rs.camera_info.name):
                    name = str(dev.get_info(rs.camera_info.name))
            except Exception:
                name = ""
            try:
                if dev.supports(rs.camera_info.serial_number):
                    serial = str(dev.get_info(rs.camera_info.serial_number))
            except Exception:
                serial = ""
            if name or serial:
                items.append((name, serial))
        return items

    def _select_device_serial(self, preferred_name: str) -> tuple[str, bool]:
        """Return a serial number and whether it matched the requested camera.

        The RealSense SDK starts streams by serial number, while config files are
        easier for humans to maintain by product/name. If no preferred name is
        configured, the first detected camera is used.
        """
        preferred = preferred_name.strip().lower()
        serial_first = ""
        if not preferred:
            available = self._list_available_devices()
            if available:
                name, serial = available[0]
                self.device_name = name
                return serial, True
            return "", False
        try:
            devices = list(rs.context().query_devices())
        except Exception:
            return "", False
        for dev in devices:
            name = ""
            serial = ""
            try:
                if dev.supports(rs.camera_info.name):
                    name = str(dev.get_info(rs.camera_info.name)).lower()
            except Exception:
                name = ""
            try:
                if dev.supports(rs.camera_info.serial_number):
                    serial = str(dev.get_info(rs.camera_info.serial_number))
            except Exception:
                serial = ""
            if not serial:
                continue
            if not serial_first:
                serial_first = serial
            if preferred in name:
                self.device_name = str(dev.get_info(rs.camera_info.name))
                return serial, True
        return "", False

    def _stop_pipeline(self) -> None:
        """Stop capture and release SDK/video resources before reconfiguration."""
        self.stop_event.set()
        if self.capture_thread is not None and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=2.0)
        self.capture_thread = None
        try:
            if self.pipeline is not None:
                self.pipeline.stop()
        except Exception:
            pass
        self.pipeline = None
        self.pipeline_profile = None
        self.pipeline_mode = "stopped"

    def _wait_for_first_color_frame(
        self,
        pipeline: rs.pipeline,
        *,
        timeout_ms: int = 2000,
        attempts: int = 2,
    ) -> tuple[bool, str]:
        """Verify that a newly started pipeline actually produces color frames."""
        last_err = ""
        for _ in range(max(1, attempts)):
            try:
                frames = pipeline.wait_for_frames(timeout_ms)
                color = frames.get_color_frame()
                if not color:
                    last_err = "first frameset had no color frame"
                    continue
                _ = np.asanyarray(color.get_data())
                return True, ""
            except Exception as exc:
                last_err = str(exc)
        return False, last_err or "no color frame received after pipeline start"

    def _remove_partial_bag(self, bag_path: Path | None) -> None:
        if bag_path is None:
            return
        try:
            if bag_path.exists():
                bag_path.unlink()
        except Exception:
            pass

    def _maybe_export_frame_artifacts(
        self,
        frame_idx: int,
        color_frame: rs.frame,
        depth_frame: rs.frame | None,
    ) -> None:
        """Optionally write per-frame RGB/depth artifacts during live capture."""
        if self.frame_export_dir is None:
            return
        if self.export_frame_every_n <= 0:
            return
        if frame_idx % self.export_frame_every_n != 0:
            return
        export_idx = frame_idx // self.export_frame_every_n
        if self.max_export_frames > 0 and export_idx >= self.max_export_frames:
            return

        try:
            self.frame_export_dir.mkdir(parents=True, exist_ok=True)
            color_np = np.asanyarray(color_frame.get_data())
            color_path = self.frame_export_dir / f"{self.camera_label}_color_{frame_idx:06d}.png"
            cv2.imwrite(str(color_path), color_np)
        except Exception:
            pass

        if depth_frame is not None:
            try:
                depth_np = np.asanyarray(depth_frame.get_data())
                depth_png_path = self.frame_export_dir / f"{self.camera_label}_depth_{frame_idx:06d}.png"
                cv2.imwrite(str(depth_png_path), depth_np)
                depth_vis = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_np, alpha=0.03),
                    cv2.COLORMAP_JET,
                )
                depth_vis_path = self.frame_export_dir / f"{self.camera_label}_depth_vis_{frame_idx:06d}.png"
                cv2.imwrite(str(depth_vis_path), depth_vis)
                if self.depth_csv_dir is not None:
                    self.depth_csv_dir.mkdir(parents=True, exist_ok=True)
                    depth_csv_path = self.depth_csv_dir / f"{self.camera_label}_depth_{frame_idx:06d}.csv"
                    np.savetxt(depth_csv_path, depth_np, fmt="%u", delimiter=",")
            except Exception:
                pass

    def _capture_loop(self) -> None:
        """Background loop that records frames and timestamp rows.

        Timestamps are captured from three clocks: host monotonic time for
        robot/camera alignment, wall time for inspection, and RealSense frame
        timestamp/number for SDK-level matching.
        """
        assert self.pipeline is not None
        while not self.stop_event.is_set():
            try:
                frames = self.pipeline.wait_for_frames(1000)
                color = frames.get_color_frame()
                if not color:
                    continue
                depth = frames.get_depth_frame() if self.depth_enabled else None

                if self.recording:
                    if self.writer is not None:
                        frame = np.asanyarray(color.get_data())
                        self.writer.write(frame)
                    host_ns = time.monotonic_ns()
                    system_ns = time.time_ns()
                    rs_ts_ms = float(color.get_timestamp())
                    rs_frame_num = int(color.get_frame_number())
                    rel_s = 0.0
                    if self.record_start_host_ns > 0:
                        rel_s = (host_ns - self.record_start_host_ns) / 1e9
                    with self.frame_rows_lock:
                        self.frame_rows.append(
                            (
                                self.frame_idx,
                                host_ns,
                                system_ns,
                                rs_frame_num,
                                rs_ts_ms,
                                rel_s,
                                int(color.get_width()) * int(color.get_height()),
                            )
                        )
                    if not self.deferred_postprocess:
                        self._maybe_export_frame_artifacts(self.frame_idx, color, depth)
                    self.frame_idx += 1
            except Exception:
                time.sleep(0.01)

    def _open_writer(self, out_path: Path, fps: float, size: tuple[int, int]) -> tuple[cv2.VideoWriter | None, Path, str]:
        candidates = [
            (out_path.with_suffix(".mp4"), "mp4v"),
            (out_path.with_suffix(".avi"), "MJPG"),
            (out_path.with_suffix(".avi"), "XVID"),
        ]
        for path, codec in candidates:
            try:
                fourcc = cv2.VideoWriter_fourcc(*codec)
                writer = cv2.VideoWriter(str(path), fourcc, fps, size)
            except Exception:
                continue
            if writer is not None and writer.isOpened():
                return writer, path, codec
            try:
                writer.release()
            except Exception:
                pass
        return None, out_path, ""

    def _start_pipeline(
        self,
        preferred_name: str,
        color_w: int,
        color_h: int,
        color_fps: int,
        depth_w: int,
        depth_h: int,
        depth_fps: int,
        infra_w: int,
        infra_h: int,
        infra_fps: int,
        depth_on: bool,
        infra_on: bool,
        save_bag: bool,
        bag_path: Path | None,
        ) -> tuple[bool, str]:
        """Start RealSense with graceful fallback from rich streams to color-only.

        Some USB ports, firmware versions, and camera models cannot support every
        requested stream combination. The service tries the most complete mode
        first, then falls back while reporting the final `pipeline_mode`.
        """
        self._stop_pipeline()
        try:
            devices = list(rs.context().query_devices())
        except Exception as exc:
            return False, f"realsense device query failed: {exc}"
        if not devices:
            return False, "no realsense devices detected by SDK"
        serial, matched_preferred = self._select_device_serial(preferred_name)
        if preferred_name.strip() and not matched_preferred:
            available = self._list_available_devices()
            if available:
                detail = ", ".join(f"{name or '<unknown>'} [{serial or '<no-serial>'}]" for name, serial in available)
            else:
                detail = "none"
            return False, f"preferred device '{preferred_name}' not found; available: {detail}"
        self.device_serial = serial
        self.depth_enabled = False
        self.infra_enabled = False

        plans: list[tuple[str, bool, bool, bool]] = [
            ("color+depth+infra+bag", depth_on, infra_on, save_bag),
            ("color+depth+bag", depth_on, False, save_bag),
            ("color+bag", False, False, save_bag),
            ("color+depth+infra", depth_on, infra_on, False),
            ("color+depth", depth_on, False, False),
            ("color-only", False, False, False),
        ]

        last_err = ""
        for mode, use_depth, use_infra, use_bag in plans:
            try:
                p = rs.pipeline()
                cfg = rs.config()
                if serial:
                    cfg.enable_device(serial)
                cfg.enable_stream(rs.stream.color, color_w, color_h, rs.format.bgr8, color_fps)
                if use_depth:
                    cfg.enable_stream(rs.stream.depth, depth_w, depth_h, rs.format.z16, depth_fps)
                if use_infra:
                    cfg.enable_stream(rs.stream.infrared, 1, infra_w, infra_h, rs.format.y8, infra_fps)
                    cfg.enable_stream(rs.stream.infrared, 2, infra_w, infra_h, rs.format.y8, infra_fps)
                if use_bag and bag_path is not None:
                    cfg.enable_record_to_file(str(bag_path))
                profile = p.start(cfg)
                frame_ok, frame_err = self._wait_for_first_color_frame(p)
                if not frame_ok:
                    last_err = f"{mode}: {frame_err}"
                    try:
                        p.stop()
                    except Exception:
                        pass
                    self._remove_partial_bag(bag_path if use_bag else None)
                    continue
                self.pipeline = p
                self.pipeline_profile = profile
                self.pipeline_mode = mode
                self.depth_enabled = use_depth
                self.infra_enabled = use_infra
                try:
                    device = profile.get_device()
                    self.device_name = str(device.get_info(rs.camera_info.name))
                    for sensor in device.query_sensors():
                        try:
                            if sensor.supports(rs.option.depth_units):
                                self.depth_scale = float(sensor.get_option(rs.option.depth_units))
                                break
                        except Exception:
                            continue
                except Exception:
                    pass
                self.stop_event.clear()
                self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
                self.capture_thread.start()
                if not (use_bag and bag_path is not None):
                    self.bag_path = None
                return True, mode
            except Exception as exc:
                last_err = str(exc)
                self._stop_pipeline()
                self._remove_partial_bag(bag_path if use_bag else None)
                continue
        return False, last_err or "unknown pipeline start error"

    def cmd_ping(self) -> dict[str, object]:
        return {
            "ok": True,
            "recording": self.recording,
            "pipeline_mode": self.pipeline_mode,
            "serial": self.device_serial,
            "device_name": self.device_name,
            "frame_count": self._frame_rows_count(),
        }

    def _profile_intrinsics(self) -> dict[str, object]:
        profile = self.pipeline_profile
        if profile is None:
            return {}
        payload: dict[str, object] = {
            "serial": self.device_serial,
            "device_name": self.device_name,
            "depth_scale": self.depth_scale,
        }
        try:
            streams = []
            for sp in profile.get_streams():
                try:
                    vsp = sp.as_video_stream_profile()
                    intr = vsp.get_intrinsics()
                    streams.append(
                        {
                            "stream_type": str(vsp.stream_type()),
                            "stream_index": int(vsp.stream_index()),
                            "width": int(vsp.width()),
                            "height": int(vsp.height()),
                            "fps": int(vsp.fps()),
                            "fx": float(intr.fx),
                            "fy": float(intr.fy),
                            "ppx": float(intr.ppx),
                            "ppy": float(intr.ppy),
                            "coeffs": [float(v) for v in intr.coeffs],
                            "model": str(intr.model),
                        }
                    )
                except Exception:
                    continue
            payload["streams"] = streams
        except Exception:
            payload["streams"] = []
        return payload

    def _write_metadata_json(self) -> None:
        if self.metadata_path is None:
            return
        payload = {
            "camera_label": self.camera_label,
            "device_name": self.device_name,
            "serial": self.device_serial,
            "pipeline_mode": self.pipeline_mode,
            "video_path": str(self.video_path or ""),
            "bag_path": str(self.bag_path or ""),
            "frame_ts_path": str(self.frame_ts_path or ""),
            "intrinsics_path": str(self.intrinsics_path or ""),
            "frame_export_dir": str(self.frame_export_dir or ""),
            "depth_csv_dir": str(self.depth_csv_dir or ""),
            "video_codec": self.video_codec,
            "video_fps": self.video_fps,
            "video_size": list(self.video_size),
            "frame_count": self._frame_rows_count(),
            "postprocess_required": self.deferred_postprocess,
            "postprocess_done": False,
            "depth_scale": self.depth_scale,
            "depth_enabled": self.depth_enabled,
            "infra_enabled": self.infra_enabled,
            "export_frame_every_n": self.export_frame_every_n,
            "export_max_frames": self.max_export_frames,
            "record_start_host_ns": self.record_start_host_ns,
        }
        self.metadata_path.parent.mkdir(parents=True, exist_ok=True)
        self.metadata_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    def _write_intrinsics_json(self) -> None:
        if self.intrinsics_path is None:
            return
        payload = self._profile_intrinsics()
        self.intrinsics_path.parent.mkdir(parents=True, exist_ok=True)
        self.intrinsics_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    def cmd_start(self, payload: dict[str, object]) -> dict[str, object]:
        """Start a session and return all paths needed by the teleop manifest."""
        if self.recording:
            return {"ok": False, "error": "already recording"}
        session_id = str(payload.get("session_id", "")).strip()
        if not session_id:
            return {"ok": False, "error": "missing session_id"}

        self.camera_label = str(payload.get("camera_label", "camera")).strip() or "camera"

        out_dir = Path(str(payload.get("camera_output_dir", str(DEFAULT_CAMERA_VIDEO_DIR))))
        out_dir.mkdir(parents=True, exist_ok=True)
        frame_ts_dir = Path(str(payload.get("camera_frame_ts_output_dir", str(DEFAULT_CAMERA_TS_DIR))))
        frame_ts_dir.mkdir(parents=True, exist_ok=True)
        meta_dir = Path(str(payload.get("camera_metadata_output_dir", str(DEFAULT_CAMERA_META_DIR))))
        meta_dir.mkdir(parents=True, exist_ok=True)
        intrinsics_dir = Path(str(payload.get("camera_intrinsics_output_dir", str(DEFAULT_CAMERA_INTRINSICS_DIR))))
        intrinsics_dir.mkdir(parents=True, exist_ok=True)
        frames_dir = Path(str(payload.get("camera_frames_output_dir", str(DEFAULT_CAMERA_FRAMES_DIR))))
        frames_dir.mkdir(parents=True, exist_ok=True)
        depth_csv_dir = Path(str(payload.get("camera_depth_csv_output_dir", str(DEFAULT_CAMERA_DEPTH_CSV_DIR))))
        depth_csv_dir.mkdir(parents=True, exist_ok=True)

        color_w = int(payload.get("camera_width", 640))
        color_h = int(payload.get("camera_height", 480))
        color_fps = int(payload.get("camera_fps", 60))
        depth_w = int(payload.get("camera_depth_width", color_w))
        depth_h = int(payload.get("camera_depth_height", color_h))
        depth_fps = int(payload.get("camera_depth_fps", color_fps))
        infra_w = int(payload.get("camera_infra_width", depth_w))
        infra_h = int(payload.get("camera_infra_height", depth_h))
        infra_fps = int(payload.get("camera_infra_fps", depth_fps))
        preferred = str(payload.get("camera_device_name", "D455"))

        depth_on = bool(payload.get("camera_record_depth", True))
        infra_on = bool(payload.get("camera_record_infra", True))
        save_bag = bool(payload.get("camera_save_bag", True))
        self.deferred_postprocess = bool(payload.get("camera_deferred_postprocess", True))

        self.video_fps = float(color_fps)
        self.video_size = (color_w, color_h)
        self.record_start_host_ns = int(payload.get("record_start_host_ns", 0) or 0)
        with self.frame_rows_lock:
            self.frame_rows = []
        self.frame_idx = 0
        self.export_frame_every_n = max(1, int(payload.get("camera_export_frame_every_n", 30) or 30))
        self.max_export_frames = max(0, int(payload.get("camera_export_max_frames", 0) or 0))

        bag_path: Path | None = None
        if save_bag:
            bag_dir = Path(str(payload.get("camera_bag_output_dir", str(DEFAULT_CAMERA_BAG_DIR))))
            bag_dir.mkdir(parents=True, exist_ok=True)
            bag_path = bag_dir / f"{self.camera_label}_{session_id}.bag"
        self.bag_path = bag_path
        self.frame_ts_path = frame_ts_dir / f"{self.camera_label}_frames_{session_id}.csv"
        self.metadata_path = meta_dir / f"{self.camera_label}_metadata_{session_id}.json"
        self.intrinsics_path = intrinsics_dir / f"{self.camera_label}_intrinsics_{session_id}.json"
        self.frame_export_dir = frames_dir / f"{self.camera_label}_{session_id}"
        self.depth_csv_dir = depth_csv_dir / f"{self.camera_label}_{session_id}"

        ok, detail = self._start_pipeline(
            preferred_name=preferred,
            color_w=color_w,
            color_h=color_h,
            color_fps=color_fps,
            depth_w=depth_w,
            depth_h=depth_h,
            depth_fps=depth_fps,
            infra_w=infra_w,
            infra_h=infra_h,
            infra_fps=infra_fps,
            depth_on=depth_on,
            infra_on=infra_on,
            save_bag=save_bag,
            bag_path=bag_path,
        )
        if not ok:
            return {"ok": False, "error": f"pipeline start failed: {detail}"}
        if self.deferred_postprocess and save_bag and self.bag_path is None:
            self._stop_pipeline()
            return {"ok": False, "error": "deferred camera recording requires BAG mode, but the pipeline fell back without BAG"}

        try:
            self._write_intrinsics_json()
        except Exception:
            pass

        self.video_path = out_dir / f"{self.camera_label}_{session_id}.mp4"
        if self.deferred_postprocess:
            self.writer = None
            self.video_codec = "bag_deferred_postprocess"
        else:
            writer, video_path, codec = self._open_writer(self.video_path, self.video_fps, self.video_size)
            if writer is None:
                self._stop_pipeline()
                return {"ok": False, "error": "video writer open failed (mp4v/mjpg/xvid)"}
            self.writer = writer
            self.video_path = video_path
            self.video_codec = codec
        self.recording = True

        return {
            "ok": True,
            "recording": True,
            "video_path": str(self.video_path),
            "bag_path": str(self.bag_path) if self.bag_path else "",
            "frame_ts_path": str(self.frame_ts_path) if self.frame_ts_path else "",
            "metadata_path": str(self.metadata_path) if self.metadata_path else "",
            "intrinsics_path": str(self.intrinsics_path) if self.intrinsics_path else "",
            "frame_export_dir": str(self.frame_export_dir) if self.frame_export_dir else "",
            "depth_csv_dir": str(self.depth_csv_dir) if self.depth_csv_dir else "",
            "serial": self.device_serial,
            "device_name": self.device_name,
            "video_codec": self.video_codec,
            "postprocess_required": self.deferred_postprocess,
            "postprocess_done": False,
            "export_frame_every_n": self.export_frame_every_n,
            "export_max_frames": self.max_export_frames,
            "started_depth": "depth" in self.pipeline_mode,
            "started_infra": "infra" in self.pipeline_mode,
            "started_bag": "bag" in self.pipeline_mode,
            "pipeline_mode": self.pipeline_mode,
            "camera_fps": color_fps,
            "record_start_host_ns": self.record_start_host_ns,
        }

    def _flush_frame_ts_csv(self) -> None:
        """Persist the current timestamp buffer so progress survives interruptions."""
        if self.frame_ts_path is None:
            return
        self.frame_ts_path.parent.mkdir(parents=True, exist_ok=True)
        rows = self._frame_rows_snapshot()
        with self.frame_ts_path.open("w", encoding="utf-8", newline="") as f:
            w = csv.writer(f)
            w.writerow(
                [
                    "frame_idx",
                    "host_monotonic_ns",
                    "system_time_ns",
                    "rs_frame_number",
                    "rs_timestamp_ms",
                    "t_rel_s",
                    "pixel_count",
                ]
            )
            w.writerows(rows)

    def cmd_status(self) -> dict[str, object]:
        try:
            self._flush_frame_ts_csv()
        except Exception:
            pass
        try:
            self._write_metadata_json()
        except Exception:
            pass
        try:
            self._write_intrinsics_json()
        except Exception:
            pass

        video_size = 0
        try:
            if self.video_path is not None and self.video_path.exists():
                video_size = int(self.video_path.stat().st_size)
        except Exception:
            video_size = 0

        return {
            "ok": True,
            "recording": self.recording,
            "video_path": str(self.video_path or ""),
            "video_size": video_size,
            "bag_path": str(self.bag_path or ""),
            "frame_ts_path": str(self.frame_ts_path or ""),
            "metadata_path": str(self.metadata_path or ""),
            "intrinsics_path": str(self.intrinsics_path or ""),
            "frame_export_dir": str(self.frame_export_dir or ""),
            "depth_csv_dir": str(self.depth_csv_dir or ""),
            "frame_count": self._frame_rows_count(),
            "device_name": self.device_name,
            "serial": self.device_serial,
            "pipeline_mode": self.pipeline_mode,
            "camera_label": self.camera_label,
            "camera_fps": int(self.video_fps),
            "started_depth": self.depth_enabled,
            "started_infra": self.infra_enabled,
            "started_bag": self.bag_path is not None,
            "record_start_host_ns": self.record_start_host_ns,
            "video_codec": self.video_codec,
            "postprocess_required": self.deferred_postprocess,
            "postprocess_done": False,
            "export_frame_every_n": self.export_frame_every_n,
            "export_max_frames": self.max_export_frames,
        }

    def cmd_mark_start(self, payload: dict[str, object]) -> dict[str, object]:
        self.record_start_host_ns = int(payload.get("record_start_host_ns", 0) or 0)
        with self.frame_rows_lock:
            self.frame_rows = []
        self.frame_idx = 0
        return {
            "ok": True,
            "recording": self.recording,
            "record_start_host_ns": self.record_start_host_ns,
            "frame_count": 0,
        }

    def cmd_stop(self) -> dict[str, object]:
        if self.writer is not None:
            try:
                self.writer.release()
            except Exception:
                pass
        self.writer = None
        self.recording = False

        self._stop_pipeline()
        try:
            self._flush_frame_ts_csv()
        except Exception:
            pass
        try:
            self._write_metadata_json()
        except Exception:
            pass

        if self.video_path is None:
            return {
                "ok": True,
                "video_path": "",
                "video_size": 0,
                "bag_path": "",
                "frame_ts_path": "",
                "metadata_path": "",
                "intrinsics_path": "",
                "frame_export_dir": "",
                "depth_csv_dir": "",
            }
        size = 0
        try:
            if self.video_path.exists():
                size = int(self.video_path.stat().st_size)
        except Exception:
            size = 0

        frame_count = self._frame_rows_count()
        bag_size = 0
        try:
            if self.bag_path is not None and self.bag_path.exists():
                bag_size = int(self.bag_path.stat().st_size)
        except Exception:
            bag_size = 0
        bag_ok = self.bag_path is None or bag_size > 1024
        video_ok = self.deferred_postprocess or size > 1024
        frames_ok = frame_count > 0
        if not (bag_ok and video_ok and frames_ok):
            return {
                "ok": False,
                "error": "bag/video file not written or no frames captured",
                "video_path": str(self.video_path),
                "video_size": size,
                "bag_path": str(self.bag_path) if self.bag_path else "",
                "frame_ts_path": str(self.frame_ts_path) if self.frame_ts_path else "",
                "metadata_path": str(self.metadata_path) if self.metadata_path else "",
                "intrinsics_path": str(self.intrinsics_path) if self.intrinsics_path else "",
                "frame_export_dir": str(self.frame_export_dir) if self.frame_export_dir else "",
                "depth_csv_dir": str(self.depth_csv_dir) if self.depth_csv_dir else "",
                "frame_count": frame_count,
                "postprocess_required": self.deferred_postprocess,
            }
        return {
            "ok": True,
            "video_path": str(self.video_path),
            "video_size": size,
            "bag_path": str(self.bag_path) if self.bag_path else "",
            "frame_ts_path": str(self.frame_ts_path) if self.frame_ts_path else "",
            "metadata_path": str(self.metadata_path) if self.metadata_path else "",
            "intrinsics_path": str(self.intrinsics_path) if self.intrinsics_path else "",
            "frame_export_dir": str(self.frame_export_dir) if self.frame_export_dir else "",
            "depth_csv_dir": str(self.depth_csv_dir) if self.depth_csv_dir else "",
            "frame_count": frame_count,
            "device_name": self.device_name,
            "serial": self.device_serial,
            "camera_fps": int(self.video_fps),
            "video_codec": self.video_codec,
            "postprocess_required": self.deferred_postprocess,
            "postprocess_done": False,
            "export_frame_every_n": self.export_frame_every_n,
            "export_max_frames": self.max_export_frames,
        }

    def cmd_shutdown(self) -> dict[str, object]:
        try:
            self.cmd_stop()
        except Exception:
            pass
        self.running = False
        return {"ok": True}

    def handle(self, payload: dict[str, object]) -> dict[str, object]:
        cmd = str(payload.get("cmd", "")).strip().lower()
        if cmd == "ping":
            return self.cmd_ping()
        if cmd == "start":
            return self.cmd_start(payload)
        if cmd == "mark_start":
            return self.cmd_mark_start(payload)
        if cmd == "status":
            return self.cmd_status()
        if cmd == "stop":
            return self.cmd_stop()
        if cmd == "shutdown":
            return self.cmd_shutdown()
        return {"ok": False, "error": f"unknown cmd: {cmd}"}


def run_server(host: str, port: int) -> int:
    svc = RealSenseRecorderService()
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(8)
    print(f"[camera-service] listening on {host}:{port}", flush=True)
    try:
        while svc.running:
            conn, _ = srv.accept()
            with conn:
                conn.settimeout(3.0)
                data = b""
                while not data.endswith(b"\n"):
                    chunk = conn.recv(4096)
                    if not chunk:
                        break
                    data += chunk
                try:
                    payload = json.loads(data.decode("utf-8", errors="replace").strip()) if data else {}
                except Exception as exc:
                    resp = {"ok": False, "error": f"invalid json: {exc}"}
                else:
                    resp = svc.handle(payload)
                conn.sendall((json.dumps(resp, ensure_ascii=True) + "\n").encode("utf-8"))
    finally:
        try:
            svc.cmd_shutdown()
        except Exception:
            pass
        srv.close()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="RealSense recorder service")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=61337)
    args = parser.parse_args()
    return run_server(args.host, args.port)


if __name__ == "__main__":
    raise SystemExit(main())

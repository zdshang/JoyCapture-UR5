#!/usr/bin/env python3
"""Export videos, RGB/depth frames, and metadata from a RealSense BAG file."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import pyrealsense2 as rs


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "paths"


@dataclass(frozen=True)
class FrameTarget:
    frame_idx: int
    rs_frame_number: int | None
    rs_timestamp_ms: float | None


class FfmpegBgrWriter:
    """Tiny pipe-based writer used because OpenCV codecs vary across machines."""

    def __init__(self, path: Path, fps: float) -> None:
        self.path = path
        self.fps = max(1.0, float(fps))
        self.proc: subprocess.Popen[bytes] | None = None
        self.size: tuple[int, int] | None = None

    def open(self, width: int, height: int) -> None:
        if self.proc is not None:
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        cmd = [
            "ffmpeg",
            "-y",
            "-loglevel",
            "error",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "bgr24",
            "-s",
            f"{width}x{height}",
            "-r",
            f"{self.fps:g}",
            "-i",
            "pipe:0",
            "-an",
            "-c:v",
            "mpeg4",
            "-q:v",
            "5",
            "-tag:v",
            "mp4v",
            "-pix_fmt",
            "yuv420p",
            str(self.path),
        ]
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
        self.size = (width, height)

    def write(self, frame_bgr: np.ndarray) -> None:
        if frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
            raise ValueError("video frame must be BGR uint8 HxWx3")
        height, width = frame_bgr.shape[:2]
        self.open(width, height)
        if self.size != (width, height):
            raise ValueError(f"video frame size changed from {self.size} to {(width, height)}")
        if self.proc is None or self.proc.stdin is None:
            raise RuntimeError("ffmpeg writer is not open")
        self.proc.stdin.write(frame_bgr.astype(np.uint8, copy=False).tobytes())

    def close(self) -> None:
        if self.proc is None:
            return
        if self.proc.stdin is not None:
            self.proc.stdin.close()
        ret = self.proc.wait()
        self.proc = None
        if ret != 0:
            raise RuntimeError(f"ffmpeg failed with exit code {ret}")


def _json_dump(payload: Any) -> str:
    return json.dumps(payload, ensure_ascii=False, indent=2)


def _safe_label(value: str, fallback: str = "camera") -> str:
    text = "".join(ch if ch.isalnum() or ch in ("_", "-") else "_" for ch in value.strip().lower())
    return text or fallback


def _as_path(value: str | Path | None) -> Path | None:
    if not value:
        return None
    return Path(str(value)).expanduser()


def _read_json(path: Path | None) -> dict[str, Any]:
    if path is None or not path.exists():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(_json_dump(payload) + "\n", encoding="utf-8")


def _float_or_none(value: str | None) -> float | None:
    if value is None or value == "":
        return None
    try:
        return float(value)
    except ValueError:
        return None


def _int_or_none(value: str | None) -> int | None:
    number = _float_or_none(value)
    if number is None:
        return None
    return int(number)


def load_frame_targets(path: Path | None) -> list[FrameTarget]:
    """Load target frame indices from the live capture timestamp CSV."""
    if path is None or not path.exists():
        return []
    targets: list[FrameTarget] = []
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            frame_idx = _int_or_none(row.get("frame_idx"))
            if frame_idx is None:
                continue
            targets.append(
                FrameTarget(
                    frame_idx=frame_idx,
                    rs_frame_number=_int_or_none(row.get("rs_frame_number")),
                    rs_timestamp_ms=_float_or_none(row.get("rs_timestamp_ms")),
                )
            )
    return targets


def target_lookup_maps(targets: list[FrameTarget]) -> tuple[dict[int, int], dict[int, int]]:
    by_frame_number: dict[int, int] = {}
    by_timestamp_us: dict[int, int] = {}
    for target in targets:
        if target.rs_frame_number is not None and target.rs_frame_number not in by_frame_number:
            by_frame_number[target.rs_frame_number] = target.frame_idx
        if target.rs_timestamp_ms is not None:
            key = int(round(target.rs_timestamp_ms * 1000.0))
            by_timestamp_us.setdefault(key, target.frame_idx)
    return by_frame_number, by_timestamp_us


def parse_media_selection(value: str) -> set[str]:
    selected = {item.strip().lower() for item in value.split(",") if item.strip()}
    if not selected or "all" in selected:
        return {"video", "frames", "depth_csv"}
    if "none" in selected:
        return set()
    allowed = {"video", "frames", "depth_csv"}
    unknown = selected - allowed
    if unknown:
        raise SystemExit(f"Unsupported media output(s): {', '.join(sorted(unknown))}")
    return selected


def cleanup_previous_outputs(
    video_path: Path,
    frame_dir: Path,
    depth_csv_dir: Path,
    label: str,
    media: set[str],
) -> None:
    if "video" in media and video_path.exists():
        video_path.unlink()
    cleanup_specs: list[tuple[Path, tuple[str, ...]]] = []
    if "frames" in media:
        cleanup_specs.append((frame_dir, (f"{label}_color_*.png", f"{label}_depth_*.png", f"{label}_depth_vis_*.png")))
    if "depth_csv" in media:
        cleanup_specs.append((depth_csv_dir, (f"{label}_depth_*.csv",)))
    for directory, patterns in cleanup_specs:
        if not directory.exists():
            continue
        for pattern in patterns:
            for path in directory.glob(pattern):
                if path.is_file():
                    path.unlink()


def color_frame_to_bgr(color: rs.video_frame) -> np.ndarray | None:
    data = np.asanyarray(color.get_data())
    if data.size == 0:
        return None
    fmt = color.profile.format()
    if fmt == rs.format.bgr8:
        return data
    if fmt == rs.format.rgb8:
        return cv2.cvtColor(data, cv2.COLOR_RGB2BGR)
    if fmt == rs.format.rgba8:
        return cv2.cvtColor(data, cv2.COLOR_RGBA2BGR)
    if fmt == rs.format.bgra8:
        return cv2.cvtColor(data, cv2.COLOR_BGRA2BGR)
    if data.ndim == 3 and data.shape[2] == 3:
        return data
    return None


def should_export_artifact(frame_idx: int, every_n: int, exported_count: int, max_frames: int) -> bool:
    if every_n <= 0:
        return False
    if frame_idx % every_n != 0:
        return False
    if max_frames > 0 and exported_count >= max_frames:
        return False
    return True


def resolve_target_frame_idx(
    *,
    color: rs.video_frame,
    bag_sequence_idx: int,
    targets: list[FrameTarget],
    by_frame_number: dict[int, int],
    by_timestamp_us: dict[int, int],
    match_mode: str,
) -> int | None:
    """Map a BAG color frame back to the frame index used during live capture."""
    if not targets:
        return bag_sequence_idx
    if match_mode == "order":
        if bag_sequence_idx >= len(targets):
            return None
        return targets[bag_sequence_idx].frame_idx
    frame_number = int(color.get_frame_number())
    if frame_number in by_frame_number:
        return by_frame_number[frame_number]
    timestamp_key = int(round(float(color.get_timestamp()) * 1000.0))
    return by_timestamp_us.get(timestamp_key)


def export_depth_artifacts(
    label: str,
    frame_idx: int,
    depth: rs.depth_frame | None,
    frame_dir: Path,
    depth_csv_dir: Path,
    *,
    write_frames: bool,
    write_csv: bool,
) -> bool:
    if depth is None:
        return False
    depth_np = np.asanyarray(depth.get_data())
    if depth_np.size == 0:
        return False
    if write_frames:
        frame_dir.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(frame_dir / f"{label}_depth_{frame_idx:06d}.png"), depth_np)
        depth_vis = cv2.applyColorMap(cv2.convertScaleAbs(depth_np, alpha=0.03), cv2.COLORMAP_JET)
        cv2.imwrite(str(frame_dir / f"{label}_depth_vis_{frame_idx:06d}.png"), depth_vis)
    if write_csv:
        depth_csv_dir.mkdir(parents=True, exist_ok=True)
        np.savetxt(depth_csv_dir / f"{label}_depth_{frame_idx:06d}.csv", depth_np, fmt="%u", delimiter=",")
    return True


def export_bag(args: argparse.Namespace, *, match_mode: str) -> dict[str, Any]:
    """Replay a BAG once and export the requested media outputs."""
    label = _safe_label(args.label)
    bag_path = Path(args.bag).expanduser()
    video_path = Path(args.video_path).expanduser()
    frame_dir = Path(args.frame_output_dir).expanduser()
    depth_csv_dir = Path(args.depth_csv_dir).expanduser()
    timestamp_csv = _as_path(args.timestamp_csv)
    targets = load_frame_targets(timestamp_csv)
    by_frame_number, by_timestamp_us = target_lookup_maps(targets)
    media = parse_media_selection(str(args.media))
    do_video = "video" in media
    do_frames = "frames" in media
    do_depth_csv = "depth_csv" in media

    cleanup_previous_outputs(video_path, frame_dir, depth_csv_dir, label, media)
    if do_frames:
        frame_dir.mkdir(parents=True, exist_ok=True)
    if do_depth_csv:
        depth_csv_dir.mkdir(parents=True, exist_ok=True)

    pipeline = rs.pipeline()
    config = rs.config()
    rs.config.enable_device_from_file(config, str(bag_path), repeat_playback=False)
    profile = pipeline.start(config)
    playback = profile.get_device().as_playback()
    playback.set_real_time(False)

    writer = FfmpegBgrWriter(video_path, float(args.fps)) if do_video else None
    bag_sequence_idx = 0
    matched_frames = 0
    video_frames = 0
    artifact_frames = 0
    depth_frames = 0
    skipped_no_target = 0
    try:
        while True:
            try:
                frames = pipeline.wait_for_frames(1000)
            except RuntimeError:
                break
            color = frames.get_color_frame()
            if not color:
                continue
            frame_idx = resolve_target_frame_idx(
                color=color,
                bag_sequence_idx=bag_sequence_idx,
                targets=targets,
                by_frame_number=by_frame_number,
                by_timestamp_us=by_timestamp_us,
                match_mode=match_mode,
            )
            bag_sequence_idx += 1
            if frame_idx is None:
                skipped_no_target += 1
                continue

            bgr = color_frame_to_bgr(color)
            if bgr is None:
                continue
            if writer is not None:
                writer.write(bgr)
            matched_frames += 1
            if do_video:
                video_frames += 1

            if (do_frames or do_depth_csv) and should_export_artifact(
                frame_idx,
                max(1, int(args.export_frame_every_n)),
                artifact_frames,
                max(0, int(args.export_max_frames)),
            ):
                if do_frames:
                    cv2.imwrite(str(frame_dir / f"{label}_color_{frame_idx:06d}.png"), bgr)
                depth = frames.get_depth_frame()
                if export_depth_artifacts(
                    label,
                    frame_idx,
                    depth,
                    frame_dir,
                    depth_csv_dir,
                    write_frames=do_frames,
                    write_csv=do_depth_csv,
                ):
                    depth_frames += 1
                artifact_frames += 1
    finally:
        try:
            if writer is not None:
                writer.close()
        finally:
            pipeline.stop()

    video_size = video_path.stat().st_size if video_path.exists() else 0
    ok = matched_frames > 0 and (not do_video or video_size > 1024)
    return {
        "ok": ok,
        "label": label,
        "media": sorted(media),
        "bag_path": str(bag_path),
        "video_path": str(video_path),
        "video_size": video_size,
        "frame_export_dir": str(frame_dir),
        "depth_csv_dir": str(depth_csv_dir),
        "timestamp_csv": str(timestamp_csv or ""),
        "match_mode": match_mode,
        "timestamp_target_count": len(targets),
        "bag_color_frames_seen": bag_sequence_idx,
        "matched_frames": matched_frames,
        "video_frames": video_frames,
        "artifact_frames": artifact_frames,
        "depth_frames": depth_frames,
        "skipped_no_target": skipped_no_target,
        "fps": float(args.fps),
        "export_frame_every_n": int(args.export_frame_every_n),
        "export_max_frames": int(args.export_max_frames),
    }


def update_metadata_json(args: argparse.Namespace, summary: dict[str, Any]) -> None:
    metadata_path = _as_path(args.metadata_json)
    if metadata_path is None:
        return
    meta = _read_json(metadata_path)
    meta.update(
        {
            "video_path": summary["video_path"],
            "frame_export_dir": summary["frame_export_dir"],
            "depth_csv_dir": summary["depth_csv_dir"],
            "video_codec": "ffmpeg_mpeg4_postprocess",
            "postprocess_required": False,
            "postprocess_done": True,
            "postprocess_summary": summary,
            "frame_count": summary["matched_frames"],
        }
    )
    _write_json(metadata_path, meta)


def fill_args_from_manifest(args: argparse.Namespace) -> argparse.Namespace:
    manifest_path = _as_path(args.manifest)
    if manifest_path is None:
        return args
    manifest = _read_json(manifest_path)
    session_id = str(manifest.get("created_at", "") or args.session_id or "")
    raw_root = manifest_path.parents[1]
    selected: dict[str, Any] | None = None
    requested_label = _safe_label(args.label, "") if args.label else ""
    for camera in manifest.get("cameras", []) or []:
        if not isinstance(camera, dict):
            continue
        label = _safe_label(str(camera.get("label", "") or camera.get("device_name", "")))
        if requested_label and label != requested_label:
            continue
        if selected is None:
            selected = camera
        if bool(camera.get("postprocess_required", False)) or "postprocess" in str(camera.get("video_codec", "")).lower():
            selected = camera
            break
    if selected is None:
        raise SystemExit(f"No camera record found in manifest: {manifest_path}")
    label = _safe_label(str(selected.get("label", "") or selected.get("device_name", "")))
    args.label = args.label or label
    args.session_id = args.session_id or session_id
    args.bag = args.bag or str(selected.get("bag_path", ""))
    args.video_path = args.video_path or str(
        selected.get("video_path", "") or raw_root / "camera_video" / label / f"{label}_{session_id}.mp4"
    )
    args.frame_output_dir = args.frame_output_dir or str(
        selected.get("frame_export_dir", "") or raw_root / "camera_frames" / label / f"{label}_{session_id}"
    )
    args.depth_csv_dir = args.depth_csv_dir or str(
        selected.get("depth_csv_dir", "") or raw_root / "camera_depth_csv" / label / f"{label}_{session_id}"
    )
    args.timestamp_csv = args.timestamp_csv or str(selected.get("frame_ts_path", ""))
    args.metadata_json = args.metadata_json or str(selected.get("metadata_path", ""))
    args.intrinsics_json = args.intrinsics_json or str(selected.get("intrinsics_path", ""))
    args.fps = args.fps or float(selected.get("camera_fps", 30) or 30)
    args.export_frame_every_n = args.export_frame_every_n or int(selected.get("export_frame_every_n", 1) or 1)
    args.export_max_frames = args.export_max_frames or int(selected.get("export_max_frames", 0) or 0)
    return args


def validate_args(args: argparse.Namespace) -> None:
    missing = []
    for name in ("bag", "label", "video_path", "frame_output_dir", "depth_csv_dir"):
        if not getattr(args, name, ""):
            missing.append(f"--{name.replace('_', '-')}")
    if missing:
        raise SystemExit("Missing required arguments: " + ", ".join(missing))
    bag_path = Path(args.bag).expanduser()
    if not bag_path.exists():
        raise SystemExit(f"BAG file not found: {bag_path}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Post-process a RealSense .bag into MP4, PNG frames, and depth CSV.")
    parser.add_argument("--manifest", default="", help="Optional session_manifest_*.json. Missing paths are read from it.")
    parser.add_argument("--bag", default="", help="Input RealSense .bag file.")
    parser.add_argument("--label", default="", help="Camera label, e.g. l515.")
    parser.add_argument("--session-id", default="", help="Session id, e.g. 20260523_221812.")
    parser.add_argument("--video-path", default="", help="Output MP4 path.")
    parser.add_argument("--frame-output-dir", default="", help="Output directory for color/depth/depth_vis PNG files.")
    parser.add_argument("--depth-csv-dir", default="", help="Output directory for depth CSV files.")
    parser.add_argument("--timestamp-csv", default="", help="Existing frame timestamp CSV used for frame-number alignment.")
    parser.add_argument("--metadata-json", default="", help="Metadata JSON to update after successful export.")
    parser.add_argument("--intrinsics-json", default="", help="Reserved for compatibility; intrinsics are recorded live.")
    parser.add_argument(
        "--media",
        default="video,frames,depth_csv",
        help="Comma-separated media outputs: video,frames,depth_csv,all,none.",
    )
    parser.add_argument("--fps", type=float, default=0.0, help="Output MP4 FPS. Defaults to camera_fps from manifest or 30.")
    parser.add_argument("--export-frame-every-n", type=int, default=0, help="Export one PNG/CSV artifact every N frames.")
    parser.add_argument("--export-max-frames", type=int, default=0, help="Maximum PNG/CSV artifact frames, 0 means no limit.")
    return fill_args_from_manifest(parser.parse_args())


def main() -> int:
    args = parse_args()
    if not args.fps:
        args.fps = 30.0
    if not args.export_frame_every_n:
        args.export_frame_every_n = 1
    validate_args(args)

    summary = export_bag(args, match_mode="exact")
    if not summary["ok"] and summary["timestamp_target_count"] and summary["matched_frames"] == 0:
        summary = export_bag(args, match_mode="order")
    if summary["ok"]:
        update_metadata_json(args, summary)
    else:
        summary["error"] = "postprocess did not produce selected media or no frames were matched"
    print(json.dumps(summary, ensure_ascii=False, separators=(",", ":")), flush=True)
    return 0 if summary["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

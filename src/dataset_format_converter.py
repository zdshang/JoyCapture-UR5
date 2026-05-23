#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import shutil
from pathlib import Path
from typing import Any

import cv2
import h5py
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RAW_ROOT = REPO_ROOT / "paths"


def _json_dump(obj: Any) -> str:
    return json.dumps(obj, ensure_ascii=False, indent=2)


def _json_line(obj: Any) -> str:
    return json.dumps(obj, ensure_ascii=False, separators=(",", ":"))


def _safe_label(value: str, fallback: str = "camera") -> str:
    text = "".join(ch if ch.isalnum() or ch in ("_", "-") else "_" for ch in value.strip().lower())
    return text or fallback


def _session_from_manifest(manifest: dict[str, Any]) -> str:
    return str(manifest.get("created_at", "") or "unknown_session")


def _resolve_existing(path_value: str | Path | None, raw_root: Path) -> Path | None:
    if not path_value:
        return None
    path = Path(str(path_value)).expanduser()
    if path.exists():
        return path
    if not path.is_absolute():
        candidate = raw_root / path
        if candidate.exists():
            return candidate
    return None


def _latest_manifest(raw_root: Path) -> Path:
    manifests = sorted(
        set(raw_root.rglob("session_metadata/session_manifest_*.json")),
        key=lambda path: path.stat().st_mtime,
    )
    if not manifests:
        raise FileNotFoundError(f"No session manifests found under {raw_root}")
    return manifests[-1]


def resolve_manifest(input_path: str | Path, session_id: str = "latest") -> Path:
    path = Path(input_path).expanduser()
    if path.is_file() and path.suffix.lower() == ".json":
        return path.resolve()
    raw_root = path.resolve()
    if session_id == "latest":
        return _latest_manifest(raw_root).resolve()
    matches = sorted(
        set(raw_root.rglob(f"session_metadata/session_manifest_{session_id}.json")),
        key=lambda path: path.stat().st_mtime,
    )
    if matches:
        return matches[-1].resolve()
    raise FileNotFoundError(f"Session manifest not found under {raw_root}: {session_id}")


def load_manifest(manifest_path: Path) -> dict[str, Any]:
    return json.loads(manifest_path.read_text(encoding="utf-8"))


def read_csv_rows(path: Path | None) -> tuple[list[str], list[dict[str, str]]]:
    if path is None or not path.exists():
        return [], []
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        return list(reader.fieldnames or []), [dict(row) for row in reader]


def rows_to_float_array(columns: list[str], rows: list[dict[str, str]]) -> np.ndarray:
    data = np.full((len(rows), len(columns)), np.nan, dtype=np.float64)
    for r_idx, row in enumerate(rows):
        for c_idx, name in enumerate(columns):
            value = row.get(name, "")
            if value is None or value == "":
                continue
            try:
                data[r_idx, c_idx] = float(value)
            except ValueError:
                continue
    return data


def write_csv_rows(path: Path, columns: list[str], rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=columns)
        writer.writeheader()
        for row in rows:
            writer.writerow({name: row.get(name, "") for name in columns})


def _copy_if_exists(src: Path | None, dst: Path) -> bool:
    if src is None or not src.exists():
        return False
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    return True


def _resolve_rlds_media_path(rlds_dir: Path, value: Any) -> Path | None:
    raw = str(value or "").strip()
    if not raw:
        return None
    path = Path(raw).expanduser()
    if path.is_absolute():
        return path
    return rlds_dir / path


def _camera_frame_dir(raw_root: Path, session_id: str, label: str, camera: dict[str, Any]) -> Path:
    meta_path = _resolve_existing(camera.get("metadata_path", ""), raw_root)
    if meta_path is not None:
        try:
            meta = json.loads(meta_path.read_text(encoding="utf-8"))
            frame_dir = _resolve_existing(meta.get("frame_export_dir", ""), raw_root)
            if frame_dir is not None:
                return frame_dir
        except Exception:
            pass
    return raw_root / "camera_frames" / label / f"{label}_{session_id}"


def _camera_depth_csv_dir(raw_root: Path, session_id: str, label: str, camera: dict[str, Any]) -> Path:
    meta_path = _resolve_existing(camera.get("metadata_path", ""), raw_root)
    if meta_path is not None:
        try:
            meta = json.loads(meta_path.read_text(encoding="utf-8"))
            depth_dir = _resolve_existing(meta.get("depth_csv_dir", ""), raw_root)
            if depth_dir is not None:
                return depth_dir
        except Exception:
            pass
    return raw_root / "camera_depth_csv" / label / f"{label}_{session_id}"


def _image_or_empty(path: Path, flags: int) -> np.ndarray | None:
    if not path.exists():
        return None
    img = cv2.imread(str(path), flags)
    if img is None:
        return None
    return img


def _read_depth(path_png: Path, path_csv: Path) -> np.ndarray | None:
    depth = _image_or_empty(path_png, cv2.IMREAD_UNCHANGED)
    if depth is not None:
        return depth
    if path_csv.exists():
        try:
            return np.loadtxt(path_csv, delimiter=",", dtype=np.uint16)
        except Exception:
            return None
    return None


def _write_json_dataset(group: h5py.Group, name: str, value: Any) -> None:
    if name in group:
        del group[name]
    group.create_dataset(name, data=np.bytes_(_json_dump(value).encode("utf-8")))


def _read_json_dataset(group: h5py.Group, name: str, fallback: Any = None) -> Any:
    if name not in group:
        return fallback
    data = group[name][()]
    if isinstance(data, bytes):
        text = data.decode("utf-8")
    else:
        text = bytes(data).decode("utf-8")
    return json.loads(text)


def _write_raw_table(h5: h5py.File, table_name: str, csv_path: Path | None) -> None:
    columns, rows = read_csv_rows(csv_path)
    group = h5.require_group("raw_tables").require_group(table_name)
    _write_json_dataset(group, "rows", {"columns": columns, "rows": rows})
    if columns:
        arr = rows_to_float_array(columns, rows)
        group.create_dataset("values", data=arr, compression="gzip", compression_opts=4)
        group.attrs["columns_json"] = json.dumps(columns)
    if csv_path is not None:
        group.attrs["source_path"] = str(csv_path)


def _relative_to_root(path: Path, raw_root: Path) -> str:
    try:
        return str(path.resolve().relative_to(raw_root.resolve()))
    except Exception:
        return path.name


def _binary_sources(manifest: dict[str, Any], raw_root: Path) -> list[tuple[Path, str]]:
    out: list[tuple[Path, str]] = []
    for key in ("playback_script", "pickle"):
        source = _resolve_existing(manifest.get(key, ""), raw_root)
        if source is not None and source.exists():
            out.append((source, _relative_to_root(source, raw_root)))
    for camera in manifest.get("cameras", []) or []:
        if not isinstance(camera, dict):
            continue
        for key in ("video_path", "bag_path"):
            source = _resolve_existing(camera.get(key, ""), raw_root)
            if source is not None and source.exists():
                out.append((source, _relative_to_root(source, raw_root)))
    return out


def _write_binary_files(h5: h5py.File, manifest: dict[str, Any], raw_root: Path) -> None:
    bin_group = h5.require_group("raw_binary_files")
    for idx, (source, rel_path) in enumerate(_binary_sources(manifest, raw_root)):
        item = bin_group.require_group(f"file_{idx:04d}")
        item.attrs["relative_path"] = rel_path
        item.attrs["source_path"] = str(source)
        item.create_dataset("data", data=np.frombuffer(source.read_bytes(), dtype=np.uint8), compression="gzip")


def _write_steps(h5: h5py.File, dataset_csv: Path | None) -> None:
    columns, rows = read_csv_rows(dataset_csv)
    if not rows:
        return
    arr = rows_to_float_array(columns, rows)
    idx = {name: i for i, name in enumerate(columns)}
    group = h5.require_group("steps")
    obs_group = group.require_group("observation")
    action_group = group.require_group("action")
    group.create_dataset("t_s", data=arr[:, idx["t_s"]], compression="gzip")
    pose_cols = ["pose_x", "pose_y", "pose_z", "pose_rx", "pose_ry", "pose_rz"]
    action_cols = ["cmd_vx", "cmd_vy", "cmd_vz", "cmd_wx", "cmd_wy", "cmd_wz"]
    controller_cols = ["left_x", "left_y", "right_x", "right_y", "left_trigger", "right_trigger", "lb", "rb"]
    obs_group.create_dataset(
        "robot_pose",
        data=np.column_stack([arr[:, idx[name]] for name in pose_cols]).astype(np.float32),
        compression="gzip",
        compression_opts=4,
    )
    action_group.create_dataset(
        "tcp_velocity",
        data=np.column_stack([arr[:, idx[name]] for name in action_cols]).astype(np.float32),
        compression="gzip",
        compression_opts=4,
    )
    action_group.create_dataset(
        "controller",
        data=np.column_stack([arr[:, idx[name]] for name in controller_cols]).astype(np.float32),
        compression="gzip",
        compression_opts=4,
    )
    if "gripper_open" in idx:
        obs_group.create_dataset("gripper_open", data=arr[:, idx["gripper_open"]].astype(np.uint8), compression="gzip")
        action_group.create_dataset("gripper_open", data=arr[:, idx["gripper_open"]].astype(np.uint8), compression="gzip")
    n = len(rows)
    group.create_dataset("reward", data=np.zeros(n, dtype=np.float32), compression="gzip")
    group.create_dataset("discount", data=np.ones(n, dtype=np.float32), compression="gzip")
    is_first = np.zeros(n, dtype=np.bool_)
    is_last = np.zeros(n, dtype=np.bool_)
    is_terminal = np.zeros(n, dtype=np.bool_)
    if n:
        is_first[0] = True
        is_last[-1] = True
        is_terminal[-1] = True
    group.create_dataset("is_first", data=is_first, compression="gzip")
    group.create_dataset("is_last", data=is_last, compression="gzip")
    group.create_dataset("is_terminal", data=is_terminal, compression="gzip")


def _camera_sync_rows(raw_root: Path, camera: dict[str, Any], label: str) -> tuple[list[str], list[dict[str, str]]]:
    sync_path = _resolve_existing(camera.get("sync_path", ""), raw_root)
    return read_csv_rows(sync_path)


def _write_camera_group(h5: h5py.File, raw_root: Path, session_id: str, camera: dict[str, Any]) -> None:
    label = _safe_label(str(camera.get("label", "") or camera.get("device_name", "camera")))
    group = h5.require_group("cameras").require_group(label)
    _write_json_dataset(group, "metadata", camera)
    source_metadata = _resolve_existing(camera.get("metadata_path", ""), raw_root)
    if source_metadata is not None:
        try:
            _write_json_dataset(group, "camera_metadata_json", json.loads(source_metadata.read_text(encoding="utf-8")))
        except Exception:
            pass
    source_intrinsics = _resolve_existing(camera.get("intrinsics_path", ""), raw_root)
    if source_intrinsics is not None:
        try:
            _write_json_dataset(group, "camera_intrinsics_json", json.loads(source_intrinsics.read_text(encoding="utf-8")))
        except Exception:
            pass
    ts_cols, ts_rows = read_csv_rows(_resolve_existing(camera.get("frame_ts_path", ""), raw_root))
    if ts_rows:
        ts_group = group.require_group("frame_timestamps")
        _write_json_dataset(ts_group, "rows", {"columns": ts_cols, "rows": ts_rows})
        ts_group.create_dataset("values", data=rows_to_float_array(ts_cols, ts_rows), compression="gzip", compression_opts=4)
        ts_group.attrs["columns_json"] = json.dumps(ts_cols)
    sync_cols, sync_rows = _camera_sync_rows(raw_root, camera, label)
    if sync_rows:
        sync_group = group.require_group("sync")
        _write_json_dataset(sync_group, "rows", {"columns": sync_cols, "rows": sync_rows})
        sync_group.create_dataset("values", data=rows_to_float_array(sync_cols, sync_rows), compression="gzip", compression_opts=4)
        sync_group.attrs["columns_json"] = json.dumps(sync_cols)

    frame_dir = _camera_frame_dir(raw_root, session_id, label, camera)
    depth_csv_dir = _camera_depth_csv_dir(raw_root, session_id, label, camera)
    frame_indices: list[int] = []
    rgb_frames: list[np.ndarray] = []
    depth_frames: list[np.ndarray] = []
    for row in sync_rows:
        try:
            frame_idx = int(float(row.get("frame_idx", "")))
        except ValueError:
            continue
        color_path = frame_dir / f"{label}_color_{frame_idx:06d}.png"
        depth_png_path = frame_dir / f"{label}_depth_{frame_idx:06d}.png"
        depth_csv_path = depth_csv_dir / f"{label}_depth_{frame_idx:06d}.csv"
        color = _image_or_empty(color_path, cv2.IMREAD_COLOR)
        depth = _read_depth(depth_png_path, depth_csv_path)
        if color is None and depth is None:
            continue
        frame_indices.append(frame_idx)
        if color is not None:
            rgb_frames.append(cv2.cvtColor(color, cv2.COLOR_BGR2RGB))
        if depth is not None:
            depth_frames.append(depth)

    frames_group = group.require_group("frames")
    frames_group.create_dataset("frame_idx", data=np.asarray(frame_indices, dtype=np.int64), compression="gzip")
    if rgb_frames and len(rgb_frames) == len(frame_indices):
        frames_group.create_dataset(
            "rgb",
            data=np.stack(rgb_frames).astype(np.uint8),
            compression="gzip",
            compression_opts=4,
            chunks=(1, *rgb_frames[0].shape),
        )
    if depth_frames and len(depth_frames) == len(frame_indices):
        frames_group.create_dataset(
            "depth",
            data=np.stack(depth_frames).astype(np.uint16),
            compression="gzip",
            compression_opts=4,
            chunks=(1, *depth_frames[0].shape),
        )


def raw_to_hdf5(manifest_path: Path, output_path: Path, *, embed_binary: bool = False) -> Path:
    manifest = load_manifest(manifest_path)
    raw_root = manifest_path.parents[1]
    session_id = _session_from_manifest(manifest)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with h5py.File(output_path, "w") as h5:
        h5.attrs["format"] = "ur5_hdf5_v1"
        h5.attrs["session_id"] = session_id
        h5.attrs["source_format"] = "raw"
        _write_json_dataset(h5.require_group("metadata"), "session_manifest", manifest)
        _write_raw_table(h5, "robot_path", _resolve_existing(manifest.get("robot_path_csv", ""), raw_root))
        _write_raw_table(h5, "actions", _resolve_existing(manifest.get("actions_csv", ""), raw_root))
        _write_raw_table(h5, "dataset_samples", _resolve_existing(manifest.get("dataset_samples_csv", ""), raw_root))
        _write_raw_table(h5, "gripper_events", _resolve_existing(manifest.get("gripper_events_csv", ""), raw_root))
        _write_steps(h5, _resolve_existing(manifest.get("dataset_samples_csv", ""), raw_root))
        for camera in manifest.get("cameras", []) or []:
            if isinstance(camera, dict):
                _write_camera_group(h5, raw_root, session_id, camera)
        if embed_binary:
            _write_binary_files(h5, manifest, raw_root)
    return output_path


def _step_rows_from_manifest(manifest_path: Path) -> tuple[list[str], list[dict[str, str]]]:
    manifest = load_manifest(manifest_path)
    raw_root = manifest_path.parents[1]
    return read_csv_rows(_resolve_existing(manifest.get("dataset_samples_csv", ""), raw_root))


def _rlds_step_from_row(row: dict[str, str], idx: int, total: int) -> dict[str, Any]:
    def f(name: str, default: float = 0.0) -> float:
        try:
            return float(row.get(name, default))
        except ValueError:
            return default

    return {
        "step_index": idx,
        "timestamp_s": f("t_s"),
        "observation": {
            "robot_pose": [f(name) for name in ("pose_x", "pose_y", "pose_z", "pose_rx", "pose_ry", "pose_rz")],
            "gripper_open": int(f("gripper_open")),
        },
        "action": {
            "tcp_velocity": [f(name) for name in ("cmd_vx", "cmd_vy", "cmd_vz", "cmd_wx", "cmd_wy", "cmd_wz")],
            "gripper_open": int(f("gripper_open")),
            "controller": {
                "left_x": f("left_x"),
                "left_y": f("left_y"),
                "right_x": f("right_x"),
                "right_y": f("right_y"),
                "left_trigger": f("left_trigger"),
                "right_trigger": f("right_trigger"),
                "lb": int(f("lb")),
                "rb": int(f("rb")),
            },
        },
        "reward": 0.0,
        "discount": 1.0,
        "is_first": idx == 0,
        "is_last": idx == total - 1,
        "is_terminal": idx == total - 1,
    }


def _write_rlds_camera_index(out_dir: Path, raw_root: Path, session_id: str, camera: dict[str, Any], copy_media: bool) -> None:
    label = _safe_label(str(camera.get("label", "") or camera.get("device_name", "camera")))
    camera_dir = out_dir / "cameras" / label
    camera_dir.mkdir(parents=True, exist_ok=True)
    sync_cols, sync_rows = _camera_sync_rows(raw_root, camera, label)
    frame_dir = _camera_frame_dir(raw_root, session_id, label, camera)
    depth_dir = _camera_depth_csv_dir(raw_root, session_id, label, camera)
    with (camera_dir / "frames.jsonl").open("w", encoding="utf-8") as f:
        for row in sync_rows:
            try:
                frame_idx = int(float(row.get("frame_idx", "")))
            except ValueError:
                continue
            color = frame_dir / f"{label}_color_{frame_idx:06d}.png"
            depth_png = frame_dir / f"{label}_depth_{frame_idx:06d}.png"
            depth_vis = frame_dir / f"{label}_depth_vis_{frame_idx:06d}.png"
            depth_csv = depth_dir / f"{label}_depth_{frame_idx:06d}.csv"
            record = {
                "frame_idx": frame_idx,
                "camera_t_rel_s": row.get("camera_t_rel_s", ""),
                "robot_t_s_interp": row.get("robot_t_s_interp", ""),
                "rgb_path": str(color),
                "depth_png_path": str(depth_png),
                "depth_vis_path": str(depth_vis),
                "depth_csv_path": str(depth_csv),
            }
            if copy_media:
                media_dir = camera_dir / "media"
                media_dir.mkdir(parents=True, exist_ok=True)
                for key in ("rgb_path", "depth_png_path", "depth_vis_path", "depth_csv_path"):
                    src = Path(record[key])
                    if src.exists():
                        dst = media_dir / src.name
                        shutil.copy2(src, dst)
                        record[key] = str(dst.relative_to(out_dir))
            f.write(_json_line(record) + "\n")
    _write_json_file(camera_dir / "sync_schema.json", {"columns": sync_cols})


def _write_json_file(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(_json_dump(payload) + "\n", encoding="utf-8")


def raw_to_rlds(manifest_path: Path, output_dir: Path, *, copy_media: bool = False) -> Path:
    manifest = load_manifest(manifest_path)
    raw_root = manifest_path.parents[1]
    session_id = _session_from_manifest(manifest)
    output_dir.mkdir(parents=True, exist_ok=True)
    _write_json_file(
        output_dir / "episode_metadata.json",
        {
            "format": "ur5_rlds_directory_v1",
            "schema_note": "RLDS-style directory with episode/step fields; not a TensorFlow Datasets package.",
            "session_manifest": manifest,
        },
    )
    columns, rows = _step_rows_from_manifest(manifest_path)
    _write_json_file(output_dir / "step_schema.json", {"columns": columns})
    with (output_dir / "steps.jsonl").open("w", encoding="utf-8") as f:
        for idx, row in enumerate(rows):
            f.write(_json_line(_rlds_step_from_row(row, idx, len(rows))) + "\n")
    for camera in manifest.get("cameras", []) or []:
        if isinstance(camera, dict):
            _write_rlds_camera_index(output_dir, raw_root, session_id, camera, copy_media)
    return output_dir


def hdf5_to_raw(hdf5_path: Path, output_root: Path) -> Path:
    output_root.mkdir(parents=True, exist_ok=True)
    with h5py.File(hdf5_path, "r") as h5:
        manifest = _read_json_dataset(h5["metadata"], "session_manifest", {})
        session_id = _session_from_manifest(manifest)
        manifest_path = output_root / "session_metadata" / f"session_manifest_{session_id}.json"
        _write_json_file(manifest_path, manifest)
        table_map = {
            "robot_path": output_root / "csv" / f"ur5_path_{session_id}.csv",
            "actions": output_root / "actions" / f"ur5_actions_{session_id}.csv",
            "dataset_samples": output_root / "dataset_samples" / f"ur5_dataset_samples_{session_id}.csv",
            "gripper_events": output_root / "gripper_events" / f"ur5_gripper_events_{session_id}.csv",
        }
        if "raw_tables" in h5:
            for name, path in table_map.items():
                if name in h5["raw_tables"]:
                    payload = _read_json_dataset(h5["raw_tables"][name], "rows", {"columns": [], "rows": []})
                    write_csv_rows(path, payload.get("columns", []), payload.get("rows", []))
        manifest["robot_path_csv"] = str(table_map["robot_path"])
        manifest["actions_csv"] = str(table_map["actions"])
        manifest["dataset_samples_csv"] = str(table_map["dataset_samples"])
        manifest["gripper_events_csv"] = str(table_map["gripper_events"])
        if "cameras" in h5:
            for label, cam_group in h5["cameras"].items():
                camera_manifest = _read_json_dataset(cam_group, "metadata", {})
                if isinstance(camera_manifest, dict):
                    for camera in manifest.get("cameras", []) or []:
                        if isinstance(camera, dict) and _safe_label(str(camera.get("label", "") or camera.get("device_name", "camera"))) == label:
                            camera.update(camera_manifest)
                if "camera_metadata_json" in cam_group:
                    metadata_path = output_root / "camera_metadata" / label / f"{label}_metadata_{session_id}.json"
                    _write_json_file(metadata_path, _read_json_dataset(cam_group, "camera_metadata_json", {}))
                if "camera_intrinsics_json" in cam_group:
                    intrinsics_path = output_root / "camera_intrinsics" / label / f"{label}_intrinsics_{session_id}.json"
                    _write_json_file(intrinsics_path, _read_json_dataset(cam_group, "camera_intrinsics_json", {}))
                if "frame_timestamps" in cam_group:
                    payload = _read_json_dataset(cam_group["frame_timestamps"], "rows", {"columns": [], "rows": []})
                    frame_ts_path = output_root / "camera_timestamps" / label / f"{label}_frames_{session_id}.csv"
                    write_csv_rows(
                        frame_ts_path,
                        payload.get("columns", []),
                        payload.get("rows", []),
                    )
                if "sync" in cam_group:
                    payload = _read_json_dataset(cam_group["sync"], "rows", {"columns": [], "rows": []})
                    sync_path = output_root / "sync" / f"ur5_camera_sync_{label}_{session_id}.csv"
                    write_csv_rows(
                        sync_path,
                        payload.get("columns", []),
                        payload.get("rows", []),
                    )
                frame_idx = cam_group.get("frames/frame_idx")
                frames_dir = output_root / "camera_frames" / label / f"{label}_{session_id}"
                depth_dir = output_root / "camera_depth_csv" / label / f"{label}_{session_id}"
                frame_count = 0
                if frame_idx is not None:
                    rgb = cam_group.get("frames/rgb")
                    depth = cam_group.get("frames/depth")
                    frames_dir.mkdir(parents=True, exist_ok=True)
                    depth_dir.mkdir(parents=True, exist_ok=True)
                    indices = frame_idx[:].astype(int).tolist()
                    frame_count = len(indices)
                    for i, idx in enumerate(indices):
                        if rgb is not None and i < rgb.shape[0]:
                            cv2.imwrite(str(frames_dir / f"{label}_color_{idx:06d}.png"), cv2.cvtColor(rgb[i], cv2.COLOR_RGB2BGR))
                        if depth is not None and i < depth.shape[0]:
                            cv2.imwrite(str(frames_dir / f"{label}_depth_{idx:06d}.png"), depth[i])
                            np.savetxt(depth_dir / f"{label}_depth_{idx:06d}.csv", depth[i], fmt="%u", delimiter=",")
                for camera in manifest.get("cameras", []) or []:
                    if not isinstance(camera, dict):
                        continue
                    if _safe_label(str(camera.get("label", "") or camera.get("device_name", "camera"))) != label:
                        continue
                    metadata_path = output_root / "camera_metadata" / label / f"{label}_metadata_{session_id}.json"
                    intrinsics_path = output_root / "camera_intrinsics" / label / f"{label}_intrinsics_{session_id}.json"
                    frame_ts_path = output_root / "camera_timestamps" / label / f"{label}_frames_{session_id}.csv"
                    sync_path = output_root / "sync" / f"ur5_camera_sync_{label}_{session_id}.csv"
                    camera["metadata_path"] = str(metadata_path)
                    camera["intrinsics_path"] = str(intrinsics_path)
                    camera["frame_ts_path"] = str(frame_ts_path)
                    camera["frame_export_dir"] = str(frames_dir)
                    camera["depth_csv_dir"] = str(depth_dir)
                    if sync_path.exists():
                        camera["sync_path"] = str(sync_path)
                    if frame_count:
                        camera["frame_count"] = frame_count
        if "raw_binary_files" in h5:
            for _, item in h5["raw_binary_files"].items():
                rel_path = str(item.attrs.get("relative_path", ""))
                if not rel_path or "data" not in item:
                    continue
                out_path = output_root / rel_path
                out_path.parent.mkdir(parents=True, exist_ok=True)
                out_path.write_bytes(item["data"][:].tobytes())
        _write_json_file(manifest_path, manifest)
    return output_root


def rlds_to_raw(rlds_dir: Path, output_root: Path) -> Path:
    meta = json.loads((rlds_dir / "episode_metadata.json").read_text(encoding="utf-8"))
    manifest = meta.get("session_manifest", {})
    session_id = _session_from_manifest(manifest)
    output_root.mkdir(parents=True, exist_ok=True)
    manifest_path = output_root / "session_metadata" / f"session_manifest_{session_id}.json"
    columns = [
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
    rows: list[dict[str, Any]] = []
    with (rlds_dir / "steps.jsonl").open("r", encoding="utf-8") as f:
        for line in f:
            step = json.loads(line)
            obs = step.get("observation", {})
            action = step.get("action", {})
            controller = action.get("controller", {})
            pose = obs.get("robot_pose", [0] * 6)
            tcp = action.get("tcp_velocity", [0] * 6)
            rows.append(
                {
                    "t_s": step.get("timestamp_s", 0),
                    "pose_x": pose[0],
                    "pose_y": pose[1],
                    "pose_z": pose[2],
                    "pose_rx": pose[3],
                    "pose_ry": pose[4],
                    "pose_rz": pose[5],
                    "cmd_vx": tcp[0],
                    "cmd_vy": tcp[1],
                    "cmd_vz": tcp[2],
                    "cmd_wx": tcp[3],
                    "cmd_wy": tcp[4],
                    "cmd_wz": tcp[5],
                    "left_x": controller.get("left_x", 0),
                    "left_y": controller.get("left_y", 0),
                    "right_x": controller.get("right_x", 0),
                    "right_y": controller.get("right_y", 0),
                    "left_trigger": controller.get("left_trigger", 0),
                    "right_trigger": controller.get("right_trigger", 0),
                    "lb": controller.get("lb", 0),
                    "rb": controller.get("rb", 0),
                    "gripper_open": obs.get("gripper_open", 0),
                }
            )
    dataset_csv = output_root / "dataset_samples" / f"ur5_dataset_samples_{session_id}.csv"
    write_csv_rows(dataset_csv, columns, rows)
    robot_rows = [
        {
            "t_s": row["t_s"],
            "x": row["pose_x"],
            "y": row["pose_y"],
            "z": row["pose_z"],
            "rx": row["pose_rx"],
            "ry": row["pose_ry"],
            "rz": row["pose_rz"],
        }
        for row in rows
    ]
    action_rows = [
        {
            "t_s": row["t_s"],
            "cmd_vx": row["cmd_vx"],
            "cmd_vy": row["cmd_vy"],
            "cmd_vz": row["cmd_vz"],
            "cmd_wx": row["cmd_wx"],
            "cmd_wy": row["cmd_wy"],
            "cmd_wz": row["cmd_wz"],
            "left_x": row["left_x"],
            "left_y": row["left_y"],
            "right_x": row["right_x"],
            "right_y": row["right_y"],
            "left_trigger": row["left_trigger"],
            "right_trigger": row["right_trigger"],
            "lb": row["lb"],
            "rb": row["rb"],
            "a": "",
            "b": "",
            "x": "",
            "y": "",
            "back": "",
            "start": "",
            "controller_connected": "",
            "gripper_open": row["gripper_open"],
        }
        for row in rows
    ]
    robot_csv = output_root / "csv" / f"ur5_path_{session_id}.csv"
    actions_csv = output_root / "actions" / f"ur5_actions_{session_id}.csv"
    gripper_csv = output_root / "gripper_events" / f"ur5_gripper_events_{session_id}.csv"
    write_csv_rows(robot_csv, ["t_s", "x", "y", "z", "rx", "ry", "rz"], robot_rows)
    write_csv_rows(
        actions_csv,
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
        ],
        action_rows,
    )
    write_csv_rows(gripper_csv, ["t_s", "action"], [])

    manifest["robot_path_csv"] = str(robot_csv)
    manifest["actions_csv"] = str(actions_csv)
    manifest["dataset_samples_csv"] = str(dataset_csv)
    manifest["gripper_events_csv"] = str(gripper_csv)
    manifest["point_count"] = len(rows)
    manifest["action_row_count"] = len(rows)

    for camera in manifest.get("cameras", []) or []:
        if not isinstance(camera, dict):
            continue
        label = _safe_label(str(camera.get("label", "") or camera.get("device_name", "camera")))
        metadata_dst = output_root / "camera_metadata" / label / f"{label}_metadata_{session_id}.json"
        intrinsics_dst = output_root / "camera_intrinsics" / label / f"{label}_intrinsics_{session_id}.json"
        timestamp_dst = output_root / "camera_timestamps" / label / f"{label}_frames_{session_id}.csv"
        sync_dst = output_root / "sync" / f"ur5_camera_sync_{label}_{session_id}.csv"
        frame_dst = output_root / "camera_frames" / label / f"{label}_{session_id}"
        depth_dst = output_root / "camera_depth_csv" / label / f"{label}_{session_id}"

        _copy_if_exists(_resolve_existing(camera.get("metadata_path", ""), Path(".")), metadata_dst)
        _copy_if_exists(_resolve_existing(camera.get("intrinsics_path", ""), Path(".")), intrinsics_dst)
        _copy_if_exists(_resolve_existing(camera.get("frame_ts_path", ""), Path(".")), timestamp_dst)
        _copy_if_exists(_resolve_existing(camera.get("sync_path", ""), Path(".")), sync_dst)

        camera_dir = rlds_dir / "cameras" / label
        frame_rows = 0
        frames_jsonl = camera_dir / "frames.jsonl"
        if frames_jsonl.exists():
            with frames_jsonl.open("r", encoding="utf-8") as f:
                for line in f:
                    record = json.loads(line)
                    frame_idx = int(record.get("frame_idx", frame_rows))
                    color_src = _resolve_rlds_media_path(rlds_dir, record.get("rgb_path"))
                    depth_png_src = _resolve_rlds_media_path(rlds_dir, record.get("depth_png_path"))
                    depth_vis_src = _resolve_rlds_media_path(rlds_dir, record.get("depth_vis_path"))
                    depth_csv_src = _resolve_rlds_media_path(rlds_dir, record.get("depth_csv_path"))
                    if _copy_if_exists(color_src, frame_dst / f"{label}_color_{frame_idx:06d}.png"):
                        frame_rows += 1
                    _copy_if_exists(depth_png_src, frame_dst / f"{label}_depth_{frame_idx:06d}.png")
                    _copy_if_exists(depth_vis_src, frame_dst / f"{label}_depth_vis_{frame_idx:06d}.png")
                    _copy_if_exists(depth_csv_src, depth_dst / f"{label}_depth_{frame_idx:06d}.csv")
        camera["metadata_path"] = str(metadata_dst)
        camera["intrinsics_path"] = str(intrinsics_dst)
        camera["frame_ts_path"] = str(timestamp_dst)
        camera["sync_path"] = str(sync_dst)
        camera["frame_export_dir"] = str(frame_dst)
        camera["depth_csv_dir"] = str(depth_dst)
        if frame_rows:
            camera["frame_count"] = frame_rows

    _write_json_file(manifest_path, manifest)
    return output_root


def hdf5_to_rlds(hdf5_path: Path, output_dir: Path) -> Path:
    tmp_raw = output_dir.parent / f".tmp_raw_from_hdf5_{hdf5_path.stem}"
    if tmp_raw.exists():
        shutil.rmtree(tmp_raw)
    try:
        hdf5_to_raw(hdf5_path, tmp_raw)
        manifest = _latest_manifest(tmp_raw)
        return raw_to_rlds(manifest, output_dir, copy_media=False)
    finally:
        if tmp_raw.exists():
            shutil.rmtree(tmp_raw)


def rlds_to_hdf5(rlds_dir: Path, output_path: Path) -> Path:
    tmp_raw = output_path.parent / f".tmp_raw_from_rlds_{output_path.stem}"
    if tmp_raw.exists():
        shutil.rmtree(tmp_raw)
    try:
        rlds_to_raw(rlds_dir, tmp_raw)
        manifest = _latest_manifest(tmp_raw)
        return raw_to_hdf5(manifest, output_path)
    finally:
        if tmp_raw.exists():
            shutil.rmtree(tmp_raw)


def export_session(
    manifest_path: Path,
    formats: list[str],
    *,
    output_root: Path | None = None,
    rlds_copy_media: bool = False,
    hdf5_embed_binary: bool = False,
) -> dict[str, str]:
    manifest = load_manifest(manifest_path)
    raw_root = manifest_path.parents[1]
    output_root = output_root or raw_root
    session_id = _session_from_manifest(manifest)
    selected = {fmt.strip().lower() for fmt in formats if fmt.strip()}
    results: dict[str, str] = {"raw": str(raw_root)}
    if "hdf5" in selected:
        out = output_root / "hdf5" / f"ur5_episode_{session_id}.hdf5"
        results["hdf5"] = str(raw_to_hdf5(manifest_path, out, embed_binary=hdf5_embed_binary))
    if "rlds" in selected:
        out = output_root / "rlds" / f"ur5_episode_{session_id}"
        results["rlds"] = str(raw_to_rlds(manifest_path, out, copy_media=rlds_copy_media))
    return results


def parse_formats(value: str) -> list[str]:
    return [item.strip().lower() for item in value.split(",") if item.strip()]


def convert(args: argparse.Namespace) -> list[Path]:
    source = args.from_format.lower()
    targets = parse_formats(args.to)
    multiple_targets = len(targets) > 1
    outputs: list[Path] = []
    input_path = Path(args.input).expanduser()
    output = Path(args.output).expanduser() if args.output else None
    if source == "raw":
        manifest = resolve_manifest(input_path, args.session)
        raw_root = manifest.parents[1]
        session_id = _session_from_manifest(load_manifest(manifest))
        for target in targets:
            if target == "raw":
                outputs.append(raw_root)
            elif target == "hdf5":
                out = (output / "hdf5" / f"ur5_episode_{session_id}.hdf5") if output and multiple_targets else (
                    output or (raw_root / "hdf5" / f"ur5_episode_{session_id}.hdf5")
                )
                outputs.append(raw_to_hdf5(manifest, out, embed_binary=args.embed_binary))
            elif target == "rlds":
                out = (output / "rlds" / f"ur5_episode_{session_id}") if output and multiple_targets else (
                    output or (raw_root / "rlds" / f"ur5_episode_{session_id}")
                )
                outputs.append(raw_to_rlds(manifest, out, copy_media=args.copy_media))
            else:
                raise SystemExit(f"Unsupported target format: {target}")
    elif source == "hdf5":
        for target in targets:
            if target == "raw":
                out = (output / "raw") if output and multiple_targets else (output or (input_path.parent / f"{input_path.stem}_raw"))
                outputs.append(hdf5_to_raw(input_path, out))
            elif target == "rlds":
                out = (output / "rlds") if output and multiple_targets else (output or (input_path.parent / f"{input_path.stem}_rlds"))
                outputs.append(hdf5_to_rlds(input_path, out))
            elif target == "hdf5":
                outputs.append(input_path)
            else:
                raise SystemExit(f"Unsupported target format: {target}")
    elif source == "rlds":
        for target in targets:
            if target == "raw":
                out = (output / "raw") if output and multiple_targets else (output or (input_path.parent / f"{input_path.name}_raw"))
                outputs.append(rlds_to_raw(input_path, out))
            elif target == "hdf5":
                out = (output / "hdf5" / f"{input_path.name}.hdf5") if output and multiple_targets else (
                    output or (input_path.parent / f"{input_path.name}.hdf5")
                )
                outputs.append(rlds_to_hdf5(input_path, out))
            elif target == "rlds":
                outputs.append(input_path)
            else:
                raise SystemExit(f"Unsupported target format: {target}")
    else:
        raise SystemExit(f"Unsupported source format: {source}")
    return outputs


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Convert UR5 Xbox datasets between raw, HDF5, and RLDS-style formats.")
    parser.add_argument("--from", dest="from_format", choices=["raw", "hdf5", "rlds"], required=True)
    parser.add_argument("--to", required=True, help="Comma-separated target formats: raw,hdf5,rlds")
    parser.add_argument("--input", required=True, help="Input raw paths dir/session manifest, .hdf5 file, or RLDS directory")
    parser.add_argument("--output", default="", help="Output path. If omitted, a default under the input directory is used.")
    parser.add_argument("--session", default="latest", help="Raw session id, or latest. Only used with --from raw.")
    parser.add_argument("--copy-media", action="store_true", help="Copy image/depth media into RLDS directory instead of referencing raw paths.")
    parser.add_argument("--embed-binary", action="store_true", help="Embed small binary raw files such as pkl/script into HDF5.")
    return parser.parse_args()


def main() -> int:
    outputs = convert(parse_args())
    for path in outputs:
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

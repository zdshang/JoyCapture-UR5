#!/usr/bin/env python3
from __future__ import annotations

import argparse
from types import SimpleNamespace
import json
from pathlib import Path
from typing import Any

from dataset_format_converter import export_session
from postprocess_realsense_bag import export_bag, update_metadata_json


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RAW_ROOT = REPO_ROOT / "paths"


def _json_dump(payload: Any) -> str:
    return json.dumps(payload, ensure_ascii=False, indent=2)


def _safe_label(value: str, fallback: str = "camera") -> str:
    text = "".join(ch if ch.isalnum() or ch in ("_", "-") else "_" for ch in value.strip().lower())
    return text or fallback


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(_json_dump(payload) + "\n", encoding="utf-8")


def _latest_manifest(raw_root: Path) -> Path:
    manifests = sorted(
        set(raw_root.rglob("session_metadata/session_manifest_*.json")),
        key=lambda path: path.stat().st_mtime,
    )
    if not manifests:
        raise FileNotFoundError(f"No session manifests found under {raw_root}")
    return manifests[-1]


def resolve_manifest(input_path: str, session: str) -> Path:
    path = Path(input_path).expanduser()
    if path.is_file() and path.suffix.lower() == ".json":
        return path.resolve()
    raw_root = path.resolve()
    if session == "latest":
        return _latest_manifest(raw_root).resolve()
    matches = sorted(
        set(raw_root.rglob(f"session_metadata/session_manifest_{session}.json")),
        key=lambda path: path.stat().st_mtime,
    )
    if matches:
        return matches[-1].resolve()
    raise FileNotFoundError(f"Session manifest not found under {raw_root}: {session}")


def parse_outputs(value: str) -> set[str]:
    selected = {item.strip().lower() for item in value.split(",") if item.strip()}
    if not selected or "all" in selected:
        return {"video", "frames", "depth_csv", "hdf5", "rlds"}
    allowed = {"video", "frames", "depth_csv", "hdf5", "rlds", "raw"}
    unknown = selected - allowed
    if unknown:
        raise SystemExit(f"Unsupported output(s): {', '.join(sorted(unknown))}")
    return selected


def selected_camera_labels(value: str) -> set[str] | None:
    labels = {_safe_label(item, "") for item in value.split(",") if item.strip()}
    labels.discard("")
    if not labels or "all" in labels:
        return None
    return labels


def _resolve_path(value: Any, fallback: Path) -> Path:
    raw = str(value or "").strip()
    if raw:
        return Path(raw).expanduser()
    return fallback


def build_bag_args(
    *,
    raw_root: Path,
    session_id: str,
    camera: dict[str, Any],
    media: set[str],
) -> SimpleNamespace:
    label = _safe_label(str(camera.get("label", "") or camera.get("device_name", "camera")))
    metadata_path = _resolve_path(camera.get("metadata_path"), raw_root / "camera_metadata" / label / f"{label}_metadata_{session_id}.json")
    metadata = _load_json(metadata_path) if metadata_path.exists() else {}
    fps = float(camera.get("camera_fps", 0) or metadata.get("video_fps", 0) or 30)
    export_every_n = int(camera.get("export_frame_every_n", 0) or metadata.get("export_frame_every_n", 0) or 1)
    export_max_frames = int(camera.get("export_max_frames", 0) or metadata.get("export_max_frames", 0) or 0)
    return SimpleNamespace(
        manifest="",
        bag=str(_resolve_path(camera.get("bag_path"), raw_root / "camera_bag" / label / f"{label}_{session_id}.bag")),
        label=label,
        session_id=session_id,
        video_path=str(_resolve_path(camera.get("video_path"), raw_root / "camera_video" / label / f"{label}_{session_id}.mp4")),
        frame_output_dir=str(
            _resolve_path(camera.get("frame_export_dir") or metadata.get("frame_export_dir"), raw_root / "camera_frames" / label / f"{label}_{session_id}")
        ),
        depth_csv_dir=str(
            _resolve_path(camera.get("depth_csv_dir") or metadata.get("depth_csv_dir"), raw_root / "camera_depth_csv" / label / f"{label}_{session_id}")
        ),
        timestamp_csv=str(
            _resolve_path(camera.get("frame_ts_path"), raw_root / "camera_timestamps" / label / f"{label}_frames_{session_id}.csv")
        ),
        metadata_json=str(metadata_path),
        intrinsics_json=str(
            _resolve_path(camera.get("intrinsics_path"), raw_root / "camera_intrinsics" / label / f"{label}_intrinsics_{session_id}.json")
        ),
        media=",".join(sorted(media)),
        fps=fps,
        export_frame_every_n=export_every_n,
        export_max_frames=export_max_frames,
    )


def process_camera_media(raw_root: Path, session_id: str, camera: dict[str, Any], media: set[str]) -> dict[str, Any]:
    args = build_bag_args(raw_root=raw_root, session_id=session_id, camera=camera, media=media)
    summary = export_bag(args, match_mode="exact")
    if not summary["ok"] and summary["timestamp_target_count"] and summary["matched_frames"] == 0:
        summary = export_bag(args, match_mode="order")
    if not summary["ok"]:
        summary["error"] = "postprocess did not produce selected media or no frames were matched"
        return summary

    update_metadata_json(args, summary)
    camera.update(
        {
            "video_path": summary["video_path"],
            "frame_export_dir": summary["frame_export_dir"],
            "depth_csv_dir": summary["depth_csv_dir"],
            "video_codec": "ffmpeg_mpeg4_postprocess" if "video" in media else str(camera.get("video_codec", "")),
            "postprocess_done": True,
            "postprocess_required": False,
            "postprocess_media": sorted(media),
            "postprocess_summary": summary,
            "frame_count": int(summary["matched_frames"]),
        }
    )
    return summary


def run(args: argparse.Namespace) -> dict[str, Any]:
    outputs = parse_outputs(args.outputs)
    media_outputs = outputs & {"video", "frames", "depth_csv"}
    dataset_outputs = sorted(outputs & {"hdf5", "rlds"})
    manifest_path = resolve_manifest(args.input, args.session)
    raw_root = manifest_path.parents[1]
    manifest = _load_json(manifest_path)
    session_id = str(manifest.get("created_at", "") or manifest_path.stem.removeprefix("session_manifest_"))
    labels = selected_camera_labels(args.cameras)
    summaries: list[dict[str, Any]] = []

    if media_outputs:
        for camera in manifest.get("cameras", []) or []:
            if not isinstance(camera, dict):
                continue
            label = _safe_label(str(camera.get("label", "") or camera.get("device_name", "camera")))
            if labels is not None and label not in labels:
                continue
            bag_path = _resolve_path(camera.get("bag_path"), raw_root / "camera_bag" / label / f"{label}_{session_id}.bag")
            if not bag_path.exists():
                summaries.append({"ok": False, "label": label, "error": f"BAG file not found: {bag_path}"})
                continue
            print(f"[postprocess] exporting {label}: {','.join(sorted(media_outputs))}", flush=True)
            summaries.append(process_camera_media(raw_root, session_id, camera, media_outputs))
        _write_json(manifest_path, manifest)

    dataset_results: dict[str, str] = {}
    if dataset_outputs:
        print(f"[postprocess] converting dataset: {','.join(dataset_outputs)}", flush=True)
        dataset_results = export_session(
            manifest_path,
            dataset_outputs,
            output_root=raw_root,
            rlds_copy_media=bool(args.rlds_copy_media),
            hdf5_embed_binary=bool(args.hdf5_embed_binary),
        )

    return {
        "ok": all(bool(item.get("ok", False)) for item in summaries) if summaries else True,
        "manifest": str(manifest_path),
        "session_id": session_id,
        "media_outputs": sorted(media_outputs),
        "dataset_outputs": dataset_results,
        "camera_summaries": summaries,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Offline post-process UR5 Xbox raw recording sessions.")
    parser.add_argument("--input", default=str(DEFAULT_RAW_ROOT), help="Raw paths directory or session_manifest_*.json.")
    parser.add_argument("--session", default="latest", help="Session id, or latest when --input is a raw paths directory.")
    parser.add_argument(
        "--outputs",
        default="video,frames,depth_csv,hdf5,rlds",
        help="Comma-separated outputs: video,frames,depth_csv,hdf5,rlds,raw,all.",
    )
    parser.add_argument("--cameras", default="all", help="Comma-separated camera labels, or all.")
    parser.add_argument("--rlds-copy-media", action="store_true", help="Copy image/depth media into RLDS output.")
    parser.add_argument("--hdf5-embed-binary", action="store_true", help="Embed BAG/MP4 and other raw binaries in HDF5.")
    return parser.parse_args()


def main() -> int:
    summary = run(parse_args())
    print(json.dumps(summary, ensure_ascii=False, separators=(",", ":")), flush=True)
    return 0 if summary["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

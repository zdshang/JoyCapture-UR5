from __future__ import annotations

import os
import sys
from pathlib import Path


def _add_local_vendor_paths() -> None:
    root = Path(__file__).resolve().parents[1]
    py_tag = f"python{sys.version_info.major}.{sys.version_info.minor}"
    env_name = os.environ.get("UR5_ENV_NAME", "UR_xbox").strip() or "UR_xbox"
    candidates = [
        root / ".vendor" / env_name / "lib" / py_tag / "site-packages",
        root / ".vendor" / "shared" / "lib" / py_tag / "site-packages",
    ]
    for path in candidates:
        if not path.exists():
            continue
        text = str(path)
        if text not in sys.path:
            sys.path.insert(0, text)


_add_local_vendor_paths()

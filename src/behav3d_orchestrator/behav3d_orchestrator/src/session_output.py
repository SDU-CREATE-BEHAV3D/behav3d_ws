"""Resolve the existing capture-session convention for execution reports."""
import os
from pathlib import Path


def active_session_path():
    root = Path(os.environ.get("BEHAV3D_CAPTURES_ROOT") or Path.home() / "behav3d_ws/captures").expanduser()
    root.mkdir(parents=True, exist_ok=True)
    active = os.environ.get("BEHAV3D_ACTIVE_SESSION", "").strip()
    if active:
        path = Path(active).expanduser().resolve()
        if not path.is_dir():
            raise FileNotFoundError(f"Active capture session does not exist: {path}")
        return path
    dirs = [p for p in root.iterdir() if p.is_dir()]
    timestamp_dirs = [p for p in dirs if len(p.name) == 13 and p.name[6] == "_"
                      and p.name[:6].isdigit() and p.name[7:].isdigit()]
    return max(timestamp_dirs or dirs, key=lambda p: p.name).resolve() if dirs else root.resolve()


def session_child(value, session_root):
    text = str(value).strip()
    if text == "@session":
        return Path(session_root)
    if text.startswith("@session/"):
        return Path(session_root) / text[len("@session/"):]
    return Path(text).expanduser().resolve()

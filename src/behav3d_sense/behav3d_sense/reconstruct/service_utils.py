import os
from pathlib import Path


def get_captures_root():
    env_root = os.environ.get('BEHAV3D_CAPTURES_ROOT', '')
    if env_root:
        return Path(env_root).expanduser()
    return Path.home() / 'behav3d_ws' / 'captures'


def get_latest_session(captures_root: Path) -> Path:
    subdirs = [d for d in captures_root.iterdir() if d.is_dir()]
    if not subdirs:
        return captures_root
    return max(subdirs, key=lambda d: d.stat().st_mtime)


def resolve_session_path(path_arg: str, use_latest: bool, captures_root: Path) -> Path:
    p = (path_arg or '').strip()
    if use_latest and not p:
        return get_latest_session(captures_root)
    if p.startswith('@session'):
        active = os.environ.get('BEHAV3D_ACTIVE_SESSION', '')
        if active and Path(active).exists():
            session_root = Path(active)
        else:
            session_root = get_latest_session(captures_root)
        sub = p.replace('@session', '').lstrip('/')
        return session_root / sub if sub else session_root
    if not p:
        return get_latest_session(captures_root)
    if not os.path.isabs(p):
        return (captures_root / p).resolve()
    return Path(p).expanduser().resolve()

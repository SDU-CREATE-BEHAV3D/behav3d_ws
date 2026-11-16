"""Read-only completion checks for the asynchronous reconstruction handoff."""
import hashlib
import json
from pathlib import Path


def alignment_is_complete(output_path, start_ts):
    """A fresh image alone is insufficient: require this run's complete frame set."""
    directory = Path(output_path)
    try:
        data = json.loads((directory / "alignment.json").read_text())
        count = data.get("frame_count")
        if (data.get("schema") != 2 or data.get("complete") is not True
                or not isinstance(data.get("run_id"), str) or not data["run_id"]
                or not isinstance(count, int) or count <= 0
                or float(data.get("started_at", 0)) < float(start_ts)
                or float(data.get("completed_at", 0)) < float(data["started_at"])):
            return False
        expected = [f"color_in_depth_{i:04d}.png" for i in range(count)]
        files = data.get("files", [])
        if [row.get("name") for row in files] != expected:
            return False
        if {p.name for p in directory.glob("color_in_depth*.png")} != set(expected):
            return False
        manifest = directory.parent.parent / "manifest.yaml"
        if hashlib.sha256(manifest.read_bytes()).hexdigest() != data["input_signature"]["manifest"]:
            return False
        return all(hashlib.sha256((directory / row["name"]).read_bytes()).hexdigest() == row["sha256"]
                   for row in files)
    except (OSError, ValueError, TypeError, KeyError, AttributeError):
        return False

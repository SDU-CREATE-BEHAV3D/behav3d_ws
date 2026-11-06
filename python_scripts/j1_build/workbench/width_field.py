"""Input adaptation only; width mapping and sampling remain in lib_scalar."""
from pathlib import Path
import hashlib
import numpy as np

from lib_scalar.bead_profile import load_normalized_width_map, normalized_width_to_mm


def original_field_path(fixed_path):
    """Follow the provenance recorded by the extra-subdivision input preparer."""
    fixed_path = Path(fixed_path)
    with np.load(fixed_path, allow_pickle=False) as payload:
        if "source_field_state" in payload:
            return fixed_path.parent / str(payload["source_field_state"].item())
        if "extra_subdivision_levels" in payload and int(payload["extra_subdivision_levels"].item()) > 0:
            raise ValueError("Subdivided goal is missing its original field path")
    return fixed_path


class WidthField:
    def __init__(self, path, count):
        self.path = Path(path).expanduser().resolve()
        self.values = load_normalized_width_map(self.path, expected_count=count)
        self.sha256 = hashlib.sha256(self.path.read_bytes()).hexdigest()

    def mapped(self, parameters):
        return normalized_width_to_mm(self.values, width_min_mm=parameters.width_min_mm,
                                      width_max_mm=parameters.width_max_mm) * 0.001

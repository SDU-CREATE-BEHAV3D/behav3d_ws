"""Generate a cached field input with one extra Loop subdivision level."""
from pathlib import Path
import sys
import numpy as np


def prepare_field(root: Path) -> Path:
    sys.path.insert(0, str(root / "src/behav3d_py/behav3d_py/scalar_field"))
    from field_scan_loop_simulator_dds_v2 import subdivide_field_mesh_loop
    from lib_scalar.geometry import sample_vertex_scalar_on_surface
    source = root / "mesh" / "fields" / "field_state_init.npz"
    destination = source.with_name("field_state_init_subdiv1.npz")
    if destination.is_file():
        return destination
    with np.load(source, allow_pickle=False) as original:
        old_vertices = original["field_vertices_scaled"]
        old_faces = original["field_faces"]
        vertices, faces = subdivide_field_mesh_loop(old_vertices, old_faces, 1)
        heat = sample_vertex_scalar_on_surface(vertices, old_vertices, old_faces, original["heat_norm"])
        scale = float(np.asarray(original["field_scale"]).reshape(-1)[0])
        np.savez_compressed(
            destination,
            field_vertices_scaled=vertices, field_faces=faces,
            heat_norm=heat, offset_xyz=original["offset_xyz"],
            field_vertices=vertices / scale, field_scale=scale,
            seed_level=original["seed_level"],
            source_field_state=source.name, extra_subdivision_levels=1,
            subdivision_method="Loop", heat_transfer="surface barycentric interpolation",
        )
    print(f"Prepared subdivided field: {destination}", flush=True)
    return destination


if __name__ == "__main__":
    prepare_field(Path(__file__).resolve().parents[2])

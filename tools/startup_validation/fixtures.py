"""Synthetic OCSR source scenes for scientific startup integration validation.

This writes controlled software fixtures, never camera evidence or a material
certificate. The helper itself requires only Python's standard library + NumPy.
Call the real Agent007 exporter separately to validate and freeze the source.
"""
import json
from pathlib import Path

import numpy as np

CLASS_VALUES = {"CLEAR_CANDIDATE": 1, "MATERIAL_REVIEW": 2, "CAUTION_CLEARANCE": 3,
                "BLOCKED": 4, "NON_PRINTABLE_TOPOGRAPHY": 5, "REQUIRES_RESCAN": 6}
SYNTHETIC_CONTROL = {"kind": "synthetic_control", "purpose": "software integration only",
                     "physical_scan": False, "material_or_structural_validation": False,
                     "robot_commands_sent": False, "fabrication_authorized": False}


def task_example():
    """Explicit demonstration profile; dimensions/process settings are not learned."""
    return {"task_id": "wall-300-synthetic-control-v1", "width_m": .3, "height_m": .3,
            "thickness_m": .03, "bead_spacing_m": .036, "bead_height_m": .016,
            "bead_diameter_m": .03, "count": 7, "yaw_degrees": [0, 45, 90, 135],
            "search_step_m": .05, "max_pose_evaluations": 2000,
            "min_confidence": .6, "min_perception_score": .6, "clearance_m": .02,
            "footprint_margin_m": .01, "max_duplicate_layer_separation_m": .005,
            "max_support_triangle_height_span_m": .01, "base_z_offset_m": .005,
            "support_contact_tolerance_m": .005, "max_frontier_height_span_m": .016,
            "heat_seed_level_m": .005, "heat_t_coef": 2000.,
            "iso_level_m": 0., "normal_continuity_rule": True}


def _write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, sort_keys=True, indent=2, allow_nan=False)
        stream.write("\n")


def _points(path, xyz):
    with path.open("x", encoding="utf-8") as stream:
        stream.write("ply\nformat ascii 1.0\nelement vertex " + str(len(xyz))
                     + "\nproperty float x\nproperty float y\nproperty float z\nend_header\n")
        for row in xyz:
            stream.write(" ".join(format(float(value), ".12g") for value in row) + "\n")


def create_source_scene(root, frame_id="startup_test_world"):
    """Create exactly the new ``root`` directory and return its canonical Path.

    A 0.6 m square flat table has 3,600 explicit 1 cm support cells at Z=.735 m.
    An uncertain object remains represented outside the placement region. Every
    product is labelled synthetic_control, with no mechanical/material claims.
    Existing directories are refused rather than overwritten.
    """
    if not isinstance(frame_id, str) or not frame_id or frame_id != frame_id.strip():
        raise ValueError("explicit frame_id required")
    root = Path(root).resolve()
    root.mkdir(parents=True, exist_ok=False)
    cells = np.asarray([(x, y) for x in range(-30, 30) for y in range(-30, 30)], dtype=np.int32)
    xyz = np.column_stack(((cells + .5) * .01, np.full(len(cells), .735)))
    objects = []
    cloud_specs = (("work_table", "support", True, xyz),
                   ("uncertain_tool", "unknown_object", False,
                    np.asarray([[.9, .9, .75], [.91, .9, .75], [.9, .91, .76]])))
    for index, (label, role, eligible, cloud) in enumerate(cloud_specs):
        object_id = f"object_{index:04d}"
        folder = root / object_id
        folder.mkdir()
        _points(folder / "points.ply", cloud)
        np.savez_compressed(folder / "cloud_attributes.npz", confidence=np.full(len(cloud), .9))
        _write(folder / "metadata.json", {"object_id": object_id, "provisional_label": label,
            "semantic_role": role, "track_status": "confirmed" if eligible else "tentative",
            "point_count": len(cloud), "points_file": "points.ply", "cloud_attributes_file": "cloud_attributes.npz",
            "intent_confirmed": False, "synthetic_control": SYNTHETIC_CONTROL,
            "perception_files": {"semantic": "perception/semantic.json"}})
        _write(folder / "perception/semantic.json", {"object_id": object_id,
            "operational_geometry_eligible": eligible, "perception_affordances": [],
            "synthetic_control": SYNTHETIC_CONTROL})
        objects.append({"object_id": object_id, "metadata_file": f"{object_id}/metadata.json"})
    _write(root / "scene.json", {"format": "ocsr.object_scene.v3", "objects": objects, "object_count": len(objects),
        "frame_id": frame_id, "coordinate_units": "m", "synthetic_control": SYNTHETIC_CONTROL,
        "support_topography": {"report_file": "support_topography/support_topography.json", "primary_surface_id": "surface_0001"},
        "decision_context": {"grid_npz_file": "decision_context_grid.npz"},
        "surface_suitability": {"report_file": "surface_suitability.json"}})
    _write(root / "semantic_scene.json", {"objects": objects, "relations": [], "frame_id": frame_id,
        "coordinate_units": "m", "synthetic_control": SYNTHETIC_CONTROL})
    _write(root / "decision_context.json", {"grid_npz_file": "decision_context_grid.npz",
        "synthetic_control": SYNTHETIC_CONTROL})
    np.savez_compressed(root / "decision_context_grid.npz", points=xyz, surface_cells=cells,
        review_class=np.ones(len(cells), dtype=np.uint8), perception_score=np.full(len(cells), .9),
        clearance_m=np.full(len(cells), np.inf), suitability_region_index=np.zeros(len(cells), dtype=np.int32),
        owner_index=np.full(len(cells), -1, dtype=np.int32))
    _write(root / "surface_suitability.json", {"format": "ocsr.surface_suitability.v1",
        "grid_file": "decision_context_grid.npz", "primary_surface_id": "surface_0001", "cell_size_m": .01,
        "class_values": CLASS_VALUES, "synthetic_control": SYNTHETIC_CONTROL,
        "regions": [{"region_id": "region_0001", "region_index": 0, "eligible": True, "cell_count": len(cells)}]})
    _write(root / "support_topography/support_topography.json", {"format": "ocsr.support_topography.v1",
        "config": {"cell_size_m": .01, "vertical_axis": 2}, "primary_surface_id": "surface_0001",
        "synthetic_control": SYNTHETIC_CONTROL, "surfaces": [{"surface_id": "surface_0001",
            "metadata_file": "surface_0001/metadata.json", "data_file": "surface_0001/topography_grid.npz",
            "selection_rank": 1, "semantic_class": "PRIMARY_PRINT_SURFACE"}]})
    support = root / "support_topography/surface_0001"
    _write(support / "metadata.json", {"surface_id": "surface_0001", "material_label": "synthetic_unknown_substrate",
        "cell_size_m": .01, "synthetic_control": SYNTHETIC_CONTROL})
    np.savez_compressed(support / "topography_grid.npz", cells=cells, points=xyz, confidence=np.full(len(cells), .9))
    _write(root / "surface_ownership.json", {"synthetic_control": SYNTHETIC_CONTROL, "surfaces": [
        {"surface_id": "surface_0001", "owner": {"object_id": "object_0000", "label": "work_table", "semantic_role": "support"},
         "print_candidate_allowed": True}]})
    return root

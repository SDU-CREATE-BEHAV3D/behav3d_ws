import numpy as np

from behav3d_py.scalar_field.lib_scalar.print_targets import OrientedLineTargets
from behav3d_py.scalar_field.lib_scalar.target_rules import (
    apply_secondary_target_rules,
)


def test_variable_width_endpoint_rule_returns_aligned_indices():
    points = np.array(
        [
            [0.000, 0.0, 0.0],
            [0.019, 0.0, 0.0],
            [0.050, 0.0, 0.0],
        ],
        dtype=np.float64,
    )
    z_dirs = np.tile([0.0, 0.0, 1.0], (3, 1))
    targets = OrientedLineTargets(points, points.copy(), z_dirs, z_dirs.copy())

    filtered, stats = apply_secondary_target_rules(
        targets,
        candidate_mode="z_lift",
        bead_height_m=0.012,
        bead_separation_m=0.016,
        normal_continuity_rule=True,
        bead_widths_m=np.array([0.020, 0.030, 0.020]),
        bead_overlap_m=0.004,
    )

    assert filtered.count == 2
    assert stats["endpoint_spacing_removed"] == 1
    assert stats["kept_indices"].tolist() == [0, 2]

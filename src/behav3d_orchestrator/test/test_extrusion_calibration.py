import math

import pytest

from behav3d_orchestrator.src.extrusion_calibration import (
    MAX_PRINT_STEPS,
    compensated_forward_steps,
    segment_motion_from_steps,
    volume_mm3_to_steps,
)


def test_reference_piston_volume_maps_to_reference_steps():
    reference_volume_mm3 = math.pi * 50.0**2 * 10.0
    assert volume_mm3_to_steps(reference_volume_mm3) == 64_000


def test_forward_steps_include_retract_compensation():
    assert compensated_forward_steps(8_500, retract_steps=4_000) == 12_500


def test_forward_steps_do_not_compensate_disabled_retract():
    assert (
        compensated_forward_steps(
            8_500,
            retract_steps=4_000,
            retract_enabled=False,
        )
        == 8_500
    )


def test_forward_steps_reject_uint32_overflow():
    with pytest.raises(ValueError, match="exceeds uint32"):
        compensated_forward_steps(MAX_PRINT_STEPS, retract_steps=1)


def test_segment_motion_speed_delivers_requested_steps():
    forward_steps = compensated_forward_steps(
        9_850,
        retract_steps=4_900,
    )
    duration_s, speed_mm_s = segment_motion_from_steps(
        13.0,
        forward_steps=forward_steps,
        steps_per_second=2950,
    )

    assert forward_steps == 14_750
    assert duration_s == pytest.approx(5.0)
    assert speed_mm_s == pytest.approx(2.6)

"""Conversion between decoded policy actions and DDS deposition primitives."""

from __future__ import annotations

from typing import TYPE_CHECKING

from .action_decoder import DecodedAction

if TYPE_CHECKING:
    from dds import PointDeposit


def point_deposit_from_action(action: DecodedAction) -> PointDeposit:
    """Build the DDS point deposit represented by one decoded policy action.

    Constructing the value does not mutate a DDS simulation. The environment
    must validate the action before applying this deposit to episode state.
    """

    from dds import BeadProfile, DepositionTarget, PointDeposit

    if not isinstance(action, DecodedAction):
        raise TypeError("action must be a DecodedAction")
    return PointDeposit(
        target=DepositionTarget(
            position=action.target_point,
            normal=action.z_axis,
        ),
        profile=BeadProfile(
            width=action.width_m,
            height=action.height_m,
        ),
    )

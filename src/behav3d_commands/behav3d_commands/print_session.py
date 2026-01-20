#!/usr/bin/env python3
from __future__ import annotations

from typing import Any, Dict, Optional

from .session import Session


class PrintSession(Session):
    """
    Session extension for paired motion + print workflows.
    TODO: add sync modes (goal-accept, trajectory fraction) and resource locks.
    """

    def goto_with_print_time(
        self,
        *,
        secs: float,
        speed: int,
        offset_s: float = 0.0,
        use_previous_speed: bool = False,
        on_move_done=None,
        on_print_done=None,
        **motion_kwargs: Any,
    ) -> None:
        motion_item = self._build_goto_item(on_done=on_move_done, **motion_kwargs)
        print_item = self._build_print_time_item(
            secs=secs,
            speed=speed,
            offset_s=offset_s,
            use_previous_speed=use_previous_speed,
            on_done=on_print_done,
        )
        self.run_simultaneously([motion_item, print_item])

    def goto_with_print_steps(
        self,
        *,
        steps: int,
        speed: int,
        offset_s: float = 0.0,
        use_previous_speed: bool = False,
        on_move_done=None,
        on_print_done=None,
        **motion_kwargs: Any,
    ) -> None:
        motion_item = self._build_goto_item(on_done=on_move_done, **motion_kwargs)
        print_item = self._build_print_steps_item(
            steps=steps,
            speed=speed,
            offset_s=offset_s,
            use_previous_speed=use_previous_speed,
            on_done=on_print_done,
        )
        self.run_simultaneously([motion_item, print_item])

    def _build_goto_item(self, *, on_done=None, **motion_kwargs: Any):
        payload = {
            "pose": motion_kwargs.get("pose"),
            "x": motion_kwargs.get("x"),
            "y": motion_kwargs.get("y"),
            "z": motion_kwargs.get("z"),
            "rx": motion_kwargs.get("rx"),
            "ry": motion_kwargs.get("ry"),
            "rz": motion_kwargs.get("rz"),
            "eef": motion_kwargs.get("eef"),
            "vel_scale": motion_kwargs.get("vel_scale"),
            "accel_scale": motion_kwargs.get("accel_scale"),
            "exec": bool(motion_kwargs.get("exec", True)),
            "motion": motion_kwargs.get("motion"),
        }
        return self.build_item("goto", payload, cmd_kind="goto", on_done=on_done)

    def _build_print_time_item(
        self,
        *,
        secs: float,
        speed: int,
        offset_s: float,
        use_previous_speed: bool,
        on_done=None,
    ):
        payload = {
            "secs": float(secs),
            "speed": int(speed),
            "use_prev": bool(use_previous_speed),
            "offset_s": float(offset_s),
        }
        kind = "print_time_delayed" if offset_s > 0.0 else "print_time"
        return self.build_item(kind, payload, cmd_kind="print", on_done=on_done)

    def _build_print_steps_item(
        self,
        *,
        steps: int,
        speed: int,
        offset_s: float,
        use_previous_speed: bool,
        on_done=None,
    ):
        payload = {
            "steps": int(steps),
            "speed": int(speed),
            "use_prev": bool(use_previous_speed),
            "offset_s": float(offset_s),
        }
        kind = "print_steps_delayed" if offset_s > 0.0 else "print_steps"
        return self.build_item(kind, payload, cmd_kind="print_steps", on_done=on_done)

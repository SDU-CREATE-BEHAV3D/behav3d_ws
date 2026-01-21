#!/usr/bin/env python3
from __future__ import annotations

from typing import Any

from behav3d_commands.session import Session


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
        motion_item = self.motion.goto(on_done=on_move_done, enqueue=False, **motion_kwargs)
        print_item = self.extruder.print_time(
            secs=secs,
            speed=speed,
            offset_s=offset_s,
            use_previous_speed=use_previous_speed,
            on_done=on_print_done,
            enqueue=False,
        )
        self.run_group([motion_item, print_item])

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
        motion_item = self.motion.goto(on_done=on_move_done, enqueue=False, **motion_kwargs)
        print_item = self.extruder.print_steps(
            steps=steps,
            speed=speed,
            offset_s=offset_s,
            use_previous_speed=use_previous_speed,
            on_done=on_print_done,
            enqueue=False,
        )
        self.run_group([motion_item, print_item])

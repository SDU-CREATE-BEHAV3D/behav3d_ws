#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional


OnCommandDone = Optional[Callable[[Dict[str, Any]], None]]


@dataclass
class Command:
    """
    A minimal command wrapper that standardizes the result contract and
    guarantees FIFO release via queue_finish_callback (even if callback raises).
    """
    kind: str
    on_done: OnCommandDone
    queue_finish_callback: Callable[[], None]

    def finish_flag(
        self,
        *,
        ok: bool,
        phase: str,
        error: Optional[str] = None,
        metrics: Optional[Dict[str, Any]] = None,
        planned_only: bool = False,
        extra: Optional[Dict[str, Any]] = None,
    ) -> None:
        res = {
            "ok": ok,
            "kind": self.kind,
            "phase": phase,
            "planned_only": planned_only,
            "error": error,
            "metrics": metrics or {},
        }
        if extra:
            res.update(extra)

        try:
            if self.on_done:
                self.on_done(res)
        finally:
            self.queue_finish_callback()

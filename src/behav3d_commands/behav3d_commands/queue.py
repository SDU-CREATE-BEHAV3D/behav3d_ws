#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Deque, Dict, Optional, Tuple
from collections import deque


QueueExecutor = Callable[[str, Dict[str, Any]], None]


@dataclass
class QueueItem:
    kind: str
    payload: Dict[str, Any]


class FifoQueue:
    """
    Minimal FIFO executor:
    - enqueue(kind, payload)
    - pause/resume
    - runs exactly one item at a time
    - the executor must call queue.finish_item(...) when done
    """

    def __init__(self, *, executor: QueueExecutor):
        self._executor = executor
        self._q: Deque[QueueItem] = deque()
        self._busy: bool = False
        self._paused: bool = False

    def enqueue(self, kind: str, payload: Dict[str, Any]) -> None:
        self._q.append(QueueItem(kind=kind, payload=payload))
        if not self._busy and not self._paused:
            self._process_next()

    def pause(self) -> None:
        self._paused = True

    def resume(self) -> None:
        if not self._paused:
            return
        self._paused = False
        if not self._busy:
            self._process_next()

    @property
    def is_busy(self) -> bool:
        return self._busy

    @property
    def is_paused(self) -> bool:
        return self._paused

    def _process_next(self) -> None:
        if self._paused:
            self._busy = False
            return
        if not self._q:
            self._busy = False
            return

        self._busy = True
        item = self._q.popleft()
        self._executor(item.kind, item.payload)

    def finish_item(self) -> None:
        """Call this exactly once when the current item is completed."""
        self._busy = False
        self._process_next()

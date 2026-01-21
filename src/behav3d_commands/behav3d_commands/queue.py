#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass
from collections import deque
from typing import Any, Callable, Deque, Dict, Iterable, List, Optional, Union

from .command import OnCommandDone


@dataclass(frozen=True)
class QueueItem:
    kind: str
    payload: Dict[str, Any]
    cmd_kind: Optional[str] = None
    on_done: OnCommandDone = None


@dataclass(frozen=True)
class QueueGroup:
    items: List[QueueItem]


QueueEntry = Union[QueueItem, QueueGroup]
QueueExecutor = Callable[[QueueItem, Callable[[], None]], None]


class SessionQueue:
    """
    FIFO queue with optional parallel groups.
    - enqueue(...) adds a single item to the tail.
    - prepend(...) inserts a single item at the head.
    - enqueue_group([...]) schedules a set of items to run simultaneously.
    TODO: add resource locks and cancellation for safe parallelism.
    """

    def __init__(self, *, executor: QueueExecutor):
        self._executor = executor
        self._q: Deque[QueueEntry] = deque()
        self._busy: bool = False
        self._paused: bool = False
        self._group_remaining: int = 0

    def enqueue(self, item: QueueItem) -> None:
        self._q.append(item)
        if not self._busy and not self._paused:
            self._process_next()

    def prepend(self, item: QueueItem) -> None:
        self._q.appendleft(item)
        if not self._busy and not self._paused:
            self._process_next()

    def enqueue_group(self, items: Iterable[QueueItem]) -> None:
        group = QueueGroup(items=list(items))
        self._q.append(group)
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

        entry = self._q.popleft()
        if isinstance(entry, QueueGroup):
            self._start_group(entry)
        else:
            self._start_single(entry)

    def _start_single(self, item: QueueItem) -> None:
        self._busy = True
        self._executor(item, self._finish_single)

    def _finish_single(self) -> None:
        self._busy = False
        self._process_next()

    def _start_group(self, group: QueueGroup) -> None:
        if not group.items:
            self._busy = False
            self._process_next()
            return
        self._busy = True
        self._group_remaining = len(group.items)
        for item in group.items:
            self._executor(item, self._finish_group_item)

    def _finish_group_item(self) -> None:
        if self._group_remaining <= 0:
            return
        self._group_remaining -= 1
        if self._group_remaining == 0:
            self._busy = False
            self._process_next()

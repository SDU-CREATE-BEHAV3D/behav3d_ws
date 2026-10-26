"""Opt-in observation commands through the existing shared pause and FIFO."""
from __future__ import annotations

from behav3d_commands.session import Session
from behav3d_commands.sense_commands.observation_commands import ObservationCommands

from .control_session import ControlAwareSession


class ObservationSession(ControlAwareSession):
    """Register only evidence evaluation, never physical command handlers."""

    def __init__(self, node):
        super().__init__(node, register_default_commands=False, enabled_command_sets=())
        self.observation = ObservationCommands(node, queue=self.queue)
        self.observation.register(self._router)

    def evaluate(self, request, *, timeout_s, cancel_event):
        item = self.observation.evaluate_observation(
            request, timeout_s=timeout_s, cancel_event=cancel_event, enqueue=False)
        if (not self.control_pause.wait('evaluate_observation', cancel_event=cancel_event)
                or cancel_event.is_set()):
            return dict(ok=False, kind='evaluate_observation', phase='exec',
                        planned_only=True, error='Review cancelled before dispatch',
                        metrics={'status': 'CANCELLED', 'request_id': request['request_id'],
                                 'requires_review': True, 'fabrication_authorized': False})
        return Session.run_sync(self, item, timeout_s=timeout_s + 5.0)

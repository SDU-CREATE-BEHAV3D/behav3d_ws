"""Startup preview uses the shared cooperative pause and no physical clients."""

from behav3d_commands.sense_commands.initial_wall_commands import InitialWallCommands
from behav3d_commands.session import Session

from .control_session import ControlAwareSession


class InitialWallSession(ControlAwareSession):
    def __init__(self, node):
        super().__init__(node, register_default_commands=False, enabled_command_sets=())
        self.initial = InitialWallCommands(node, queue=self.queue)
        self.initial.register(self._router)

    def evaluate(self, request, *, timeout_s, cancel_event):
        item = self.initial.initial_wall(
            request, timeout_s=timeout_s, cancel_event=cancel_event, enqueue=False
        )
        if (
            not self.control_pause.wait("initial_wall", cancel_event=cancel_event)
            or cancel_event.is_set()
        ):
            return {
                "ok": False,
                "kind": "initial_wall",
                "phase": "plan",
                "planned_only": True,
                "error": "Startup cancelled before dispatch",
                "metrics": {
                    "status": "CANCELLED",
                    "requires_review": True,
                    "fabrication_authorized": False,
                },
            }
        return Session.run_sync(self, item, timeout_s=timeout_s + 5)

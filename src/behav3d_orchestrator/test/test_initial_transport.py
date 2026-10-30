"""ROS-shaped startup transport tests; no ROS server or robot is started."""

import importlib.util
import sys
import tempfile
import threading
import types
import unittest
from concurrent.futures import Future
from pathlib import Path
from typing import ClassVar
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[3]
PACKAGE = ROOT / "src/behav3d_orchestrator/behav3d_orchestrator"
COMMANDS = ROOT / "src/behav3d_commands/behav3d_commands"
sys.path.insert(0, str(PACKAGE.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_initial_workflow import request, success


def module(name, root=None):
    value = types.ModuleType(name)
    value.__path__ = [] if root is None else [str(root)]
    return value


class Timer:
    def __init__(self, callback):
        self.callback, self.cancelled = callback, False

    def cancel(self):
        self.cancelled = True


class Client:
    def __init__(self):
        self.ready, self.goals, self.future = True, [], Future()

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal):
        self.goals.append(goal)
        return self.future


class Node:
    parameters: ClassVar[dict] = {}

    def __init__(self, name="fake"):
        self.name, self.timers, self.services = name, [], []
        self.client = Client()

    def declare_parameter(self, name, default):
        return types.SimpleNamespace(value=self.parameters.get(name, default))

    def create_timer(self, period, callback, **kwargs):
        timer = Timer(callback)
        self.timers.append(timer)
        return timer

    def destroy_timer(self, timer):
        timer.cancel()

    def create_service(self, *args, **kwargs):
        self.services.append(args)
        return object()

    def create_publisher(self, *args, **kwargs):
        return types.SimpleNamespace(publish=lambda message: None)

    def destroy_node(self):
        return True


class Handle:
    def __init__(self, goal=None):
        self.request, self.accepted = goal, True
        self.is_cancel_requested, self.cancel_count, self.terminals = False, 0, []
        self.result = Future()

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.cancel_count += 1
        return Future()

    def publish_feedback(self, _message):
        pass

    def succeed(self):
        self.terminals.append("SUCCEEDED")

    def abort(self):
        self.terminals.append("ABORTED")

    def canceled(self):
        if not self.is_cancel_requested:
            raise AssertionError("Invalid ROS cancel transition")
        self.terminals.append("CANCELLED")


def load():
    values = {}
    for name in (
        "rclpy",
        "rclpy.node",
        "rclpy.action",
        "rclpy.clock",
        "rclpy.callback_groups",
        "rclpy.executors",
        "rclpy.impl",
        "rclpy.impl.implementation_singleton",
        "rclpy.qos",
        "action_msgs",
        "action_msgs.msg",
        "std_msgs",
        "std_msgs.msg",
        "behav3d_interfaces",
        "behav3d_interfaces.action",
        "behav3d_interfaces.srv",
    ):
        values[name] = module(name)
    values["behav3d_commands"] = module("behav3d_commands", COMMANDS)
    values["behav3d_orchestrator"] = module("behav3d_orchestrator", PACKAGE)
    values["behav3d_orchestrator.src"] = module(
        "behav3d_orchestrator.src", PACKAGE / "src"
    )
    fake_session = module("behav3d_orchestrator.src.initial_session")

    class Session:
        def __init__(self, _node):
            self.calls = []

        def evaluate(self, value, **kwargs):
            self.calls.append(value)
            return {"ok": True, "metrics": success(value)}

    fake_session.InitialWallSession = Session
    values[fake_session.__name__] = fake_session
    values["rclpy.node"].Node = Node
    values["rclpy.clock"].Clock = lambda **kwargs: kwargs
    values["rclpy.clock"].ClockType = types.SimpleNamespace(STEADY_TIME=1)
    values["rclpy.action"].ActionClient = lambda node, *_args: node.client
    values["rclpy.action"].ActionServer = lambda *_args, **kwargs: (
        types.SimpleNamespace(destroy=lambda: None)
    )
    values["rclpy.action"].GoalResponse = types.SimpleNamespace(
        ACCEPT="ACCEPT", REJECT="REJECT"
    )
    values["rclpy.action"].CancelResponse = types.SimpleNamespace(
        ACCEPT="ACCEPT", REJECT="REJECT"
    )
    values["rclpy.callback_groups"].ReentrantCallbackGroup = object
    values["rclpy.executors"].ExternalShutdownException = type(
        "ExternalShutdownException", (Exception,), {}
    )
    values["rclpy.executors"].MultiThreadedExecutor = object
    values[
        "rclpy.impl.implementation_singleton"
    ].rclpy_implementation = types.SimpleNamespace(
        RCLError=type("RCLError", (Exception,), {})
    )
    values["rclpy.qos"].QoSProfile = lambda **kwargs: kwargs
    values["rclpy.qos"].QoSDurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL=1)
    values["rclpy.qos"].QoSReliabilityPolicy = types.SimpleNamespace(RELIABLE=1)
    values["action_msgs.msg"].GoalStatus = types.SimpleNamespace(STATUS_SUCCEEDED=4)
    values["std_msgs.msg"].String = lambda **kwargs: types.SimpleNamespace(**kwargs)
    values["behav3d_interfaces.action"].InitialWall = types.SimpleNamespace(
        Goal=type("Goal", (), {}),
        Result=type("Result", (), {}),
        Feedback=type("Feedback", (), {}),
    )
    values["behav3d_interfaces.srv"].ApproveSupervisedStep = object
    values["behav3d_interfaces.srv"].SetSupervisorArmed = object
    with patch.dict(sys.modules, values):
        result = []
        for name, path in (
            ("behav3d_commands.command", COMMANDS / "command.py"),
            ("behav3d_commands.queue", COMMANDS / "queue.py"),
            (
                "isolated_initial_commands",
                COMMANDS / "sense_commands/initial_wall_commands.py",
            ),
            (
                "behav3d_orchestrator.initial_startup_node",
                PACKAGE / "initial_startup_node.py",
            ),
        ):
            spec = importlib.util.spec_from_file_location(name, path)
            imported = importlib.util.module_from_spec(spec)
            sys.modules[name] = imported
            spec.loader.exec_module(imported)
            result.append(imported)
        return result[0].Command, result[2], result[3]


class InitialTransportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.Command, cls.commands, cls.nodes = load()

    def setUp(self):
        self.node, self.results, self.finished = Node(), [], []
        self.api = self.commands.InitialWallCommands(self.node)
        self.clock = patch.object(self.commands.time, "monotonic", return_value=100)
        self.now = self.clock.start()
        self.addCleanup(self.clock.stop)

    def start(self, event=None):
        item = self.api.initial_wall(
            request(), enqueue=False, cancel_event=event, timeout_s=2
        )
        command = self.Command(
            item.kind, self.results.append, lambda: self.finished.append(True)
        )
        self.api._handle(item.payload, command)

    def test_builder_registers_queue_route_and_no_physical_fields(self):
        routes = []
        self.api.register(
            types.SimpleNamespace(register=lambda *args: routes.append(args))
        )
        self.assertEqual(routes[0][0], "initial_wall")
        self.start(threading.Event())
        self.assertEqual(vars(self.node.client.goals[0]), request())
        self.assertNotIn("cancel_event", vars(self.node.client.goals[0]))

    def test_async_result_finishes_once_and_requires_review(self):
        self.start()
        handle = Handle()
        self.node.client.future.set_result(handle)
        handle.result.set_result(
            types.SimpleNamespace(
                status=4, result=types.SimpleNamespace(**success(request()))
            )
        )
        self.node.timers[0].callback()
        self.assertEqual(self.finished, [True])
        self.assertTrue(self.results[0]["ok"])
        self.assertFalse(self.results[0]["metrics"]["fabrication_authorized"])

    def test_cancel_before_dispatch_and_deadline_finish(self):
        event = threading.Event()
        event.set()
        self.start(event)
        self.assertEqual(self.node.client.goals, [])
        self.assertEqual(self.results[0]["metrics"]["status"], "CANCELLED")

    def test_server_unavailable_is_bounded(self):
        self.node.client.ready = False
        self.start()
        self.now.return_value = 103
        self.node.timers[0].callback()
        self.assertEqual(self.results[0]["metrics"]["status"], "TIMED_OUT")
        self.assertEqual(self.finished, [True])

    def test_late_accepted_goal_cancelled_after_timeout(self):
        self.start()
        self.now.return_value = 103
        self.node.timers[0].callback()
        handle = Handle()
        self.node.client.future.set_result(handle)
        self.assertEqual(handle.cancel_count, 1)
        self.assertEqual(len(self.results), 1)

    def test_result_cannot_authorize_fabrication(self):
        self.start()
        handle = Handle()
        self.node.client.future.set_result(handle)
        bad = success(request())
        bad["fabrication_authorized"] = True
        handle.result.set_result(
            types.SimpleNamespace(status=4, result=types.SimpleNamespace(**bad))
        )
        self.assertFalse(self.results[0]["ok"])

    def test_coordinator_disarmed_and_exact_approval_gate(self):
        with tempfile.TemporaryDirectory() as temporary:
            Node.parameters = {
                "journal_path": str(Path(temporary).resolve() / "journal.jsonl")
            }
            coordinator = self.nodes.InitialStartupCoordinator()
            try:
                goal = types.SimpleNamespace(**request())
                self.assertEqual(coordinator._goal(goal), "REJECT")
                coordinator.supervisor.set_armed(True, "offline-test-operator")
                self.assertEqual(coordinator._goal(goal), "ACCEPT")
                result = coordinator._execute(Handle(goal))
                self.assertTrue(result.success)
                self.assertEqual(
                    coordinator.supervisor.snapshot()["state"],
                    "AWAITING_PLACEMENT_REVIEW",
                )
                generate = types.SimpleNamespace(**request("GENERATE"))
                self.assertEqual(coordinator._goal(generate), "REJECT")
                coordinator.supervisor.approve(
                    "startup-a", result.proposal_sha256, "offline-test-operator"
                )
                self.assertEqual(coordinator._goal(generate), "ACCEPT")
                self.assertTrue(coordinator._execute(Handle(generate)).success)
            finally:
                coordinator.destroy_node()
                Node.parameters = {}

    def test_backend_no_configuration_and_no_journal_generation_reject(self):
        Node.parameters = {}
        backend = self.nodes.InitialWallBackend()
        try:
            self.assertEqual(
                backend._goal(types.SimpleNamespace(**request())), "REJECT"
            )
        finally:
            backend.destroy_node()

    def test_session_explicitly_disables_physical_command_sets_and_uses_shared_pause(
        self,
    ):
        source = (PACKAGE / "src/initial_session.py").read_text()
        self.assertIn("enabled_command_sets=()", source)
        self.assertIn("self.control_pause.wait", source)
        self.assertIn("Session.run_sync", source)

    def test_main_only_suppresses_shutdown_context_waitset_race(self):
        for active in (False, True):
            with self.subTest(context_active=active):
                node, executor = Mock(), Mock()
                executor.spin.side_effect = self.nodes._rclpy.RCLError(
                    "invalid context"
                )
                with (
                    patch.object(self.nodes.rclpy, "init", create=True),
                    patch.object(self.nodes.rclpy, "try_shutdown", create=True),
                    patch.object(
                        self.nodes.rclpy, "ok", return_value=active, create=True
                    ),
                    patch.object(
                        self.nodes, "MultiThreadedExecutor", return_value=executor
                    ),
                ):
                    if active:
                        with self.assertRaises(self.nodes._rclpy.RCLError):
                            self.nodes._main(lambda node=node: node)
                    else:
                        self.nodes._main(lambda node=node: node)
                node.begin_shutdown.assert_called_once_with()
                executor.shutdown.assert_called_once_with(timeout_sec=10)
                node.destroy_node.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()

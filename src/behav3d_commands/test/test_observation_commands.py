"""Exercise observation transport without importing ROS or starting a server."""
from __future__ import annotations

import importlib.util
import sys
import threading
import types
import unittest
from concurrent.futures import Future
from pathlib import Path
from unittest.mock import patch


def load_isolated():
    root = Path(__file__).resolve().parents[1] / "behav3d_commands"
    modules = {}
    for name in (
        "rclpy", "rclpy.node", "rclpy.action", "rclpy.clock", "action_msgs",
        "action_msgs.msg", "behav3d_interfaces", "behav3d_interfaces.action",
        "behav3d_commands",
    ):
        modules[name] = types.ModuleType(name)
        modules[name].__path__ = []
    modules["rclpy.node"].Node = object
    modules["rclpy.clock"].Clock = lambda **kwargs: kwargs
    modules["rclpy.clock"].ClockType = types.SimpleNamespace(STEADY_TIME=1)
    modules["rclpy.action"].ActionClient = lambda node, action, name: node.client
    modules["action_msgs.msg"].GoalStatus = types.SimpleNamespace(STATUS_SUCCEEDED=4)
    modules["behav3d_interfaces.action"].EvaluateObservation = types.SimpleNamespace(
        Goal=type("Goal", (), {}),
    )
    with patch.dict(sys.modules, modules):
        loaded = {}
        for name, path in (
            ("behav3d_commands.command", root / "command.py"),
            ("behav3d_commands.queue", root / "queue.py"),
            ("isolated_observation_commands", root / "sense_commands/observation_commands.py"),
        ):
            spec = importlib.util.spec_from_file_location(name, path)
            module = importlib.util.module_from_spec(spec)
            sys.modules[name] = module
            spec.loader.exec_module(module)
            loaded[name] = module
        command = loaded["behav3d_commands.command"].Command
        observation = loaded["isolated_observation_commands"]
    return observation, command


OBS, Command = load_isolated()


def request():
    return {
        "request_id": "request-1", "artifact_id": "wall", "observation_id": "scan-1",
        "event_type": "inspection", "candidate_cycle": 0, "scan_cycle": 1,
        "previous_evaluation_id": "", "frame_id": "ur20_base_link",
        "coordinate_units": "m", "source_manifest_path": "/offline/manifest.json",
        "source_manifest_sha256": "a" * 64,
    }


def result(**overrides):
    values = {
        "success": True, "status": "REVIEW_REQUIRED", "message": "review required",
        "request_id": "request-1", "evaluation_id": "evaluation-1",
        "packet_path": "/offline/packet.json", "packet_sha256": "b" * 64,
        "evidence_manifest_path": "/offline/evidence.json",
        "evidence_manifest_sha256": "c" * 64,
        "requires_review": True, "fabrication_authorized": False,
    }
    values.update(overrides)
    return types.SimpleNamespace(**values)


class Timer:
    def __init__(self, callback):
        self.callback = callback
        self.cancelled = False

    def cancel(self):
        self.cancelled = True

    def fire(self):
        if not self.cancelled:
            self.callback()


class Client:
    def __init__(self):
        self.ready = True
        self.goal_future = Future()
        self.goals = []
        self.send_error = None

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal):
        self.goals.append(goal)
        if self.send_error:
            raise self.send_error
        return self.goal_future


class Node:
    def __init__(self):
        self.client = Client()
        self.timers = []
        self.destroyed = []
        self.timer_error = None
        self.fire_timer_on_create = False

    def create_timer(self, period, callback, **kwargs):
        if self.timer_error:
            raise self.timer_error
        assert kwargs["clock"] == {"clock_type": 1}
        timer = Timer(callback)
        self.timers.append(timer)
        if self.fire_timer_on_create:
            timer.fire()
        return timer

    def destroy_timer(self, timer):
        self.destroyed.append(timer)

    def get_logger(self):
        return types.SimpleNamespace(warning=lambda message: None)


class Handle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.result_future = Future()
        self.cancel_count = 0
        self.get_result_error = None
        self.cancel_error = None

    def get_result_async(self):
        if self.get_result_error:
            raise self.get_result_error
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_count += 1
        if self.cancel_error:
            raise self.cancel_error
        return Future()


class ObservationCommandsTest(unittest.TestCase):
    def setUp(self):
        self.node = Node()
        self.api = OBS.ObservationCommands(self.node)
        self.results = []
        self.finished = []
        self.clock = patch.object(OBS.time, "monotonic", return_value=100.0)
        self.now = self.clock.start()
        self.addCleanup(self.clock.stop)

    def start(self, **kwargs):
        item = self.api.evaluate_observation(request(), enqueue=False, **kwargs)
        command = Command(item.kind, self.results.append, lambda: self.finished.append(True))
        self.api._handle_evaluate_observation(item.payload, command)
        return item

    def accept(self, handle=None):
        handle = handle or Handle()
        self.node.client.goal_future.set_result(handle)
        return handle

    def complete(self, handle, response=None, status=4):
        handle.result_future.set_result(types.SimpleNamespace(status=status, result=response or result()))

    def assert_single(self, ok, status):
        self.assertEqual(len(self.results), 1)
        self.assertEqual(self.finished, [True])
        self.assertEqual(self.results[0]["ok"], ok)
        self.assertTrue(self.results[0]["planned_only"])
        self.assertEqual(self.results[0]["phase"], "plan")
        self.assertEqual(self.results[0]["metrics"]["command_status"], status)
        self.assertEqual(len(self.node.destroyed), len(self.node.timers))

    def test_register_and_builder_match_queue_pattern(self):
        calls = []
        self.api.register(types.SimpleNamespace(register=lambda *args: calls.append(args)))
        self.assertEqual(calls[0][0], "evaluate_observation")
        self.assertEqual(calls[0][1], self.api._handle_evaluate_observation)
        queued = []
        api = OBS.ObservationCommands(self.node, queue=types.SimpleNamespace(enqueue=queued.append))
        original = request()
        event = threading.Event()
        self.assertIsNone(api.evaluate_observation(original, cancel_event=event))
        original["request_id"] = "changed"
        self.assertEqual(queued[0].payload["request"]["request_id"], "request-1")
        self.assertIs(queued[0].payload["cancel_event"], event)
        with self.assertRaises(RuntimeError):
            self.api.evaluate_observation(request())

    def test_invalid_builder_input_never_sends(self):
        for timeout in (0, -1, float("nan"), float("inf"), True):
            with self.subTest(timeout=timeout), self.assertRaises(ValueError):
                self.api.evaluate_observation(request(), timeout_s=timeout, enqueue=False)
        for changed in (
            {**request(), "candidate_cycle": True}, {**request(), "scan_cycle": 2 ** 63},
            {**request(), "request_id": ""}, {**request(), "unknown": "x"},
        ):
            with self.assertRaises(ValueError):
                self.api.evaluate_observation(changed, enqueue=False)
        with self.assertRaises(ValueError):
            self.api.evaluate_observation(request(), cancel_event=True, enqueue=False)
        self.assertEqual(self.node.client.goals, [])

    def test_success_preserves_fields_and_never_sends_cancel_event(self):
        self.start(cancel_event=threading.Event())
        self.assertEqual(vars(self.node.client.goals[0]), request())
        handle = self.accept()
        self.complete(handle)
        self.assert_single(True, "REVIEW_REQUIRED")
        for name, value in vars(result()).items():
            self.assertEqual(self.results[0]["metrics"][name], value)
        self.assertEqual(handle.cancel_count, 0)

    def test_goal_rejection(self):
        self.start()
        self.accept(Handle(accepted=False))
        self.assert_single(False, "REJECTED")

    def test_waits_asynchronously_for_server(self):
        self.node.client.ready = False
        self.start()
        self.assertFalse(self.node.client.goals)
        self.node.client.ready = True
        self.node.timers[0].fire()
        self.assertEqual(len(self.node.client.goals), 1)
        self.node.timers[0].fire()
        self.assertEqual(len(self.node.client.goals), 1)
        self.complete(self.accept())
        self.assert_single(True, "REVIEW_REQUIRED")

    def test_unavailable_server_has_finite_deadline(self):
        self.node.client.ready = False
        self.start(timeout_s=2)
        self.now.return_value = 102
        self.node.timers[0].fire()
        self.assert_single(False, "TIMED_OUT")
        self.assertEqual(self.node.client.goals, [])

    def test_timeout_before_goal_acceptance_cancels_late_goal(self):
        self.start(timeout_s=1)
        self.now.return_value = 101
        self.node.timers[0].fire()
        self.assert_single(False, "TIMED_OUT")
        handle = self.accept()
        self.assertEqual(handle.cancel_count, 1)
        self.complete(handle)
        self.assert_single(False, "TIMED_OUT")
        self.assertEqual(self.results[0]["metrics"]["status"], "TIMED_OUT")

    def test_timeout_after_acceptance_suppresses_late_result(self):
        self.start(timeout_s=1)
        handle = self.accept()
        self.now.return_value = 101
        self.node.timers[0].fire()
        self.complete(handle)
        self.assert_single(False, "TIMED_OUT")
        self.assertEqual(handle.cancel_count, 1)

    def test_result_at_deadline_is_not_success(self):
        self.start(timeout_s=1)
        handle = self.accept()
        self.now.return_value = 101
        self.complete(handle)
        self.assert_single(False, "TIMED_OUT")

    def test_cancel_before_send(self):
        event = threading.Event()
        event.set()
        self.start(cancel_event=event)
        self.assert_single(False, "CANCELLED")
        self.assertEqual(self.node.client.goals, [])

    def test_cancel_while_waiting_for_acceptance(self):
        event = threading.Event()
        self.start(cancel_event=event)
        event.set()
        self.node.timers[0].fire()
        handle = self.accept()
        self.assertEqual(handle.cancel_count, 1)
        self.assert_single(False, "CANCELLED")

    def test_cancel_after_acceptance(self):
        event = threading.Event()
        self.start(cancel_event=event)
        handle = self.accept()
        event.set()
        self.node.timers[0].fire()
        self.complete(handle)
        self.assert_single(False, "CANCELLED")
        self.assertEqual(handle.cancel_count, 1)

    def test_result_identity_safety_and_ros_status_fail_closed(self):
        for overrides, status in (
            ({"request_id": "another"}, 4), ({"status": "PRINT_NOW"}, 4),
            ({"fabrication_authorized": True}, 4), ({"requires_review": False}, 4),
            ({}, 6), ({}, 5),
            ({"success": 1}, 4), ({"message": None}, 4),
        ):
            with self.subTest(overrides=overrides, ros_status=status):
                self.node = Node()
                self.api = OBS.ObservationCommands(self.node)
                self.results.clear()
                self.finished.clear()
                self.start()
                handle = self.accept()
                self.complete(handle, result(**overrides), status)
                self.assert_single(False, "INVALID_RESULT")

    def test_known_backend_failure_is_failure_not_unknown_status(self):
        self.start()
        self.complete(self.accept(), result(success=False, status="FAILED"), status=6)
        self.assert_single(False, "FAILED")

    def test_immediate_timer_completion_still_destroys_timer(self):
        self.node.fire_timer_on_create = True
        self.now.side_effect = [100, 101, 101]
        self.start(timeout_s=1)
        self.assert_single(False, "TIMED_OUT")
        self.assertEqual(self.node.client.goals, [])

    def test_goal_and_result_future_errors_release_once(self):
        self.start()
        self.node.client.goal_future.set_exception(RuntimeError("goal failure"))
        self.assert_single(False, "FAILED")

    def test_result_future_exception(self):
        self.start()
        handle = self.accept()
        handle.result_future.set_exception(RuntimeError("result failure"))
        self.assert_single(False, "INVALID_RESULT")
        self.assertEqual(handle.cancel_count, 1)

    def test_get_result_exception(self):
        self.start()
        handle = Handle()
        handle.get_result_error = RuntimeError("cannot get result")
        self.accept(handle)
        self.assert_single(False, "FAILED")

    def test_send_exception(self):
        self.node.client.send_error = RuntimeError("send failure")
        self.start()
        self.assert_single(False, "FAILED")

    def test_timer_creation_exception(self):
        self.node.timer_error = RuntimeError("timer failure")
        self.start()
        self.assert_single(False, "FAILED")
        self.assertEqual(self.node.client.goals, [])

    def test_cancel_exception_does_not_prevent_queue_release(self):
        event = threading.Event()
        self.start(cancel_event=event)
        handle = self.accept()
        handle.cancel_error = RuntimeError("cancel failed")
        event.set()
        self.node.timers[0].fire()
        self.assert_single(False, "CANCELLED")


if __name__ == "__main__":
    unittest.main()

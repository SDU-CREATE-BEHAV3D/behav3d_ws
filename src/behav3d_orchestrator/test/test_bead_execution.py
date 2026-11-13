"""Exercise real print loops with command outcomes, never robot hardware."""
from contextlib import nullcontext
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
import open3d as o3d
import yaml
import rclpy

from geometry_msgs.msg import PoseStamped
from behav3d_orchestrator.src.control_session import ControlAwareSession
from behav3d_orchestrator.src.print_session import PrintSession
from behav3d_orchestrator.src.yaml_session import DotTarget, TargetSegment
from behav3d_orchestrator.src.fault_session import FaultSession
from behav3d_orchestrator.print_yaml_fault_and_scan_sequence import PrintYamlFaultAndScanSequenceNode
from behav3d_orchestrator.print_yaml_and_scan_sequence import PrintYamlAndScanSequenceNode


def pose(x=0.5, y=0.5, z=0.5):
    result = PoseStamped()
    result.header.frame_id = "world"
    result.pose.position.x, result.pose.position.y, result.pose.position.z = x, y, z
    result.pose.orientation.w = 1.0
    return result


class Commands:
    def __getattr__(self, kind):
        def build(*args, **kwargs):
            callback = kwargs.pop("on_done", None)
            kwargs.pop("enqueue", None)
            return SimpleNamespace(kind=kind, payload=kwargs, on_done=callback)
        return build


class FakePrintSession(PrintSession):
    def __init__(self, root):
        self.motion = self.extruder = self.util = Commands()
        self.node = SimpleNamespace(get_logger=lambda: SimpleNamespace(info=lambda *a: None, warn=lambda *a: None, error=lambda *a: None))
        self.joint_current_logger = SimpleNamespace(mark_event=lambda *a, **k: None)
        self.fault_session = None
        self._active_bead = None
        self.commands = []
        self.failure = None
        self.plans = []
        self.configure_bead_logging(root, "field_loop/cycle_0001")

    def execute(self, item, timeout_s=None):
        self.commands.append(item)
        if self.failure is not None:
            value = self.failure(item)
            if value is not None:
                return value
        return dict(ok=True, metrics=dict(accepted_steps=item.payload["steps"])) if item.kind == "print_steps" else dict(ok=True)

    def run_group(self, items):
        for item in items:
            item.on_done(self.execute(item))

    def _start_joint_current_logging(self, *a, **k):
        return False

    def wait_if_paused(self, *a):
        return True

    def defer_pause(self):
        return nullcontext()

    def _plan_print_segment(self, **kwargs):
        self.plans.append(kwargs)
        return dict(ok=True, metrics=dict(planned_duration_s=2.0))


class BeadExecutionTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.session = FakePrintSession(self.root)
        override = patch.object(ControlAwareSession, "run_sync", lambda obj, item, timeout_s=None: obj.execute(item, timeout_s))
        override.start()
        self.addCleanup(override.stop)

    def report(self):
        return yaml.safe_load(self.session._print_manifest_path.read_text())["cycles"][self.session.bead_cycle]

    def region(self, pct=40):
        path = self.root / "region.stl"
        mesh = o3d.geometry.TriangleMesh.create_box(1000, 1000, 1000)
        mesh.compute_vertex_normals()
        o3d.io.write_triangle_mesh(str(path), mesh)
        return FaultSession().load_region(path, reduction_pct=pct)

    def dots(self, **kwargs):
        return self.session.run_print_dots_targets(targets=[pose()], dot_steps=100,
            steps_per_mm3=2, post_dot_retract_steps=20, post_dot_retract_speed=100,
            dwell_s=0, **kwargs)

    def segments(self, **kwargs):
        return self.session.run_print_segments(segments=[TargetSegment(17, pose(z=0.48), pose())],
            segment_steps=100, steps_per_mm3=2, steps_per_second=60,
            post_segment_retract_steps=20, post_segment_retract_speed=100, **kwargs)

    def test_mesh_selects_endpoints_including_boundary_and_respects_scale(self):
        region = self.region()
        self.assertTrue(region.contains((0.5, 0.5, 0.5)))
        self.assertTrue(region.contains((0, 0.5, 0.5)))
        self.assertFalse(region.contains((1.1, 0.5, 0.5)))
        self.assertEqual(region.material_steps(100, pose()), 60)
        self.assertEqual(region.material_steps(100, pose(x=2)), 100)
        with self.assertRaises(ValueError):
            region.set_reduction(101)
        with self.assertRaises(FileNotFoundError):
            FaultSession().load_region(self.root / "absent.stl")
        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector([[0, 0, 0], [1, 0, 0], [0, 1, 0]])
        mesh.triangles = o3d.utility.Vector3iVector([[0, 1, 2]])
        o3d.io.write_triangle_mesh(str(self.root / "open.ply"), mesh)
        with self.assertRaises(ValueError):
            FaultSession().load_region(self.root / "open.ply")

    def test_manifest_separates_net_forward_and_retract_for_dots_and_segments(self):
        for mode in ["dots", "segments"]:
            for reduction in [0, 40]:
                with self.subTest(mode=mode, reduction=reduction):
                    self.session.fault_session = self.region(reduction) if reduction else None
                    self.session.configure_bead_logging(self.root, f"accounting/{mode}_{reduction}")
                    getattr(self, mode)()
                    row = self.report()["successful_beads"][0]
                    net = 100 - reduction
                    self.assertEqual(row["record_schema_version"], 2)
                    self.assertEqual(row["extrusion_steps_per_mm3"], 2)
                    self.assertEqual(row["nominal_net_steps"], 100)
                    self.assertEqual(row["requested_net_steps"], net)
                    self.assertEqual(row["commanded_forward_steps"], net + 20)
                    self.assertEqual(row["commanded_retract_steps"], 20)
                    self.assertEqual(row["accepted_forward_steps"], net + 20)
                    self.assertEqual(row["accepted_retract_steps"], 20)
                    self.assertEqual(row["accepted_net_steps"], net)
                    self.assertEqual(row["net_volume_mm3"], net / 2)
                    self.assertEqual(row["volume_mm3"], row["net_volume_mm3"])

    def test_cycle_checkpoints_are_cached_and_do_not_rewrite_history(self):
        self.dots()
        first = self.session._print_manifest_path
        self.session.finalize_bead_logging(dict(ok=True))
        original = first.read_bytes()
        self.session.configure_bead_logging(self.root, "field_loop/cycle_0002")
        with patch("behav3d_orchestrator.src.print_session._read_print_yaml",
                   side_effect=AssertionError("checkpoint reread a YAML file")):
            self.dots(target_indices=[12])
            self.session.configure_bead_logging(self.root, "field_loop/cycle_0002")
            self.dots(target_indices=[13])
            self.session.finalize_bead_logging(dict(ok=True))
        self.assertEqual(first.read_bytes(), original)
        self.assertEqual(len(self.report()["attempted_beads"]), 2)
        self.assertFalse((self.root / "print_manifest.yaml").exists())
        self.assertEqual(self.session._print_manifest_path,
                         self.root / "field_loop/cycle_0002/print_manifest.yaml")
        self.session.finish_bead_logging()
        consolidated = yaml.safe_load((self.root / "print_manifest.yaml").read_text())
        self.assertEqual(set(consolidated["cycles"]), {"field_loop/cycle_0001", "field_loop/cycle_0002"})
        self.assertEqual(first.read_bytes(), original)

    def test_cycle_summary_distinguishes_known_material_from_unknown_and_failed_beads(self):
        self.dots(target_indices=[1])
        def timeout(item):
            if item.kind == "print_steps":
                raise TimeoutError("reply lost")
        self.session.failure = timeout
        with self.assertRaises(TimeoutError):
            self.dots(target_indices=[2])
        self.session.failure = None
        self.session.fault_session = self.region(100)
        self.dots(target_indices=[3])
        self.session.fault_session = None
        self.session.failure = lambda item: dict(ok=False, error="motion aborted") if item.kind == "exec" else None
        self.session.run_print_segments(segments=[TargetSegment(4, pose(z=0.48), pose())],
                                       segment_steps=100, steps_per_mm3=2)
        self.session.finalize_bead_logging(dict(ok=False))
        bucket = self.report()
        self.assertEqual(bucket["print_status"], "failed")
        summary = bucket["summary"]
        self.assertEqual(summary["attempted_beads"], 4)
        self.assertEqual(summary["successful_beads"], 1)
        self.assertEqual(summary["failed_beads"], 1)
        self.assertEqual(summary["unknown_beads"], 1)
        self.assertEqual(summary["status_counts"], dict(success=1, unknown=1, not_extruded=1, motion_failed=1))
        self.assertEqual(summary["known_net_steps"], 200)
        self.assertEqual(summary["unknown_net_steps_beads"], 1)
        self.assertEqual(summary["known_net_volume_mm3"], 100)
        self.assertEqual(summary["unknown_net_volume_beads"], 1)
        self.assertEqual(summary["successful_net_volume_mm3"], 50)

    def test_restart_recovers_cycle_newer_than_the_last_consolidated_snapshot(self):
        self.dots(target_indices=[42])
        self.session.finish_bead_logging()
        snapshot = (self.root / "print_manifest.yaml").read_bytes()
        self.session.configure_bead_logging(self.root, "field_loop/cycle_0005")
        def timeout(item):
            if item.kind == "print_steps":
                raise TimeoutError("interrupted")
        self.session.failure = timeout
        with self.assertRaises(TimeoutError):
            self.dots(target_indices=[43])
        self.assertEqual((self.root / "print_manifest.yaml").read_bytes(), snapshot)
        self.assertEqual(PrintSession.next_print_cycle_number(self.root, "field_loop"), 6)
        recovered = PrintSession.load_print_manifest(self.root)["cycles"]
        self.assertEqual(recovered["field_loop/cycle_0005"]["attempted_beads"][0]["status"], "unknown")
        fresh = FakePrintSession(self.root)
        fresh.configure_bead_logging(self.root, "field_loop/cycle_0005")
        fresh.run_print_dots_targets(targets=[pose()], target_indices=[44], dot_steps=100,
                                     steps_per_mm3=2, dwell_s=0)
        fresh.finalize_bead_logging(dict(ok=True))
        rows = yaml.safe_load(fresh._print_manifest_path.read_text())["cycles"][fresh.bead_cycle]
        self.assertEqual([r["index"] for r in rows["attempted_beads"]], [43, 44])
        self.assertEqual([r["index"] for r in rows["successful_beads"]], [44])
        self.assertEqual(rows["summary"]["unknown_beads"], 1)
        self.assertEqual((self.root / "print_manifest.yaml").read_bytes(), snapshot)

    def test_manifest_uses_response_counts_and_handles_disabled_retract(self):
        self.session.failure = lambda item: dict(ok=True, metrics=dict(accepted_steps=110)) if (
            item.kind == "print_steps" and not item.payload.get("reverse")) else None
        self.dots()
        row = self.report()["successful_beads"][0]
        self.assertEqual(row["requested_net_steps"], 100)
        self.assertEqual(row["commanded_forward_steps"], 120)
        self.assertEqual(row["accepted_forward_steps"], 110)
        self.assertEqual(row["accepted_net_steps"], 90)
        self.assertEqual(row["net_volume_mm3"], 45)
        self.session.failure = None
        self.session.configure_bead_logging(self.root, "no_retract")
        self.session.run_print_dots_targets(targets=[pose()], dot_steps=100,
            steps_per_mm3=2, post_dot_retract_steps=20, post_dot_retract_speed=0, dwell_s=0)
        row = self.report()["successful_beads"][0]
        self.assertEqual(row["commanded_forward_steps"], 100)
        self.assertEqual(row["commanded_retract_steps"], 0)
        self.assertEqual(row["accepted_retract_steps"], 0)
        self.assertEqual(row["accepted_net_steps"], 100)

    def test_manifest_schema_upgrade_preserves_legacy_bead_records(self):
        self.dots()
        cycle_manifest = self.session._print_manifest_path
        manifest = self.root / "print_manifest.yaml"
        data = yaml.safe_load(cycle_manifest.read_text())
        data["schema_version"] = 1
        data.pop("step_accounting")
        added = {"record_schema_version", "extrusion_steps_per_mm3", "nominal_net_steps",
                 "requested_net_steps", "commanded_retract_steps", "accepted_forward_steps",
                 "accepted_retract_steps", "accepted_net_steps", "net_volume_mm3"}
        bucket = data["cycles"][self.session.bead_cycle]
        for rows in (bucket["attempted_beads"], bucket["successful_beads"]):
            for row in rows:
                for key in added:
                    row.pop(key)
        legacy = yaml.safe_load(yaml.safe_dump(bucket))
        manifest.write_text(yaml.safe_dump(data))
        cycle_manifest.unlink()
        original = manifest.read_bytes()
        self.session = FakePrintSession(self.root)
        self.session.configure_bead_logging(self.root, "new_cycle")
        self.dots()
        self.assertEqual(manifest.read_bytes(), original)
        updated = self.session.load_print_manifest(self.root)
        self.assertEqual(updated["schema_version"], 3)
        self.assertIn("net_volume_mm3", updated["step_accounting"])
        self.assertEqual(updated["cycles"]["field_loop/cycle_0001"], legacy)
        self.assertEqual(updated["cycles"]["new_cycle"]["successful_beads"][0]["record_schema_version"], 2)

    def test_dot_reduction_changes_only_material_and_preserves_nominal_duration(self):
        self.session.fault_session = self.region()
        self.dots(target_indices=[91])
        row = self.report()["successful_beads"][0]
        self.assertEqual(row["index"], 91)
        self.assertEqual(row["nominal_volume_mm3"], 50)
        self.assertEqual(row["volume_mm3"], 30)
        commands = [i for i in self.session.commands if i.kind == "print_steps"]
        self.assertEqual([c.payload["steps"] for c in commands], [80, 20])
        self.assertEqual(commands[0].payload["speed"], 800)

    def test_segment_reduction_keeps_planned_speed_and_records_acknowledged_volume(self):
        self.segments()
        nominal_speed = self.session.plans[-1]["target_print_speed_mm_s"]
        self.session.configure_bead_logging(self.root, "field_loop/cycle_0002")
        self.session.fault_session = self.region()
        result = self.segments()
        self.assertAlmostEqual(self.session.plans[-1]["target_print_speed_mm_s"], nominal_speed)
        self.assertEqual(result["extrusion_requests"][0]["commanded_steps_per_second"], 40)
        row = self.report()["successful_beads"][0]
        self.assertEqual(row["index"], 17)
        self.assertEqual(row["volume_mm3"], 30)
        self.assertIn("start", row)
        self.assertEqual(len(self.session.load_print_manifest(self.root)["cycles"]), 2)

    def test_planning_failure_records_attempt_with_no_extrusion(self):
        self.session._plan_print_segment = lambda **k: dict(ok=False, stage="plan_segment", error="planning failed")
        self.segments()
        row = self.report()["attempted_beads"][0]
        self.assertEqual(row["volume_mm3"], 0)
        self.assertFalse(row["extruded"])
        self.assertEqual(row["accepted_forward_steps"], 0)
        self.assertEqual(row["accepted_retract_steps"], 0)
        self.assertEqual(row["accepted_net_steps"], 0)
        self.assertEqual(row["net_volume_mm3"], 0)
        self.assertFalse(self.report()["successful_beads"])
        self.assertFalse(any(i.kind == "print_steps" for i in self.session.commands))

    def test_first_approach_failure_and_timeout_are_not_lost(self):
        self.session.failure = lambda i: dict(ok=False, error="unreachable") if i.kind == "goto" else None
        self.segments()
        self.assertEqual(self.report()["attempted_beads"][0]["volume_mm3"], 0)
        self.assertFalse(self.report()["successful_beads"])

    def test_extruder_timeout_is_unknown_and_durable_before_return(self):
        def failure(item):
            if item.kind == "print_steps":
                row = self.report()["attempted_beads"][-1]
                self.assertIsNone(row["volume_mm3"])
                raise TimeoutError("communication lost")
        self.session.failure = failure
        with self.assertRaises(TimeoutError):
            self.dots()
        row = self.report()["attempted_beads"][0]
        self.assertIsNone(row["extruded"])
        self.assertIsNone(row["volume_mm3"])
        self.assertIsNone(row["accepted_forward_steps"])
        self.assertIsNone(row["accepted_net_steps"])
        self.assertIsNone(row["net_volume_mm3"])
        self.assertFalse(self.report()["successful_beads"])

    def test_retract_timeout_retains_forward_count_and_keeps_net_unknown(self):
        def failure(item):
            if item.kind == "print_steps" and item.payload.get("reverse"):
                row = self.report()["attempted_beads"][-1]
                self.assertEqual(row["accepted_forward_steps"], 120)
                self.assertIsNone(row["accepted_retract_steps"])
                self.assertIsNone(row["accepted_net_steps"])
                self.assertIsNone(row["net_volume_mm3"])
                raise TimeoutError("retract reply lost")
        self.session.failure = failure
        with self.assertRaises(TimeoutError):
            self.dots()
        row = self.report()["attempted_beads"][0]
        self.assertEqual(row["accepted_forward_steps"], 120)
        self.assertIsNone(row["accepted_retract_steps"])
        self.assertIsNone(row["accepted_net_steps"])
        self.assertIsNone(row["net_volume_mm3"])

    def test_missing_acknowledged_steps_or_failed_retract_are_not_success(self):
        for mode in ["missing", "retract"]:
            with self.subTest(mode=mode):
                self.session.configure_bead_logging(self.root, mode)
                self.session.failure = lambda i: (dict(ok=True, metrics={}) if mode == "missing" else
                    dict(ok=False, error="retract failed")) if i.kind == "print_steps" and (mode == "missing" or i.payload.get("reverse")) else None
                self.dots()
                self.assertIsNone(self.report()["attempted_beads"][0]["volume_mm3"])
                row = self.report()["attempted_beads"][0]
                self.assertIsNone(row["accepted_retract_steps"])
                self.assertIsNone(row["accepted_net_steps"])
                self.assertIsNone(row["net_volume_mm3"])
                if mode == "retract":
                    self.assertEqual(row["accepted_forward_steps"], 120)
                self.assertFalse(self.report()["successful_beads"])

    def test_motion_failure_is_not_success_even_when_extrusion_completes(self):
        self.session.failure = lambda i: dict(ok=False, error="motion aborted") if i.kind == "exec" else None
        self.session.run_print_segments(segments=[TargetSegment(1, pose(z=0.48), pose())],
                                       segment_steps=100, steps_per_mm3=2)
        row = self.report()["attempted_beads"][0]
        self.assertEqual(row["status"], "motion_failed")
        self.assertEqual(row["volume_mm3"], 50)
        self.assertFalse(self.report()["successful_beads"])

    def test_missing_motion_reply_does_not_count_as_success_after_forward_ack(self):
        callbacks = []
        def run_group(items):
            for item in items:
                if item.kind == "print_steps":
                    item.on_done(self.session.execute(item))
                else:
                    callbacks.append(item.on_done)
        self.session.run_group = run_group
        self.session.run_print_segments(segments=[TargetSegment(1, pose(z=0.48), pose())],
                                       segment_steps=100, steps_per_mm3=2, timeout_s=0.001)
        row = self.report()["attempted_beads"][0]
        self.assertEqual(row["status"], "motion_unknown")
        self.assertEqual(row["volume_mm3"], 50)
        self.assertFalse(self.report()["successful_beads"])
        callbacks[0](dict(ok=True))
        self.assertEqual(self.report()["attempted_beads"][0]["status"], "motion_unknown")

    def test_full_reduction_never_sends_zero_step_or_retract_commands(self):
        self.session.fault_session = self.region(100)
        self.segments()
        self.assertFalse(any(i.kind == "print_steps" for i in self.session.commands))
        self.assertEqual(self.report()["attempted_beads"][0]["volume_mm3"], 0)
        row = self.report()["attempted_beads"][0]
        self.assertEqual(row["requested_net_steps"], 0)
        self.assertEqual(row["commanded_forward_steps"], 0)
        self.assertEqual(row["commanded_retract_steps"], 0)
        self.assertEqual(row["accepted_net_steps"], 0)
        self.assertFalse(self.report()["successful_beads"])

    def test_operator_stop_does_not_create_attempted_beads(self):
        result = self.dots(before_target=lambda _: False)
        self.assertTrue(result["stopped_by_user"])
        self.assertFalse(self.session._print_manifest_path.exists())

    def test_repeat_attempts_and_restart_cycle_numbers_preserve_history(self):
        self.dots(target_indices=[42])
        self.dots(target_indices=[42])
        report = self.report()
        self.assertEqual([row["index"] for row in report["attempted_beads"]], [42, 42])
        self.assertEqual(len(report["successful_beads"]), 2)
        self.assertEqual(self.session.next_print_cycle_number(self.root, "field_loop"), 2)

    def test_fault_node_preserves_segment_yaml_indices_volumes_and_original_file(self):
        path = self.root / "targets.yaml"
        path.write_text('segments:\n  - index: 12\n    volume_mm3: 50\n    plane_type: E\n    start:\n      plane: "O(500,500,480) Z(0,0,1)"\n    end:\n      plane: "O(500,500,500) Z(0,0,1)"\n')
        original = path.read_bytes()
        node = object.__new__(PrintYamlFaultAndScanSequenceNode)
        node.print_session = self.session
        targets = node._load_print_targets(yaml_path=str(path), frame_id="world")
        self.assertEqual(len(targets), 1)
        self.assertEqual(targets[0].index, 12)
        self.assertEqual(targets[0].volume_mm3, 50)
        self.assertEqual(targets[0].pose.pose.position.z, 0.5)
        self.assertEqual(node._load_yaml_plane_types(str(path), target_count=1), ["E"])
        self.assertEqual(path.read_bytes(), original)

    def test_yaml_dot_sequence_prints_only_segment_ends_in_index_order(self):
        path = self.root / "segments.yaml"
        path.write_text(yaml.safe_dump(dict(segments=[
            dict(index=12, volume_mm3=27000, plane_type="E",
                 start=dict(plane="O(500,500,480) Z(0,0,1)"),
                 end=dict(plane="O(500,500,500) Z(0,0,1)")),
            dict(index=3, start=dict(plane="O(100,500,480) Z(0,0,1)"),
                 end=dict(plane="O(100,500,500) Z(0,0,1)")),
        ])))
        original = path.read_bytes()
        node = object.__new__(PrintYamlAndScanSequenceNode)
        node.print_session = self.session
        targets = node._load_print_targets(yaml_path=str(path), frame_id="world")
        self.assertEqual(node._loaded_target_indices, [3, 12])
        self.assertEqual([t.volume_mm3 for t in targets], [None, 27000])
        self.assertEqual([t.pose.pose.position.x for t in targets], [0.1, 0.5])
        self.assertEqual([t.pose.pose.position.z for t in targets], [0.5, 0.5])
        self.assertEqual(node._load_yaml_plane_types(str(path), target_count=2), ["N", "E"])
        self.assertEqual(path.read_bytes(), original)

    def test_offline_print_modes_execute_same_segment_yaml_without_rewriting(self):
        path = self.root / "segments.yaml"
        rows = []
        for index, volume in [(20, 25), (3, 50), (12, None)]:
            row = dict(index=index, start=dict(plane="O(500,500,480) Z(0,0,1)"),
                       end=dict(plane="O(500,500,500) Z(0,0,1)"))
            if volume is not None:
                row["volume_mm3"] = volume
            rows.append(row)
        path.write_text(yaml.safe_dump(dict(segments=rows)))
        original = path.read_bytes()
        cfg = yaml.safe_load((Path(__file__).resolve().parents[1] / "config/print_yaml_and_scan_sequence_config.yaml").read_text())["print_yaml_and_scan_sequence"]["ros__parameters"]
        cfg.update(prompt_before_print=False, dot_steps_n=100, segment_steps=100,
                   extrusion_steps_per_mm3=2, post_dot_retract_steps=20,
                   post_segment_retract_steps=20, dwell_s=0, post_segment_wait_s=0)
        node = object.__new__(PrintYamlAndScanSequenceNode)
        node.print_session = self.session
        node.get_logger = self.session.node.get_logger
        targets = node._load_print_targets(yaml_path=str(path), frame_id="world")
        for mode in ["dots", "segments", "auto"]:
            with self.subTest(mode=mode):
                cfg["print_mode"] = mode
                self.session.commands.clear()
                self.session.plans.clear()
                self.session.configure_bead_logging(self.root, f"mode_{mode}/cycle_0001")
                # Two chunks ensure segment slicing follows the loaded order.
                for start, end in [(0, 2), (2, 3)]:
                    result = node._print_chunk(
                        [t.pose for t in targets[start:end]],
                        volumes_mm3=[t.volume_mm3 for t in targets[start:end]],
                        plane_types=["N"] * (end - start), cfg=cfg, timeout_s=5,
                        target_start_index=start, total_target_count=len(targets))
                    self.assertTrue(result["ok"])
                    self.assertEqual(result["printed"], end - start)
                beads = self.report()["successful_beads"]
                self.assertEqual([b["index"] for b in beads], [3, 12, 20])
                self.assertEqual([b["volume_mm3"] for b in beads], [50, 50, 25])
                commands = [c.payload["steps"] for c in self.session.commands if c.kind == "print_steps"]
                self.assertEqual(commands, [120, 20, 120, 20, 70, 20])
                self.assertEqual([p["segment"].index for p in self.session.plans],
                                 [] if mode == "dots" else [3, 12, 20])
                self.assertEqual(path.read_bytes(), original)

    def test_offline_print_mode_rejects_invalid_mode_and_segments_without_paths(self):
        for cls in [PrintYamlAndScanSequenceNode, PrintYamlFaultAndScanSequenceNode]:
            node = object.__new__(cls)
            node._loaded_segments = []
            for mode in ["segment", "segments"]:
                with self.subTest(node=cls.__name__, mode=mode), self.assertRaises(ValueError):
                    node._resolve_print_mode(dict(print_mode=mode))
            self.assertEqual(node._resolve_print_mode(dict(print_mode="auto")), "dots")
            self.assertEqual(node._resolve_print_mode({}), "dots")
            node._loaded_segments = [TargetSegment(1, pose(z=0.48), pose())]
            self.assertEqual(node._resolve_print_mode({}),
                             "segments" if cls is PrintYamlFaultAndScanSequenceNode else "dots")

    def test_legacy_yaml_node_also_logs_actual_extrusion(self):
        cfg = yaml.safe_load((Path(__file__).resolve().parents[1] / "config/print_yaml_and_scan_sequence_config.yaml").read_text())["print_yaml_and_scan_sequence"]["ros__parameters"]
        cfg["prompt_before_print"] = False
        node = object.__new__(PrintYamlAndScanSequenceNode)
        node.print_session = self.session
        node._loaded_target_indices = [73]
        node.get_logger = self.session.node.get_logger
        node._print_chunk([pose()], volumes_mm3=[50], plane_types=["N"], cfg=cfg,
                          timeout_s=5, target_start_index=0, total_target_count=1)
        row = self.report()["successful_beads"][0]
        self.assertEqual(row["index"], 73)
        self.assertAlmostEqual(row["volume_mm3"], round(50 * cfg["extrusion_steps_per_mm3"]) / cfg["extrusion_steps_per_mm3"])

    def test_each_bead_reloads_steps_speeds_retract_and_fault_settings(self):
        for cls in [PrintYamlAndScanSequenceNode, PrintYamlFaultAndScanSequenceNode]:
            for mode in ["dots", "segments"]:
                for prompts in [False, True]:
                    with self.subTest(node=cls.__name__, mode=mode, prompts=prompts):
                        raw = yaml.safe_load((Path(__file__).resolve().parents[1] / "config" / cls.config_filename).read_text())
                        name = next(iter(raw))
                        cfg = dict(raw[name]["ros__parameters"], print_mode=mode, prompt_before_print=prompts,
                                   extrusion_steps_per_mm3=2, dot_steps_n=100, segment_steps=100,
                                   post_dot_retract_steps=20, post_segment_retract_steps=20,
                                   dwell_s=0, post_segment_wait_s=0, fault_region_mesh_path=str(self.root / "region.stl"))
                        self.region()
                        self.session.fault_session = None
                        self.session.failure = None
                        self.session.commands.clear()
                        self.session.plans.clear()
                        self.session.configure_bead_logging(self.root, f"beads_{name}_{mode}_{prompts}/cycle_0001")
                        config = self.root / cls.config_filename
                        node = object.__new__(cls)
                        node.print_session = self.session
                        node.get_name = lambda: name
                        node.get_logger = self.session.node.get_logger
                        node._execution_session, node._execution_frame = self.root, cfg["frame_id"]
                        node._default_config_path = lambda: str(config)
                        node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None))
                        node.create_publisher = lambda *a, **k: SimpleNamespace(publish=lambda *a: None)
                        node.fault_session = None
                        node._loaded_segments = [TargetSegment(i, pose(z=0.48), pose()) for i in [3, 12, 20]]
                        node._loaded_target_indices = [3, 12, 20]
                        steps = [200, 300, 400] if prompts else [100, 200, 300]
                        retracts = [30, 40, 50] if prompts else [20, 30, 40]
                        speeds = [1200, 1300, 1400]
                        reductions = [0, 25, 50]

                        def save():
                            config.write_text(yaml.safe_dump({name: {"ros__parameters": cfg}}))

                        def set_bead(i):
                            cfg.update(dot_steps_n=steps[i], dot_steps_e=steps[i], segment_steps=steps[i],
                                       dot_speed=speeds[i], segment_steps_per_second=speeds[i],
                                       post_dot_retract_steps=retracts[i], post_segment_retract_steps=retracts[i],
                                       extrusion_reduction_pct=reductions[i], segment_accel_scale=0.1 * (i + 1))
                            save()

                        if prompts:
                            save()
                        else:
                            set_bead(0)
                        current = dict(cfg)
                        reasons = []

                        def refresh(reason):
                            nonlocal current
                            reasons.append(reason)
                            current = node._reload_runtime_config_or_keep(current, config_path=str(config), reason=reason)
                            return current

                        prompt_count = forward_count = 0

                        def prompt(message):
                            nonlocal prompt_count
                            set_bead(prompt_count)
                            prompt_count += 1
                            return False

                        def during_command(item):
                            nonlocal forward_count
                            if item.kind == "print_steps" and not item.payload.get("reverse", False):
                                forward_count += 1
                                if prompts:
                                    # File edits during extrusion must not change this bead's retract.
                                    cfg.update(post_dot_retract_steps=999, post_segment_retract_steps=999)
                                    save()
                                elif forward_count < 3:
                                    set_bead(forward_count)

                        node._operator_requested_stop = prompt
                        self.session.failure = during_command
                        with patch.object(rclpy, "ok", return_value=True):
                            result = node._print_group([s.end for s in node._loaded_segments],
                                volumes_mm3=[None] * 3, plane_types=["N"] * 3, cfg=current, timeout_s=5,
                                target_start_index=0, total_target_count=3, refresh_config=refresh)
                        self.assertTrue(result["ok"])
                        self.assertEqual(result["printed"], 3)
                        self.assertEqual(prompt_count, 3 if prompts else 0)
                        self.assertEqual(len(reasons), 6 if prompts else 3)
                        material = [int(s * (1 - pct / 100) + 0.5) for s, pct in zip(steps, reductions)] if cls is PrintYamlFaultAndScanSequenceNode else steps
                        commands = [c.payload for c in self.session.commands if c.kind == "print_steps"]
                        self.assertEqual([c["steps"] for c in commands],
                                         [v for s, r in zip(material, retracts) for v in (s + r, r)])
                        beads = self.report()["successful_beads"]
                        self.assertEqual([b["index"] for b in beads], [3, 12, 20])
                        self.assertEqual([b["nominal_volume_mm3"] for b in beads], [s / 2 for s in steps])
                        self.assertEqual([b["volume_mm3"] for b in beads], [s / 2 for s in material])
                        if mode == "dots":
                            self.assertEqual([c["speed"] for c in commands[::2]],
                                             [max(1, round(v * (s + r) / (n + r))) for v, s, n, r in zip(speeds, material, steps, retracts)])
                        else:
                            self.assertEqual([round(p["seed_accel_scale"], 3) for p in self.session.plans], [0.1, 0.2, 0.3])
                            for plan, rate, step, retract in zip(self.session.plans, speeds, steps, retracts):
                                self.assertAlmostEqual(plan["target_print_speed_mm_s"], 20 * rate / (step + retract))
                        approaches = [c for c in self.session.commands if c.kind == "goto"
                                      and abs(c.payload["pose"].pose.position.z - (0.48 if mode == "segments" else 0.5) - 0.1) < 1e-9]
                        self.assertEqual(len(approaches), 1)
                        self.session.failure = None

    def test_per_bead_reload_stops_before_next_bead_when_operator_quits(self):
        node = object.__new__(PrintYamlAndScanSequenceNode)
        cfg = dict(prompt_before_print=True, timeout_s=0)
        node._refresh_execution_config = lambda cfg: None
        answers = iter([False, True])
        node._operator_requested_stop = lambda message: next(answers)
        printed = []

        def bead(targets, **kwargs):
            printed.extend(targets)
            return dict(ok=True, printed=1, printed_targets=targets)

        node._print_chunk = bead
        with patch.object(rclpy, "ok", return_value=True):
            result = node._print_group([pose()] * 3, volumes_mm3=[None] * 3,
                plane_types=["N"] * 3, cfg=cfg, timeout_s=None,
                target_start_index=0, total_target_count=3, refresh_config=lambda reason: cfg)
        self.assertTrue(result["stopped_by_user"])
        self.assertEqual(result["printed"], 1)
        self.assertEqual(len(printed), 1)

    def test_yaml_sequences_scan_after_each_group_and_respect_stop_and_failure(self):
        cases = [
            (12, True, None, ["print", "scan", "print", "scan", "print", "scan"]),
            (10, True, None, ["print", "scan", "print", "scan"]),
            (3, True, None, ["print", "scan"]),
            (12, False, None, ["print", "print", "print"]),
            (12, True, "scan_failure", ["print", "scan"]),
            (12, True, "print_failure", ["print"]),
            (12, True, "print_stop", ["print"]),
            (12, True, "scan_prompt_stop", ["print"]),
        ]
        for cls in [PrintYamlAndScanSequenceNode, PrintYamlFaultAndScanSequenceNode]:
            for count, enabled, interruption, expected in cases:
                with self.subTest(node=cls.__name__, count=count, enabled=enabled, interruption=interruption):
                    raw = yaml.safe_load((Path(__file__).resolve().parents[1] / "config" / cls.config_filename).read_text())
                    cfg = dict(next(iter(raw.values()))["ros__parameters"],
                               points_per_cycle=5, enable_scan=enabled, home_before=False,
                               home_after=False, debug_mode=False,
                               prompt_before_scan=interruption == "scan_prompt_stop")
                    events, scans, errors = [], [], []
                    node = object.__new__(cls)
                    node.get_logger = lambda: SimpleNamespace(info=lambda *a: None, warn=lambda *a: None,
                                                              error=lambda msg: errors.append(msg))
                    node._default_config_path = lambda: "mock_config.yaml"
                    node._load_config = lambda path: dict(cfg)
                    node._configure_run = node._refresh_execution_config = lambda cfg: None
                    node._execution_session, node._execution_scope = self.root, "scan_schedule"
                    node._first_cycle_number = 8
                    node.print_session = SimpleNamespace(configure_bead_logging=lambda *a: None,
                        finalize_bead_logging=lambda *a: None, finish_bead_logging=lambda: None)
                    targets = [DotTarget(i, pose(x=float(i))) for i in range(count)]
                    node._load_print_targets = lambda **kwargs: targets
                    node._load_yaml_plane_types = lambda *a, **k: ["N"] * count
                    node._publish_yaml_targets = lambda *a, **k: None
                    node._operator_requested_stop = lambda *a: True

                    def print_chunk(chunk, **kwargs):
                        events.append(("print", len(chunk)))
                        return dict(ok=interruption != "print_failure", printed=len(chunk),
                                    printed_targets=list(chunk), stopped_by_user=interruption == "print_stop")

                    def scan_cycle(**kwargs):
                        events.append(("scan", kwargs["cycle_tag"]))
                        scans.append(list(kwargs["printed_targets"]))
                        return interruption != "scan_failure"

                    node._print_group, node._run_scan_cycle = print_chunk, scan_cycle
                    with patch.object(rclpy, "ok", return_value=True), patch.object(rclpy, "shutdown"):
                        node._run()
                    self.assertEqual([kind for kind, _ in events], expected)
                    if interruption is None:
                        self.assertEqual([n for kind, n in events if kind == "print"],
                                         [min(5, count - i) for i in range(0, count, 5)])
                        self.assertEqual([tag for kind, tag in events if kind == "scan"],
                                         [f"cycle_{i:04d}" for i in range(8, 8 + len(scans))])
                        self.assertEqual([len(s) for s in scans],
                                         [min(i + 5, count) for i in range(0, count, 5)] if enabled else [])
                    self.assertEqual(bool(errors), interruption in ("scan_failure", "print_failure"))

    def test_yaml_sequences_reload_at_print_and_scan_boundaries(self):
        for cls in [PrintYamlAndScanSequenceNode, PrintYamlFaultAndScanSequenceNode]:
            with self.subTest(node=cls.__name__):
                raw = yaml.safe_load((Path(__file__).resolve().parents[1] / "config" / cls.config_filename).read_text())
                name = next(iter(raw))
                cfg = dict(raw[name]["ros__parameters"], points_per_cycle=5,
                           enable_scan=False, home_before=False, home_after=False,
                           debug_mode=True, prompt_before_scan=False, dot_steps_n=100)
                config = self.root / cls.config_filename

                def save(**updates):
                    cfg.update(updates)
                    config.write_text(yaml.safe_dump({name: {"ros__parameters": cfg}}))

                save()
                node = object.__new__(cls)
                node.get_name = lambda: name
                errors, events = [], []
                node.get_logger = lambda: SimpleNamespace(info=lambda *a: None, warn=lambda *a: None,
                                                          error=lambda msg: errors.append(msg))
                node._default_config_path = lambda: str(config)
                node._configure_run = node._refresh_execution_config = lambda cfg: None
                node._execution_session, node._execution_scope = self.root, "live_config"
                node._first_cycle_number = 1
                node.print_session = SimpleNamespace(configure_bead_logging=lambda *a: None,
                    finalize_bead_logging=lambda *a: None, finish_bead_logging=lambda: None)
                node._load_print_targets = lambda **k: [DotTarget(i, pose()) for i in range(7)]
                node._load_yaml_plane_types = lambda *a, **k: ["N"] * 7
                node._publish_yaml_targets = lambda *a, **k: None

                def prompt(message):
                    if "Debug gate" in message:
                        save(dot_steps_n=150, debug_mode=False)
                    else:
                        save(scan_vel_scale=0.31)
                    return False

                def print_chunk(chunk, **kwargs):
                    events.append(("print", len(chunk), kwargs["cfg"]["dot_steps_n"]))
                    if len(events) == 1:
                        save(enable_scan=True, prompt_before_scan=True, scan_vel_scale=0.23)
                    return dict(ok=True, printed=len(chunk), printed_targets=list(chunk))

                def scan_cycle(**kwargs):
                    events.append(("scan", kwargs["cfg"]["scan_vel_scale"]))
                    save(points_per_cycle=2, dot_steps_n=200, enable_scan=False, prompt_before_scan=False)
                    return True

                node._operator_requested_stop = prompt
                node._print_group, node._run_scan_cycle = print_chunk, scan_cycle
                with patch.object(rclpy, "ok", return_value=True), patch.object(rclpy, "shutdown"):
                    node._run()
                self.assertEqual(errors, [])
                self.assertEqual(events, [("print", 5, 150), ("scan", 0.31), ("print", 2, 200)])

    def test_runtime_config_reload_keeps_previous_on_failure_and_pins_target_source(self):
        node = object.__new__(PrintYamlAndScanSequenceNode)
        node.get_logger = self.session.node.get_logger
        node.get_name = lambda: "print_yaml_and_scan_sequence"
        cfg = yaml.safe_load((Path(__file__).resolve().parents[1] / "config" / node.config_filename).read_text())[node.get_name()]["ros__parameters"]
        config = self.root / "runtime.yaml"
        for content in ["[broken", yaml.safe_dump({node.get_name(): {"ros__parameters": dict(cfg, points_per_cycle=0)}}),
                        yaml.safe_dump({node.get_name(): {"ros__parameters": dict(cfg, print_mode="invalid")}})]:
            config.write_text(content)
            self.assertIs(node._reload_runtime_config_or_keep(cfg, config_path=str(config), reason="test"), cfg)
        config.write_text(yaml.safe_dump({node.get_name(): {"ros__parameters": dict(
            cfg, yaml_path="other.yaml", frame_id="other_frame", dot_steps_n=123)}}))
        updated = node._reload_runtime_config_or_keep(cfg, config_path=str(config), reason="test")
        self.assertEqual(updated["yaml_path"], cfg["yaml_path"])
        self.assertEqual(updated["frame_id"], cfg["frame_id"])
        self.assertEqual(updated["dot_steps_n"], 123)

    def test_fault_sequence_print_modes_reduce_material_then_scan_each_group_and_publish_region(self):
        for segments, mode in [(False, "dots"), (False, "auto"),
                               (True, "dots"), (True, "segments"), (True, "auto")]:
            with self.subTest(segments=segments, mode=mode):
                self.region()
                self.session.plans.clear()
                cfg = yaml.safe_load((Path(__file__).resolve().parents[1] / "config/print_yaml_fault_and_scan_sequence_config.yaml").read_text())["print_yaml_fault_and_scan_sequence"]["ros__parameters"]
                prefix = f"fault_{'segment' if segments else 'dot'}_{mode}"
                cfg.update(fault_region_mesh_path=str(self.root / "region.stl"), prompt_before_print=False,
                           points_per_cycle=2, extrusion_steps_per_mm3=2, segment_steps_per_second=60,
                           dot_steps_n=100, post_dot_retract_steps=20, post_segment_retract_steps=20,
                           run_reconstruction=False, scan_root_folder=f"@session/{prefix}", print_mode=mode)
                source = self.root / "targets.yaml"
                rows = []
                for index, x in [(20, 500), (30, 2000), (40, 500)]:
                    end = dict(plane=f"O({x},500,500) Z(0,0,1)")
                    row = dict(index=index, volume_mm3=50)
                    row.update(start=dict(plane=f"O({x},500,480) Z(0,0,1)"), end=end) if segments else row.update(end)
                    rows.append(row)
                source.write_text(yaml.safe_dump({"segments" if segments else "targets": rows}))
                cfg["yaml_path"] = str(source)
                original = source.read_bytes()
                config = self.root / "run_config.yaml"
                config.write_text(yaml.safe_dump({"print_yaml_fault_and_scan_sequence": {"ros__parameters": cfg}}))
                node = object.__new__(PrintYamlFaultAndScanSequenceNode)
                node.print_session = self.session
                node.fault_session = None
                node._loaded_segments = []
                node._default_config_path = lambda: str(config)
                node.get_logger = self.session.node.get_logger
                node.get_name = lambda: "print_yaml_fault_and_scan_sequence"
                node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None))
                markers = []
                node.create_publisher = lambda *a, **k: SimpleNamespace(publish=markers.append)
                scans = []
                node._run_scan_cycle = lambda **kwargs: scans.append(
                    dict(kwargs, printed_targets=list(kwargs["printed_targets"]))) or True
                with patch("behav3d_orchestrator.print_yaml_and_scan_sequence.PrintSession.active_session_path", return_value=self.root), patch.object(rclpy, "ok", return_value=True), patch.object(rclpy, "shutdown", create=True):
                    node._run()
                self.assertEqual(source.read_bytes(), original)
                self.assertEqual([s["cycle_tag"] for s in scans], ["cycle_0001", "cycle_0002"])
                self.assertEqual([len(s["printed_targets"]) for s in scans],
                                 [4, 6] if segments and mode != "dots" else [2, 3])
                self.assertEqual(len(markers), 1)
                self.assertEqual(markers[0].color.a, 0.2)
                self.assertEqual(markers[0].header.frame_id, "world")
                self.assertLessEqual(max(p.x for p in markers[0].points), 1)
                data = PrintSession.load_print_manifest(self.root)["cycles"]
                success = [row for key, cycle in data.items() if key.startswith(prefix)
                           for row in cycle["successful_beads"]]
                self.assertEqual([row["index"] for row in success], [20, 30, 40])
                self.assertEqual([row["volume_mm3"] for row in success], [30, 50, 30])
                self.assertEqual([p["segment"].index for p in self.session.plans],
                                 [20, 30, 40] if segments and mode != "dots" else [])

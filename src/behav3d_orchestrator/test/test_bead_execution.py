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
from behav3d_orchestrator.src.yaml_session import TargetSegment
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
        return yaml.safe_load((self.root / "print_manifest.yaml").read_text())["cycles"][self.session.bead_cycle]

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
        self.assertEqual(len(yaml.safe_load((self.root / "print_manifest.yaml").read_text())["cycles"]), 2)

    def test_planning_failure_records_attempt_with_no_extrusion(self):
        self.session._plan_print_segment = lambda **k: dict(ok=False, stage="plan_segment", error="planning failed")
        self.segments()
        row = self.report()["attempted_beads"][0]
        self.assertEqual(row["volume_mm3"], 0)
        self.assertFalse(row["extruded"])
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
        self.assertFalse(self.report()["successful_beads"])

    def test_missing_acknowledged_steps_or_failed_retract_are_not_success(self):
        for mode in ["missing", "retract"]:
            with self.subTest(mode=mode):
                self.session.configure_bead_logging(self.root, mode)
                self.session.failure = lambda i: (dict(ok=True, metrics={}) if mode == "missing" else
                    dict(ok=False, error="retract failed")) if i.kind == "print_steps" and (mode == "missing" or i.payload.get("reverse")) else None
                self.dots()
                self.assertIsNone(self.report()["attempted_beads"][0]["volume_mm3"])
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
        self.assertFalse(self.report()["successful_beads"])

    def test_operator_stop_does_not_create_attempted_beads(self):
        result = self.dots(before_target=lambda _: False)
        self.assertTrue(result["stopped_by_user"])
        self.assertFalse((self.root / "print_manifest.yaml").exists())

    def test_repeat_attempts_and_restart_cycle_numbers_preserve_history(self):
        self.dots(target_indices=[42])
        self.dots(target_indices=[42])
        report = self.report()
        self.assertEqual([row["index"] for row in report["attempted_beads"]], [42, 42])
        self.assertEqual(len(report["successful_beads"]), 2)
        self.assertEqual(self.session.next_print_cycle_number(self.root, "field_loop"), 2)

    def test_fault_node_preserves_segment_yaml_indices_volumes_and_original_file(self):
        path = self.root / "targets.yaml"
        path.write_text('segments:\n  - index: 12\n    volume_mm3: 50\n    start:\n      plane: "O(500,500,480) Z(0,0,1)"\n    end:\n      plane: "O(500,500,500) Z(0,0,1)"\n')
        original = path.read_bytes()
        node = object.__new__(PrintYamlFaultAndScanSequenceNode)
        node.print_session = self.session
        targets = node._load_print_targets(yaml_path=str(path), frame_id="world")
        self.assertEqual(len(targets), 1)
        self.assertEqual(targets[0].index, 12)
        self.assertEqual(targets[0].volume_mm3, 50)
        self.assertEqual(targets[0].pose.pose.position.z, 0.5)
        self.assertEqual(node._load_yaml_plane_types(str(path), target_count=1), ["N"])
        self.assertEqual(path.read_bytes(), original)

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

    def test_fault_sequence_runs_both_schemas_then_scans_once_and_publishes_region(self):
        for segments in [False, True]:
            with self.subTest(segments=segments):
                self.region()
                cfg = yaml.safe_load((Path(__file__).resolve().parents[1] / "config/print_yaml_fault_and_scan_sequence_config.yaml").read_text())["print_yaml_fault_and_scan_sequence"]["ros__parameters"]
                cfg.update(fault_region_mesh_path=str(self.root / "region.stl"), prompt_before_print=False,
                           points_per_cycle=2, extrusion_steps_per_mm3=2, segment_steps_per_second=60,
                           dot_steps_n=100, post_dot_retract_steps=20, post_segment_retract_steps=20,
                           run_reconstruction=False, scan_root_folder="@session/fault_segment" if segments else "@session/fault_dot")
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
                node._run_scan_cycle = lambda **kwargs: scans.append(kwargs)
                with patch("behav3d_orchestrator.print_yaml_and_scan_sequence.PrintSession.active_session_path", return_value=self.root), patch.object(rclpy, "ok", return_value=True), patch.object(rclpy, "shutdown", create=True):
                    node._run()
                self.assertEqual(source.read_bytes(), original)
                self.assertEqual(len(scans), 1)
                self.assertEqual(len(markers), 1)
                self.assertEqual(markers[0].color.a, 0.2)
                self.assertEqual(markers[0].header.frame_id, "world")
                self.assertLessEqual(max(p.x for p in markers[0].points), 1)
                data = yaml.safe_load((self.root / "print_manifest.yaml").read_text())["cycles"]
                prefix = "fault_segment" if segments else "fault_dot"
                success = [row for key, cycle in data.items() if key.startswith(prefix)
                           for row in cycle["successful_beads"]]
                self.assertEqual([row["index"] for row in success], [20, 30, 40])
                self.assertEqual([row["volume_mm3"] for row in success], [30, 50, 30])

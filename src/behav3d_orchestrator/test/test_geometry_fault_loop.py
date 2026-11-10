"""Exercise the inherited geometry loop with simulated ROS service replies."""
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import open3d as o3d
import rclpy
import yaml

from behav3d_orchestrator.geometry_representation_scan_print_loop import GeometryRepresentationScanPrintLoopNode
from behav3d_orchestrator.geometry_representation_fault_scan_print_loop import GeometryRepresentationFaultScanPrintLoopNode
from behav3d_orchestrator.src.control_session import ControlAwareSession
from test_bead_execution import FakePrintSession, Commands, pose


class GeometryFaultLoopTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        mesh = o3d.geometry.TriangleMesh.create_box(1000, 1000, 1000)
        mesh.compute_vertex_normals()
        o3d.io.write_triangle_mesh(str(self.root / "region.stl"), mesh)
        backend = patch.object(ControlAwareSession, "run_sync", lambda obj, item, timeout_s=None: obj.execute(item, timeout_s))
        backend.start()
        self.addCleanup(backend.stop)

    def run_loop(self, *, fault=True, segments=True, update_pct=None, invalid_mesh=False, request_rescan=False):
        name = "geometry_representation_fault_scan_print_loop" if fault else "geometry_representation_scan_print_loop"
        output = self.root / name
        output.mkdir()
        package = Path(__file__).resolve().parents[1]
        cfg = yaml.safe_load((package / "config" / f"{name}_config.yaml").read_text())[name]["ros__parameters"]
        cfg.update(home_before_scan=invalid_mesh, home_after=False, max_cycles=2,
                   prompt_before_print=False, prompt_before_scan=False, debug_mode=request_rescan,
                   enable_scan=True, run_reconstruction=True, scan_before_generate_first_cycle=True,
                   contour_offset_estimation=False, log_joint_currents=False,
                   resume_scan_mesh_path=str(output / "scan_initial.ply"),
                   print_mode="segments" if segments else "dots", extrusion_steps_per_mm3=2,
                   segment_steps_per_second=60, post_segment_retract_steps=20,
                   post_dot_retract_steps=20, dot_dwell_s=0)
        if fault:
            cfg.update(fault_region_mesh_path="absent.stl" if invalid_mesh else "region.stl",
                       fault_region_mesh_scale=0.001, extrusion_reduction_pct=40)
        config_path = self.root / f"{name}_config.yaml"

        def write_config():
            config_path.write_text(yaml.safe_dump({name: {"ros__parameters": cfg}}), encoding="utf-8")
        write_config()
        cls = GeometryRepresentationFaultScanPrintLoopNode if fault else GeometryRepresentationScanPrintLoopNode
        node = object.__new__(cls)
        node.fault_session = None
        node.print_session = FakePrintSession(output)
        node._default_runtime_config_path = lambda: str(config_path)
        node._latest_capture_session_path = lambda: output
        node.get_name = lambda: name
        errors, generations, scans, reconstructions, markers, originals, rescans = [], [], [], [], [], [], []
        node.get_logger = lambda: SimpleNamespace(info=lambda *a: None, warn=lambda *a: None, error=errors.append)
        node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None))
        node.create_publisher = lambda *a, **k: SimpleNamespace(publish=markers.append)
        node._wait_if_control_paused = lambda *a: True
        node._load_configured_field_state = lambda rt: (str(output / "field.npz"), (0., 0., 0.), (0., 0., 0.))
        node._set_fields_node_candidate_params = lambda rt: None

        def dispatch(item, timeout_s=None):
            if item.kind == "generate_print_candidates":
                generations.append(dict(item.payload))
                destination = Path(item.payload["output_dir"]) / "targets.yaml"
                destination.parent.mkdir(parents=True, exist_ok=True)
                rows = []
                for index, x in [(12, 500), (29, 2000)]:
                    row = dict(index=index, volume_mm3=50)
                    endpoint = dict(plane=f"O({x},500,500) Z(0,0,1)")
                    row.update(start=dict(plane=f"O({x},500,480) Z(0,0,1)"), end=endpoint) if segments else row.update(endpoint)
                    rows.append(row)
                destination.write_text(yaml.safe_dump({"segments" if segments else "targets": rows}))
                originals.append((destination, destination.read_bytes()))
                return dict(ok=True, metrics=dict(targets_yaml_path=str(destination), candidate_count=2))
            if item.kind == "input":
                return dict(ok=True, metrics=dict(value="r" if request_rescan and not rescans else ""))
            return dict(ok=True, metrics={})

        node.session = SimpleNamespace(motion=Commands(), field=Commands(), camera=Commands(), util=Commands(), run_sync=dispatch)

        def scan(**kwargs):
            scans.append(kwargs)
            if len(scans) == 2 and update_pct is not None:
                cfg["extrusion_reduction_pct"] = update_pct
                write_config()
            return [pose()]

        def reconstruct(**kwargs):
            reconstructions.append(kwargs)
            return str(output / f"scan_reconstructed_{len(reconstructions)}.ply"), {}

        def rescan(**kwargs):
            rescans.append(kwargs)
            return str(output / "scan_manual.ply")

        node._run_configured_scan = scan
        node._run_reconstruction_for_scan = reconstruct
        node._rescan_cycle = rescan
        with patch.object(rclpy, "ok", return_value=True), patch.object(rclpy, "shutdown", create=True):
            node._run()
        if not invalid_mesh:
            self.assertFalse(errors, errors)
        for path, original in originals:
            self.assertEqual(path.read_bytes(), original)
        return SimpleNamespace(node=node, output=output, errors=errors, generations=generations,
                               scans=scans, reconstructions=reconstructions, markers=markers, rescans=rescans)

    def manifest(self, result):
        return yaml.safe_load((result.output / "print_manifest.yaml").read_text())["cycles"]

    def test_fault_loop_prints_then_rescans_and_uses_the_updated_mesh(self):
        result = self.run_loop()
        self.assertEqual(len(result.generations), 2)
        self.assertEqual(len(result.scans), 2)
        self.assertEqual(len(result.reconstructions), 2)
        self.assertTrue(result.generations[0]["scan_mesh_paths"][0].endswith("scan_reconstructed_1.ply"))
        self.assertTrue(result.generations[1]["scan_mesh_paths"][0].endswith("scan_reconstructed_2.ply"))
        manifest = self.manifest(result)
        self.assertEqual(set(manifest), {"field_loop/cycle_0000", "field_loop/cycle_0001"})
        for cycle in manifest.values():
            self.assertEqual([r["index"] for r in cycle["attempted_beads"]], [12, 29])
            self.assertEqual([r["volume_mm3"] for r in cycle["successful_beads"]], [30, 50])
        self.assertEqual(len(result.markers), 1)
        self.assertEqual(result.markers[0].color.a, 0.2)
        self.assertEqual(result.markers[0].header.frame_id, "world")

    def test_dot_mode_uses_the_same_fault_and_manifest(self):
        result = self.run_loop(segments=False)
        for cycle in self.manifest(result).values():
            self.assertEqual([r["volume_mm3"] for r in cycle["successful_beads"]], [30, 50])
            self.assertTrue(all("start" not in r for r in cycle["attempted_beads"]))

    def test_runtime_percentage_changes_apply_to_the_next_cycle(self):
        result = self.run_loop(update_pct=20)
        manifest = self.manifest(result)
        self.assertEqual(manifest["field_loop/cycle_0000"]["successful_beads"][0]["volume_mm3"], 30)
        self.assertEqual(manifest["field_loop/cycle_0001"]["successful_beads"][0]["volume_mm3"], 40)
        self.assertEqual(len(result.markers), 1)
        for number, pct in [(0, 40), (1, 20)]:
            snapshot = next((result.output / f"field_loop/cycle_{number:04d}/config").glob("*.yaml"))
            values = yaml.safe_load(snapshot.read_text())[result.node.get_name()]["ros__parameters"]
            self.assertEqual(values["extrusion_reduction_pct"], pct)

    def test_invalid_region_is_rejected_before_home_scan_or_extrusion(self):
        result = self.run_loop(invalid_mesh=True)
        self.assertTrue(result.errors)
        self.assertFalse(result.generations)
        self.assertFalse(result.scans)
        self.assertFalse(result.node.print_session.commands)
        self.assertFalse((result.output / "print_manifest.yaml").exists())

    def test_original_loop_keeps_nominal_material_and_generation_parameters(self):
        baseline = self.run_loop(fault=False)
        faulty = self.run_loop()
        for cycle in self.manifest(baseline).values():
            self.assertEqual([r["volume_mm3"] for r in cycle["successful_beads"]], [50, 50])
        self.assertFalse(baseline.markers)
        self.assertIsNone(baseline.node.print_session.fault_session)
        for left, right in zip(baseline.generations, faulty.generations):
            for key in left.keys() - {"session_path", "output_dir", "field_state_path", "scan_mesh_paths"}:
                self.assertEqual(left[key], right[key], key)

    def test_operator_rescan_regenerates_candidates_before_creating_attempts(self):
        result = self.run_loop(request_rescan=True)
        self.assertEqual(len(result.rescans), 1)
        self.assertEqual(len(result.generations), 3)
        self.assertEqual(sum(len(c["attempted_beads"]) for c in self.manifest(result).values()), 4)

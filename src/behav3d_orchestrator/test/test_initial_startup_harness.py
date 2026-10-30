"""Pure validation-harness contract checks, without ROS or scientific imports."""

import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "scripts"))
spec = importlib.util.spec_from_file_location(
    "ros_initial_startup_helpers", ROOT / "scripts/ros_initial_startup.py"
)
harness = importlib.util.module_from_spec(spec)
spec.loader.exec_module(harness)


class StartupHarnessTests(unittest.TestCase):
    def setUp(self):
        self.request = {"frame_id": "synthetic_world"}
        self.planning = {
            "selected": {"center_m": [0, 0, 0.735]},
            "scene_sha256": "a",
            "geometry_sha256": "b",
            "task_sha256": "c",
            "proposal_id": "proposal-a",
        }
        self.batch = dict(
            self.planning,
            frame_id="synthetic_world",
            candidate_count=6,
            requested_count=7,
            partial_batch=True,
            requires_review=True,
            fabrication_authorized=False,
        )

    def test_partial_native_batch_is_retained_without_inventing_seventh(self):
        harness.verify_preview(self.batch, self.planning, self.request)

    def test_pose_frame_source_or_authority_changes_rejected(self):
        for changes in (
            {"selected": {}},
            {"frame_id": "robot_alias"},
            {"scene_sha256": "changed"},
            {"proposal_id": "changed"},
            {"fabrication_authorized": True},
        ):
            with self.subTest(changes=changes), self.assertRaises(AssertionError):
                harness.verify_preview(
                    dict(self.batch, **changes), self.planning, self.request
                )

    def test_bad_count_and_false_partial_claim_rejected(self):
        for changes in (
            {"candidate_count": 0},
            {"candidate_count": 8},
            {"candidate_count": True},
            {"partial_batch": False},
        ):
            with self.subTest(changes=changes), self.assertRaises(AssertionError):
                harness.verify_preview(
                    dict(self.batch, **changes), self.planning, self.request
                )

    def test_software_approval_flag_must_be_explicit(self):
        with (
            patch.object(harness, "require_isolation"),
            self.assertRaises(AssertionError),
        ):
            harness.run(types.SimpleNamespace(synthetic_software_test_approvals=False))

    def test_process_type_is_separate_runtime_process(self):
        self.assertTrue(issubclass(harness.StartupProcess, harness.RuntimeProcess))
        source = (ROOT / "scripts/ros_initial_startup.py").read_text()
        self.assertIn("start_new_session=True", source.replace(" ", ""))
        self.assertIn("synthetic-fixture-software-test-operator-not-human", source)


if __name__ == "__main__":
    unittest.main()

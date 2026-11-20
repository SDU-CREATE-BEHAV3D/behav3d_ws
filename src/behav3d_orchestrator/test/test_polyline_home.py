"""Run the polyline orchestration with fake commands and no robot connection."""

from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import yaml
from geometry_msgs.msg import PoseStamped

from behav3d_orchestrator.polyline_motion_sequence import PolylineMotionSequenceNode


def run_sequence(*, execute_motion, home_ok=True):
    config = Path(__file__).parents[1] / "config/polyline_motion_sequence_config.yaml"
    parameters = yaml.safe_load(config.read_text())["polyline_motion_sequence"]["ros__parameters"]
    assert parameters["home_before"] is True
    parameters.update(execute_motion=execute_motion, orientation_mode="fixed", publish_markers=False)
    commands = []
    motion = SimpleNamespace()
    for kind in ("home", "goto", "plan", "exec", "setLIN", "setSpd", "setAcc", "setEef"):
        setattr(motion, kind, lambda *args, _kind=kind, **kwargs: _kind)
    motion._planned_jt = SimpleNamespace(
        joint_names=["joint"], points=[SimpleNamespace(positions=[0.0])]
    )

    def run_sync(command, **kwargs):
        commands.append(command)
        return {"ok": home_ok if command == "home" else True, "error": "home rejected"}

    poses = [PoseStamped(), PoseStamped()]
    session = SimpleNamespace(
        motion=motion,
        run_sync=run_sync,
        parse_yaml_polylines=lambda **kwargs: [SimpleNamespace(index=0, poses=poses)],
    )
    logger = Mock()
    node = SimpleNamespace(
        session=session,
        get_parameter=lambda name: SimpleNamespace(value=parameters[name]),
        get_logger=lambda: logger,
        _plan_virtual_polyline=Mock(return_value={"ok": True}),
    )
    with patch("behav3d_orchestrator.polyline_motion_sequence.rclpy.shutdown") as shutdown:
        PolylineMotionSequenceNode._run(node)
        shutdown.assert_called_once()
    return commands, logger, node


def test_home_completes_before_approach_and_execution():
    commands, logger, _ = run_sequence(execute_motion=True)
    assert commands.index("home") < commands.index("goto") < commands.index("exec")
    logger.error.assert_not_called()


def test_failed_home_aborts_before_approach_and_execution():
    commands, logger, _ = run_sequence(execute_motion=True, home_ok=False)
    assert commands == ["home"]
    assert "home_before failed" in logger.error.call_args.args[0]


def test_planned_preview_never_executes_home_or_motion():
    commands, logger, node = run_sequence(execute_motion=False)
    assert "home" not in commands
    assert "goto" not in commands
    assert "exec" not in commands
    assert "plan" in commands
    node._plan_virtual_polyline.assert_called_once()
    logger.error.assert_not_called()

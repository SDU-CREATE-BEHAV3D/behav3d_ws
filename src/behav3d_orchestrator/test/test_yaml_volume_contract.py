import pytest

from behav3d_orchestrator.src.yaml_session import YamlSession


def test_dot_target_volume_is_optional_and_preserves_index_order(tmp_path):
    path = tmp_path / "targets.yaml"
    path.write_text(
        "targets:\n"
        "  - index: 2\n"
        '    plane: "O(20,0,0) Z(0,0,1)"\n'
        "  - index: 1\n"
        '    plane: "O(10,0,0) Z(0,0,1)"\n'
        "    volume_mm3: 1234.5\n",
        encoding="utf-8",
    )
    session = object.__new__(YamlSession)

    targets = session.parse_yaml_dot_targets(yaml_path=str(path), frame_id="world")

    assert [target.index for target in targets] == [1, 2]
    assert targets[0].volume_mm3 == pytest.approx(1234.5)
    assert targets[1].volume_mm3 is None


def test_segment_volume_is_optional_and_preserves_index_order(tmp_path):
    path = tmp_path / "segments.yaml"
    path.write_text(
        "segments:\n"
        "  - index: 2\n"
        "    start:\n"
        '      plane: "O(20,0,0) Z(0,0,1)"\n'
        "    end:\n"
        '      plane: "O(20,0,10) Z(0,0,1)"\n'
        "  - index: 1\n"
        "    volume_mm3: 1234.5\n"
        "    start:\n"
        '      plane: "O(10,0,0) Z(0,0,1)"\n'
        "    end:\n"
        '      plane: "O(10,0,10) Z(0,0,1)"\n',
        encoding="utf-8",
    )
    session = object.__new__(YamlSession)

    segments = session.parse_yaml_segments(yaml_path=str(path), frame_id="world")

    assert [segment.index for segment in segments] == [1, 2]
    assert segments[0].volume_mm3 == pytest.approx(1234.5)
    assert segments[1].volume_mm3 is None

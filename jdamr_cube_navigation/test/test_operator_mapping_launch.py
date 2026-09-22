"""Static contract checks for unrestricted manual mapping."""

from pathlib import Path


LAUNCH = (
    Path(__file__).parents[1] / 'launch' / 'operator_mapping.launch.py'
).read_text(encoding='utf-8')


def test_manual_command_has_no_drive_gate():
    """Manual commands go to the base without preflight or obstacle gates."""
    assert "'output_topic': 'cmd_vel'" in LAUNCH
    assert "'require_preflight': False" in LAUNCH
    assert "executable='velocity_smoother'" not in LAUNCH
    assert "executable='collision_monitor'" not in LAUNCH
    assert "executable='operator_mapping_preflight'" not in LAUNCH


def test_mapping_launch_has_no_autonomous_explorer():
    """Operator mapping never starts autonomous frontier movement."""
    assert 'frontier_explorer' not in LAUNCH
    assert 'cartographer_real.launch.py' in LAUNCH


def test_map_saver_and_required_shutdown_are_present():
    """The map can be saved and a failed required node stops the stack."""
    assert "executable='map_saver_server'" in LAUNCH
    assert 'required operator-mapping process exited' in LAUNCH

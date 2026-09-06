"""Tests for the deterministic simulated SLAM experiment harness."""

from pathlib import Path
import subprocess
import sys

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'evaluation'))

from run_sim_slam_experiment import (  # noqa: E402,I100
    _stop,
    backend_command,
    classify_experiment_outcome,
    process_group_members,
    resolve_cartographer_config,
    summarize_resources,
    valid_run_label,
    validate_json_schema,
)


def test_experiment_outcome_separates_fail_from_invalid():
    """A valid completed evaluation may fail performance without invalidity."""
    assert classify_experiment_outcome(
        valid=True, completed=False, execution_ok=True) == (
            'FAIL', 'ground-truth completion gate failed')
    assert classify_experiment_outcome(
        valid=False, completed=False, execution_ok=True)[0] == 'INVALID'
    assert classify_experiment_outcome(
        valid=True, completed=True, execution_ok=True)[0] == 'PASS'


def test_cartographer_command_is_headless_and_uses_sim_time():
    """The experiment backend must not start RViz or wall-time mapping."""
    command = backend_command('cartographer', None, Path('/tmp/config'))

    assert command[:4] == [
        'ros2', 'run', 'cartographer_ros', 'cartographer_node']
    assert 'jdamr_cube_2d_real.lua' in command
    assert 'use_sim_time:=true' in command
    assert 'rviz2' not in command


def test_cartographer_config_resolves_absolute_path_or_safe_basename(
        tmp_path):
    """Generated configs split into a directory and slash-free basename."""
    installed = tmp_path / 'installed'
    installed.mkdir()
    packaged = installed / 'packaged.lua'
    generated = tmp_path / 'generated' / 'subdivision_2.lua'
    generated.parent.mkdir()
    packaged.write_text('return {}')
    generated.write_text('return {}')

    assert resolve_cartographer_config('packaged.lua', installed) == (
        installed.resolve(), 'packaged.lua')
    assert resolve_cartographer_config(generated.resolve(), installed) == (
        generated.parent.resolve(), 'subdivision_2.lua')
    with pytest.raises(ValueError, match='basename'):
        resolve_cartographer_config('../packaged.lua', installed)


def test_slam_toolbox_command_requires_explicit_parameters(tmp_path):
    """Every Toolbox result must remain attributable to one config file."""
    with pytest.raises(ValueError, match='requires'):
        backend_command('slam_toolbox', None)

    params = tmp_path / 'params.yaml'
    command = backend_command('slam_toolbox', params)

    assert f'slam_params_file:={params}' in command


def test_unknown_backend_is_rejected():
    """A typo must not silently select another mapping process."""
    with pytest.raises(ValueError, match='unsupported backend'):
        backend_command('unknown', None)


def test_resource_summary_retains_cpu_units_and_peak_rss():
    """Resource evidence must state multicore CPU semantics explicitly."""
    summary = summarize_resources([
        {'label': 'backend', 'cpu_pct_one_core': None, 'rss_mb': 10.0},
        {'label': 'backend', 'cpu_pct_one_core': 80.0, 'rss_mb': 12.0},
        {'label': 'backend', 'cpu_pct_one_core': 120.0, 'rss_mb': 11.0},
    ])

    assert 'exceed 100' in summary['scope']
    assert summary['by_process_group']['backend'][
        'cpu_mean_pct_one_core'] == 100.0
    assert summary['by_process_group']['backend']['rss_peak_mb'] == 12.0


def test_schema_validator_rejects_unknown_keyword_and_naive_datetime():
    """The local schema subset must fail closed instead of skipping rules."""
    with pytest.raises(ValueError, match='unsupported keywords'):
        validate_json_schema('x', {'type': 'string', 'maxLength': 2}, {})
    with pytest.raises(ValueError, match='unsupported keywords'):
        schema = {
            'type': 'object',
            'properties': {'optional': {'type': 'string', 'maxLength': 2}},
        }
        validate_json_schema({}, schema, schema)
    with pytest.raises(ValueError, match='timezone'):
        validate_json_schema(
            '2026-09-05T12:00:00',
            {'type': 'string', 'format': 'date-time'}, {})


def test_run_label_cannot_escape_the_output_root():
    """Run identifiers are single safe directory names."""
    assert valid_run_label('scan_dropout__cartographer__seed_42')
    assert not valid_run_label('../../outside')
    assert not valid_run_label('Uppercase')


def test_stop_cleans_orphaned_members_of_the_launched_group():
    """A launcher exit must not leave its child process in the ROS domain."""
    process = subprocess.Popen(
        ['/bin/sh', '-c', 'sleep 30 &'], start_new_session=True)
    process.wait(timeout=2.0)

    returncode = _stop(process, timeout_s=0.2)

    assert returncode == 0
    assert process_group_members(process.pid) == []

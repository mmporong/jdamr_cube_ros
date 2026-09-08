"""Exercise real child failures and bounded G005 diagnostic evidence."""

from pathlib import Path
import subprocess
import sys

from frontier_policy_contract import strict_json_load
import pytest
import run_frontier_policy_full as runner


@pytest.mark.parametrize('failure', ['timeout', 'coordinator_exit'])
def test_child_failure_preserves_bounded_log_and_teardown(
        tmp_path, monkeypatch, failure):
    command = [sys.executable, '-u', '-c', 'import time; time.sleep(60)']
    failing = [sys.executable, '-u', '-c',
               'print("x" * 200000); print("diagnostic end"); raise SystemExit(7)']
    monkeypatch.setattr(runner, '_runtime_commands', lambda *args: (
        command, command if failure == 'timeout' else failing))
    monkeypatch.setattr(runner, '_rehash_production_inputs', lambda value: value)
    monkeypatch.setattr(runner, 'LAUNCH_ALIVE_PROBE_DELAY_S', 0.01)
    request = {'run_id': 'current__seed_11', 'ros_domain_id': 170}
    expected = 'wall timeout' if failure == 'timeout' else 'exited with code 7'
    with pytest.raises(RuntimeError, match=expected):
        runner._execute_one_request(request, tmp_path, 0.3, {'source': {}})
    execution = strict_json_load(tmp_path / 'current__seed_11.execution.json')
    assert execution['survivor_count'] == 0
    assert execution['launch_exit_code'] is not None
    assert execution['coordinator_exit_code'] is not None
    log = tmp_path / 'current__seed_11.runtime.log'
    assert log.stat().st_size <= runner.RUNTIME_LOG_TAIL_BYTES + 64
    if failure == 'coordinator_exit':
        assert b'diagnostic end' in log.read_bytes()


@pytest.mark.parametrize('renamed', [False, True])
def test_failed_matrix_keeps_completed_work_and_no_valid_manifest(
        tmp_path, renamed):
    output = tmp_path / 'final'
    with pytest.raises(RuntimeError, match='failure at second run'):
        with runner._preserved_result_stage(
                output, '.g005-frontier-full-') as stage:
            (stage / 'first.runtime.json').write_text('{}', encoding='utf-8')
            (stage / 'manifest.json').write_text('{}', encoding='utf-8')
            if renamed:
                stage.rename(output)
            raise RuntimeError('failure at second run')
    assert not output.exists()
    failed, = tmp_path.glob('g005-frontier-full-*.failed')
    assert (failed / 'first.runtime.json').is_file()
    assert not (failed / 'manifest.json').exists()
    assert (failed / 'manifest.unvalidated.json').exists()
    assert strict_json_load(failed / 'failure.json')['status'] == 'INCOMPLETE'


@pytest.mark.parametrize('invalid', [False, True])
def test_diagnostic_runs_only_selected_case_and_never_claims_full_pass(
        tmp_path, monkeypatch, invalid):
    calls = []
    request = {'run_id': 'nearest__seed_23', 'policy': 'nearest',
               'layout_seed': 23}
    monkeypatch.setattr(runner, 'validate_assets',
                        lambda *args: {'production_inputs': {}})
    monkeypatch.setattr(runner, 'build_execution_plan', lambda *args: {
        'requests': [request]})

    def execute(selected, stage, timeout_s, production):
        calls.append((selected['run_id'], timeout_s))
        runner._write_atomic_json(stage / 'nearest__seed_23.execution.json', {
            'survivor_count': 0, 'production_inputs_unchanged': True})
        (stage / 'nearest__seed_23.runtime.log').write_text(
            ('G005 runtime invalid: sensor_failure\n' if invalid
             else 'actual diagnostic output'), encoding='utf-8')
        raise runner.RuntimeDeadlineExceeded('G005 runtime wall timeout')

    monkeypatch.setattr(runner, '_execute_one_request', execute)
    output = tmp_path / 'diagnostic'
    report = runner.run_diagnostic(
        tmp_path / 'assets', output, 170, 60.0, 'nearest', 23)
    assert calls == [('nearest__seed_23', 60.0)]
    assert report['status'] == (
        'RUNTIME_INVALID' if invalid else 'TIME_BUDGET_REACHED')
    assert report['full_matrix_complete'] is False
    assert report['production_change_authorized'] is False
    assert (output / 'diagnostic.json').is_file()
    assert not (output / 'manifest.json').exists()
    for record in report['files']:
        assert (output / record['path']).is_file()


def test_diagnostic_rejects_survivors_and_retains_evidence(tmp_path, monkeypatch):
    request = {'run_id': 'current__seed_11', 'policy': 'current',
               'layout_seed': 11}
    monkeypatch.setattr(runner, 'validate_assets',
                        lambda *args: {'production_inputs': {}})
    monkeypatch.setattr(runner, 'build_execution_plan', lambda *args: {
        'requests': [request]})

    def execute(selected, stage, *args):
        runner._write_atomic_json(stage / 'current__seed_11.execution.json', {
            'survivor_count': 1})
        raise runner.RuntimeDeadlineExceeded('G005 runtime wall timeout')

    monkeypatch.setattr(runner, '_execute_one_request', execute)
    with pytest.raises(RuntimeError, match='live processes'):
        runner.run_diagnostic(
            tmp_path / 'assets', tmp_path / 'result', 170, 60.0, 'current', 11)
    failed, = tmp_path.glob('g005-diagnostic-*.failed')
    assert (failed / 'failure.json').is_file()


@pytest.mark.parametrize('budget', [0.0, -1.0, float('nan'), float('inf'), 901.0])
def test_diagnostic_rejects_invalid_budget_before_launch(tmp_path, budget):
    with pytest.raises(ValueError, match='budget'):
        runner.run_diagnostic(
            Path('/absent'), tmp_path / 'result', 170, budget, 'current', 11)


def test_case_selection_without_diagnostic_cannot_start_full_matrix(tmp_path):
    completed = subprocess.run([
        sys.executable, str(Path(runner.__file__)),
        '--asset-root', str(tmp_path / 'assets'),
        '--output-root', str(tmp_path / 'output'), '--policy', 'nearest'],
        capture_output=True, text=True, check=False)
    assert completed.returncode == 2
    assert 'case selection requires --diagnostic' in completed.stderr
    assert not (tmp_path / 'output').exists()

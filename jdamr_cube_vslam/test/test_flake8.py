"""Run Python style checks as part of colcon test."""

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """Reject Python style regressions in source and tests."""
    return_code, errors = main_with_errors(
        argv=['--exclude', 'build', 'install', 'log'])
    assert return_code == 0, (
        f'Found {len(errors)} code style errors or warnings:\n'
        + '\n'.join(errors))

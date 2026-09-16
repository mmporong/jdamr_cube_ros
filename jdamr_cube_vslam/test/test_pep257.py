"""Run docstring checks as part of colcon test."""

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    """Reject missing or malformed public docstrings."""
    assert main(argv=['.']) == 0

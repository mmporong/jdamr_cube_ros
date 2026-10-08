"""Shared import setup for this package's pytest and colcon test runs."""

from pathlib import Path
import sys


TEST_ROOT = Path(__file__).resolve().parent
PACKAGE_ROOT = TEST_ROOT.parent

# Tests exercise the source package and the evaluation-only modules. The
# evaluation directory is installed as package data, not as an importable
# package, so it must be on sys.path before any test module is collected.
# The resulting order matches PYTHONPATH=<package>:<package>/evaluation; the
# test directory itself provides the sealed_inputs helper.
for _path in (TEST_ROOT, PACKAGE_ROOT / 'evaluation', PACKAGE_ROOT):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

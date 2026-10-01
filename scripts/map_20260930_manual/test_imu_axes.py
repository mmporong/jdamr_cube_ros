"""imu_axes.analyse on a synthetic board mounted like the robot's (z down, y fwd, x left).

    python3 -m pytest -q scripts/map_20260930_manual/test_imu_axes.py
"""
from pathlib import Path
import sys

import numpy as np
import pytest

pytest.importorskip('rosbag2_py')
sys.path.insert(0, str(Path(__file__).resolve().parent))
import imu_axes  # noqa: E402


def test_board_axes_are_recovered():
    t = np.arange(0.0, 120.0, 0.02)
    v = 0.06 * (1 - np.cos(2 * np.pi * t / 7.0))         # 0..0.12 m/s
    w = 0.4 * np.sin(2 * np.pi * t / 11.0)                # left positive
    a_fwd = np.gradient(v, t)
    a_lat = v * w                                         # toward base +y
    odom = np.column_stack([t, v, w])
    # Board x = base y (left), board y = base x (forward), board z = -base z.
    imu = np.column_stack([t, a_lat + 0.3, a_fwd - 0.4, np.full_like(t, -9.4),
                           np.zeros_like(t), np.zeros_like(t), -w])
    out, rows = imu_axes.analyse(odom, imu)
    assert out['gyro_z_per_wheel_yaw'] == pytest.approx(-1.0, abs=0.01)
    fx, fy = out['fit']['board_x']['base_xy'], out['fit']['board_y']['base_xy']
    assert fx[0] == pytest.approx(0.0, abs=0.1) and fx[1] == pytest.approx(1.0, abs=0.1)
    assert fy[0] == pytest.approx(1.0, abs=0.1) and fy[1] == pytest.approx(0.0, abs=0.1)

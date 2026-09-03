"""Regression tests for the offline SLAM comparison maths."""

# Both defects these tests pin were found by disbelieving an output rather
# than by reading the code: a 23.6 m drive reported 304 m of path length
# because 50 Hz odometry noise was summed pair by pair, and a 60 s replay
# reported a 19.7 m "error" against AMCL because the backend's map frame
# starts at the robot rather than at the pre-built map's origin.

import math
from pathlib import Path
import sys

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'evaluation'))

pytest.importorskip('mcap_ros2',
                    reason='evaluation deps are installed separately')

from compare_slam_runs import (  # noqa: E402,I100
    apply_rigid,
    compose,
    decimate,
    path_length,
    rigid_align,
    yaw_of,
)


class _Quaternion:
    def __init__(self, z, w):
        self.x = 0.0
        self.y = 0.0
        self.z = z
        self.w = w


def _straight_line(metres, step, noise=0.0):
    points = []
    distance = 0.0
    index = 0
    while distance <= metres:
        offset = noise if index % 2 else -noise
        points.append((float(index), distance, offset, 0.0))
        distance += step
        index += 1
    return points


def test_path_length_bounds_high_rate_sensor_noise():
    """Bound the noise inflation and state the bound explicitly."""
    # Summing every consecutive pair of the noisy stream gives over 40 m for a
    # 10 m line.  Decimation does not remove the inflation, it bounds it to
    # the lateral noise carried across each retained step, so the contract is
    # "within about ten percent", not "exact".
    clean = _straight_line(10.0, 0.004)
    noisy = _straight_line(10.0, 0.004, noise=0.01)
    naive = sum(math.dist(a[1:3], b[1:3])
                for a, b in zip(noisy, noisy[1:]))

    assert path_length(clean) == pytest.approx(10.0, abs=0.1)
    assert naive > 40.0
    assert path_length(noisy) < 0.25 * naive
    assert path_length(noisy) == pytest.approx(10.0, rel=0.10)


def test_decimate_keeps_the_endpoints():
    """Dropping the last sample would shorten every measured trajectory."""
    points = _straight_line(1.0, 0.004)
    kept = decimate(points, 0.05)

    assert kept[0] == points[0]
    assert kept[-1] == points[-1]
    assert len(kept) < len(points)


def test_rigid_align_recovers_a_known_frame_offset():
    """The alignment must remove the frame offset, not the estimate error."""
    theta = math.radians(30.0)
    source = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (2.0, 1.0)]
    target = [(math.cos(theta) * x - math.sin(theta) * y + 5.0,
               math.sin(theta) * x + math.cos(theta) * y - 3.0)
              for x, y in source]

    transform = rigid_align(source, target)

    assert transform is not None
    assert math.degrees(transform[0]) == pytest.approx(30.0, abs=0.01)
    for point, expected in zip(source, target):
        moved = apply_rigid(transform, *point)
        assert math.dist(moved, expected) < 1e-6


def test_rigid_align_never_rescales_the_trajectory():
    """Scaling would hide a backend that under- or over-estimates distance."""
    source = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    target = [(0.0, 0.0), (2.0, 0.0), (4.0, 0.0)]

    transform = rigid_align(source, target)
    moved = [apply_rigid(transform, *point) for point in source]

    assert math.dist(moved[0], moved[-1]) == pytest.approx(2.0, abs=1e-6)


def test_compose_matches_a_hand_computed_transform():
    """map->odom composed with odom->base must rotate the child offset."""
    composed = compose((1.0, 2.0, math.radians(90.0)), (3.0, 0.0, 0.0))

    assert composed[0] == pytest.approx(1.0, abs=1e-9)
    assert composed[1] == pytest.approx(5.0, abs=1e-9)


def test_yaw_of_reads_a_quarter_turn():
    """A quaternion at 90 degrees must not read as zero."""
    assert math.degrees(
        yaw_of(_Quaternion(math.sin(math.pi / 4), math.cos(math.pi / 4)))
    ) == pytest.approx(90.0, abs=1e-6)

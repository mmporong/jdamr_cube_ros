"""Regression tests for read-only live alignment sample decoding."""

import importlib.util
import itertools
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


SCRIPT = (Path(__file__).resolve().parents[1]
          / 'evaluation' / 'capture_depth_alignment.py')
SPEC = importlib.util.spec_from_file_location('alignment_capture_test', SCRIPT)
CAPTURE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CAPTURE)


@pytest.mark.parametrize('big_endian', [False, True])
def test_padded_depth_preserves_metres_input_units(big_endian):
    """Read padded rows and preserve original uint16 millimetre values."""
    dtype = '>u2' if big_endian else '<u2'
    values = np.array([[400, 1250], [2500, 0]], dtype=dtype)
    data = b''.join(row.tobytes() + b'\xff\xff' for row in values)
    message = SimpleNamespace(encoding='16UC1', width=2, height=2,
                              step=6, is_bigendian=big_endian, data=data)
    decoded = CAPTURE.image_array(message)
    np.testing.assert_array_equal(decoded, values)
    assert decoded.dtype == np.uint16
    assert decoded.flags.c_contiguous


@pytest.mark.parametrize('encoding,pixel', [
    ('rgb8', [1, 2, 3]), ('bgr8', [3, 2, 1]),
    ('rgba8', [1, 2, 3, 255]), ('bgra8', [3, 2, 1, 255]),
])
def test_rgb_channel_order_and_row_padding(encoding, pixel):
    """Normalize all supported encodings to three-channel RGB."""
    message = SimpleNamespace(encoding=encoding, width=1, height=2,
                              step=len(pixel)+2, is_bigendian=False,
                              data=bytes(pixel+[99, 98])*2)
    result = CAPTURE.image_array(message, channels=3)
    np.testing.assert_array_equal(result, [[[1, 2, 3]], [[1, 2, 3]]])


@pytest.mark.parametrize('patch', [
    {'width': 0}, {'height': 0}, {'step': 1}, {'data': b'\x00'},
    {'encoding': '32FC1'},
])
def test_invalid_depth_buffers_are_rejected(patch):
    """Do not reinterpret malformed or differently scaled depth images."""
    fields = {'encoding': '16UC1', 'width': 1, 'height': 1, 'step': 2,
              'is_bigendian': False, 'data': b'\x01\x00'}
    fields.update(patch)
    with pytest.raises(ValueError):
        CAPTURE.image_array(SimpleNamespace(**fields))


@pytest.mark.parametrize('duration', ['0', '-1', 'nan', 'inf'])
def test_capture_duration_must_be_finite_and_positive(duration, tmp_path):
    """Every capture is bounded rather than an accidental infinite run."""
    with pytest.raises(SystemExit):
        CAPTURE.parse_args(['--output-dir', str(tmp_path),
                            '--duration', duration])


def test_stamp_nanoseconds_do_not_round_large_seconds():
    """Keep exact integer time for nearest-scan matching."""
    message = SimpleNamespace(header=SimpleNamespace(
        stamp=SimpleNamespace(sec=1789954059, nanosec=924560894)))
    assert CAPTURE.stamp_ns(message) == 1789954059924560894


def stamped(seconds):
    """Construct exact millisecond test stamps without ROS node startup."""
    millis = round(seconds * 1000)
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(
        sec=millis // 1000, nanosec=(millis % 1000) * 1_000_000)))


def test_latest_depth_without_scan_uses_recent_valid_pair():
    """Account for unequal sensor rates without relaxing the time limit."""
    older, newest, scan = stamped(9.85), stamped(9.99), stamped(9.85)
    result = CAPTURE.select_depth_scan_pair([older, newest], [scan], 10**10)
    assert result == (older, scan)


def test_pair_selection_uses_stamp_order_not_arrival_order():
    """An out-of-order DDS callback cannot replace a newer valid sample."""
    older, newer, scan = stamped(9.8), stamped(9.95), stamped(9.96)
    result = CAPTURE.select_depth_scan_pair([newer, older], [scan], 10**10)
    assert result == (newer, scan)


@pytest.mark.parametrize('depth,scan', [
    (9.49, 9.49), (9.99, 9.49), (10.01, 10.01),
    (0, 0), (9.99, 9.88),
])
def test_stale_future_zero_or_unpaired_samples_are_rejected(depth, scan):
    """Fail instead of producing a stale or loosely synchronized snapshot."""
    with pytest.raises(ValueError):
        CAPTURE.select_depth_scan_pair(
            [stamped(depth)], [stamped(scan)], 10**10)


@pytest.mark.parametrize('failures', [0, 1, 4])
def test_capture_reports_tf_retry_evidence(monkeypatch, tmp_path, failures):
    """A recovered TF lookup must not masquerade as first-attempt success."""
    calls = []
    destroyed = []

    def capture():
        calls.append(True)
        if len(calls) <= failures:
            raise ValueError('transform_lookup_failed: delayed TF')
        return {'example': np.array([1])}, {'pair_delta_s': 0.01}

    node = SimpleNamespace(capture=capture, diagnostics=lambda: {},
                           destroy_node=lambda: destroyed.append(True))
    monkeypatch.setattr(CAPTURE, 'AlignmentCapture', lambda: node)
    monkeypatch.setattr(CAPTURE.rclpy, 'init', lambda **kwargs: None)
    monkeypatch.setattr(CAPTURE.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(CAPTURE.rclpy, 'shutdown', lambda: None)
    monkeypatch.setattr(CAPTURE.rclpy, 'spin_once', lambda *a, **kw: None)
    ticks = itertools.count()
    monkeypatch.setattr(CAPTURE.time, 'monotonic', lambda: next(ticks))
    output = tmp_path / 'capture'
    result = CAPTURE.main(['--output-dir', str(output), '--duration', '1'])
    report = json.loads((output / 'capture.json').read_text())
    assert result == (1 if failures == 4 else 0)
    assert report['capture_attempts'] == min(failures+1, 4)
    assert len(report['tf_lookup_retry_errors']) == min(failures, 3)
    assert report['success'] == (failures < 4)
    assert report['success_scope'] == 'snapshot_only_not_sensor_alignment_approval'
    assert destroyed == [True]

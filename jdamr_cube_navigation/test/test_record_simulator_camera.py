"""Tests for simulator camera timestamp provenance."""

import importlib.util
import json
from pathlib import Path


EVALUATION = Path(__file__).resolve().parents[1] / 'evaluation'
SPEC = importlib.util.spec_from_file_location(
    'record_simulator_camera', EVALUATION / 'record_simulator_camera.py')
RECORDER_MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RECORDER_MODULE)


class _Bridge:
    def imgmsg_to_cv2(self, _message, desired_encoding):
        assert desired_encoding == 'bgr8'
        return type('Frame', (), {'shape': (720, 1280, 3)})()


class _Writer:
    def __init__(self):
        self.frames = []
        self.released = False

    def write(self, frame):
        self.frames.append(frame)

    def release(self):
        self.released = True


def _message(sec, nanosec):
    """Build the timestamp fields used by the recorder callback."""
    stamp = type('Stamp', (), {'sec': sec, 'nanosec': nanosec})()
    header = type('Header', (), {'stamp': stamp})()
    return type('Image', (), {'header': header, 'encoding': 'rgb8'})()


def _recorder(tmp_path):
    """Build a recorder without starting an ROS graph node."""
    recorder = RECORDER_MODULE.CameraRecorder.__new__(
        RECORDER_MODULE.CameraRecorder)
    recorder.topic = '/portfolio/camera/image_raw'
    recorder.output = tmp_path / 'capture.mp4'
    recorder.metadata = tmp_path / 'capture.json'
    recorder.ready_file = tmp_path / 'ready'
    recorder.fps = 30.0
    recorder.bridge = _Bridge()
    recorder.writer = None
    recorder.frames = 0
    recorder.width = 1280
    recorder.height = 720
    recorder.source_encoding = 'rgb8'
    recorder.first_frame_steady_ns = None
    recorder.last_frame_steady_ns = None
    recorder.first_frame_wall_ns = None
    recorder.last_frame_wall_ns = None
    recorder.frame_timestamps = []
    return recorder


def test_records_callback_and_sim_time_for_each_written_frame(
        tmp_path, monkeypatch):
    """Persist one indexed timestamp record for every encoded frame."""
    recorder = _recorder(tmp_path)
    writer = _Writer()
    monkeypatch.setattr(
        RECORDER_MODULE.cv2, 'VideoWriter', lambda *_args: writer)
    writer.isOpened = lambda: True
    steady_times = iter((10_000, 20_000))
    wall_times = iter((30_000, 40_000))
    monkeypatch.setattr(
        RECORDER_MODULE.time, 'monotonic_ns', lambda: next(steady_times))
    monkeypatch.setattr(
        RECORDER_MODULE.time, 'time_ns', lambda: next(wall_times))

    recorder._on_image(_message(7, 123))
    recorder._on_image(_message(8, 456))
    recorder.finalize()

    document = json.loads(recorder.metadata.read_text(encoding='utf-8'))
    assert document['schema_version'] == 2
    assert document['frames'] == 2
    assert document['first_frame_steady_ns'] == 10_000
    assert document['last_frame_steady_ns'] == 20_000
    assert document['first_frame_wall_ns'] == 30_000
    assert document['last_frame_wall_ns'] == 40_000
    assert document['frame_timestamps'] == [
        {'index': 0, 'steady_ns': 10_000, 'wall_ns': 30_000,
         'sim_ns': 7_000_000_123},
        {'index': 1, 'steady_ns': 20_000, 'wall_ns': 40_000,
         'sim_ns': 8_000_000_456},
    ]
    assert writer.released is True

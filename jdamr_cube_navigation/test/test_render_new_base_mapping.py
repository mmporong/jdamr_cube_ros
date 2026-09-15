"""Offline mapping media must be rooted in actual ROS 2 map records."""

import hashlib
import json
from pathlib import Path
import sys

from mcap.writer import Writer
from nav_msgs.msg import OccupancyGrid
import pytest
from rclpy.serialization import serialize_message
from tf2_msgs.msg import TFMessage


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'evaluation'))

from render_new_base_mapping import (  # noqa: E402,I100,I201
    _map_image,
    _read_maps,
    _world_bounds,
    render,
)


def _map(width, height, cells, origin_x=0.0):
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.resolution = 0.1
    grid.info.width = width
    grid.info.height = height
    grid.info.origin.position.x = origin_x
    grid.info.origin.orientation.w = 1.0
    grid.data = cells
    return grid


def _write_mcap(path, maps):
    with path.open('wb') as stream:
        writer = Writer(stream)
        writer.start(profile='ros2')
        map_schema = writer.register_schema(
            'nav_msgs/msg/OccupancyGrid', '', b'')
        map_channel = writer.register_channel('/map', 'cdr', map_schema)
        tf_schema = writer.register_schema('tf2_msgs/msg/TFMessage', '', b'')
        tf_channel = writer.register_channel('/tf', 'cdr', tf_schema)
        for index, grid in enumerate(maps):
            time_ns = (index + 1) * 1_000_000_000
            writer.add_message(map_channel, time_ns,
                               bytes(serialize_message(grid)), time_ns)
        writer.add_message(tf_channel, 4_000_000_000,
                           bytes(serialize_message(TFMessage())),
                           4_000_000_000)
        writer.finish()


def test_map_pixels_and_provenance_come_from_changed_mcap_snapshots(tmp_path):
    """Render a tiny real CDR MCAP without implying camera footage."""
    mcap = tmp_path / 'output.mcap'
    _write_mcap(mcap, [
        _map(2, 2, [-1, 0, 100, -1]),
        _map(2, 2, [-1, 0, 100, -1]),
        _map(3, 2, [-1, 0, 100, 0, 100, 0], origin_x=-0.1),
    ])
    bag = tmp_path / 'source'
    bag.mkdir()
    (bag / 'input.mcap').write_bytes(b'original input sample')
    log = tmp_path / 'run.log'
    log.write_text('offline replay completed\n', encoding='utf-8')
    out = tmp_path / 'media'

    manifest = render(mcap, bag, out, log, fps=2, duration_s=1.5)

    assert manifest['observed_topics'] == {
        '/map_changed_snapshots': 2, '/tf_messages': 1}
    assert manifest['map']['last_recorded_known_cells'] == 5
    assert manifest['map']['last_recorded_width_cells'] == 3
    assert manifest['map']['resolution_m_per_cell'] == pytest.approx(0.1)
    assert manifest['timing']['frame_times'][0]['known_cells'] == 2
    assert manifest['timing']['frame_times'][-1]['known_cells'] == 5
    assert manifest['not_claimed'] == [
        'physical camera video', '3D map',
        'automatic human/obstacle classification']
    assert len(manifest['inputs']['source_bag']) == 1
    assert manifest['inputs']['source_bag'][0]['sha256'] == hashlib.sha256(
        b'original input sample').hexdigest()
    for output in manifest['outputs'].values():
        path = Path(output['path'])
        assert path.is_file() and path.stat().st_size > 0
        assert output['sha256'] == hashlib.sha256(path.read_bytes()).hexdigest()
    sidecar = json.loads((out / 'new_base_mapping_provenance.json').read_text())
    assert sidecar == manifest


def test_world_alignment_and_occupancy_colors(tmp_path):
    """Expanded maps retain their measured origin and +y orientation."""
    mcap = tmp_path / 'maps.mcap'
    _write_mcap(mcap, [
        _map(2, 2, [-1, 0, 100, -1]),
        _map(3, 2, [-1, 0, 100, 0, 100, 0], origin_x=-0.1),
    ])
    samples, _ = _read_maps(mcap, 10)
    bounds = _world_bounds(samples)
    image = _map_image(samples[0], bounds)
    assert image.size == (3, 2)
    assert image.getpixel((1, 0)) == 30
    assert image.getpixel((2, 0)) == 88
    assert image.getpixel((1, 1)) == 88
    assert image.getpixel((2, 1)) == 224


def test_missing_map_or_reused_source_is_rejected_before_output(tmp_path):
    """Do not manufacture mapping evidence from a missing or same source."""
    out = tmp_path / 'out'
    with pytest.raises(FileNotFoundError):
        render(tmp_path / 'none.mcap', tmp_path / 'source', out)
    assert not out.exists()
    mcap = tmp_path / 'only.mcap'
    _write_mcap(mcap, [_map(1, 1, [0])])
    with pytest.raises(ValueError, match='must differ'):
        render(mcap, mcap, out)
    assert not out.exists()


def test_non_map_frame_is_rejected(tmp_path):
    """Avoid labeling an arbitrary occupancy grid as the SLAM map."""
    mcap = tmp_path / 'wrong.mcap'
    grid = _map(1, 1, [0])
    grid.header.frame_id = 'odom'
    _write_mcap(mcap, [grid])
    with pytest.raises(ValueError, match='map frame'):
        _read_maps(mcap, 10)

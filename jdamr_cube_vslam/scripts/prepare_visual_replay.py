#!/usr/bin/env python3
"""Keep camera-local TF for visual replay, leaving the input bag untouched."""

import argparse
from collections import Counter
import json
from pathlib import Path

from rclpy.serialization import deserialize_message, serialize_message

import rosbag2_py

from tf2_msgs.msg import TFMessage


ALLOWED_TOPICS = {
    '/camera/color/image_raw', '/camera/depth/image_raw',
    '/camera/color/camera_info', '/camera/depth/camera_info',
    '/tf', '/tf_static', '/odom',
}


def camera_descendants(edges, root='camera_link'):
    """Find camera descendants without retaining any incoming root edge."""
    frames = {root}
    while True:
        found = {child for parent, child in edges if parent in frames}
        if found <= frames:
            return frames
        frames.update(found)


def keep_camera_edges(message, frames, root='camera_link'):
    """Remove wheel/base transforms and the recorded camera mount edge."""
    return TFMessage(transforms=[
        edge for edge in message.transforms
        if edge.header.frame_id in frames
        and edge.child_frame_id in frames
        and edge.child_frame_id != root])


def open_reader(path):
    """Open an MCAP bag without publishing ROS topics."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    return reader


def prepare(source, output):
    """Write a separate camera-only TF replay and its filtering provenance."""
    if output.exists():
        raise FileExistsError(output)
    reader = open_reader(source)
    topics = reader.get_all_topics_and_types()
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/tf', '/tf_static']))
    edges = set()
    while reader.has_next():
        _, raw, _ = reader.read_next()
        for edge in deserialize_message(raw, TFMessage).transforms:
            edges.add((edge.header.frame_id, edge.child_frame_id))
    frames = camera_descendants(edges)
    if len(frames) < 2:
        raise ValueError('camera-local TF tree is missing')
    reader = open_reader(source)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(
        uri=str(output), storage_id='mcap',
        storage_preset_profile='zstd_fast'),
        rosbag2_py.ConverterOptions('', ''))
    for topic in topics:
        if topic.name in ALLOWED_TOPICS:
            writer.create_topic(topic)
    counts = Counter()
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        if topic not in ALLOWED_TOPICS:
            continue
        if topic in {'/tf', '/tf_static'}:
            message = keep_camera_edges(
                deserialize_message(raw, TFMessage), frames)
            if not message.transforms:
                continue
            raw = serialize_message(message)
        writer.write(topic, raw, stamp)
        counts[topic] += 1
    del writer
    summary = {
        'source': str(source), 'output': str(output),
        'camera_frames': sorted(frames),
        'removed_tf_edges': sorted([list(edge) for edge in edges
                                    if edge[0] not in frames
                                    or edge[1] not in frames
                                    or edge[1] == 'camera_link']),
        'written_counts': dict(counts),
        'wheel_odom_role': 'reference only; visual uses /rtabmap/odom',
        'command_topics_replayed': False,
    }
    (output / 'visual_replay_summary.json').write_text(
        json.dumps(summary, indent=2) + '\n')
    return summary


def main():
    """Prepare one non-overwriting, offline visual replay input."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(prepare(args.input, args.output), indent=2))


if __name__ == '__main__':
    main()

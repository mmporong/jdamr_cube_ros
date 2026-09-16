"""Create a synchronized processing bag from an original RGB-D recording."""

import argparse
from bisect import bisect_left
from collections import Counter
from dataclasses import dataclass
import json
from pathlib import Path

from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import Image


COLOR_TOPIC = '/camera/color/image_raw'
DEPTH_TOPIC = '/camera/depth/image_raw'
RGBD_IMAGE_TOPICS = {COLOR_TOPIC, DEPTH_TOPIC}


@dataclass(frozen=True)
class ImageFrame:
    """Identify one image message by topic ordinal and sensor timestamp."""

    ordinal: int
    stamp_ns: int


def select_synchronized_frames(
        color_frames: list[ImageFrame], depth_frames: list[ImageFrame],
        image_fps: float, max_pair_delta_ms: float
        ) -> tuple[set[int], set[int], list[int]]:
    """Select rate-limited one-to-one RGB-D pairs using sensor header time."""
    if image_fps <= 0.0:
        raise ValueError('image_fps must be positive')
    if max_pair_delta_ms <= 0.0:
        raise ValueError('max_pair_delta_ms must be positive')
    color_frames = sorted(color_frames, key=lambda frame: frame.stamp_ns)
    depth_frames = sorted(depth_frames, key=lambda frame: frame.stamp_ns)
    minimum_interval_ns = round(1e9 / image_fps)
    maximum_delta_ns = round(max_pair_delta_ms * 1e6)
    depth_stamps_ns = [frame.stamp_ns for frame in depth_frames]
    selected_color = set()
    selected_depth = set()
    pair_deltas_ns = []
    used_depth_ordinals = set()
    last_color_stamp_ns = None

    for color in color_frames:
        if last_color_stamp_ns is not None \
                and color.stamp_ns - last_color_stamp_ns \
                < minimum_interval_ns:
            continue
        insertion = bisect_left(depth_stamps_ns, color.stamp_ns)
        nearby = range(max(0, insertion - 2),
                       min(len(depth_frames), insertion + 3))
        candidates = [
            depth_frames[index] for index in nearby
            if depth_frames[index].ordinal not in used_depth_ordinals
        ]
        if not candidates:
            continue
        depth = min(
            candidates,
            key=lambda frame: abs(frame.stamp_ns - color.stamp_ns))
        delta_ns = abs(depth.stamp_ns - color.stamp_ns)
        if delta_ns > maximum_delta_ns:
            continue
        selected_color.add(color.ordinal)
        selected_depth.add(depth.ordinal)
        used_depth_ordinals.add(depth.ordinal)
        pair_deltas_ns.append(delta_ns)
        last_color_stamp_ns = color.stamp_ns

    return selected_color, selected_depth, pair_deltas_ns


def _open_reader(input_bag: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(input_bag), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''))
    return reader


def _scan_images(input_bag: Path):
    reader = _open_reader(input_bag)
    topic_metadata = reader.get_all_topics_and_types()
    read_counts = Counter()
    frames = {COLOR_TOPIC: [], DEPTH_TOPIC: []}
    while reader.has_next():
        topic, serialized_message, _ = reader.read_next()
        ordinal = read_counts[topic]
        read_counts[topic] += 1
        if topic in RGBD_IMAGE_TOPICS:
            message = deserialize_message(serialized_message, Image)
            stamp = message.header.stamp
            frames[topic].append(ImageFrame(
                ordinal=ordinal,
                stamp_ns=stamp.sec * 1_000_000_000 + stamp.nanosec,
            ))
    return topic_metadata, read_counts, frames


def downsample_bag(input_bag: Path, output_bag: Path, image_fps: float,
                   max_pair_delta_ms: float) -> dict:
    """Copy all non-image data and synchronized, rate-limited RGB-D pairs."""
    if not input_bag.is_dir():
        raise FileNotFoundError(f'input bag not found: {input_bag}')
    if output_bag.exists():
        raise FileExistsError(f'output path already exists: {output_bag}')

    topic_metadata, read_counts, frames = _scan_images(input_bag)
    selected_color, selected_depth, pair_deltas_ns = \
        select_synchronized_frames(
            frames[COLOR_TOPIC], frames[DEPTH_TOPIC],
            image_fps, max_pair_delta_ms)
    if not pair_deltas_ns:
        raise RuntimeError('no synchronized RGB-D pairs found')
    selected_ordinals = {
        COLOR_TOPIC: selected_color,
        DEPTH_TOPIC: selected_depth,
    }

    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = _open_reader(input_bag)
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(
            uri=str(output_bag),
            storage_id='mcap',
            storage_preset_profile='zstd_fast'),
        converter_options)
    for metadata in topic_metadata:
        writer.create_topic(metadata)

    message_ordinals = Counter()
    written_counts = Counter()
    while reader.has_next():
        topic, serialized_message, timestamp_ns = reader.read_next()
        ordinal = message_ordinals[topic]
        message_ordinals[topic] += 1
        if topic in RGBD_IMAGE_TOPICS \
                and ordinal not in selected_ordinals[topic]:
            continue
        writer.write(topic, serialized_message, timestamp_ns)
        written_counts[topic] += 1
    del writer

    ordered_deltas_ns = sorted(pair_deltas_ns)
    p95_index = round(0.95 * (len(ordered_deltas_ns) - 1))
    summary = {
        'input_bag': str(input_bag),
        'output_bag': str(output_bag),
        'image_fps': image_fps,
        'max_pair_delta_ms': max_pair_delta_ms,
        'synchronized_pair_count': len(pair_deltas_ns),
        'pair_delta_mean_ms': (
            sum(pair_deltas_ns) / len(pair_deltas_ns) / 1e6),
        'pair_delta_p95_ms': ordered_deltas_ns[p95_index] / 1e6,
        'pair_delta_max_ms': ordered_deltas_ns[-1] / 1e6,
        'read_counts': dict(sorted(read_counts.items())),
        'written_counts': dict(sorted(written_counts.items())),
    }
    (output_bag / 'downsample_summary.json').write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    return summary


def _parse_args(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--image-fps', type=float, default=10.0)
    parser.add_argument('--max-pair-delta-ms', type=float, default=30.0)
    return parser.parse_args(argv)


def main(args=None):
    """Run the sensor-timestamp RGB-D bag processor."""
    parsed = _parse_args(args)
    summary = downsample_bag(
        parsed.input, parsed.output, parsed.image_fps,
        parsed.max_pair_delta_ms)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()

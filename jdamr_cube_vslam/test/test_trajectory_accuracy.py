"""Regression tests for metric distance and yaw evaluation."""

import csv
import json
import math
from pathlib import Path

from jdamr_cube_vslam.trajectory_accuracy import (
    associate_trajectories,
    evaluate_controlled_segments,
    evaluate_pairs,
    main,
    Pose,
    read_trajectory,
)
import pytest
import yaml


def _transform(reference: list[Pose], scale: float, yaw_offset_rad: float,
               translation_x_m: float, translation_y_m: float) -> list[Pose]:
    cosine = math.cos(yaw_offset_rad)
    sine = math.sin(yaw_offset_rad)
    result = []
    for pose in reference:
        result.append(Pose(
            timestamp_s=pose.timestamp_s,
            x_m=(scale * (cosine * pose.x_m - sine * pose.y_m)
                 + translation_x_m),
            y_m=(scale * (sine * pose.x_m + cosine * pose.y_m)
                 + translation_y_m),
            yaw_rad=pose.yaw_rad + yaw_offset_rad,
        ))
    return result


def _write_csv(path: Path, poses: list[Pose]) -> None:
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=[
            'timestamp_s', 'x_m', 'y_m', 'z_m',
            'qx', 'qy', 'qz', 'qw', 'frame_id', 'child_frame_id',
        ])
        writer.writeheader()
        for pose in poses:
            writer.writerow({
                'timestamp_s': pose.timestamp_s,
                'x_m': pose.x_m,
                'y_m': pose.y_m,
                'z_m': 0.0,
                'qx': 0.0,
                'qy': 0.0,
                'qz': math.sin(pose.yaw_rad / 2.0),
                'qw': math.cos(pose.yaw_rad / 2.0),
                'frame_id': 'odom',
                'child_frame_id': 'camera_link',
            })


def test_scale_error_is_not_removed_by_alignment():
    reference = [
        Pose(float(index), float(index), 0.0, 0.0)
        for index in range(11)
    ]
    visual = _transform(
        reference,
        scale=1.1,
        yaw_offset_rad=math.radians(20.0),
        translation_x_m=3.0,
        translation_y_m=-2.0,
    )
    metrics = evaluate_pairs(list(zip(visual, reference)))
    assert metrics['distance_scale_error_percent'] == pytest.approx(10.0)
    assert metrics['yaw_rmse_deg'] == pytest.approx(0.0, abs=1e-9)
    assert metrics['translation_ate_rmse_m'] > 0.0


def test_timestamp_association_rejects_distant_samples():
    visual = [Pose(0.0, 0.0, 0.0, 0.0), Pose(1.0, 1.0, 0.0, 0.0)]
    reference = [Pose(0.02, 0.0, 0.0, 0.0), Pose(2.0, 2.0, 0.0, 0.0)]
    pairs = associate_trajectories(visual, reference, max_time_delta_s=0.05)
    assert len(pairs) == 1
    assert pairs[0][0].timestamp_s == 0.0


def test_controlled_distance_and_yaw_segments(tmp_path):
    visual = [
        Pose(0.0, 0.0, 0.0, 0.0),
        Pose(1.0, 1.0, 0.0, 0.0),
        Pose(2.0, 2.0, 0.0, math.pi / 2.0),
    ]
    protocol = {
        'controlled_measurements': {
            'segments': [{
                'name': 'measured_move',
                'start_sec': 0.0,
                'end_sec': 2.0,
                'expected_distance_m': 2.0,
                'expected_yaw_deg': 90.0,
            }],
        },
    }
    protocol_path = tmp_path / 'protocol.yaml'
    protocol_path.write_text(yaml.safe_dump(protocol), encoding='utf-8')
    results = evaluate_controlled_segments(visual, protocol_path)
    assert results[0]['distance_error_m'] == pytest.approx(0.0)
    assert results[0]['yaw_error_deg'] == pytest.approx(0.0)


def test_cli_writes_json_and_markdown(tmp_path):
    reference = [
        Pose(index * 0.1, index * 0.1, 0.0, 0.0)
        for index in range(40)
    ]
    visual = _transform(reference, 1.0, math.radians(-15.0), 2.0, 1.0)
    visual_path = tmp_path / 'visual.csv'
    reference_path = tmp_path / 'reference.csv'
    report_json = tmp_path / 'accuracy.json'
    report_md = tmp_path / 'accuracy.md'
    _write_csv(visual_path, visual)
    _write_csv(reference_path, reference)
    main([
        '--visual', str(visual_path),
        '--reference', str(reference_path),
        '--output-json', str(report_json),
        '--output-md', str(report_md),
    ])
    report = json.loads(report_json.read_text(encoding='utf-8'))
    assert report['inputs']['scale_alignment_applied'] is False
    assert report['trajectory_metrics']['pair_count'] == 40
    assert '거리 스케일 오차' in report_md.read_text(encoding='utf-8')


def test_read_trajectory_skips_null_odometry_pose(tmp_path):
    path = tmp_path / 'trajectory.csv'
    _write_csv(path, [
        Pose(0.0, 0.0, 0.0, 0.0),
        Pose(1.0, 1.0, 0.0, 0.0),
        Pose(2.0, 2.0, 0.0, 0.0),
    ])
    with path.open(encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    rows.insert(2, {
        'timestamp_s': 1.5,
        'x_m': 0.0,
        'y_m': 0.0,
        'z_m': 0.0,
        'qx': 0.0,
        'qy': 0.0,
        'qz': 0.0,
        'qw': 0.0,
        'frame_id': 'vslam_odom',
        'child_frame_id': 'camera_link',
    })
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    poses = read_trajectory(path)
    assert [pose.timestamp_s for pose in poses] == [0.0, 1.0, 2.0]

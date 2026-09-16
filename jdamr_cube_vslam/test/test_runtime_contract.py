"""Static contract tests for capture and RTAB-Map orchestration."""

from pathlib import Path
import subprocess

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_shell_scripts_are_syntax_valid():
    for script in sorted((PACKAGE_ROOT / 'scripts').glob('*.sh')):
        subprocess.run(['bash', '-n', str(script)], check=True)


def test_capture_records_metric_rgbd_and_reference_topics():
    script = (PACKAGE_ROOT / 'scripts' / 'capture_rgbd_bag.sh').read_text(
        encoding='utf-8')
    for required in (
        'depth_registration:=true',
        'color_depth_synchronization:=true',
        '/camera/color/image_raw',
        '/camera/color/camera_info',
        '/camera/depth/image_raw',
        '/camera/depth/camera_info',
        '/tf',
        '/tf_static',
        '/odom',
        '--storage-preset-profile zstd_fast',
        '--qos-profile-overrides-path',
        'All requested topics are subscribed',
    ):
        assert required in script
    assert 'systemctl start jdamr-astra-camera.service' in script


def test_tf_static_qos_is_transient_local():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'rosbag_qos_overrides.yaml').read_text(
            encoding='utf-8'))
    assert config['/tf_static']['durability'] == 'transient_local'
    assert config['/tf_static']['reliability'] == 'reliable'


def test_sensor_provenance_is_complete():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'astra_s_rgbd.yaml').read_text(
            encoding='utf-8'))
    assert config['sensor']['usb_id'] == '2bc5:0402'
    assert config['streams']['depth']['registered_to_color'] is True
    provenance = config['provenance']
    assert set(provenance) >= {
        'verified_at', 'valid_for', 'method', 'invalidate_when'}


def test_unmeasured_mount_blocks_sensor_fusion():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'camera_mount.yaml').read_text(
            encoding='utf-8'))
    assert config['camera_mount']['status'] == 'unmeasured'
    assert all(
        value is None
        for value in config['camera_mount']['transform'].values())
    assert config['usage_gate']['camera_only_rgbd_slam'] == 'allowed'
    assert config['usage_gate']['lidar_rgbd_fusion'] \
        == 'blocked_until_measured'


def test_offline_mapping_does_not_consume_lidar_as_visual_ground_truth():
    script = (
        PACKAGE_ROOT / 'scripts' / 'process_rgbd_bag_in_container.sh'
    ).read_text(encoding='utf-8')
    assert 'subscribe_scan:=false' in script
    assert 'visual_odometry:=true' in script
    assert '--Grid/Sensor 1' in script
    assert 'reference_topic /odom' not in script
    assert '--reference-topic /odom' in script


def test_visual_odometry_subscription_uses_best_effort_qos():
    source = (
        PACKAGE_ROOT / 'jdamr_cube_vslam' / 'trajectory_csv_recorder.py'
    ).read_text(encoding='utf-8')
    assert 'ReliabilityPolicy.BEST_EFFORT' in source

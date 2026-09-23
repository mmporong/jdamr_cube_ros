"""Static contract tests for capture and RTAB-Map orchestration."""

import os
from pathlib import Path
import subprocess
import tempfile

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
        'ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET',
        'No sample received from ${reference_topic}',
    ):
        assert required in script
    assert 'systemctl start jdamr-astra-camera.service' in script


def test_capture_has_low_bandwidth_navigation_profile():
    script = (PACKAGE_ROOT / 'scripts' / 'capture_rgbd_bag.sh').read_text(
        encoding='utf-8')
    for required in (
        '--low-bandwidth',
        'color_width=320',
        'color_height=240',
        '--record-navigation',
        '/cmd_vel',
        '/cmd_vel_smoothed',
        '/collision_monitor_state',
        'navigation_telemetry: ${record_navigation}',
    ):
        assert required in script


def test_installed_capture_can_find_qos_overrides():
    script = (PACKAGE_ROOT / 'scripts' / 'capture_rgbd_bag.sh').read_text(
        encoding='utf-8')
    assert '../../../share/jdamr_cube_vslam/config/' in script
    assert 'rosbag QoS overrides not found' in script


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


def test_measured_mount_allows_wheel_odometry_fusion():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'camera_mount.yaml').read_text(
            encoding='utf-8'))
    assert config['camera_mount']['status'] == 'measured'
    assert config['camera_mount']['transform'] == {
        'x_m': 0.065,
        'y_m': 0.0,
        'z_m': 0.215,
        'roll_rad': 0.0,
        'pitch_rad': 0.0,
        'yaw_rad': 0.0,
    }
    assert config['usage_gate']['camera_only_rgbd_slam'] == 'allowed'
    assert config['usage_gate']['wheel_odom_fusion'] == 'allowed'
    assert config['usage_gate']['lidar_rgbd_fusion'] \
        == 'blocked_until_cross_sensor_validation'


def test_wrapper_rejects_wheel_guess_with_unmeasured_mount():
    script = PACKAGE_ROOT / 'scripts' / 'run_rtabmap_docker.sh'
    with tempfile.TemporaryDirectory() as temporary_directory:
        temporary_path = Path(temporary_directory)
        bag_path = temporary_path / 'bag'
        bag_path.mkdir()
        config_path = temporary_path / 'camera_mount.yaml'
        config_path.write_text(
            """camera_mount:
  status: unmeasured
  parent_frame: base_link
  child_frame: camera_link
  transform:
    x_m: null
    y_m: null
    z_m: null
    roll_rad: null
    pitch_rad: null
    yaw_rad: null
usage_gate:
  wheel_odom_fusion: blocked_until_measured
""",
            encoding='utf-8',
        )
        completed = subprocess.run(
            [
                str(script),
                str(bag_path),
                str(temporary_path / 'output'),
                '--odom-guess-frame',
                'odom',
                '--camera-mount',
                str(config_path),
            ],
            check=False,
            capture_output=True,
            text=True,
        )
    assert completed.returncode != 0
    assert 'camera mount status must be measured' in completed.stderr


def test_wheel_guess_injects_measured_camera_transform():
    wrapper = (
        PACKAGE_ROOT / 'scripts' / 'run_rtabmap_docker.sh'
    ).read_text(encoding='utf-8')
    processor = (
        PACKAGE_ROOT / 'scripts' / 'process_rgbd_bag_in_container.sh'
    ).read_text(encoding='utf-8')
    for variable in (
        'CAMERA_MOUNT_PARENT',
        'CAMERA_MOUNT_CHILD',
        'CAMERA_MOUNT_X',
        'CAMERA_MOUNT_Y',
        'CAMERA_MOUNT_Z',
        'CAMERA_MOUNT_ROLL',
        'CAMERA_MOUNT_PITCH',
        'CAMERA_MOUNT_YAW',
        'ODOM_GUESS_MIN_TRANSLATION',
        'ODOM_GUESS_MIN_ROTATION',
    ):
        assert variable in wrapper
        assert variable in processor
    assert 'static_transform_publisher' in processor
    assert 'scripts/wait_for_tf.py' in processor
    assert '--from-frame "$tf_check_from_frame"' in processor
    assert '--to-frame "$CAMERA_MOUNT_CHILD"' in processor
    assert 'guess_frame_id         = ${ODOM_GUESS_FRAME_ID}' in processor
    assert 'odom_guess_min_translation:=' in processor
    assert 'odom_guess_min_rotation:=' in processor
    assert 'vo_frame_id:=vslam_odom' in processor
    assert 'odom_frame_id          = vslam_odom' in processor
    assert 'wait_for_transform_s=1.5' in processor
    assert 'bag_play_rate=0.5' in processor
    assert 'retrying without radius-noise filtering' in processor
    assert 'ODOM_SOURCE_MODE' in wrapper
    assert '--external-odom' in wrapper
    assert 'visual_odometry:="$visual_odometry"' in processor
    assert 'publish_tf_odom=true' in processor
    assert 'publish_tf_odom=false' in processor
    assert 'publish_tf_odom:="$publish_tf_odom"' in processor
    assert 'rtabmap_odom_topic=/rtabmap/odom' in processor
    assert 'rtabmap_odom_topic=/odom' in processor
    assert 'odom_topic:="$rtabmap_odom_topic"' in processor
    assert 'rtabmap_frame_id=base_link' in processor
    assert 'ODOM_GUESS_FRAME_ID:-$CAMERA_MOUNT_PARENT' in processor
    assert 'odom_guess_frame_id:=${ODOM_GUESS_FRAME_ID}' in processor


def test_external_mount_is_forwarded_but_ambiguous_wheel_guess_is_blocked():
    script = PACKAGE_ROOT / 'scripts' / 'run_rtabmap_docker.sh'
    with tempfile.TemporaryDirectory() as temporary_directory:
        temporary_path = Path(temporary_directory)
        bag_path = temporary_path / 'bag'
        bag_path.mkdir()
        config_path = temporary_path / 'camera_mount.yaml'
        config_path.write_text(
            """camera_mount:
  status: measured
  parent_frame: base_link
  child_frame: camera_link
  transform:
    x_m: 0.065
    y_m: 0.0
    z_m: 0.2
    roll_rad: 0.0
    pitch_rad: 0.0
    yaw_rad: 0.0
usage_gate:
  wheel_odom_fusion: allowed
""",
            encoding='utf-8',
        )
        fake_bin = temporary_path / 'bin'
        fake_bin.mkdir()
        fake_docker = fake_bin / 'docker'
        fake_docker.write_text(
            '#!/usr/bin/env bash\nprintf "%s\\n" "$@"\n',
            encoding='utf-8',
        )
        fake_docker.chmod(0o755)
        environment = os.environ.copy()
        environment['PATH'] = f'{fake_bin}:{environment["PATH"]}'
        blocked = subprocess.run(
            [
                str(script),
                str(bag_path),
                str(temporary_path / 'output'),
                '--odom-guess-frame',
                'odom',
                '--camera-mount',
                str(config_path),
            ],
            check=False,
            capture_output=True,
            text=True,
            env=environment,
        )
        completed = subprocess.run(
            [str(script), str(bag_path), str(temporary_path / 'external'),
             '--external-odom', '--camera-mount', str(config_path)],
            check=False, capture_output=True, text=True, env=environment)
    assert blocked.returncode == 2
    assert 'wheel-guess replay blocked' in blocked.stderr
    assert completed.returncode == 0, completed.stderr
    for expected in (
        'ODOM_SOURCE_MODE=external',
        'CAMERA_MOUNT_PARENT=base_link',
        'CAMERA_MOUNT_CHILD=camera_link',
        'CAMERA_MOUNT_X=0.065',
        'CAMERA_MOUNT_Y=0.0',
        'CAMERA_MOUNT_Z=0.2',
        'CAMERA_MOUNT_ROLL=0.0',
        'CAMERA_MOUNT_PITCH=0.0',
        'CAMERA_MOUNT_YAW=0.0',
        'ODOM_GUESS_MIN_TRANSLATION=0.005',
        'ODOM_GUESS_MIN_ROTATION=0.005',
    ):
        assert expected in completed.stdout


def test_wrapper_rejects_invalid_wheel_guess_threshold():
    script = PACKAGE_ROOT / 'scripts' / 'run_rtabmap_docker.sh'
    with tempfile.TemporaryDirectory() as temporary_directory:
        temporary_path = Path(temporary_directory)
        bag_path = temporary_path / 'bag'
        bag_path.mkdir()
        completed = subprocess.run(
            [
                str(script),
                str(bag_path),
                str(temporary_path / 'output'),
                '--odom-guess-frame',
                'odom',
                '--odom-guess-min-translation',
                '-0.001',
            ],
            check=False,
            capture_output=True,
            text=True,
        )
    assert completed.returncode != 0
    assert 'translation must be finite and nonnegative' in completed.stderr


def test_wrapper_rejects_boolean_camera_transform():
    script = PACKAGE_ROOT / 'scripts' / 'run_rtabmap_docker.sh'
    with tempfile.TemporaryDirectory() as temporary_directory:
        temporary_path = Path(temporary_directory)
        bag_path = temporary_path / 'bag'
        bag_path.mkdir()
        config_path = temporary_path / 'camera_mount.yaml'
        config_path.write_text(
            """camera_mount:
  status: measured
  parent_frame: base_link
  child_frame: camera_link
  transform:
    x_m: true
    y_m: 0.0
    z_m: 0.2
    roll_rad: 0.0
    pitch_rad: 0.0
    yaw_rad: 0.0
usage_gate:
  wheel_odom_fusion: allowed
""",
            encoding='utf-8',
        )
        completed = subprocess.run(
            [
                str(script),
                str(bag_path),
                str(temporary_path / 'output'),
                '--odom-guess-frame',
                'odom',
                '--camera-mount',
                str(config_path),
            ],
            check=False,
            capture_output=True,
            text=True,
        )
    assert completed.returncode != 0
    assert 'transform x_m must be finite' in completed.stderr


def test_wrapper_rejects_base_footprint_as_guess_frame():
    script = PACKAGE_ROOT / 'scripts' / 'run_rtabmap_docker.sh'
    with tempfile.TemporaryDirectory() as temporary_directory:
        temporary_path = Path(temporary_directory)
        bag_path = temporary_path / 'bag'
        bag_path.mkdir()
        completed = subprocess.run(
            [
                str(script),
                str(bag_path),
                str(temporary_path / 'output'),
                '--odom-guess-frame',
                'base_footprint',
            ],
            check=False,
            capture_output=True,
            text=True,
        )
    assert completed.returncode != 0
    assert 'wheel guess frame must be odom' in completed.stderr


def test_offline_mapping_does_not_consume_lidar_as_visual_ground_truth():
    script = (
        PACKAGE_ROOT / 'scripts' / 'process_rgbd_bag_in_container.sh'
    ).read_text(encoding='utf-8')
    assert 'subscribe_scan:=false' in script
    assert 'visual_odometry=true' in script
    assert 'visual_odometry:="$visual_odometry"' in script
    assert '--Grid/Sensor 1' in script
    assert 'reference_topic /odom' not in script
    assert '--reference-topic /odom' in script


def test_visual_odometry_subscription_uses_best_effort_qos():
    source = (
        PACKAGE_ROOT / 'jdamr_cube_vslam' / 'trajectory_csv_recorder.py'
    ).read_text(encoding='utf-8')
    assert 'ReliabilityPolicy.BEST_EFFORT' in source


def test_low_texture_profile_keeps_odometry_only_arguments_separate():
    script = (
        PACKAGE_ROOT / 'scripts' / 'process_rgbd_bag_in_container.sh'
    ).read_text(encoding='utf-8')
    assert "rtabmap_odom_extra_args='--Odom/ResetCountdown 5 " \
        "--OdomF2M/MaxSize 3000'" in script
    assert 'odom_extra_launch_args=("odom_args:=${rtabmap_odom_extra_args}")' in script
    assert '"${odom_extra_launch_args[@]}"' in script
    slam_argument_lines = [
        line for line in script.splitlines()
        if 'rtabmap_extra_args=' in line
        and 'rtabmap_odom_extra_args=' not in line
    ]
    assert slam_argument_lines
    assert all('--Odom/' not in line for line in slam_argument_lines)


def test_asset_export_reoptimizes_graph_instead_of_reusing_last_map():
    script = (
        PACKAGE_ROOT / 'scripts' / 'export_3d_assets.sh'
    ).read_text(encoding='utf-8')
    assert script.count('--opt 0') == 2
    assert '--opt 2' not in script

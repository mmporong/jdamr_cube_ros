"""Tests for traceable Gazebo sensor-profile variants."""

from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest


ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'jdamr_cube_description' / 'urdf' / 'jdamr_cube.urdf'
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'evaluation'))

from make_sim_sensor_variant import (  # noqa: E402,I100
    _sensor,
    apply_sensor_settings,
)


def test_sensor_variant_changes_only_requested_fields():
    """A rate-only experiment must keep every noise value unchanged."""
    root = ET.parse(BASE).getroot()
    laser = _sensor(root, 'laser_link', 'laser_sensor')
    imu = _sensor(root, 'base_link', 'imu_sensor')
    original_lidar_noise = laser.findtext('lidar/noise/stddev')
    original_imu_rate = imu.findtext('update_rate')

    changes = apply_sensor_settings(root, {'lidar_update_rate_hz': 10.0})

    assert laser.findtext('update_rate') == '10.0'
    assert laser.findtext('lidar/noise/stddev') == original_lidar_noise
    assert imu.findtext('update_rate') == original_imu_rate
    assert changes == {
        'lidar_update_rate_hz': {
            'from': 2.0,
            'to': 10.0,
            'unit': 'Hz',
        },
    }


def test_sensor_variant_applies_imu_noise_to_all_axes():
    """One declared IMU profile must cover all three axes consistently."""
    root = ET.parse(BASE).getroot()
    imu = _sensor(root, 'base_link', 'imu_sensor')

    apply_sensor_settings(root, {
        'imu_angular_noise_stddev_radps': 0.02,
        'imu_linear_noise_stddev_mps2': 0.1,
    })

    for axis in ('x', 'y', 'z'):
        assert float(imu.findtext(
            f'imu/angular_velocity/{axis}/noise/stddev')) == 0.02
        assert float(imu.findtext(
            f'imu/linear_acceleration/{axis}/noise/stddev')) == 0.1


def test_sensor_variant_rejects_unknown_settings():
    """The generator must not silently mutate an undeclared field."""
    root = ET.parse(BASE).getroot()

    with pytest.raises(ValueError, match='unsupported settings'):
        apply_sensor_settings(root, {'wheel_slip': 0.5})

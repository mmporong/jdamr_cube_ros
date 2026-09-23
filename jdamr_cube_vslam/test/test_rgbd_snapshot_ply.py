"""Unit tests for metric RGB-D projection and PLY output."""

from jdamr_cube_vslam.rgbd_snapshot_ply import (
    project_registered_depth,
    write_binary_ply,
)
import numpy as np
from sensor_msgs.msg import CameraInfo


def test_registered_depth_projects_in_meters(tmp_path):
    depth_mm = np.array([[1000, 2000], [0, 6000]], dtype=np.uint16)
    color_rgb = np.array([
        [[255, 0, 0], [0, 255, 0]],
        [[0, 0, 255], [255, 255, 255]],
    ], dtype=np.uint8)
    camera_info = CameraInfo()
    camera_info.width = 2
    camera_info.height = 2
    camera_info.k = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

    points_m, colors = project_registered_depth(
        depth_mm, color_rgb, camera_info, pixel_step=1)
    assert points_m.shape == (2, 3)
    np.testing.assert_allclose(points_m[0], [0.0, 0.0, 1.0])
    np.testing.assert_allclose(points_m[1], [2.0, 0.0, 2.0])
    np.testing.assert_array_equal(colors[0], [255, 0, 0])

    output_path = tmp_path / 'snapshot.ply'
    write_binary_ply(output_path, points_m, colors)
    assert output_path.read_bytes().startswith(b'ply\n')
    assert output_path.stat().st_size > 100

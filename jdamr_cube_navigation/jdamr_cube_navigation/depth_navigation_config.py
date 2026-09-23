"""Build an opt-in depth-obstacle overlay for the new-base Nav2 profile."""

from copy import deepcopy
from pathlib import Path

import yaml


DEPTH_OBSTACLE_TOPIC = '/depth_navigation/obstacles'
DEPTH_RAY_TOPIC = '/depth_navigation/rays'
DEPTH_SENSOR_FRAME = 'camera_color_optical_frame'
DEPTH_MIN_HEIGHT_M = 0.05
DEPTH_MAX_HEIGHT_M = 1.5
DEPTH_FILTER_DEFAULTS = {
    'depth_topic': '/camera/depth/image_raw',
    'camera_info_topic': '/camera/depth/camera_info',
    'obstacle_topic': DEPTH_OBSTACLE_TOPIC,
    'ray_topic': DEPTH_RAY_TOPIC,
    'status_topic': '/depth_navigation/status',
    'optical_frame': DEPTH_SENSOR_FRAME,
    'base_frame': 'base_footprint',
    'calibration_model': 'rectified_projection',
    'min_depth_m': 0.4,
    'max_depth_m': 2.5,
    'min_height_m': DEPTH_MIN_HEIGHT_M,
    'max_height_m': DEPTH_MAX_HEIGHT_M,
    'max_forward_m': 2.5,
    'max_lateral_m': 1.5,
    'pixel_stride': 4,
    'voxel_size_m': 0.04,
    'min_valid_fraction': 0.05,
    'max_points': 20000,
    'max_image_age_s': 0.5,
    'max_processing_hz': 5.0,
    'transform_timeout_s': 0.05,
}


def build_depth_navigation_params(base_params):
    """Return a deep-copied Nav2 configuration with approved depth sources."""
    params = deepcopy(base_params)
    for costmap_name in ('local_costmap', 'global_costmap'):
        costmap = params[costmap_name][costmap_name]['ros__parameters']
        inflation_index = costmap['plugins'].index('inflation_layer')
        costmap['plugins'].insert(inflation_index, 'depth_obstacle_layer')
        costmap['depth_obstacle_layer'] = {
            'plugin': 'nav2_costmap_2d::VoxelLayer',
            'enabled': True,
            'combination_method': 1,
            'origin_z': 0.0,
            'z_resolution': 0.1,
            'z_voxels': 16,
            'mark_threshold': 0,
            'unknown_threshold': 15,
            'publish_voxel_map': False,
            'min_obstacle_height': -0.05,
            'max_obstacle_height': DEPTH_MAX_HEIGHT_M,
            'observation_sources': 'depth_marks depth_rays',
            'depth_marks': {
                'topic': DEPTH_OBSTACLE_TOPIC,
                'sensor_frame': DEPTH_SENSOR_FRAME,
                'data_type': 'PointCloud2',
                'marking': True,
                'clearing': False,
                'min_obstacle_height': DEPTH_MIN_HEIGHT_M,
                'max_obstacle_height': DEPTH_MAX_HEIGHT_M,
                'obstacle_min_range': 0.4,
                'obstacle_max_range': 2.5,
            },
            'depth_rays': {
                'topic': DEPTH_RAY_TOPIC,
                'sensor_frame': DEPTH_SENSOR_FRAME,
                'data_type': 'PointCloud2',
                'marking': False,
                'clearing': True,
                # Include small floor-height jitter in 3D clearing rays.
                'min_obstacle_height': -0.05,
                'max_obstacle_height': DEPTH_MAX_HEIGHT_M,
                'raytrace_min_range': 0.4,
                'raytrace_max_range': 2.5,
            },
        }

    monitor = params['collision_monitor']['ros__parameters']
    monitor['observation_sources'] = ['scan', 'depth_obstacles']
    monitor['depth_obstacles'] = {
        'type': 'pointcloud',
        'topic': DEPTH_OBSTACLE_TOPIC,
        'enabled': True,
        'source_timeout': 1.0,
        'min_height': DEPTH_MIN_HEIGHT_M,
        'max_height': DEPTH_MAX_HEIGHT_M,
    }
    return params


def load_depth_navigation_params(path):
    """Load a base YAML file and return the approved depth-enabled overlay."""
    base_params = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(base_params, dict):
        raise RuntimeError('Nav2 parameter file must contain a mapping')
    return build_depth_navigation_params(base_params)

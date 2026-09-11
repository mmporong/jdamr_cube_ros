"""Validate the independent Gazebo ground-truth pose path."""

import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
URDF = ROOT / 'jdamr_cube_description' / 'urdf' / 'jdamr_cube.urdf'
BRIDGE = ROOT / 'jdamr_cube_gazebo' / 'params' / 'bridge.yaml'
LAUNCH = ROOT / 'jdamr_cube_gazebo' / 'launch' / 'gazebo.launch.py'


def _ground_truth_plugin():
    root = ET.parse(URDF).getroot()
    for gazebo in root.findall('gazebo'):
        plugin = gazebo.find('plugin')
        if (plugin is not None
                and plugin.get('filename')
                == 'gz-sim-pose-publisher-system'):
            return plugin
    raise AssertionError('Gazebo PosePublisher plugin is missing')


def test_ground_truth_publishes_only_the_model_pose():
    """The truth stream must not contain link or sensor transforms."""
    plugin = _ground_truth_plugin()

    assert plugin.findtext('publish_model_pose') == 'true'
    assert plugin.findtext('publish_nested_model_pose') == 'false'
    for field in (
            'publish_link_pose',
            'publish_sensor_pose',
            'publish_collision_pose',
            'publish_visual_pose'):
        assert plugin.findtext(field) == 'false'
    assert plugin.findtext('use_pose_vector_msg') == 'false'
    assert plugin.find('topic') is None
    assert float(plugin.findtext('update_frequency')) == 20.0


def test_ground_truth_bridge_is_one_way_and_not_tf():
    """The evaluator receives PoseStamped without polluting the TF tree."""
    entries = yaml.safe_load(BRIDGE.read_text(encoding='utf-8'))
    matches = [entry for entry in entries
               if entry.get('ros_topic_name') == 'ground_truth_pose']

    assert matches == [{
        'ros_topic_name': 'ground_truth_pose',
        'gz_topic_name': 'model/jdamr_cube/pose',
        'ros_type_name': 'geometry_msgs/msg/PoseStamped',
        'gz_type_name': 'gz.msgs.Pose',
        'direction': 'GZ_TO_ROS',
    }]


def test_launch_accepts_generated_urdf_and_resolves_controller_path(tmp_path):
    """Sensor variants must retain the gz_ros2_control configuration."""
    spec = importlib.util.spec_from_file_location('gazebo_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    variant = tmp_path / 'variant.urdf'
    variant.write_text(
        '<robot>package://jdamr_cube_description/config/'
        'so101_controllers.yaml</robot>', encoding='utf-8')

    content = module._load_robot_description(
        variant, '/tmp/controllers.yaml')

    assert content == '<robot>/tmp/controllers.yaml</robot>'


def test_robot_state_publisher_respawn_bool_is_strict():
    """Capture runs may disable respawn without weakening the default."""
    spec = importlib.util.spec_from_file_location('gazebo_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    assert module._strict_bool('true', 'respawn') is True
    assert module._strict_bool('false', 'respawn') is False
    for invalid in ('True', 'False', '1', '0', 'yes', ''):
        try:
            module._strict_bool(invalid, 'respawn')
        except ValueError:
            continue
        raise AssertionError(f'accepted invalid bool: {invalid!r}')

    source = LAUNCH.read_text(encoding='utf-8')
    assert "'robot_state_publisher_respawn', default_value='true'" in source


def test_launch_skips_arm_spawners_for_base_only_runs():
    """Base-only replays must not wait for absent arm controllers."""
    source = LAUNCH.read_text(encoding='utf-8')

    assert "'enable_arm_controllers', default_value='true'" in source
    assert "'enable_arm_controllers'" in source


def test_launch_exposes_gazebo_set_pose_as_a_ros_service():
    """Dynamic actors must not spawn one shell process per animation frame."""
    source = LAUNCH.read_text(encoding='utf-8')

    assert '/world/slam_corridor/set_pose@' in source
    assert 'ros_gz_interfaces/srv/SetEntityPose' in source

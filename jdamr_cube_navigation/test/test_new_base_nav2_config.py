"""The physical candidate rejects a stale JD-AMR footprint before launch."""

import hashlib
import importlib.util
from pathlib import Path
from types import SimpleNamespace

from jdamr_cube_navigation.corridor_route import revisit_plan_length_ok
from launch import LaunchContext
from nav2_common.launch import RewrittenYaml
import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
LAUNCH = ROOT / 'jdamr_cube_navigation/launch/onboard_nav2_core.launch.py'
PARAMS = ROOT / 'jdamr_cube_navigation/config/new_base_nav2_params.yaml'
OLD_PARAMS = ROOT / 'jdamr_cube_navigation/config/nav2_params.yaml'
WRAPPER = ROOT / 'jdamr_cube_navigation/launch/onboard_keepout_navigation.launch.py'
LEGACY_AUTORUN = ROOT / 'jdamr_cube_navigation/scripts/corridor_autorun.sh'


def _load_validator(path, map_name='new_base_live_20260915T1407.yaml',
                    mask_name='new_base_live_20260915T1407_keepout.yaml'):
    spec = importlib.util.spec_from_file_location('new_base_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    class FixedConfiguration:
        def __init__(self, name):
            self.name = name

        def perform(self, _context):
            return {
                'params_file': str(path),
                'map': map_name,
                'keepout_mask': mask_name,
            }[self.name]

    module.LaunchConfiguration = FixedConfiguration
    module.get_package_share_directory = lambda _name: str(
        ROOT / 'jdamr_cube_description')
    return module._validate_new_base_params


def test_physical_candidate_is_accepted():
    assert _load_validator(PARAMS)(None) == []
    document = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))
    assert document['amcl']['ros__parameters']['set_initial_pose'] is False
    assert document['velocity_smoother']['ros__parameters']['max_velocity'][0] == 0.08


def test_revisit_rejects_long_detour_outside_recorded_corridor():
    assert revisit_plan_length_ok(78.0, 76.42)
    assert not revisit_plan_length_ok(120.0, 76.42)


def test_jazzy_rewrite_seeds_only_the_revisit_home_pose():
    rewritten = RewrittenYaml(
        source_file=str(PARAMS), root_key='', convert_types=True,
        param_rewrites={
            'amcl.ros__parameters.set_initial_pose': 'true',
            'amcl.ros__parameters.initial_pose.x': '0.0',
            'amcl.ros__parameters.initial_pose.y': '-0.1',
            'amcl.ros__parameters.initial_pose.yaw': '0.0',
        })
    effective = yaml.safe_load(Path(rewritten.perform(LaunchContext()))
                               .read_text(encoding='utf-8'))
    amcl = effective['amcl']['ros__parameters']
    assert amcl['set_initial_pose'] is True
    assert amcl['initial_pose']['y'] == -0.1
    assert yaml.safe_load(PARAMS.read_text(encoding='utf-8'))[
        'amcl']['ros__parameters']['set_initial_pose'] is False


def test_revisit_accepts_only_verified_legacy_map_with_new_base_geometry():
    old_map = Path.home() / 'maps/autonomous_20260826T161908.yaml'
    old_mask = Path.home() / 'maps/autonomous_20260826T161908_keepout_multi.yaml'
    if not old_map.is_file() or not old_mask.is_file():
        pytest.skip('verified old corridor artifacts are not on this host')
    validator = _load_validator(PARAMS, str(old_map), str(old_mask))
    assert validator(None, revisit=True) == []


def test_physical_wrapper_fails_closed_without_new_map_and_mask():
    source = WRAPPER.read_text(encoding='utf-8')
    assert "'navigation_profile', default_value='new_base_candidate'" in source
    assert "choices=['new_base_candidate', 'new_base_revisit_candidate']" in source
    assert "'autostart', default_value='false'" in source
    assert "package_share, 'config', 'new_base_nav2_params.yaml'" in source
    assert "'map',\n            default_value=''" in source
    assert "'keepout_mask',\n            default_value=''" in source


def test_legacy_corridor_runner_rejects_old_profile_on_new_base_before_logging():
    source = LEGACY_AUTORUN.read_text(encoding='utf-8')
    marker = 'install/jdamr_cube_description/share/jdamr_cube_description/urdf/new_base_real.urdf'
    assert marker in source
    assert source.index(marker) < source.index('A="$HOME/jdamr_artifacts"')
    assert 'exit 2' in source[source.index(marker):source.index('A="$HOME/jdamr_artifacts"')]
    assert '"$NAVIGATION_PROFILE" != new_base_revisit_candidate' in source
    assert '오래된 출발 신호가 남아' in source


def test_revisit_runner_records_inputs_and_checks_graph_before_nav2():
    source = LEGACY_AUTORUN.read_text(encoding='utf-8')
    assert 'check_revisit_isolation || exit 4' in source
    assert 'jdamr-cartographer-session.service jdamr-webteleop.service' in source
    assert 'Publisher count' in source
    assert 'autostart:=true bag_output:=' in source
    assert '.inputs.sha256' in source
    assert 'motion_start_utc=' in source
    assert 'motion_end_utc=' in source
    assert source.index('check_revisit_isolation || exit 4',
                        source.index('say "Nav2 와 기록 기동"')) < source.index(
                            'setsid nohup ros2 launch',
                            source.index('say "Nav2 와 기록 기동"'))


def test_preflight_requires_collision_monitor_not_only_one_publisher():
    source = (ROOT / 'jdamr_cube_navigation/scripts/corridor_preflight.sh'
              ).read_text(encoding='utf-8')
    assert 'ros2 topic info /cmd_vel --verbose' in source
    assert '[ "$publisher" = collision_monitor ]' in source
    assert 'Cartographer·웹 조종기 미실행' in source
    assert '로봇 정지 확인' in source


def test_revisit_reference_pins_both_yaml_and_image_hashes(tmp_path):
    spec = importlib.util.spec_from_file_location('new_base_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    artifacts = {}
    for label, yaml_name, image_name in (
            ('map', 'autonomous_20260826T161908.yaml',
             'autonomous_20260826T161908.pgm'),
            ('mask', 'autonomous_20260826T161908_keepout_multi.yaml',
             'autonomous_20260826T161908_keepout_multi.pgm')):
        image = tmp_path / image_name
        image.write_bytes(b'P5\n1 1\n255\n\xff')
        metadata = tmp_path / yaml_name
        metadata.write_text(yaml.safe_dump({'image': image_name}),
                            encoding='utf-8')
        module.REVISIT_REFERENCE[label] = (
            yaml_name, hashlib.sha256(metadata.read_bytes()).hexdigest(),
            image_name, hashlib.sha256(image.read_bytes()).hexdigest())
        artifacts[label] = (metadata, image)

    class FixedConfiguration:
        def __init__(self, name):
            self.name = name

        def perform(self, _context):
            return str(artifacts[{'map': 'map',
                                  'keepout_mask': 'mask'}[self.name]][0])

    module.LaunchConfiguration = FixedConfiguration
    assert module._validate_revisit_reference(None) == []
    artifacts['map'][0].write_text('image: wrong.pgm\n', encoding='utf-8')
    with pytest.raises(RuntimeError, match='map YAML hash mismatch'):
        module._validate_revisit_reference(None)


def test_direct_core_rejects_legacy_profile_on_installed_new_base():
    spec = importlib.util.spec_from_file_location('new_base_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    selected = {'profile': 'corridor'}

    class FixedConfiguration:
        def __init__(self, name):
            self.name = name

        def perform(self, _context):
            return {'use_sim_time': 'false',
                    'navigation_profile': selected['profile']}[self.name]

    module.LaunchConfiguration = FixedConfiguration
    module.get_package_share_directory = lambda _name: str(
        ROOT / 'jdamr_cube_description')
    with pytest.raises(RuntimeError, match='rejects legacy navigation profiles'):
        module._reject_legacy_profile_on_new_base(None)
    selected['profile'] = 'new_base_revisit_candidate'
    assert module._reject_legacy_profile_on_new_base(None) == []


def test_direct_revisit_launch_rejects_active_teleop_and_unknown_graph(
        monkeypatch):
    spec = importlib.util.spec_from_file_location('new_base_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = lambda _name: str(
        ROOT / 'jdamr_cube_description')

    class FixedConfiguration:
        def __init__(self, name):
            self.name = name

        def perform(self, _context):
            return {'use_sim_time': 'false',
                    'navigation_profile': 'new_base_revisit_candidate'}[
                        self.name]

    module.LaunchConfiguration = FixedConfiguration
    monkeypatch.setenv('ROS_DOMAIN_ID', '12')

    def active_teleop(args, **_kwargs):
        active = args[-1] == 'jdamr-webteleop.service'
        return SimpleNamespace(returncode=(0 if active else 3),
                               stdout='')

    monkeypatch.setattr(module.subprocess, 'run', active_teleop)
    with pytest.raises(RuntimeError, match='active jdamr-webteleop.service'):
        module._validate_revisit_isolation(None)

    def unknown_graph(args, **_kwargs):
        if args[:3] == ['ros2', 'node', 'list']:
            return SimpleNamespace(returncode=1, stdout='')
        return SimpleNamespace(returncode=3, stdout='')

    monkeypatch.setattr(module.subprocess, 'run', unknown_graph)
    with pytest.raises(RuntimeError, match='cannot read ROS nodes'):
        module._validate_revisit_isolation(None)


def test_legacy_params_are_rejected():
    with pytest.raises(RuntimeError, match='footprint is smaller'):
        _load_validator(OLD_PARAMS)(None)


def test_old_map_is_rejected():
    with pytest.raises(RuntimeError, match='requires a new-base map and mask'):
        _load_validator(PARAMS, map_name='autonomous_20260826T161908.yaml')(None)


def test_narrowed_stop_zone_is_rejected(tmp_path):
    document = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))
    document['collision_monitor']['ros__parameters']['StopZone']['points'] = \
        '[[0.28, 0.25], [0.28, -0.25], [-0.28, -0.25], [-0.28, 0.25]]'
    narrowed = tmp_path / 'narrowed.yaml'
    narrowed.write_text(yaml.safe_dump(document), encoding='utf-8')
    with pytest.raises(RuntimeError, match='StopZone does not contain'):
        _load_validator(narrowed)(None)


@pytest.mark.parametrize('mutate,expected', [
    (lambda monitor: monitor['StopZone'].update(enabled=False),
     'StopZone is not active'),
    (lambda monitor: monitor['StopZone'].update(action_type='slowdown'),
     'StopZone is not active'),
    (lambda monitor: monitor['scan'].update(enabled=False),
     'scan collision source is not active'),
    (lambda monitor: monitor.update(observation_sources=[]),
     'scan collision source is not active'),
    (lambda monitor: monitor['StopZone'].update(points=(
        '[[0.35, 0.35], [0.35, 0.30], [-0.38, 0.30], [-0.38, 0.35]]')),
     'polygon must cover both sides'),
    (lambda monitor: monitor['StopZone'].update(points=(
        '[[0.35, 0.35], [-0.38, -0.35], [0.35, -0.35], [-0.38, 0.35]]')),
     'corners must follow the perimeter'),
])
def test_disabled_or_one_sided_monitor_is_rejected(tmp_path, mutate, expected):
    document = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))
    mutate(document['collision_monitor']['ros__parameters'])
    invalid = tmp_path / 'invalid.yaml'
    invalid.write_text(yaml.safe_dump(document), encoding='utf-8')
    with pytest.raises(RuntimeError, match=expected):
        _load_validator(invalid)(None)

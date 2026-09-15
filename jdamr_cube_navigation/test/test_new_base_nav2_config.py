"""The physical candidate rejects a stale JD-AMR footprint before launch."""

import importlib.util
from pathlib import Path

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


def test_physical_wrapper_fails_closed_without_new_map_and_mask():
    source = WRAPPER.read_text(encoding='utf-8')
    assert "'navigation_profile', default_value='new_base_candidate'" in source
    assert "choices=['new_base_candidate']" in source
    assert "'autostart', default_value='false'" in source
    assert "package_share, 'config', 'new_base_nav2_params.yaml'" in source
    assert "'map',\n            default_value=''" in source
    assert "'keepout_mask',\n            default_value=''" in source


def test_legacy_corridor_runner_rejects_installed_new_base_before_logging():
    source = LEGACY_AUTORUN.read_text(encoding='utf-8')
    marker = 'install/jdamr_cube_description/share/jdamr_cube_description/urdf/new_base_real.urdf'
    assert marker in source
    assert source.index(marker) < source.index('A="$HOME/jdamr_artifacts"')
    assert 'exit 2' in source[source.index(marker):source.index('A="$HOME/jdamr_artifacts"')]


def test_direct_core_rejects_legacy_profile_on_installed_new_base():
    spec = importlib.util.spec_from_file_location('new_base_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    class FixedConfiguration:
        def __init__(self, name):
            self.name = name

        def perform(self, _context):
            return {'use_sim_time': 'false',
                    'navigation_profile': 'corridor'}[self.name]

    module.LaunchConfiguration = FixedConfiguration
    module.get_package_share_directory = lambda _name: str(
        ROOT / 'jdamr_cube_description')
    with pytest.raises(RuntimeError, match='rejects legacy navigation profiles'):
        module._reject_legacy_profile_on_new_base(None)


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

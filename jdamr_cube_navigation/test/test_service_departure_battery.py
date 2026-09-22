"""Exercise departure reserve independently from the running cutoff."""

import io
import json
from pathlib import Path
from unittest.mock import Mock

from jdamr_cube_navigation.restaurant_service import load_service_contract, ServiceRoute
import pytest
import yaml


CONTRACT = Path(__file__).resolve().parents[1] / 'config/restaurant_service_contract.yaml'


@pytest.mark.parametrize('voltage_v,ready', [
    (None, False), (float('nan'), False), (float('inf'), False),
    (10.464, False), (10.6, False), (10.8, True), (12.0, True),
])
def test_departure_reserve(voltage_v, ready):
    """A near-cutoff idle reading must not authorize a new mission."""
    node = object.__new__(ServiceRoute)
    node.service_contract = load_service_contract(CONTRACT)
    node.minimum_battery_v = node.service_contract['minimum_running_battery_v']
    node.battery_voltage = voltage_v
    node.emit = Mock()
    assert node._departure_battery_ready() is ready
    assert node.minimum_battery_v == 10.5
    if not ready:
        assert node.emit.call_args.kwargs['reason'] == 'battery_departure_reserve'
        json.dumps(node.emit.call_args.kwargs, allow_nan=False)


@pytest.mark.parametrize('method', ['execute', '_execute_reverse_path'])
def test_low_reserve_never_dispatches_forward_or_reverse(method):
    """Both action entrypoints reject low reserve before send_goal_async."""
    node = object.__new__(ServiceRoute)
    node.service_contract = load_service_contract(CONTRACT)
    node.minimum_battery_v = 10.5
    node.battery_voltage = 10.6
    node.emit = Mock()
    node.stop_requested = False
    node._navigation_ready = Mock(return_value=True)
    node.navigate, node.follow_reverse = Mock(), Mock()
    node.waypoints = [{'id': 'home'}]
    node.result_stream = io.StringIO()
    assert not getattr(node, method)(*([None] if method.startswith('_') else []))
    node.navigate.send_goal_async.assert_not_called()
    node.follow_reverse.send_goal_async.assert_not_called()


@pytest.mark.parametrize('value', [None, True, float('nan'), -1.0, 10.4])
def test_invalid_departure_contract(tmp_path, value):
    """Reject missing, nonfinite and weaker-than-running reserve settings."""
    document = yaml.safe_load(CONTRACT.read_text())
    document['minimum_start_battery_v'] = value
    path = tmp_path / 'contract.yaml'
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError):
        load_service_contract(path)

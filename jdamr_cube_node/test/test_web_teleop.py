"""Tests for ordering browser teleoperation commands."""

import json
import subprocess
import threading
import time

from jdamr_cube_node.web_teleop import (
    CommandOrder,
    DEFAULT_FRESH_SEC,
    TeleopNode,
    scaled_command,
)

import pytest


def _run_page_scenario(scenario):
    """Run the embedded browser controller with deterministic HTTP fakes."""
    from jdamr_cube_node.web_teleop import PAGE

    script = PAGE.split('<script>', 1)[1].split('</script>', 1)[0]
    harness = f"""
const vm = require('node:vm');
const assert = require('node:assert/strict');
const elements = new Map();
class Element {{
  constructor(id) {{
    this.id = id;
    this.value = id === 'spd' ? '50' : '';
    this.textContent = '';
    this.className = '';
    this.classList = {{toggle() {{}}}};
  }}
  addEventListener() {{}}
  setPointerCapture() {{}}
}}
const document = {{
  hidden: false,
  getElementById(id) {{
    if (!elements.has(id)) elements.set(id, new Element(id));
    return elements.get(id);
  }},
  addEventListener() {{}},
}};
const window = {{addEventListener() {{}}}};
const intervals = [];
const cmdRequests = [];
let cmdAborts = 0;
let commandMode = 'delay';
let commandDelayMs = 200;
let stateHang = false;
let stateRequests = 0;
let stateInFlight = 0;
let maxStateInFlight = 0;
const stateBody = {{
  accepted_sequence: 0,
  angular_z: 0,
  command_fresh: true,
  linear_x: 0,
  output_topic: 'cmd_vel',
  profile_label: 'test',
}};
function reply(body, ok=true) {{
  return {{ok, async json() {{ return body; }}}};
}}
function fetch(url, options={{}}) {{
  if (url === '/state') {{
    stateRequests += 1;
    stateInFlight += 1;
    maxStateInFlight = Math.max(maxStateInFlight, stateInFlight);
    if (stateHang) return new Promise(() => {{}});
    stateInFlight -= 1;
    return Promise.resolve(reply(stateBody));
  }}
  if (url === '/abort') return Promise.resolve(reply(stateBody));
  assert.equal(url, '/cmd');
  const command = JSON.parse(options.body);
  cmdRequests.push(command);
  const ordinal = cmdRequests.length;
  return new Promise((resolve, reject) => {{
    options.signal.addEventListener('abort', () => {{
      cmdAborts += 1;
      const error = new Error('aborted');
      error.name = 'AbortError';
      reject(error);
    }}, {{once:true}});
    if (commandMode === 'hang-first' && ordinal === 1) return;
    setTimeout(() => resolve(reply({{
      ...stateBody,
      accepted_sequence: command.sequence,
      linear_x: command.vx,
      angular_z: command.wz,
    }})), commandDelayMs);
  }});
}}
const context = vm.createContext({{
  AbortController,
  Date,
  Math,
  Promise,
  console,
  crypto: {{randomUUID: () => 'browser-test'}},
  document,
  fetch,
  globalThis: null,
  navigator: {{sendBeacon() {{ return true; }}}},
  performance,
  setInterval: (fn, ms) => {{
    intervals.push({{fn, ms}});
    return intervals.length;
  }},
  setTimeout,
  clearTimeout,
  window,
}});
context.globalThis = context;
vm.runInContext({json.dumps(script)}, context);
(async () => {{
  {scenario}
}})().catch(error => {{ console.error(error); process.exitCode = 1; }});
"""
    result = subprocess.run(
        ['node'], input=harness, text=True, capture_output=True,
        timeout=5, check=False)
    assert result.returncode == 0, result.stderr


def test_page_keeps_one_command_in_flight_during_200ms_rtt():
    """A slow-but-healthy ACK must not be cancelled by 100 ms keepalives."""
    _run_page_scenario("""
      vm.runInContext('press("w")', context);
      const keepalive = intervals.find(item => item.ms === 100).fn;
      await new Promise(resolve => setTimeout(resolve, 80));
      keepalive();
      await new Promise(resolve => setTimeout(resolve, 80));
      keepalive();
      await new Promise(resolve => setTimeout(resolve, 310));
      assert.equal(cmdAborts, 0);
      assert.ok(cmdRequests.filter(command => command.vx > 0).length >= 2);
    """)


def test_page_release_aborts_motion_and_only_queues_zero_afterward():
    """A key release supersedes an outstanding motion request with STOP."""
    _run_page_scenario("""
      vm.runInContext('press("w")', context);
      await new Promise(resolve => setTimeout(resolve, 20));
      vm.runInContext('release("w")', context);
      await new Promise(resolve => setTimeout(resolve, 260));
      assert.equal(cmdAborts, 1);
      assert.ok(cmdRequests.length >= 2);
      assert.ok(cmdRequests[0].vx > 0);
      assert.equal(cmdRequests.at(-1).vx, 0);
      assert.equal(cmdRequests.at(-1).wz, 0);
      assert.ok(cmdRequests.at(-1).sequence > cmdRequests[0].sequence);
    """)


def test_page_blur_stop_never_requeues_the_stale_motion():
    """Blur-style stop bursts contain only zero after superseding motion."""
    _run_page_scenario("""
      vm.runInContext('press("w")', context);
      await new Promise(resolve => setTimeout(resolve, 20));
      vm.runInContext('sendStopBurst()', context);
      await new Promise(resolve => setTimeout(resolve, 430));
      assert.ok(cmdRequests.length >= 3);
      assert.ok(cmdRequests[0].vx > 0);
      for (const command of cmdRequests.slice(1)) {
        assert.equal(command.vx, 0);
        assert.equal(command.wz, 0);
      }
    """)


def test_page_recovers_a_hung_command_before_ros_deadman():
    """A hung HTTP request is aborted and retried within the 350 ms deadman."""
    _run_page_scenario("""
      commandMode = 'hang-first';
      commandDelayMs = 10;
      vm.runInContext('press("w")', context);
      await new Promise(resolve => setTimeout(resolve, 340));
      assert.equal(cmdAborts, 1);
      assert.ok(cmdRequests.length >= 2);
      assert.ok(cmdRequests.at(-1).vx > 0);
    """)


def test_page_does_not_overlap_state_requests():
    """A delayed state poll prevents another state request from piling up."""
    _run_page_scenario("""
      await new Promise(resolve => setTimeout(resolve, 0));
      stateHang = true;
      const before = stateRequests;
      vm.runInContext('pollState(); pollState()', context);
      await new Promise(resolve => setTimeout(resolve, 20));
      assert.equal(stateRequests - before, 1);
      assert.equal(maxStateInFlight, 1);
    """)


def test_ros_deadman_remains_350ms():
    """Transport recovery must not weaken the established ROS deadman."""
    assert DEFAULT_FRESH_SEC == 0.35


def test_rejects_older_sequence_from_same_browser():
    """A delayed request cannot overwrite a newer held-key command."""
    order = CommandOrder()
    assert order.accept('browser-a', 1, 1.0)
    assert order.accept('browser-a', 3, 1.1)
    assert not order.accept('browser-a', 2, 1.2)


def test_active_browser_cannot_be_overridden_until_command_is_stale():
    """Only one browser owns the controller within the freshness window."""
    order = CommandOrder()
    assert order.accept('browser-a', 1, 1.0)
    assert not order.accept('browser-b', 1, 1.1)
    assert order.accept('browser-b', 1, 1.0 + 0.36)


def test_legacy_command_remains_compatible():
    """Existing clients without ordering metadata remain accepted."""
    order = CommandOrder()
    assert order.accept('', 0, 1.0)


def test_scaled_command_clamps_normalized_input():
    """Browser values outside the unit interval remain bounded."""
    assert scaled_command(2.0, -2.0, 0.15, 1.2) == (0.15, -1.2)


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
def test_scaled_command_rejects_nonfinite_values(value):
    """Nonfinite values never reach the motor command topic."""
    with pytest.raises(ValueError, match='finite'):
        scaled_command(value, 0.0, 0.15, 1.2)


def test_page_exposes_unrestricted_manual_state():
    """The manual controller never disables a direction from preflight."""
    from jdamr_cube_node.web_teleop import PAGE
    assert '수동 주행 · 출발 조건 차단 없음' in PAGE
    assert 'direction_blocked' not in PAGE
    assert 'button:disabled' not in PAGE


def test_nonzero_command_is_rejected_when_preflight_is_not_ready():
    """A browser cannot bypass the ROS-side readiness gate."""
    node = object.__new__(TeleopNode)
    node.require_preflight = True
    node.preflight_stale_sec = 1.5
    node._lock = threading.RLock()
    node._preflight = {'ready': False, 'blockers': ['라이다 입력 지연']}
    node._preflight_stamp = time.monotonic()
    assert node.set_cmd(0.5, 0.0, 'browser-a', 1) == (
        False, 'preflight_not_ready')


def test_stale_preflight_cannot_authorize_motion():
    """A formerly healthy graph cannot leave movement latched as ready."""
    node = object.__new__(TeleopNode)
    node.require_preflight = True
    node.preflight_stale_sec = 1.5
    node._lock = threading.RLock()
    node._preflight = {'ready': True, 'blockers': []}
    node._preflight_stamp = time.monotonic() - 2.0
    assert not node._drive_ready()


def test_blocked_direction_cannot_authorize_motion():
    """A direction blocked by current LiDAR points remains rejected."""
    node = object.__new__(TeleopNode)
    node.require_preflight = True
    node.preflight_stale_sec = 1.5
    node._lock = threading.RLock()
    node._preflight = {
        'ready': True,
        'blockers': [],
        'checks': {'directions': {
            'forward': {'clear': False, 'point_count': 3},
            'backward': {'clear': True, 'point_count': 0},
            'left': {'clear': True, 'point_count': 0},
            'right': {'clear': True, 'point_count': 0},
        }},
    }
    node._preflight_stamp = time.monotonic()
    assert node.set_cmd(0.5, 0.0, 'browser-a', 1) == (
        False, 'direction_blocked:forward')

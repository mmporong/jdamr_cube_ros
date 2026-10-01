#!/usr/bin/env python3
"""JD-AMR box service departure runbook as one PC-side command (new map, 2026-09-30).

A call runs: dock -> water_station (face approach, 5 s, escape) -> called table
(face approach, 5 s, escape) -> reverse docking. The operator's "출발 table_0N" stands in
for a web call.

The robot only moves in `go`, which starts the existing `box_service` executor on the Pi.
Every other subcommand is no-motion: PC display, Nav2 session in prepare-only mode,
localization seeding through the existing `activate_navigation`, home teaching.

    jdamr_depart.py sync            copy Phase 2 assets to the Pi (same paths)
    jdamr_depart.py display-start   PC map_server + relay import + markers + RViz + click capture
    jdamr_depart.py session-start   Pi Nav2 session (precision, prepare-only, LOCALHOST)
    jdamr_depart.py init            registered dock pose -> scan-matched pose -> activation
                                    (--click: RViz click instead, re-teaches home unless --keep-home)
    jdamr_depart.py go table_01     table cycle: observe -> align -> 5 cm -> 5 s -> escape -> dock
    jdamr_depart.py health          read-only: does a fresh Pi process receive map/TF/scan
    jdamr_depart.py recover         no-motion restart of sensors, session and display + init
    jdamr_depart.py status | stop | display-stop | session-stop
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time

import yaml

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import dock_survey  # noqa: E402
import scan_match  # noqa: E402

# mDNS (jdamr.local) failed twice on 2026-09-30; the Pi keeps 192.168.0.159 on this LAN.
HOST = 'lim@192.168.0.159'
HOME_DIR = Path(os.path.expanduser('~'))
BASE = HOME_DIR / 'jdamr_data/map_20260930_manual'
# Wall-aligned copy of the 14:23 Cartographer map (aligned/alignment.json).
P2 = BASE / 'aligned'
MAP_YAML = P2 / 'map.yaml'
# Only the map's unknown cells: satisfies the session's non-empty mask and keeps
# KeepoutFilter from turning unknown cells into free space.
KEEPOUT_YAML = P2 / 'keepout.yaml'
REGISTRY = P2 / 'service_destinations.yaml'
ANNOTATION = P2 / 'annotation.json'
CLICK = P2 / 'rviz_initialpose.json'
STATE = P2 / 'state.json'
PI_WS = '/home/lim/jdamr_ws'
PI_SESSION = f'{PI_WS}/src/jdamr_cube_ros/jdamr_cube_navigation/scripts/restaurant_session.sh'
PI_SHARE = f'{PI_WS}/install/jdamr_cube_navigation/share/jdamr_cube_navigation'
PI_BOX_CONTRACT = f'{PI_SHARE}/config/box_parking_contract.yaml'
PI_GEOMETRY = (f'{PI_WS}/install/jdamr_cube_description/share/jdamr_cube_description/'
               'config/new_base_geometry.yaml')
PI_MOUNT = '/home/lim/jdamr_deploy/camera_mount_confirmed_20260917.yaml'
ROS_ENV = ('export ROS_DOMAIN_ID=12 ROS_LOCALHOST_ONLY=0 '
           'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4')
# Nav2 session and every short-lived Pi participant use loopback UDP without shared
# memory: under LOCALHOST rmw_fastrtps always adds shared memory, and a joining
# participant froze the Nav2 container by invalidating its port (2026-09-30).
DDS_PROFILE = P2 / 'fastdds_udp_loopback.xml'
PI_DDS_ENV = ('export ROS_DOMAIN_ID=12 ROS_LOCALHOST_ONLY=0 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET '
              f'FASTRTPS_DEFAULT_PROFILES_FILE={DDS_PROFILE}; unset FASTDDS_BUILTIN_TRANSPORTS')
# Loopback UDP did not stop the container deadlock (churn test 2026-09-30 10:55);
# keep the project-standard LOCALHOST transport and test per-process Nav2 nodes.
PI_SOURCE = f'set +u; source /opt/ros/jazzy/setup.bash; source {PI_WS}/install/setup.bash; {ROS_ENV}'
SESSION_DISCOVERY = 'LOCALHOST'
# 9/29's last session ran without composition (plan v4.1 E4); a composed container
# deadlocked when a participant joined (three reproductions on 2026-09-30).
SESSION_COMPOSITION = 'false'
PC_SOURCE = ('set +u; source /opt/ros/jazzy/setup.bash; '
             f'source {HOME_DIR}/jdamr_rgbd_ws/install/setup.bash; {ROS_ENV}')
DISPLAY_UNITS = ('jdamr-p2-mapserver', 'jdamr-p2-relay', 'jdamr-p2-markers',
                 'jdamr-p2-rviz', 'jdamr-p2-click')
# PC-side bag of what the display relay already brings over (no Pi disk or load:
# the Pi had 844 MB free on 2026-09-30). /tf carries the odom trajectory and the
# AMCL corrections; the relay never carries command topics by design.
RECORD_UNIT = 'jdamr-p2-record'
RECORD_TOPICS = ('/tf', '/tf_static', '/scan', '/plan', '/amcl_pose')
# Box front-face centres and outward normals (destinations.json, operator RViz clicks).
DESTINATIONS = json.loads((P2 / 'destinations.json').read_text())['destinations']
VIA_ID = 'water_station'
TABLES = ('table_01', 'table_02')
# Distances from the face along its outward normal (box_service: final 0.115, face
# alignment 0.515, escape 0.565). Pre and observation lie on the same straight line so
# the robot arrives facing the box; their 0.42 m spacing stays above the BT goal
# tolerance (0.15 m). Pre 1.00 keeps 0.54 m between the water face and table_01's pre.
PRE_M, OBSERVE_M, ESCAPE_M = 1.00, 0.58, 0.565
HOME_EXIT_M = 0.70


def along_normal(dest_id, distance_m):
    spec = DESTINATIONS[dest_id]
    yaw = math.radians(spec['normal_deg'])
    return (spec['face_xy'][0] + distance_m * math.cos(yaw),
            spec['face_xy'][1] + distance_m * math.sin(yaw), yaw)


def approach_waypoints(dest_id):
    pre_x, pre_y, normal = along_normal(dest_id, PRE_M)
    obs_x, obs_y, _ = along_normal(dest_id, OBSERVE_M)
    facing = math.atan2(-math.sin(normal), -math.cos(normal))
    return [{'id': f'{dest_id}_pre', 'x': round(pre_x, 3), 'y': round(pre_y, 3)},
            {'id': f'{dest_id}_observation', 'x': round(obs_x, 3), 'y': round(obs_y, 3),
             'yaw': round(facing, 4)}]


REGION_RADIUS_M = 0.40
# A box face seen from the dock appears as a 0.25-0.7 m unmapped LiDAR line; smaller
# clusters (chair legs, bags) never move the region away from the user's marker.
UNMAPPED_REGION_MAX_M = 0.5
BOX_FACE_LENGTH_M = (0.25, 0.7)
# Budgets are not safety gates (the progress checker ends real stalls). Conservative
# upper bounds from plan v4.1 section 1.7: return about 500 s, table leg above 240 s.
TASK_TIMEOUT_S = 600.0
RETURN_TIMEOUT_S = 600.0
INIT_COVARIANCE = {'covariance_x_m2': 0.0025, 'covariance_y_m2': 0.0025,
                   'covariance_yaw_rad2': math.radians(5.0) ** 2}
# Three unmapped boxes near the dock lower the inlier share (0.48-0.49 at 15:20 with a
# consistent global match); the global agreement and the AMCL agreement stay the gates.
MATCH_MIN_INLIER = 0.45
MATCH_CLICK_XY_M = 0.5
MATCH_CLICK_YAW_RAD = math.radians(30.0)
AMCL_AGREE_XY_M = 0.05
AMCL_AGREE_YAW_RAD = math.radians(2.0)
CLICK_MAX_AGE_S = 1800.0
# The 2026-09-30 morning fix for new processes receiving nothing restarted all three.
SENSOR_UNITS = ('jdamr-base.service', 'jdamr-box-rgbd.service', 'jdamr-box-observer.service')
# box_service's first check (verify_live_maps, 30 s) runs before any motion command.
MAP_WAIT_FAILURE = 'live map data unavailable'


def log(message):
    print(time.strftime('%H:%M:%S'), message, flush=True)


def fail(message):
    log('STOP: ' + message)
    raise SystemExit(1)


def sh(command, timeout=120, check=True, capture=True):
    result = subprocess.run(['bash', '-c', command], text=True, timeout=timeout,
                            stdout=subprocess.PIPE if capture else None,
                            stderr=subprocess.STDOUT if capture else None)
    if check and result.returncode != 0:
        fail(f'command failed ({result.returncode}): {command}\n{result.stdout}')
    return result


def pi(command, timeout=120, check=True, stdin=None):
    args = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', HOST, command]
    result = subprocess.run(args, text=True, timeout=timeout, input=stdin,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if check and result.returncode != 0:
        fail(f'pi command failed ({result.returncode}): {command}\n{result.stdout}')
    return result


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_state():
    return json.loads(STATE.read_text()) if STATE.exists() else {}


def save_state(state):
    temporary = STATE.with_suffix('.tmp')
    temporary.write_text(json.dumps(state, indent=1))
    os.replace(temporary, STATE)


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def mark_localized(value):
    """Only a completed init localizes; a session restart or a failed init revokes it."""
    state = load_state()
    state['localized'] = value
    save_state(state)


def session_active():
    return pi('systemctl is-active jdamr-restaurant-navigation.service',
              check=False).stdout.strip() == 'active'


def refuse_during_cycle(action):
    """Session stops and sensor restarts would end a running cycle mid-drive."""
    result = pi("systemctl list-units --type=service --state=active,activating,deactivating "
                "--no-legend 'jdamr-table-cycle-*'", check=False)
    if result.returncode == 255:
        fail(f'ssh to the Pi failed; cannot tell whether a cycle is running: {result.stdout[-200:]}')
    busy = result.stdout.strip()
    if busy:
        fail(f'a table cycle is running; {action} would stop it (use stop first): {busy}')


# ---------------------------------------------------------------- assets
def write_routes(home):
    """Water route from the dock; table routes from the water station's escape pose."""
    keepout_image = KEEPOUT_YAML.parent / yaml.safe_load(KEEPOUT_YAML.read_text())['image']
    hx, hy, hyaw = home
    exit_wp = {'id': 'home_exit', 'x': round(hx + HOME_EXIT_M * math.cos(hyaw), 3),
               'y': round(hy + HOME_EXIT_M * math.sin(hyaw), 3)}
    escape = along_normal(VIA_ID, ESCAPE_M)
    plans = {VIA_ID: ((hx, hy), [exit_wp, *approach_waypoints(VIA_ID)],
                      'registry home taught at the operator placement (start = dock)')}
    for table_id in TABLES:
        plans[table_id] = ((escape[0], escape[1]), approach_waypoints(table_id),
                           'water_station escape pose (face + 0.565 m along its normal)')
    for dest_id, (start, waypoints, basis) in plans.items():
        route = {
            'schema_version': 1, 'frame_id': 'map',
            'map_yaml': str(MAP_YAML), 'keepout_mask_yaml': str(KEEPOUT_YAML),
            'expected_mask_sha256': sha256(keepout_image),
            'start_pose': {'x': round(float(start[0]), 3), 'y': round(float(start[1]), 3)},
            'max_route_start_distance_m': 0.3,
            'waypoints': waypoints,
            'purpose': f'{dest_id} box observation approach; not a final parking goal',
            'target_basis': f'destinations.json {DESTINATIONS[dest_id]["source"]}',
            'start_basis': basis,
        }
        path = P2 / f'{dest_id}_route.yaml'
        path.write_text(yaml.safe_dump(route, sort_keys=False, allow_unicode=True))
    log(f'routes written: home ({hx:.3f}, {hy:.3f}, {math.degrees(hyaw):.1f} deg), '
        f'water escape ({escape[0]:.3f}, {escape[1]:.3f})')


def cmd_sync(_args):
    pi(f'mkdir -p {shlex.quote(str(P2))}/runs')
    sh(f'rsync -a --exclude runs --exclude state.json --exclude rviz_initialpose.json '
       f'{shlex.quote(str(P2))}/ {HOST}:{shlex.quote(str(P2))}/')
    for name in ('service_destinations.yaml', 'keepout.pgm', 'keepout.yaml'):
        local = sha256(P2 / name)
        remote = pi(f'sha256sum {shlex.quote(str(P2 / name))}').stdout.split()[0]
        if local != remote:
            fail(f'{name} differs after sync')
    log('Phase 2 assets synced to the Pi (hashes match)')


def pull_registry():
    """The Pi registry is authoritative once teach-home ran there."""
    text = pi(f'cat {shlex.quote(str(REGISTRY))}').stdout
    REGISTRY.write_text(text)
    return yaml.safe_load(text)


# ---------------------------------------------------------------- display (PC)
def cmd_display_start(_args):
    missing = [u for u in DISPLAY_UNITS
               if sh(f'systemctl --user is-active {u}', check=False).stdout.strip() != 'active']
    if not missing:
        log('display already running')
        record_start()
        return
    env = '--setenv=DISPLAY=:1 --setenv=XAUTHORITY=/run/user/1000/gdm/Xauthority'
    commands = {
        'jdamr-p2-mapserver': (
            f'{PC_SOURCE}; ros2 run nav2_map_server map_server --ros-args '
            f'-p yaml_filename:={MAP_YAML} -p frame_id:=map & '
            'sleep 3; ros2 run nav2_util lifecycle_bringup map_server; wait'),
        'jdamr-p2-relay': (
            f'{PC_SOURCE}; exec python3 -m jdamr_cube_navigation.rviz_display_relay import '
            f'--host {HOST.split("@")[0]}@jdamr.local --workspace {PI_WS}'),
        'jdamr-p2-markers': (
            f'{PC_SOURCE}; exec ros2 run jdamr_cube_navigation service_visualization '
            f'--registry {REGISTRY} --annotation {ANNOTATION} --selected table_01'),
        'jdamr-p2-rviz': f'{PC_SOURCE}; exec rviz2 -d {P2}/restaurant_phase2.rviz',
        'jdamr-p2-click': f'{PC_SOURCE}; exec python3 {TOOLS}/initialpose_capture.py {CLICK}',
    }
    for unit in missing:
        command = commands[unit]
        sh(f'systemd-run --user --unit={unit} --collect {env} '
           f'--property=KillSignal=SIGINT --property=TimeoutStopSec=10 '
           f'/bin/bash -c {shlex.quote(command)}')
    time.sleep(4)
    states = {u: sh(f'systemctl --user is-active {u}', check=False).stdout.strip()
              for u in DISPLAY_UNITS}
    (P2 / 'display_units.txt').write_text(
        json.dumps({'started': time.time(), 'units': states,
                    'stop': 'jdamr_depart.py display-stop'}, indent=1))
    log(f'display units: {states}')
    if any(v != 'active' for v in states.values()):
        fail('a display unit is not active; see journalctl --user -u <unit>')
    record_start()


def record_start():
    """Start the PC bag once and confirm that it actually writes messages."""
    if sh(f'systemctl --user is-active {RECORD_UNIT}', check=False).stdout.strip() == 'active':
        log('bag recorder already running')
        return
    bag = P2 / 'runs' / f'bag_{time.strftime("%Y%m%d_%H%M%S")}'
    command = (f'{PC_SOURCE}; exec ros2 bag record -s mcap -o {shlex.quote(str(bag))} '
               + ' '.join(RECORD_TOPICS))
    sh(f'systemd-run --user --unit={RECORD_UNIT} --collect '
       f'--property=KillSignal=SIGINT --property=TimeoutStopSec=15 '
       f'/bin/bash -c {shlex.quote(command)}')
    for _ in range(15):
        time.sleep(2)
        written = sum(f.stat().st_size for f in bag.glob('*.mcap')) if bag.exists() else 0
        if written > 4096:
            state = load_state()
            state['bag'] = str(bag)
            save_state(state)
            log(f'bag recording: {bag} ({written} bytes so far)')
            return
    log(f'bag recorder started but nothing written yet: {bag} '
        '(the relay may still be connecting; check with status)')


def record_stop():
    sh(f'systemctl --user stop {RECORD_UNIT}', check=False)
    bag = load_state().get('bag')
    if bag and Path(bag).exists():
        size = sum(f.stat().st_size for f in Path(bag).glob('*'))
        log(f'bag closed: {bag} ({size} bytes)')


def cmd_display_stop(_args):
    record_stop()
    for unit in DISPLAY_UNITS:
        sh(f'systemctl --user stop {unit}', check=False)
    left = [u for u in DISPLAY_UNITS
            if sh(f'systemctl --user is-active {u}', check=False).stdout.strip() == 'active']
    log(f'display stopped; still active: {left}')


# ---------------------------------------------------------------- session (Pi)
SESSION_SCRIPT = r'''
{source}
was_active=$(systemctl is-active jdamr-restaurant-navigation.service)
start_at=$(date '+%Y-%m-%d %H:%M:%S')
if ! bash {session} start --registry {registry} --discovery-range {discovery} \
    --use-composition {composition} --precision-parking --prepare-only; then
  [ "$was_active" = active ] || exit 3
  # Identity mismatch (registry, params or contract changed): no-motion restart once.
  bash {session} stop || exit 3
  start_at=$(date '+%Y-%m-%d %H:%M:%S')
  was_active=inactive
  bash {session} start --registry {registry} --discovery-range {discovery} \
    --use-composition {composition} --precision-parking --prepare-only || exit 3
fi
[ "$was_active" = active ] && {{ echo REUSED; exit 0; }}
for i in $(seq 1 40); do
  J=$(journalctl -a -u jdamr-restaurant-navigation.service --since "$start_at" -o cat --no-pager)
  if grep -q 'lifecycle_manager_localization\].*Managed nodes are active' <<<"$J" &&
     grep -q 'lifecycle_manager_keepout\].*Managed nodes are active' <<<"$J"; then
    echo STARTED; exit 0
  fi
  sleep 3
done
exit 4
'''


def cmd_session_start(_args):
    refuse_during_cycle('session-start')
    script = SESSION_SCRIPT.format(source=PI_SOURCE, session=PI_SESSION, registry=REGISTRY,
                                   discovery=SESSION_DISCOVERY,
                                   composition=SESSION_COMPOSITION)
    try:
        result = pi('bash -s', stdin=script, timeout=240, check=False)
    except subprocess.TimeoutExpired:
        mark_localized(False)
        fail('Nav2 session start did not answer within 240 s; run status, then init')
    tail = result.stdout.strip().splitlines()[-3:]
    if not tail or tail[-1] != 'REUSED':
        # A new or restarted session holds no localization until init.
        mark_localized(False)
    if result.returncode != 0:
        fail(f'Nav2 session start failed (code {result.returncode}): {tail}')
    log(f'Nav2 session {tail[-1] if tail else ""}: precision, prepare-only, '
        f'{SESSION_DISCOVERY}, composition={SESSION_COMPOSITION}; '
        'AMCL waits for init, motion servers inactive')


def cmd_session_stop(_args):
    refuse_during_cycle('session-stop')
    mark_localized(False)
    pi(f'{PI_SOURCE}; bash {PI_SESSION} stop', timeout=90, check=False)
    log('session: ' + pi('systemctl is-active jdamr-restaurant-navigation.service',
                         check=False).stdout.strip())


# ---------------------------------------------------------------- init
def capture_scan(path):
    script = (TOOLS / 'capture_scan2.py').read_text()
    result = pi(f'{PI_SOURCE}; python3 - 40', stdin=script, timeout=60)
    line = next((l for l in result.stdout.splitlines() if l.startswith('JSON:')), None)
    if line is None:
        fail('scan capture returned no data (check linger and sensor services)')
    Path(path).write_text(line[5:])
    data = json.loads(line[5:])
    if len(data['scans']) < 20:
        fail(f'only {len(data["scans"])} scans captured')
    moving = max(max(abs(o['v']), abs(o['w'])) for o in data['odom'])
    if moving > 0.02:
        fail(f'robot is moving during capture (odom {moving:.3f})')
    return data


def refine_pose(scan_path, click):
    img, occ, free, res, origin = scan_match.load_map(str(MAP_YAML))
    pts, *_ = scan_match.laser_points(str(scan_path))
    score, _ = scan_match.make_score(occ, res, origin)
    seed = (click['x_m'], click['y_m'], click['yaw_rad'])
    local_score, pose = scan_match.search(score, pts, seed, xy_half=MATCH_CLICK_XY_M,
                                          yaw_half=MATCH_CLICK_YAW_RAD)
    inlier = float((score(pts, *pose)[1] < 0.05).mean())
    ranked = dock_survey.global_pose(score, pts, free, res, origin)
    best = ranked[0]['pose']
    agree = (math.dist(best[:2], pose[:2]) <= 0.1
             and abs(wrap(best[2] - pose[2])) <= math.radians(3.0))
    return {'pose': [float(v) for v in pose], 'mean_dist_m': float(local_score),
            'inlier_5cm': float(inlier), 'global_best': [float(v) for v in best],
            'global_agrees': bool(agree), 'click': [float(v) for v in seed],
            'click_offset_m': float(math.dist(seed[:2], pose[:2])),
            'click_offset_deg': float(math.degrees(wrap(seed[2] - pose[2])))}


def read_events(pi_path):
    text = pi(f'cat {shlex.quote(pi_path)}', check=False).stdout
    events = []
    for line in text.splitlines():
        try:
            events.append(json.loads(line))
        except json.JSONDecodeError:
            pass
    return events


def cmd_init(args):
    state = load_state()
    # Until this init completes, go must not depart on an earlier localization.
    state['localized'] = False
    save_state(state)
    refuse_during_cycle('init')
    if not session_active():
        fail('Nav2 session is not active; run session-start first')
    if args.seed_from_state:
        # Same placement only: the scan match below must agree globally with this seed.
        if 'pose' not in state:
            fail('no previous init pose; click 2D Pose Estimate in RViz')
        x, y, yaw = state['pose']
        click = {'x_m': x, 'y_m': y, 'yaw_rad': yaw, 'received_unix_s': time.time(),
                 'source': 'previous_init_scan_match'}
    elif getattr(args, 'click', False):
        if not CLICK.exists():
            fail(f'no RViz 2D Pose Estimate click saved yet ({CLICK})')
        click = json.loads(CLICK.read_text())
    else:
        # A new run starts at the dock: seed with the registered dock and keep it, so the
        # dock does not drift to wherever the last docking stopped.
        home = pull_registry().get('home') or state.get('home')
        if home is None:
            fail('no registered dock yet; run init --click at the dock once')
        click = {'x_m': home['x_m'], 'y_m': home['y_m'], 'yaw_rad': home['yaw_rad'],
                 'received_unix_s': time.time(), 'source': 'registered_dock'}
        args.keep_home = True
    age = time.time() - click['received_unix_s']
    if age > CLICK_MAX_AGE_S:
        fail(f'RViz click is {age:.0f} s old; click 2D Pose Estimate again')
    run = time.strftime('%Y%m%d_%H%M%S')
    run_dir = P2 / 'runs' / f'init_{run}'
    run_dir.mkdir(parents=True)
    pi(f'mkdir -p {shlex.quote(str(run_dir))}')
    log(f'click ({click["x_m"]:.3f}, {click["y_m"]:.3f}, '
        f'{math.degrees(click["yaw_rad"]):.1f} deg), age {age:.0f} s')
    capture_scan(run_dir / 'scan.json')
    refined = refine_pose(run_dir / 'scan.json', click)
    (run_dir / 'refine.json').write_text(json.dumps(refined, indent=1))
    pose = refined['pose']
    log(f'scan match ({pose[0]:.3f}, {pose[1]:.3f}, {math.degrees(pose[2]):.1f} deg) '
        f'inlier {refined["inlier_5cm"]:.2f}, click offset {refined["click_offset_m"]:.2f} m / '
        f'{refined["click_offset_deg"]:.1f} deg, global agrees {refined["global_agrees"]}')
    # --local-only: the operator stated the placement (e.g. pulled straight back and
    # turned to face a box); nearby boxes can outrank it globally. AMCL agreement stays.
    min_inlier = 0.35 if args.local_only else MATCH_MIN_INLIER
    if refined['inlier_5cm'] < min_inlier or not (refined['global_agrees'] or args.local_only):
        fail('scan does not match the map near the click; check placement or click again')
    initial = {'frame_id': 'map', 'x_m': pose[0], 'y_m': pose[1], 'yaw_rad': pose[2],
               **INIT_COVARIANCE,
               'provenance': {'purpose': 'localization_seed_from_scan_match',
                              'click': refined['click'], 'refine': 'refine.json',
                              'covariance_basis': 'seed spread (5 cm, 5 deg), not measured'}}
    (run_dir / 'initial_pose.yaml').write_text(yaml.safe_dump(initial, sort_keys=False))
    sh(f'rsync -a {shlex.quote(str(run_dir))}/ {HOST}:{shlex.quote(str(run_dir))}/')
    log('activating navigation (no motion) with the scan-matched seed')
    result = pi(f'{PI_SOURCE}; cd {PI_WS}; python3 -m jdamr_cube_navigation.activate_navigation '
                f'--registry {REGISTRY} --parking-contract {PI_BOX_CONTRACT} '
                f'--log {run_dir}/activation.jsonl --initial-pose {run_dir}/initial_pose.yaml',
                timeout=240, check=False)
    events = read_events(str(run_dir / 'activation.jsonl'))
    activated = [e for e in events if e.get('event') == 'navigation_activated_without_motion']
    if result.returncode != 0 or not activated:
        reasons = [e.get('reason') for e in events if e.get('event') == 'activation_failed']
        fail(f'activation failed: {reasons or result.stdout[-400:]}')
    amcl = activated[-1]['stationary_pose']
    dxy, dyaw = math.dist(amcl[:2], pose[:2]), wrap(amcl[2] - pose[2])
    log(f'AMCL ({amcl[0]:.3f}, {amcl[1]:.3f}, {math.degrees(amcl[2]):.1f} deg) vs scan match: '
        f'{dxy:.3f} m, {math.degrees(dyaw):.1f} deg')
    if dxy > AMCL_AGREE_XY_M or abs(dyaw) > AMCL_AGREE_YAW_RAD:
        fail('AMCL disagrees with the scan match after activation; do not depart')
    if not args.keep_home:
        pi(f'{PI_SOURCE}; cd {PI_WS}; python3 -m jdamr_cube_navigation.restaurant_service '
           f'teach-home --registry {REGISTRY} --log {run_dir}/teach_home.jsonl '
           '--pose-id home_dock --approach-offset-m 0.7 --parking-direction reverse --replace',
           timeout=90)
    registry = pull_registry()
    home = registry.get('home')
    if home is None:
        fail('registry has no home; run init without --keep-home')
    log(f'home_dock ({home["x_m"]:.3f}, {home["y_m"]:.3f}, '
        f'{math.degrees(home["yaw_rad"]):.1f} deg)')
    write_routes((home['x_m'], home['y_m'], home['yaw_rad']))
    sh(f'rsync -a {P2}/water_station_route.yaml {P2}/table_01_route.yaml '
       f'{P2}/table_02_route.yaml {HOST}:{P2}/')
    # Box candidates: persistent unmapped LiDAR returns near each marker.
    raw, n_scans = dock_survey.raw_points_map(str(run_dir / 'scan.json'), pose)
    img, occ, _free, res, origin = scan_match.load_map(str(MAP_YAML))
    import cv2
    import numpy as np
    dist = cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 5) * res
    groups = dock_survey.clusters(raw, n_scans, dist, res, origin, *occ.shape)
    regions = {}
    for table_id, spec in DESTINATIONS.items():
        marker = spec['face_xy']
        near = sorted((g for g in groups
                       if math.dist(g['centroid'], marker) <= UNMAPPED_REGION_MAX_M
                       and BOX_FACE_LENGTH_M[0] <= g['length_m'] <= BOX_FACE_LENGTH_M[1]),
                      key=lambda g: math.dist(g['centroid'], marker))
        center = near[0]['centroid'] if near else list(marker)
        regions[table_id] = {'xy': center, 'radius_m': REGION_RADIUS_M,
                             'basis': ('unmapped LiDAR cluster near marker' if near
                                       else 'marker (no box-sized unmapped LiDAR cluster within 0.5 m)'),
                             'candidates': near[:3]}
        log(f'{table_id} region ({center[0]:.3f}, {center[1]:.3f}) r={REGION_RADIUS_M}: '
            f'{regions[table_id]["basis"]}')
    (run_dir / 'regions.json').write_text(json.dumps(regions, indent=1))
    state.update({'init_run': str(run_dir), 'pose': pose, 'amcl': amcl, 'home': home,
                  'regions': regions, 'init_time': time.time(), 'localized': True,
                  'init_local_only': bool(args.local_only)})
    save_state(state)
    log('init complete: robot localized at the dock, navigation active, no motion sent')


# ---------------------------------------------------------------- go
def map_wait_failed(events):
    """Only the executor's first check ran and it received no map: nothing moved."""
    return (len(events) == 1 and events[0].get('event') == 'failed'
            and str(events[0].get('reason', '')).startswith(MAP_WAIT_FAILURE))


def run_cycle(args, state, table_id):
    """Start one box_service run on the Pi and follow its log until the unit ends."""
    region = state['regions'][table_id]
    via = state['regions'][VIA_ID]
    if args.region:
        region = {'xy': args.region[:2], 'radius_m': args.region[2], 'basis': 'operator-confirmed box'}
    run = time.strftime('%Y%m%d_%H%M%S')
    run_dir = P2 / 'runs' / f'{table_id}_{run}'
    run_dir.mkdir(parents=True)
    pi(f'mkdir -p {shlex.quote(str(run_dir))}')
    unit = f'jdamr-table-cycle-{run.replace("_", "-")}'
    route = Path(args.route) if args.route else P2 / f'{table_id}_route.yaml'
    command = (f'{PI_SOURCE}; exec python3 -m jdamr_cube_navigation.box_service '
               f'--registry {REGISTRY} --approach-route {route} --camera-mount {PI_MOUNT} '
               f'--geometry {PI_GEOMETRY} --parking-contract {PI_BOX_CONTRACT} '
               f'--table-id {table_id} --region-xy {region["xy"][0]} {region["xy"][1]} '
               f'--region-radius-m {region["radius_m"]} --log {run_dir}/cycle_events.jsonl '
               f'--candidate-trial --execute --search --task-timeout-s {TASK_TIMEOUT_S} '
               f'--return-home --return-timeout-s {RETURN_TIMEOUT_S} '
               + (' --home-only ' if args.dock_only else '')
               + (' --graceful-final ' if args.graceful_final else '')
               + (' --resume-at-observation ' if args.resume_at_observation else '')
               + (f' --resume-parked-from-log {shlex.quote(args.resume_parked_log)} '
                  if args.resume_parked_log else '')
               + ('' if args.skip_via or args.dock_only else
                  f'--via-id {VIA_ID} --via-route {args.via_route or P2 / (VIA_ID + "_route.yaml")} '
                  f'--via-region-xy {via["xy"][0]} {via["xy"][1]} '
                  f'--via-region-radius-m {via["radius_m"]}'))
    (run_dir / 'command.txt').write_text(command + '\n')
    pi(f'sudo -n systemd-run --unit={unit} --collect --property=User=lim '
       '--property=KillMode=mixed --property=KillSignal=SIGINT --property=TimeoutStopSec=20 '
       f'--setenv=HOME=/home/lim --working-directory={PI_WS} /bin/bash -c {shlex.quote(command)}')
    fresh = load_state()
    fresh.update({'last_unit': unit, 'last_run': str(run_dir)})
    save_state(fresh)
    stops = ('dock only' if args.dock_only else
             f'{table_id}' if args.skip_via else f'{VIA_ID} -> {table_id}')
    log(f'DEPARTED {stops} -> dock: unit {unit}, log {run_dir}/cycle_events.jsonl')
    seen = 0
    while True:
        time.sleep(10)
        events = read_events(str(run_dir / 'cycle_events.jsonl'))
        for event in events[seen:]:
            keep = {k: event[k] for k in ('event', 'phase', 'reason', 'waypoint_id',
                                          'terminal_status_code', 'nav2_error_code')
                    if k in event}
            log('  ' + json.dumps(keep, ensure_ascii=False))
        seen = len(events)
        active = pi(f'systemctl is-active {unit}', check=False).stdout.strip()
        if active not in ('active', 'activating'):
            # The last lines may land between the read above and this check.
            final = read_events(str(run_dir / 'cycle_events.jsonl'))
            for event in final[seen:]:
                keep = {k: event[k] for k in ('event', 'phase', 'reason', 'waypoint_id',
                                              'terminal_status_code', 'nav2_error_code')
                        if k in event}
                log('  ' + json.dumps(keep, ensure_ascii=False))
            return final


def stop_requested_since(started):
    """An operator stop ends this go: no recovery, no re-departure."""
    if load_state().get('stop_unix', 0.0) >= started:
        fail('stop requested; no re-departure')


def cmd_go(args):
    started = time.time()
    state = load_state()
    if 'regions' not in state or state.get('localized') is not True:
        fail('not localized: the session restarted or an init failed since the last init; '
             'run init')
    if not session_active():
        fail('Nav2 session is not active')
    refuse_during_cycle('a new departure')
    table_id = args.table_id
    events = run_cycle(args, state, table_id)
    if map_wait_failed(events):
        # 2026-09-30: new processes stopped receiving map/TF/scan from the running base
        # and session; fresh services fixed it every time. The executor's own map check
        # is the detector, so a healthy departure carries no extra check.
        resumed = (args.skip_via or args.resume_at_observation or args.resume_parked_log
                   or args.dock_only)
        if resumed or state.get('init_local_only'):
            fail('the executor received no map before moving; the robot is not at a globally '
                 'matched init pose, so run recover --seed X Y YAW_DEG [--local-only], '
                 'then go again')
        stop_requested_since(started)
        log('the executor received no map before moving; recovering without motion, '
            'then departing once more')
        args.seed, args.local_only = None, False
        cmd_recover(args)
        stop_requested_since(started)
        refuse_during_cycle('the re-departure')
        events = run_cycle(args, load_state(), table_id)
    names = [e.get('event') for e in events]
    arrived = [e for e in events if e.get('event') == 'home_arrived']
    home = bool(arrived) and arrived[-1].get('confirmation', {}).get('confirmed') is True
    log(f'cycle ended: parked={"box_approach_finished" in names} '
        f'dwell={"parked_dwell_complete" in names} escape={"box_escape_finished" in names} '
        f'home={home}')
    if not home:
        fail('the cycle did not end confirmed at the dock; see the last events above')


def dds_health():
    """Read-only probe: returns ('ok' | 'missing' | 'error', report line)."""
    script = (TOOLS / 'dds_probe.py').read_text()
    try:
        result = pi(f'{PI_SOURCE}; timeout 45 python3 - health', stdin=script, timeout=60,
                    check=False)
    except subprocess.TimeoutExpired:
        return 'error', 'probe did not finish within 60 s'
    line = next((text for text in result.stdout.splitlines() if text.startswith('PROBE ')), None)
    if line is None:
        return 'error', f'probe did not run (code {result.returncode}): {result.stdout[-300:]}'
    return ('ok' if line.split()[2:3] == ['ok'] else 'missing'), line


def cmd_health(_args):
    status, line = dds_health()
    log(f'DDS {status}: {line}')
    if status == 'missing':
        fail('fresh processes do not receive map/TF/scan; run recover')
    if status == 'error':
        fail('the probe could not run; check ssh and the Pi ROS environment')


def unit_state(unit):
    text = pi(f'systemctl show -p ActiveState -p ActiveEnterTimestampMonotonic {unit}',
              check=False).stdout
    state = dict(line.split('=', 1) for line in text.splitlines() if '=' in line)
    return state.get('ActiveState'), state.get('ActiveEnterTimestampMonotonic')


def restart_sensor_services():
    """Restart each running sensor service and confirm it really started again."""
    for unit in SENSOR_UNITS:
        before = unit_state(unit)
        if before[0] != 'active':
            # A box cycle needs all three; None means the ssh read failed.
            fail(f'{unit} is {before[0]}; start it before recovering')
        pi(f'sudo -n systemctl restart {unit}', timeout=200, check=False)
        after = unit_state(unit)
        if after[0] != 'active' or after[1] == before[1]:
            fail(f'{unit} did not restart (before {before}, after {after})')
        log(f'{unit}: restarted')
    time.sleep(8)


def cmd_recover(args):
    """Fresh DDS state without motion: sensor services, session, display, then init.

    Without --seed the robot must still be where the last init placed it (the dock
    before a departure): the scan match must then agree globally with that pose.
    --local-only (operator-stated placement) is never implied.
    """
    if args.local_only and args.seed is None:
        fail('--local-only needs --seed: without it the last init pose must agree globally')
    refuse_during_cycle('recover')
    linger = pi('loginctl show-user lim -p Linger --value', check=False).stdout.strip()
    if linger != 'yes':
        # 2026-09-30 morning: without linger the last ssh logout removed DDS shared memory.
        fail(f'Pi user lim linger is {linger or "unknown"}; run sudo loginctl enable-linger lim')
    mark_localized(False)
    cmd_display_stop(args)
    cmd_session_stop(args)
    restart_sensor_services()
    cmd_display_start(args)
    cmd_session_start(args)
    args.keep_home, args.seed_from_state = True, args.seed is None
    args.click = args.seed is not None
    if args.seed is not None:
        x, y, yaw_deg = args.seed
        CLICK.write_text(json.dumps({'x_m': x, 'y_m': y, 'yaw_rad': math.radians(yaw_deg),
                                     'received_unix_s': time.time(), 'source': 'recover seed'}))
    try:
        cmd_init(args)
    except SystemExit:
        log('recover: services are fresh but init stopped. If the robot is not at the last '
            'init pose, click 2D Pose Estimate and run init --click, or run recover --seed X Y YAW_DEG')
        raise
    status, line = dds_health()
    if status != 'ok':
        fail(f'DDS {status} after recovery ({line})')
    log('DDS OK after recovery: ' + line)


def cmd_stop(_args):
    state = load_state()
    state['stop_unix'] = time.time()
    save_state(state)
    unit = state.get('last_unit')
    if unit:
        pi(f'sudo -n systemctl stop {unit}', check=False)
        log(f'{unit}: ' + pi(f'systemctl is-active {unit}', check=False).stdout.strip())


def cmd_status(_args):
    state = load_state()
    log('session: ' + pi('systemctl is-active jdamr-restaurant-navigation.service',
                         check=False).stdout.strip())
    for unit in (*DISPLAY_UNITS, RECORD_UNIT):
        log(f'{unit}: ' + sh(f'systemctl --user is-active {unit}', check=False).stdout.strip())
    if state.get('last_unit'):
        log(f'{state["last_unit"]}: ' + pi(f'systemctl is-active {state["last_unit"]}',
                                           check=False).stdout.strip())
    log('state: ' + json.dumps({k: state[k] for k in ('pose', 'home', 'last_run', 'bag')
                                if k in state}))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)
    for name in ('sync', 'display-start', 'display-stop', 'session-start', 'session-stop',
                 'stop', 'status', 'record-start', 'record-stop'):
        sub.add_parser(name)
    init = sub.add_parser('init')
    init.add_argument('--keep-home', action='store_true',
                      help='do not re-teach home at this placement')
    init.add_argument('--local-only', action='store_true',
                      help='operator-stated placement: skip the global agreement check')
    init.add_argument('--seed-from-state', action='store_true',
                      help='robot not moved since the last init: seed with that pose')
    init.add_argument('--click', action='store_true',
                      help='robot placed away from the registered dock: seed with the RViz click')
    go = sub.add_parser('go')
    go.add_argument('table_id', choices=TABLES)
    go.add_argument('--route', help='route file (default: <table_id>_route.yaml)')
    go.add_argument('--region', nargs=3, type=float, metavar=('X', 'Y', 'R'))
    go.add_argument('--via-route', help='water_station route (default: <P2>/water_station_route.yaml)')
    go.add_argument('--skip-via', action='store_true', help='water stop already done: table then dock')
    go.add_argument('--dock-only', action='store_true', help='return to the dock only (re-dock)')
    go.add_argument('--graceful-final', action='store_true',
                    help='Graceful box approach and dock leg instead of RPP (default)')
    go.add_argument('--resume-at-observation', action='store_true',
                    help='robot already at the first stop observation point (water_station unless --skip-via)')
    go.add_argument('--resume-parked-log',
                    help='cycle_events.jsonl of a run that stopped at the first box: hold, escape, continue')
    sub.add_parser('health')
    recover = sub.add_parser('recover')
    recover.add_argument('--seed', nargs=3, type=float, metavar=('X', 'Y', 'YAW_DEG'),
                         help='placement when the robot is not at the last init pose')
    recover.add_argument('--local-only', action='store_true',
                         help='operator-stated placement: skip the global agreement check')
    routes = sub.add_parser('routes')
    routes.add_argument('--start', nargs=3, type=float, required=True,
                        metavar=('X', 'Y', 'YAW_RAD'))
    args = parser.parse_args()
    handlers = {'sync': cmd_sync, 'display-start': cmd_display_start,
                'display-stop': cmd_display_stop, 'session-start': cmd_session_start,
                'session-stop': cmd_session_stop, 'init': cmd_init, 'go': cmd_go,
                'stop': cmd_stop, 'status': cmd_status, 'health': cmd_health,
                'recover': cmd_recover,
                'record-start': lambda _a: record_start(),
                'record-stop': lambda _a: record_stop(),
                'routes': lambda a: write_routes(a.start)}
    handlers[args.command](args)


if __name__ == '__main__':
    main()

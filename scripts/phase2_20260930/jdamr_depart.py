#!/usr/bin/env python3
"""JD-AMR box service departure runbook as one PC-side command (Phase 2, 2026-09-30).

The robot only moves in `go`, which starts the existing `box_service` executor on the Pi.
Every other subcommand is no-motion: PC display, Nav2 session in prepare-only mode,
localization seeding through the existing `activate_navigation`, home teaching.

    jdamr_depart.py sync            copy Phase 2 assets to the Pi (same paths)
    jdamr_depart.py display-start   PC map_server + relay import + markers + RViz + click capture
    jdamr_depart.py session-start   Pi Nav2 session (precision, prepare-only, LOCALHOST)
    jdamr_depart.py init            RViz click -> scan-matched pose -> activation -> teach home
    jdamr_depart.py go table_01     table cycle: observe -> align -> 5 cm -> 5 s -> escape -> dock
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
BASE = HOME_DIR / 'jdamr_data/map_update_20260929_4XUrkc'
P2 = BASE / 'phase2_20260930'
MAP_YAML = P2 / 'map_dockfix.yaml'  # dock artifacts cleared (dockfix_provenance.json)
KEEPOUT_YAML = P2 / 'keepout.yaml'
REGISTRY = P2 / 'service_destinations.yaml'
ANNOTATION = BASE / 'departure_mask_v2/annotation.json'
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
HOME_EXIT = {'id': 'home_exit', 'x': -0.255, 'y': 0.2}
# Observation waypoints: 0.98 m camera-to-marker, marker 18 deg inside the 29 deg
# half field of view, rotation circle 0.43 m clear of map obstacles and keepout
# (offline check with geometry_check.py on the Phase 2 keepout copy).
TABLES = {
    'table_01': {'marker': (1.896, 0.303), 'waypoints': [
        dict(HOME_EXIT), {'id': 'table_01_pre', 'x': 0.4, 'y': 0.05},
        {'id': 'table_01_observation', 'x': 0.9, 'y': 0.0, 'yaw': 0.0}]},
    'table_02': {'marker': (3.119, -0.924), 'waypoints': [
        dict(HOME_EXIT), {'id': 'table_02_pre', 'x': 1.846, 'y': -0.076},
        {'id': 'table_02_observation', 'x': 2.2, 'y': -0.43, 'yaw': -math.pi / 4}]},
}
REGION_RADIUS_M = 0.45
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
MATCH_MIN_INLIER = 0.5
MATCH_CLICK_XY_M = 0.5
MATCH_CLICK_YAW_RAD = math.radians(30.0)
AMCL_AGREE_XY_M = 0.05
AMCL_AGREE_YAW_RAD = math.radians(2.0)
CLICK_MAX_AGE_S = 1800.0


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


# ---------------------------------------------------------------- assets
def write_routes(start_xy):
    keepout_image = KEEPOUT_YAML.parent / yaml.safe_load(KEEPOUT_YAML.read_text())['image']
    for table_id, spec in TABLES.items():
        route = {
            'schema_version': 1, 'frame_id': 'map',
            'map_yaml': str(MAP_YAML), 'keepout_mask_yaml': str(KEEPOUT_YAML),
            'expected_mask_sha256': sha256(keepout_image),
            'start_pose': {'x': float(start_xy[0]), 'y': float(start_xy[1])},
            'max_route_start_distance_m': 0.3,
            'waypoints': spec['waypoints'],
            'purpose': f'{table_id} box observation approach; not a final parking goal',
            'target_basis': ('marker from the user map annotation; observation waypoint '
                             'offline-checked for rotation and footprint clearance'),
            'start_basis': 'registry home taught at the operator placement (start = dock)',
        }
        path = P2 / f'{table_id}_route.yaml'
        path.write_text(yaml.safe_dump(route, sort_keys=False, allow_unicode=True))
    log(f'routes written with start ({start_xy[0]:.3f}, {start_xy[1]:.3f})')


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
    running = [u for u in DISPLAY_UNITS
               if sh(f'systemctl --user is-active {u}', check=False).stdout.strip() == 'active']
    if running:
        log(f'display already running: {running}')
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
    for unit, command in commands.items():
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


def cmd_display_stop(_args):
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
    script = SESSION_SCRIPT.format(source=PI_SOURCE, session=PI_SESSION, registry=REGISTRY,
                                   discovery=SESSION_DISCOVERY,
                                   composition=SESSION_COMPOSITION)
    result = pi('bash -s', stdin=script, timeout=240, check=False)
    tail = result.stdout.strip().splitlines()[-3:]
    if result.returncode != 0:
        fail(f'Nav2 session start failed (code {result.returncode}): {tail}')
    log(f'Nav2 session {tail[-1] if tail else ""}: precision, prepare-only, '
        f'{SESSION_DISCOVERY}, composition={SESSION_COMPOSITION}; '
        'AMCL waits for init, motion servers inactive')


def cmd_session_stop(_args):
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
    if pi('systemctl is-active jdamr-restaurant-navigation.service',
          check=False).stdout.strip() != 'active':
        fail('Nav2 session is not active; run session-start first')
    if args.seed_from_state:
        # Same placement only: the scan match below must agree globally with this seed.
        if 'pose' not in state:
            fail('no previous init pose; click 2D Pose Estimate in RViz')
        x, y, yaw = state['pose']
        click = {'x_m': x, 'y_m': y, 'yaw_rad': yaw, 'received_unix_s': time.time(),
                 'source': 'previous_init_scan_match'}
    else:
        if not CLICK.exists():
            fail(f'no RViz 2D Pose Estimate click saved yet ({CLICK})')
        click = json.loads(CLICK.read_text())
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
    if refined['inlier_5cm'] < MATCH_MIN_INLIER or not refined['global_agrees']:
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
    write_routes((home['x_m'], home['y_m']))
    sh(f'rsync -a {P2}/table_01_route.yaml {P2}/table_02_route.yaml {HOST}:{P2}/')
    # Box candidates: persistent unmapped LiDAR returns near each marker.
    raw, n_scans = dock_survey.raw_points_map(str(run_dir / 'scan.json'), pose)
    img, occ, _free, res, origin = scan_match.load_map(str(MAP_YAML))
    import cv2
    import numpy as np
    dist = cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 5) * res
    groups = dock_survey.clusters(raw, n_scans, dist, res, origin, *occ.shape)
    regions = {}
    for table_id, spec in TABLES.items():
        near = sorted((g for g in groups
                       if math.dist(g['centroid'], spec['marker']) <= UNMAPPED_REGION_MAX_M
                       and BOX_FACE_LENGTH_M[0] <= g['length_m'] <= BOX_FACE_LENGTH_M[1]),
                      key=lambda g: math.dist(g['centroid'], spec['marker']))
        center = near[0]['centroid'] if near else list(spec['marker'])
        regions[table_id] = {'xy': center, 'radius_m': REGION_RADIUS_M,
                             'basis': ('unmapped LiDAR cluster near marker' if near
                                       else 'marker (no box-sized unmapped LiDAR cluster within 0.5 m)'),
                             'candidates': near[:3]}
        log(f'{table_id} region ({center[0]:.3f}, {center[1]:.3f}) r={REGION_RADIUS_M}: '
            f'{regions[table_id]["basis"]}')
    (run_dir / 'regions.json').write_text(json.dumps(regions, indent=1))
    state.update({'init_run': str(run_dir), 'pose': pose, 'amcl': amcl, 'home': home,
                  'regions': regions, 'init_time': time.time()})
    save_state(state)
    log('init complete: robot localized at the dock, navigation active, no motion sent')


# ---------------------------------------------------------------- go
def cmd_go(args):
    state = load_state()
    if 'regions' not in state:
        fail('run init first')
    table_id = args.table_id
    region = state['regions'][table_id]
    if args.region:
        region = {'xy': args.region[:2], 'radius_m': args.region[2], 'basis': 'operator-confirmed box'}
    if pi('systemctl is-active jdamr-restaurant-navigation.service',
          check=False).stdout.strip() != 'active':
        fail('Nav2 session is not active')
    busy = pi("systemctl list-units --type=service --state=active --no-legend "
              "'jdamr-table-cycle-*'", check=False).stdout.strip()
    if busy:
        fail(f'a table cycle is already running: {busy}')
    run = time.strftime('%Y%m%d_%H%M%S')
    run_dir = P2 / 'runs' / f'{table_id}_{run}'
    run_dir.mkdir(parents=True)
    pi(f'mkdir -p {shlex.quote(str(run_dir))}')
    unit = f'jdamr-table-cycle-{run.replace("_", "-")}'
    route = Path(args.route) if args.route else P2 / f'{table_id}_route.yaml'
    resume = ' --resume-at-observation' if args.resume_at_observation else ''
    command = (f'{PI_SOURCE}; exec python3 -m jdamr_cube_navigation.box_service '
               f'--registry {REGISTRY} --approach-route {route} --camera-mount {PI_MOUNT} '
               f'--geometry {PI_GEOMETRY} --parking-contract {PI_BOX_CONTRACT} '
               f'--table-id {table_id} --region-xy {region["xy"][0]} {region["xy"][1]} '
               f'--region-radius-m {region["radius_m"]} --log {run_dir}/cycle_events.jsonl '
               f'--candidate-trial --execute --search --task-timeout-s {TASK_TIMEOUT_S} '
               f'--return-home --return-timeout-s {RETURN_TIMEOUT_S}{resume}')
    (run_dir / 'command.txt').write_text(command + '\n')
    pi(f'sudo -n systemd-run --unit={unit} --collect --property=User=lim '
       '--property=KillMode=mixed --property=KillSignal=SIGINT --property=TimeoutStopSec=20 '
       f'--setenv=HOME=/home/lim --working-directory={PI_WS} /bin/bash -c {shlex.quote(command)}')
    state.update({'last_unit': unit, 'last_run': str(run_dir)})
    save_state(state)
    log(f'DEPARTED {table_id}: unit {unit}, log {run_dir}/cycle_events.jsonl')
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
            break
    names = [e.get('event') for e in events]
    arrived = [e for e in events if e.get('event') == 'home_arrived']
    log(f'cycle ended: parked={"box_approach_finished" in names} '
        f'dwell={"parked_dwell_complete" in names} escape={"box_escape_finished" in names} '
        f'home={bool(arrived) and arrived[-1].get("confirmation", {}).get("confirmed") is True}')


def cmd_stop(_args):
    unit = load_state().get('last_unit')
    if unit:
        pi(f'sudo -n systemctl stop {unit}', check=False)
        log(f'{unit}: ' + pi(f'systemctl is-active {unit}', check=False).stdout.strip())


def cmd_status(_args):
    state = load_state()
    log('session: ' + pi('systemctl is-active jdamr-restaurant-navigation.service',
                         check=False).stdout.strip())
    for unit in DISPLAY_UNITS:
        log(f'{unit}: ' + sh(f'systemctl --user is-active {unit}', check=False).stdout.strip())
    if state.get('last_unit'):
        log(f'{state["last_unit"]}: ' + pi(f'systemctl is-active {state["last_unit"]}',
                                           check=False).stdout.strip())
    log('state: ' + json.dumps({k: state[k] for k in ('pose', 'home', 'last_run')
                                if k in state}))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)
    for name in ('sync', 'display-start', 'display-stop', 'session-start', 'session-stop',
                 'stop', 'status'):
        sub.add_parser(name)
    init = sub.add_parser('init')
    init.add_argument('--keep-home', action='store_true',
                      help='do not re-teach home at this placement')
    init.add_argument('--seed-from-state', action='store_true',
                      help='robot not moved since the last init: seed with that pose')
    go = sub.add_parser('go')
    go.add_argument('table_id', choices=sorted(TABLES))
    go.add_argument('--resume-at-observation', action='store_true')
    go.add_argument('--route', help='route file (default: <table_id>_route.yaml)')
    go.add_argument('--region', nargs=3, type=float, metavar=('X', 'Y', 'R'))
    routes = sub.add_parser('routes')
    routes.add_argument('--start', nargs=2, type=float, required=True)
    args = parser.parse_args()
    handlers = {'sync': cmd_sync, 'display-start': cmd_display_start,
                'display-stop': cmd_display_stop, 'session-start': cmd_session_start,
                'session-stop': cmd_session_stop, 'init': cmd_init, 'go': cmd_go,
                'stop': cmd_stop, 'status': cmd_status,
                'routes': lambda a: write_routes(a.start)}
    handlers[args.command](args)


if __name__ == '__main__':
    main()

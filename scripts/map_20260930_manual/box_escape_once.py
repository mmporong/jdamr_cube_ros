"""Run only the box escape (straight reverse to 0.565 m) of box_service, once.

Used when a box approach stopped close to the face and a new run cannot observe it
(camera inside the depth minimum) or turn (box inside the rotation guard). The face
frame is the robot's own heading with the measured face distance, so the reverse moves
straight away from the face the robot is facing. Everything else (reverse path
validation, ParkingReverse controller, stop confirmation) is the executor's own code.

    python3 box_escape_once.py <registry.yaml> <log.jsonl> <face_distance_m>
    python3 box_escape_once.py <registry.yaml> <log.jsonl> face <x> <y> <nx> <ny> [dwell_s]
      (face frame from a logged observation; optional verified dwell before the escape)
    --stop-id <id> labels the log (default water_station).
"""
import argparse
import json
import math
from pathlib import Path
import signal
import time

from ament_index_python.packages import get_package_share_directory
from jdamr_cube_navigation.box_service import BoxServiceRoute
from jdamr_cube_navigation.parking import load_parking_contract
from jdamr_cube_navigation.service_destinations import load_registry
import rclpy
from rclpy.signals import SignalHandlerOptions


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('registry', type=Path)
    parser.add_argument('log', type=Path)
    parser.add_argument('mode', help="face distance in metres, or 'face'")
    parser.add_argument('face', nargs='*', type=float, help='X Y NX NY [DWELL_S] after face')
    parser.add_argument('--stop-id', default='water_station')
    args = parser.parse_args()
    log = args.log
    logged = args.mode == 'face'
    if logged:
        if len(args.face) not in (4, 5):
            parser.error('face needs X Y NX NY [DWELL_S]')
        logged_face = args.face[0:2]
        norm = math.hypot(*args.face[2:4])
        # The executor's facing (cos 3 deg) and distance checks assume a unit normal.
        if not abs(norm - 1.0) <= 0.01:
            parser.error(f'outward normal must be a unit vector (|n| = {norm:.3f})')
        logged_normal = [args.face[2] / norm, args.face[3] / norm]
        dwell_s = args.face[4] if len(args.face) == 5 else 0.0
    else:
        if args.face:
            parser.error('extra values are only used with face')
        try:
            face_distance_m = float(args.mode)
        except ValueError:
            parser.error("mode must be a face distance in metres or 'face'")
        dwell_s = 0.0
        if not 0.10 <= face_distance_m <= 0.55:
            raise SystemExit('face distance must be within the escape range')
    registry = load_registry(args.registry)
    share = Path(get_package_share_directory('jdamr_cube_navigation'))
    contract = load_parking_contract(share / 'config/box_parking_contract.yaml')
    home_contract = load_parking_contract(share / 'config/parking_contract.yaml')
    with log.open('x', encoding='utf-8') as stream:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = BoxServiceRoute(registry, contract, stream, home_contract=home_contract)
        # A dropped ssh session (SIGHUP) or a stop cancels the reverse like box_service.
        handlers = {signum: signal.signal(signum, lambda *_: node.request_stop())
                    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
        ok = False
        try:
            node.verify_live_maps()
            if not node.wait_until_ready(timeout=10.0):
                raise RuntimeError('localization or sensor data unavailable')
            (x, y, yaw), _ = node.capture_stationary_pose()
            if logged:
                face, normal = logged_face, logged_normal
                face_distance_m = (x - face[0]) * normal[0] + (y - face[1]) * normal[1]
            else:
                face = [x + face_distance_m * math.cos(yaw), y + face_distance_m * math.sin(yaw)]
                normal = [-math.cos(yaw), -math.sin(yaw)]
            node.table_id = args.stop_id
            node.config, node.waypoints = {'frame_id': 'map', 'waypoints': []}, []
            node.last_box_face = {'face_center_map_xy_m': face, 'outward_normal_map_xy': normal}
            node.emit('escape_face_from_heading', face_center_map_xy_m=face,
                      outward_normal_map_xy=normal, face_distance_m=face_distance_m,
                      basis='logged observation' if logged else 'robot heading and measured face distance')
            if dwell_s > 0.0:
                # A new process has no final command for wait_parked(); hold the
                # stationary stop and confirm it again after the dwell.
                node.emit('parked_dwell_started', dwell_s=dwell_s)
                time.sleep(dwell_s)
                (x2, y2, _yaw2), _ = node.capture_stationary_pose()
                moved_m = math.dist((x, y), (x2, y2))
                if moved_m > 0.02:
                    # Same bound as box_service.resume_parked: a moving stop is no dwell.
                    node.emit('failed', phase='resume_parked_dwell', moved_m=moved_m)
                    raise RuntimeError('robot moved during the dwell')
                node.emit('parked_dwell_complete', dwell_s=dwell_s, moved_m=moved_m)
            ok = node._leave_parked_pose()
        finally:
            confirmed = node.finish_navigation()
            for signum, handler in handlers.items():
                signal.signal(signum, handler)
            node.destroy_node()
            rclpy.shutdown()
        print(json.dumps({'escaped': bool(ok), 'navigation_finished': bool(confirmed)}))
        return 0 if ok and confirmed else 1


if __name__ == '__main__':
    raise SystemExit(main())

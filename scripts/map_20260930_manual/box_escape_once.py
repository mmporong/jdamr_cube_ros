"""Run only the box escape (straight reverse to 0.565 m) of box_service, once.

Used when a box approach stopped close to the face and a new run cannot observe it
(camera inside the depth minimum) or turn (box inside the rotation guard). The face
frame is the robot's own heading with the measured face distance, so the reverse moves
straight away from the face the robot is facing. Everything else (reverse path
validation, ParkingReverse controller, stop confirmation) is the executor's own code.

    python3 box_escape_once.py <registry.yaml> <log.jsonl> <face_distance_m>
    python3 box_escape_once.py <registry.yaml> <log.jsonl> face <x> <y> <nx> <ny> [dwell_s]
      (face frame from a logged observation; optional verified dwell before the escape)
"""
import json
import math
from pathlib import Path
import sys
import time

from ament_index_python.packages import get_package_share_directory
from jdamr_cube_navigation.box_service import BoxServiceRoute
from jdamr_cube_navigation.parking import load_parking_contract
from jdamr_cube_navigation.service_destinations import load_registry
import rclpy
from rclpy.signals import SignalHandlerOptions


def main():
    registry = load_registry(Path(sys.argv[1]))
    log = Path(sys.argv[2])
    logged = sys.argv[3] == 'face'
    if logged:
        logged_face = [float(sys.argv[4]), float(sys.argv[5])]
        logged_normal = [float(sys.argv[6]), float(sys.argv[7])]
        dwell_s = float(sys.argv[8]) if len(sys.argv) > 8 else 0.0
    else:
        face_distance_m = float(sys.argv[3])
        dwell_s = 0.0
        if not 0.10 <= face_distance_m <= 0.55:
            raise SystemExit('face distance must be within the escape range')
    share = Path(get_package_share_directory('jdamr_cube_navigation'))
    contract = load_parking_contract(share / 'config/box_parking_contract.yaml')
    home_contract = load_parking_contract(share / 'config/parking_contract.yaml')
    with log.open('x', encoding='utf-8') as stream:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = BoxServiceRoute(registry, contract, stream, home_contract=home_contract)
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
            node.table_id = 'water_station'
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
                node.emit('parked_dwell_complete', dwell_s=dwell_s, moved_m=moved_m)
            ok = node._leave_parked_pose()
        finally:
            confirmed = node.finish_navigation()
            node.destroy_node()
            rclpy.shutdown()
        print(json.dumps({'escaped': bool(ok), 'navigation_finished': bool(confirmed)}))
        return 0 if ok and confirmed else 1


if __name__ == '__main__':
    raise SystemExit(main())

#!/usr/bin/env python3
"""Exercise the installed disarmed chain on an isolated synthetic ROS domain."""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, CameraInfo, Image, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
import yaml


def run(root, armed=False):
    """Use synthetic sensors and a fake base sink; never contact physical motors."""
    os.environ.update(ROS_DOMAIN_ID='198', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                      ROS_STATIC_PEERS='', FASTDDS_BUILTIN_TRANSPORTS='UDPv4')
    rclpy.init(domain_id=198)
    node = rclpy.create_node('jdamr_base_driver')
    publishers = {
        'odom': node.create_publisher(Odometry, '/odom', qos_profile_sensor_data),
        'scan': node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data),
        'battery': node.create_publisher(BatteryState, '/battery_state', qos_profile_sensor_data),
        'tf': node.create_publisher(TFMessage, '/tf', 10),
        'depth': node.create_publisher(Image, '/camera/depth/image_raw', qos_profile_sensor_data),
        'info': node.create_publisher(
            CameraInfo, '/camera/depth/camera_info', qos_profile_sensor_data),
    }
    commands, statuses, requested, observations = [], [], [], []
    switches = {'scan': True, 'obstacle': False}
    node.create_subscription(Twist, '/cmd_vel',
                             lambda msg: commands.append((msg.linear.x, msg.angular.z)), 10)
    node.create_subscription(String, '/box_parking/execution_status',
                             lambda msg: statuses.append(json.loads(msg.data)), 10)
    node.create_subscription(String, '/box_parking/perception_status',
                             lambda msg: observations.append(json.loads(msg.data)), 10)
    node.create_subscription(Twist, '/cmd_vel_nav',
                             lambda msg: requested.append((msg.linear.x, msg.angular.z)), 10)
    services = {name: node.create_client(Trigger, f'/box_parking/{name}_approach')
                for name in ('start', 'cancel')}

    def publish_sensors():
        stamp = node.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, 'odom'
        odom.child_frame_id, odom.pose.pose.orientation.w = 'base_footprint', 1.
        publishers['odom'].publish(odom)
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = stamp, 'laser_link'
        scan.angle_min, scan.angle_max = -math.pi, math.pi
        scan.angle_increment = math.pi / 180
        scan.range_min, scan.range_max = .28, 12.
        scan.ranges = [8.] * 361
        if switches['obstacle']:
            for index in range(267, 274):
                scan.ranges[index] = .285
        if switches['scan']:
            publishers['scan'].publish(scan)
        battery = BatteryState(voltage=12., present=True)
        battery.header.stamp = stamp
        publishers['battery'].publish(battery)
        transforms = []
        for parent, child in (('odom', 'base_footprint'), ('base_footprint', 'laser_link')):
            transform = TransformStamped()
            transform.header.stamp, transform.header.frame_id = stamp, parent
            transform.child_frame_id, transform.transform.rotation.w = child, 1.
            transforms.append(transform)
        publishers['tf'].publish(TFMessage(transforms=transforms))
        if armed:
            info = CameraInfo()
            info.header.stamp, info.header.frame_id = stamp, 'camera_color_optical_frame'
            info.width, info.height = 320, 240
            info.k = [288., 0., 160., 0., 288., 120., 0., 0., 1.]
            info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
            info.p = [288., 0., 160., 0., 0., 288., 120., 0., 0., 0., 1., 0.]
            publishers['info'].publish(info)
            depth = Image()
            depth.header = info.header
            depth.width, depth.height, depth.step = 320, 240, 640
            depth.encoding = '16UC1'
            pixels = np.zeros((240, 320), dtype='<u2')
            pixels[70:180, 100:220] = 900
            depth.data = pixels.tobytes()
            publishers['depth'].publish(depth)

    node.create_timer(.05, publish_sensors)
    nav = root / 'jdamr_cube_navigation'
    command = ['ros2', 'launch', 'jdamr_cube_navigation', 'box_approach_execution.launch.py',
               f'camera_mount_file:={root / "jdamr_cube_vslam/config/camera_mount.yaml"}',
               f'geometry_file:={root / "jdamr_cube_description/config/new_base_geometry.yaml"}',
               f'parking_contract_file:={nav / "config/parking_contract.yaml"}',
               f'nav_params_file:={nav / "config/new_base_nav2_params.yaml"}']
    process, paused_pid = None, None

    def wait_until(predicate, timeout=10.):
        until = time.monotonic() + timeout
        while time.monotonic() < until and (process is None or process.poll() is None):
            rclpy.spin_once(node, timeout_sec=.02)
            if predicate():
                return
        raise RuntimeError(f'condition_timeout: {statuses[-1] if statuses else None}; '
                           f'perception={observations[-1] if observations else None}')

    def call_service(name):
        if not services[name].service_is_ready():
            raise RuntimeError(f'service_unavailable:{name}')
        future = services[name].call_async(Trigger.Request())
        wait_until(future.done, 3.)
        if not future.result().success:
            raise RuntimeError(f'service_rejected:{name}:{future.result().message}')

    def begin_motion():
        previous_count = len(statuses)
        wait_until(lambda: len(statuses) > previous_count
                   and not statuses[-1]['blockers']
                   and statuses[-1]['zero_witness_valid']
                   and commands and commands[-1] == (0., 0.))
        commands.clear()
        call_service('start')
        wait_until(lambda: any(abs(v) > 0 or abs(w) > 0 for v, w in commands), 3.)

    def wait_stopped():
        wait_until(lambda: statuses and statuses[-1]['state'] == 'ABORTED', 3.)
        wait_until(lambda: requested and requested[-1] == (0., 0.), 2.)

    try:
        with tempfile.TemporaryFile(mode='w+') as log, tempfile.TemporaryDirectory(
                prefix='synthetic_box_approval_domain198_') as temporary:
            if armed:
                # This fixture is deliberately not bound to the physical camera file bytes.
                camera = Path(temporary) / 'synthetic_camera.yaml'
                camera_source = root / 'jdamr_cube_vslam/config/camera_mount.yaml'
                camera.write_text('# Synthetic domain198 fixture, not physical validation\n'
                                  + camera_source.read_text())
                approval = Path(temporary) / 'synthetic_approval.yaml'
                approval.write_text(yaml.safe_dump({
                    'schema_version': 1, 'physically_validated': True,
                    'evidence': 'synthetic LOCALHOST domain198 test fixture only',
                    'calibration_sha256': hashlib.sha256(camera.read_bytes()).hexdigest(),
                    'geometry_sha256': hashlib.sha256((root / (
                        'jdamr_cube_description/config/new_base_geometry.yaml')).read_bytes()
                    ).hexdigest(),
                    'nav_params_sha256': hashlib.sha256(
                        (nav / 'config/new_base_nav2_params.yaml').read_bytes()).hexdigest(),
                    'parking_contract_sha256': hashlib.sha256(
                        (nav / 'config/parking_contract.yaml').read_bytes()).hexdigest(),
                }))
                command = [arg for arg in command if not arg.startswith('camera_mount_file:=')]
                command.extend((f'camera_mount_file:={camera}',
                                f'physical_validation_file:={approval}',
                                'charger_unplugged_confirmed:=true'))
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            if armed:
                cases = []
                try:
                    begin_motion()
                    call_service('cancel')
                    wait_stopped()
                    cases.append('start_nonzero_through_nav2_then_cancel_zero')
                    begin_motion()
                    switches['scan'] = False
                    wait_stopped()
                    switches['scan'] = True
                    cases.append('scan_stale_aborts_with_zero')
                    begin_motion()
                    switches['obstacle'] = True
                    wait_stopped()
                    switches['obstacle'] = False
                    cases.append('collision_monitor_stop_aborts_with_zero')
                    begin_motion()
                    children = subprocess.check_output(
                        ['pgrep', '-P', str(process.pid)], text=True).split()
                    for child in children:
                        cmdline = Path(f'/proc/{child}/cmdline').read_bytes()
                        if b'/nav2_collision_monitor/collision_monitor\x00' in cmdline:
                            paused_pid = int(child)
                            break
                    if paused_pid is None:
                        raise RuntimeError('owned_synthetic_monitor_pid_not_found')
                    os.kill(paused_pid, signal.SIGSTOP)
                    wait_stopped()
                    os.kill(paused_pid, signal.SIGCONT)
                    paused_pid = None
                    cases.append('final_echo_loss_aborts_with_zero_request')
                    print(json.dumps({'scope': 'synthetic_armed_nav2_no_motor',
                                      'cases': cases, 'last_status': statuses[-1]}, indent=2))
                    return
                except Exception:
                    log.seek(0)
                    print(log.read())
                    raise
            deadline = time.monotonic() + 18.
            while time.monotonic() < deadline and process.poll() is None:
                rclpy.spin_once(node, timeout_sec=.05)
            result = {'scope': 'synthetic_disarmed_chain_no_physical_motion',
                      'commands_received': len(commands),
                      'all_commands_zero': bool(commands) and all(x == (0., 0.) for x in commands),
                      'last_status': statuses[-1] if statuses else None}
            print(json.dumps(result, indent=2))
            if not result['all_commands_zero'] or not statuses:
                log.seek(0)
                print(log.read())
                raise RuntimeError('disarmed_startup_failed')
            blockers = statuses[-1]['blockers']
            if any('not_active' in reason or 'lifecycle_stale' in reason for reason in blockers):
                raise RuntimeError('protection_chain_not_active')
            if any(reason in blockers for reason in (
                    'command_stale', 'final_zero_not_observed', 'final_zero_witness_missing',
                    'collision_monitor_intervention')):
                log.seek(0)
                print(log.read())
                raise RuntimeError('idle_protection_handshake_failed')
            if statuses[-1]['state'] != 'BLOCKED':
                raise RuntimeError('missing_calibration_must_block_execution')
    finally:
        if paused_pid is not None:
            os.kill(paused_pid, signal.SIGCONT)
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--armed', action='store_true', help='synthetic domain198 commands only')
    arguments = parser.parse_args()
    run(arguments.repo, arguments.armed)

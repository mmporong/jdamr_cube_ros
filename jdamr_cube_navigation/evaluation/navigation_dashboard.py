#!/usr/bin/env python3
"""Serve a localhost-only JDAMR navigation supervision dashboard.

The dashboard separates live ROS observations from the last verified run.  It
never exposes motion commands: operators use the page to understand LiDAR,
Nav2, Collision Monitor, and evidence state without adding another cmd_vel
authority.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import threading
import time
from typing import Any
from urllib.parse import urlsplit


HERE = Path(__file__).resolve().parent
DEFAULT_HTML = HERE / 'navigation_dashboard.html'
MAX_SCAN_POINTS = 180
MAX_MEDIA_BYTES = 96 * 1024 * 1024


def _finite_number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def _scenario_summary(document: dict[str, Any]) -> dict[str, Any]:
    """Extract only verified fields used by the dashboard."""
    results = document.get('results', [])
    if not isinstance(results, list) or len(results) != 1:
        return {'available': False}
    result = results[0]
    if not isinstance(result, dict):
        return {'available': False}
    scenario = result.get('scenario', {})
    detour = result.get('detour_evidence', {})
    if not isinstance(scenario, dict) or not isinstance(detour, dict):
        return {'available': False}
    final_pose = scenario.get('final_world_pose_m')
    return {
        'available': True,
        'status': result.get('status', document.get('status', 'UNKNOWN')),
        'claim_scope': result.get(
            'claim_scope', document.get('claim_scope', 'UNKNOWN')),
        'run_id': result.get('run_id'),
        'scenario': result.get('case'),
        'goal_send_count': scenario.get('goal_send_count'),
        'goal_cancel_count': scenario.get('goal_cancel_count'),
        'contact_count': scenario.get('contact_count'),
        'protected_clearance_m': scenario.get(
            'protected_envelope_minimum_clearance_m'),
        'observer_scan_to_zero_command_s': scenario.get(
            'observer_scan_to_zero_command_s'),
        'minimum_scan_range_m': scenario.get(
            'minimum_observed_scan_range_m'),
        'final_world_pose_m': final_pose,
        'same_goal_resume': (
            scenario.get('same_goal_command_verdict') == 'CONFIRMED'),
        'action_terminal': scenario.get('action_terminal'),
        'straight_centerline_blocked': detour.get(
            'straight_centerline_blocked'),
        'maximum_lateral_offset_m': detour.get(
            'maximum_abs_lateral_offset_m'),
        'static_obstacle_clearance_m': detour.get('minimum_clearance_m'),
    }


def load_verified_run(summary_path: Path | None) -> dict[str, Any]:
    """Load a single PASS/FAIL summary without inventing absent metrics."""
    if summary_path is None:
        return {'available': False}
    if not summary_path.is_file() or summary_path.is_symlink():
        raise ValueError(f'summary must be a regular file: {summary_path}')
    document = json.loads(summary_path.read_text(encoding='utf-8'))
    if not isinstance(document, dict):
        raise ValueError('summary root must be an object')
    extracted = _scenario_summary(document)
    extracted['summary_path'] = str(summary_path.resolve())
    return extracted


def load_verified_bundle(manifest_path: Path | None) -> dict[str, Any]:
    """Aggregate a portable media manifest without weakening its scope."""
    if manifest_path is None:
        return {'available': False}
    if not manifest_path.is_file() or manifest_path.is_symlink():
        raise ValueError(f'manifest must be a regular file: {manifest_path}')
    document = json.loads(manifest_path.read_text(encoding='utf-8'))
    runs = document.get('runs')
    if document.get('status') != 'PASS' or not isinstance(runs, list):
        raise ValueError('media manifest must contain passing runs')
    metrics = [run.get('metrics') for run in runs]
    if not runs or not all(isinstance(item, dict) for item in metrics):
        raise ValueError('media manifest run metrics are incomplete')
    directions = [run.get('entry_side') for run in runs]
    if any(direction not in {'left', 'right'} for direction in directions):
        raise ValueError('media manifest entry direction is invalid')
    clearances = [
        _finite_number(item.get('protected_clearance_m'))
        for item in metrics]
    latencies = [
        _finite_number(item.get('observer_scan_to_zero_command_s'))
        for item in metrics]
    if any(value is None for value in (*clearances, *latencies)):
        raise ValueError('media manifest verified metrics are invalid')
    if (not all(item.get('same_goal_resume') is True for item in metrics)
            or not all(item.get('action_terminal') == 'succeeded'
                       for item in metrics)):
        raise ValueError('media manifest goal-resume evidence is incomplete')
    return {
        'available': True,
        'status': 'PASS',
        'claim_scope': document.get('claim_scope'),
        'run_id': f"directional_{'_'.join(directions)}",
        'directional_run_count': len(runs),
        'entry_sides': directions,
        'goal_send_count': sum(item.get('goal_send_count', 0)
                               for item in metrics),
        'goal_cancel_count': sum(item.get('goal_cancel_count', 0)
                                 for item in metrics),
        'contact_count': sum(item.get('contact_count', 0)
                             for item in metrics),
        'protected_clearance_m': min(clearances),
        'observer_scan_to_zero_command_s': max(latencies),
        'observer_latency_range_s': [min(latencies), max(latencies)],
        'same_goal_resume': all(
            item['same_goal_resume'] for item in metrics),
        'action_terminal': 'succeeded',
        'manifest_path': str(manifest_path.resolve()),
    }


@dataclass
class LiveState:
    """Thread-safe, bounded state projected from live ROS topics."""

    lock: threading.Lock = field(default_factory=threading.Lock)
    started_ns: int = field(default_factory=time.monotonic_ns)
    scan_ns: int | None = None
    odom_ns: int | None = None
    cmd_ns: int | None = None
    monitor_ns: int | None = None
    goal_ns: int | None = None
    scan: dict[str, Any] = field(default_factory=lambda: {
        'points': [], 'minimum_m': None, 'front_minimum_m': None,
        'closest_angle_deg': None, 'sample_count': 0,
    })
    pose: dict[str, Any] = field(default_factory=lambda: {
        'x_m': None, 'y_m': None, 'yaw_deg': None,
        'linear_mps': 0.0, 'angular_rps': 0.0,
    })
    command: dict[str, Any] = field(default_factory=lambda: {
        'linear_mps': 0.0, 'angular_rps': 0.0,
    })
    monitor: dict[str, Any] = field(default_factory=lambda: {
        'action': 'NO_DATA', 'action_type': None, 'polygon': None,
    })
    navigation: dict[str, Any] = field(default_factory=lambda: {
        'status': 'NO_GOAL', 'status_code': None,
    })
    error: str | None = None

    def update_scan(self, ranges: list[float], angle_min: float,
                    angle_increment: float, range_min: float,
                    range_max: float) -> None:
        """Downsample one scan to bounded display data and safety metrics."""
        valid = []
        for index, raw in enumerate(ranges):
            value = _finite_number(raw)
            if value is None or not range_min < value < range_max:
                continue
            valid.append((angle_min + index * angle_increment, value))
        stride = max(1, math.ceil(len(valid) / MAX_SCAN_POINTS))
        points = [[math.degrees(angle), distance]
                  for angle, distance in valid[::stride]][:MAX_SCAN_POINTS]
        front = [distance for angle, distance in valid
                 if abs(math.atan2(math.sin(angle), math.cos(angle)))
                 <= math.radians(30.0)]
        closest = min(valid, key=lambda item: item[1]) if valid else None
        with self.lock:
            self.scan_ns = time.monotonic_ns()
            self.scan = {
                'points': points,
                'minimum_m': closest[1] if closest else None,
                'front_minimum_m': min(front) if front else None,
                'closest_angle_deg': (
                    math.degrees(closest[0]) if closest else None),
                'sample_count': len(ranges),
            }

    def snapshot(self, verified_run: dict[str, Any]) -> dict[str, Any]:
        """Return a JSON-safe snapshot with explicit freshness semantics."""
        now_ns = time.monotonic_ns()

        def age(stamp_ns: int | None) -> float | None:
            return ((now_ns - stamp_ns) / 1e9
                    if stamp_ns is not None else None)

        with self.lock:
            freshness = {
                'scan_age_s': age(self.scan_ns),
                'odom_age_s': age(self.odom_ns),
                'cmd_age_s': age(self.cmd_ns),
                'monitor_age_s': age(self.monitor_ns),
                'goal_age_s': age(self.goal_ns),
            }
            live = (freshness['scan_age_s'] is not None
                    and freshness['scan_age_s'] <= 1.0
                    and freshness['odom_age_s'] is not None
                    and freshness['odom_age_s'] <= 1.0)
            return {
                'schema_version': 1,
                'observed_at_monotonic_ns': now_ns,
                'mode': 'LIVE_ROS' if live else 'OFFLINE_REPLAY_READY',
                'live': live,
                'scan': dict(self.scan),
                'pose': dict(self.pose),
                'command': dict(self.command),
                'collision_monitor': dict(self.monitor),
                'navigation': dict(self.navigation),
                'freshness': freshness,
                'verified_run': verified_run,
                'error': self.error,
            }


class RosObserver:
    """Optional ROS observer; it has no publishers, services, or actions."""

    ACTION_NAMES = {
        0: 'DO_NOTHING', 1: 'STOP', 2: 'SLOWDOWN',
        3: 'APPROACH', 4: 'LIMIT',
    }
    GOAL_NAMES = {
        0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING',
        3: 'CANCELING', 4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED',
    }

    def __init__(self, state: LiveState) -> None:
        """Prepare the observer without starting ROS or any publishers."""
        self.state = state
        self.thread: threading.Thread | None = None
        self.node = None
        self._rclpy = None

    def start(self) -> bool:
        """Start subscriptions when ROS Python packages are available."""
        try:
            import rclpy
            from action_msgs.msg import GoalStatusArray
            from geometry_msgs.msg import Twist
            from nav2_msgs.msg import CollisionMonitorState
            from nav_msgs.msg import Odometry
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import LaserScan
        except ImportError as error:
            self.state.error = f'ROS observer unavailable: {error}'
            return False

        state = self.state
        action_names = self.ACTION_NAMES
        goal_names = self.GOAL_NAMES

        class ObserverNode(Node):
            def __init__(self) -> None:
                super().__init__('jdamr_navigation_dashboard')
                self.create_subscription(
                    LaserScan, '/scan', self.on_scan,
                    qos_profile_sensor_data)
                self.create_subscription(Odometry, '/odom', self.on_odom, 10)
                self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
                self.create_subscription(
                    CollisionMonitorState, '/collision_monitor_state',
                    self.on_monitor, 10)
                self.create_subscription(
                    GoalStatusArray, '/navigate_to_pose/_action/status',
                    self.on_goal, 10)

            def on_scan(self, message) -> None:
                state.update_scan(
                    list(message.ranges), message.angle_min,
                    message.angle_increment, message.range_min,
                    message.range_max)

            def on_odom(self, message) -> None:
                pose = message.pose.pose
                twist = message.twist.twist
                yaw = math.atan2(
                    2.0 * (pose.orientation.w * pose.orientation.z
                           + pose.orientation.x * pose.orientation.y),
                    1.0 - 2.0 * (
                        pose.orientation.y * pose.orientation.y
                        + pose.orientation.z * pose.orientation.z))
                with state.lock:
                    state.odom_ns = time.monotonic_ns()
                    state.pose = {
                        'x_m': pose.position.x,
                        'y_m': pose.position.y,
                        'yaw_deg': math.degrees(yaw),
                        'linear_mps': math.hypot(
                            twist.linear.x, twist.linear.y),
                        'angular_rps': twist.angular.z,
                    }

            def on_cmd(self, message) -> None:
                with state.lock:
                    state.cmd_ns = time.monotonic_ns()
                    state.command = {
                        'linear_mps': message.linear.x,
                        'angular_rps': message.angular.z,
                    }

            def on_monitor(self, message) -> None:
                action_type = int(message.action_type)
                with state.lock:
                    state.monitor_ns = time.monotonic_ns()
                    state.monitor = {
                        'action': action_names.get(
                            action_type, f'UNKNOWN_{action_type}'),
                        'action_type': action_type,
                        'polygon': message.polygon_name or None,
                    }

            def on_goal(self, message) -> None:
                statuses = list(message.status_list)
                if not statuses:
                    return
                code = int(statuses[-1].status)
                with state.lock:
                    state.goal_ns = time.monotonic_ns()
                    state.navigation = {
                        'status': goal_names.get(code, f'UNKNOWN_{code}'),
                        'status_code': code,
                    }

        rclpy.init(args=None)
        self._rclpy = rclpy
        self.node = ObserverNode()

        def spin() -> None:
            try:
                rclpy.spin(self.node)
            except Exception as error:  # pragma: no cover - runtime guard
                state.error = f'ROS observer stopped: {type(error).__name__}'

        self.thread = threading.Thread(
            target=spin, name='jdamr-dashboard-ros', daemon=True)
        self.thread.start()
        return True

    def stop(self) -> None:
        """Stop the observer thread without affecting other ROS processes."""
        if self.node is not None:
            self.node.destroy_node()
        if self._rclpy is not None and self._rclpy.ok():
            self._rclpy.shutdown()
        if self.thread is not None:
            self.thread.join(timeout=2.0)


def _media_range(header: str | None, size: int) -> tuple[int, int] | None:
    if not header:
        return None
    if not header.startswith('bytes=') or ',' in header:
        raise ValueError('unsupported Range header')
    start_text, end_text = header[6:].split('-', 1)
    if not start_text:
        length = int(end_text)
        if length <= 0:
            raise ValueError('invalid suffix range')
        return max(0, size - length), size - 1
    start = int(start_text)
    end = int(end_text) if end_text else size - 1
    if start < 0 or start >= size or end < start:
        raise ValueError('range outside file')
    return start, min(end, size - 1)


def make_handler(html: Path, video: Path | None, state: LiveState,
                 verified_run: dict[str, Any]):
    """Create an HTTP handler closed over immutable server configuration."""

    class Handler(BaseHTTPRequestHandler):
        server_version = 'JDAMRDashboard/1.0'

        def handle(self) -> None:
            """Ignore clients that close while a response is being written."""
            try:
                super().handle()
            except (BrokenPipeError, ConnectionResetError):
                return

        def log_message(self, *_args) -> None:
            return

        def _send(self, body: bytes, content_type: str,
                  status: HTTPStatus = HTTPStatus.OK,
                  extra: dict[str, str] | None = None) -> None:
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            for name, value in (extra or {}).items():
                self.send_header(name, value)
            self.end_headers()
            if self.command != 'HEAD':
                self.wfile.write(body)

        def _serve_video(self) -> None:
            if (video is None or not video.is_file() or video.is_symlink()
                    or video.stat().st_size > MAX_MEDIA_BYTES):
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            size = video.stat().st_size
            try:
                interval = _media_range(self.headers.get('Range'), size)
            except (ValueError, TypeError):
                self.send_error(HTTPStatus.REQUESTED_RANGE_NOT_SATISFIABLE)
                return
            start, end = interval or (0, size - 1)
            status = (HTTPStatus.PARTIAL_CONTENT
                      if interval else HTTPStatus.OK)
            self.send_response(status)
            self.send_header('Content-Type', 'video/mp4')
            self.send_header('Content-Length', str(end - start + 1))
            self.send_header('Accept-Ranges', 'bytes')
            if interval:
                self.send_header(
                    'Content-Range', f'bytes {start}-{end}/{size}')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            if self.command == 'HEAD':
                return
            with video.open('rb') as stream:
                stream.seek(start)
                remaining = end - start + 1
                while remaining:
                    chunk = stream.read(min(1024 * 1024, remaining))
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                    remaining -= len(chunk)

        def do_HEAD(self) -> None:
            """Serve metadata using the same route contract as GET."""
            self.do_GET()

        def do_GET(self) -> None:
            path = urlsplit(self.path).path
            if path == '/':
                self._send(html.read_bytes(), 'text/html; charset=utf-8')
            elif path == '/api/state':
                body = json.dumps(
                    state.snapshot(verified_run), ensure_ascii=False,
                    separators=(',', ':')).encode('utf-8')
                self._send(body, 'application/json; charset=utf-8')
            elif path == '/api/config':
                body = json.dumps({
                    'schema_version': 1,
                    'video_available': bool(
                        video is not None and video.is_file()),
                    'poll_interval_ms': 200,
                    'scan_render_limit': MAX_SCAN_POINTS,
                    'command_surface': 'read_only',
                }, separators=(',', ':')).encode('utf-8')
                self._send(body, 'application/json; charset=utf-8')
            elif path in {'/media/replay.mp4', '/media/gazebo.mp4'}:
                self._serve_video()
            else:
                self.send_error(HTTPStatus.NOT_FOUND)

    return Handler


def parse_args() -> argparse.Namespace:
    """Parse localhost dashboard configuration."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--html', type=Path, default=DEFAULT_HTML)
    parser.add_argument('--summary', type=Path)
    parser.add_argument('--media-manifest', type=Path)
    parser.add_argument('--video', type=Path)
    parser.add_argument('--no-ros', action='store_true')
    args = parser.parse_args()
    if args.host not in {'127.0.0.1', '::1', 'localhost'}:
        parser.error('dashboard binds to localhost only')
    if not 1024 <= args.port <= 65535:
        parser.error('--port must be between 1024 and 65535')
    if not args.html.is_file() or args.html.is_symlink():
        parser.error(f'HTML file unavailable: {args.html}')
    if args.video is not None and (
            not args.video.is_file() or args.video.is_symlink()):
        parser.error(f'video must be a regular file: {args.video}')
    return args


def main() -> int:
    """Serve the read-only dashboard until interrupted."""
    args = parse_args()
    verified_run = load_verified_run(args.summary)
    verified_bundle = load_verified_bundle(args.media_manifest)
    if verified_bundle['available']:
        verified_run = verified_bundle
    state = LiveState()
    observer = RosObserver(state)
    if not args.no_ros:
        observer.start()
    handler = make_handler(args.html, args.video, state, verified_run)
    server = ThreadingHTTPServer((args.host, args.port), handler)
    print(json.dumps({
        'url': f'http://{args.host}:{args.port}/',
        'read_only': True,
        'video': str(args.video.resolve()) if args.video else None,
        'verified_run': verified_run.get('status'),
    }, ensure_ascii=False, sort_keys=True), flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        observer.stop()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

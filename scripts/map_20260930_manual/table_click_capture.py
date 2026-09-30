"""PC-only (domain 78): record RViz 2D Pose Estimate clicks as table candidates.

Each click = box front-face centre, arrow = side the robot approaches from.
Clicks are appended to <map_dir>/table_clicks.json and drawn with the dock.
"""
import json
import math
import os
from pathlib import Path
import sys
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

MAP_DIR = Path(sys.argv[1])
OUT = MAP_DIR / 'table_clicks.json'
_DOCK = json.loads((MAP_DIR / 'dock_pose.json').read_text())
DOCK = _DOCK.get('dock_target', _DOCK['dock_pose_scan_match'])
NAMES = {1: 'table_01', 2: 'water_station', 3: 'table_02'}
SUGGEST = MAP_DIR / 'suggested_points.json'


def yaw_of(q):
    return 2.0 * math.atan2(q.z, q.w)


def snap(yaw):
    """Operator request: arrows follow the map axes (0/90/180/270 deg)."""
    return math.atan2(math.sin(round(yaw / (math.pi / 2)) * math.pi / 2),
                      math.cos(round(yaw / (math.pi / 2)) * math.pi / 2))


def arrow(mid, x, y, yaw, rgb, length=0.4):
    m = Marker()
    m.header.frame_id = 'map'
    m.ns, m.id, m.type, m.action = 'points', mid, Marker.ARROW, Marker.ADD
    m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.05
    m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    m.scale.x, m.scale.y, m.scale.z = length, 0.06, 0.06
    m.color.r, m.color.g, m.color.b, m.color.a = (*rgb, 1.0)
    return m


def label(mid, x, y, text, rgb):
    m = Marker()
    m.header.frame_id = 'map'
    m.ns, m.id, m.type, m.action = 'labels', mid, Marker.TEXT_VIEW_FACING, Marker.ADD
    m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y + 0.25, 0.3
    m.pose.orientation.w = 1.0
    m.scale.z = 0.22
    m.color.r, m.color.g, m.color.b, m.color.a = (*rgb, 1.0)
    m.text = text
    return m


def main():
    rclpy.init()
    node = rclpy.create_node('table_click_capture')
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(MarkerArray, '/service_visualization/destinations', latched)
    clicks = json.loads(OUT.read_text()) if OUT.exists() else []

    def redraw():
        markers = [arrow(0, DOCK[0], DOCK[1], DOCK[2], (0.1, 0.7, 0.2)),
                   label(0, DOCK[0], DOCK[1], 'dock', (0.1, 0.6, 0.2))]
        for i, c in enumerate(clicks, start=1):
            markers.append(arrow(i, c['x'], c['y'], c['yaw'], (1.0, 0.5, 0.0)))
            markers.append(label(i, c['x'], c['y'], NAMES.get(i, f'P{i}'), (0.9, 0.4, 0.0)))
        if SUGGEST.exists():
            for j, s in enumerate(json.loads(SUGGEST.read_text()), start=100):
                markers.append(arrow(j, s['x'], s['y'], s['yaw'], (0.45, 0.45, 0.45)))
                markers.append(label(j, s['x'], s['y'], s['label'], (0.35, 0.35, 0.35)))
        pub.publish(MarkerArray(markers=markers))

    def on_click(msg):
        p = msg.pose.pose
        raw = yaw_of(p.orientation)
        clicks.append({'n': len(clicks) + 1, 'x': p.position.x, 'y': p.position.y,
                       'yaw': snap(raw), 'yaw_deg': math.degrees(snap(raw)),
                       'raw_yaw_deg': math.degrees(raw),
                       'received_unix_s': time.time(), 'meaning': 'box front-face centre; '
                       'arrow = outward face normal (robot approaches from this side)'})
        tmp = OUT.with_suffix('.tmp')
        tmp.write_text(json.dumps(clicks, indent=1))
        os.replace(tmp, OUT)
        node.get_logger().info(f'P{len(clicks)} ({p.position.x:.2f}, {p.position.y:.2f}, '
                               f'{math.degrees(snap(raw)):.0f} deg, raw {math.degrees(raw):.1f})')
        redraw()

    node.create_subscription(PoseWithCovarianceStamped, '/initialpose', on_click, 10)
    redraw()
    node.create_timer(2.0, redraw)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()

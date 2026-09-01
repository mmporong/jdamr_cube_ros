"""Capture multiple keepout polygons from RViz Publish Point clicks."""

import argparse
from math import atan2
import os
from pathlib import Path
import sys

from geometry_msgs.msg import PointStamped
from jdamr_cube_navigation.keepout_mask import MINIMUM_SAFETY_MARGIN_M
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
import yaml


def zone_document(map_yaml, zone_id, margin, points):
    """Return the versioned polygon document consumed by keepout_mask."""
    return {
        'schema_version': 1,
        'map_yaml': str(map_yaml),
        'safety_margin_m': margin,
        'zones': [{
            'id': zone_id,
            'enabled': True,
            'polygon': points,
        }],
    }


def zones_document(map_yaml, margin, zones):
    """Return a versioned document containing every completed zone."""
    return {
        'schema_version': 1,
        'map_yaml': str(map_yaml),
        'safety_margin_m': margin,
        'zones': zones,
    }


def order_polygon_points(points):
    """Order convex polygon vertices around their centroid."""
    center_x = sum(point[0] for point in points) / len(points)
    center_y = sum(point[1] for point in points) / len(points)
    return sorted(
        points,
        key=lambda point: atan2(
            point[1] - center_y, point[0] - center_x),
    )


class ZoneCapture(Node):
    """Save one polygon after each fixed-size group of map-frame clicks."""

    def __init__(self, map_yaml, output, zone_id, point_count, margin):
        super().__init__('keepout_zone_capture')
        self.map_yaml = map_yaml
        self.output = output
        self.zone_id = zone_id
        self.point_count = point_count
        self.margin = margin
        self.points = []
        self.zones = []
        self.create_subscription(
            PointStamped, '/clicked_point', self._clicked, 10)
        self.get_logger().info(
            f'RViz Publish Point로 금지구역 하나당 꼭짓점 {point_count}개를 '
            '순서대로 클릭하세요. 저장 후 다음 구역을 계속 클릭할 수 '
            '있으며, 모두 끝나면 이 launch를 Ctrl-C로 종료하세요.')

    def _clicked(self, message):
        if message.header.frame_id.lstrip('/') != 'map':
            self.get_logger().error(
                f'map 프레임 점만 허용합니다: {message.header.frame_id}')
            return
        point = [round(message.point.x, 6), round(message.point.y, 6)]
        self.points.append(point)
        self.get_logger().info(
            f'point {len(self.points)}/{self.point_count}: {point}')
        if len(self.points) < self.point_count:
            return
        zone_number = len(self.zones) + 1
        zone = {
            'id': f'{self.zone_id}_{zone_number}',
            'enabled': True,
            'polygon': order_polygon_points(self.points),
        }
        self.zones.append(zone)
        document = zones_document(
            self.map_yaml, self.margin, self.zones)
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output.write_text(
            yaml.safe_dump(document, sort_keys=False), encoding='utf-8')
        self.points = []
        self.get_logger().info(
            f'금지구역 {zone_number}개 저장 완료: {self.output}. '
            f'다음 구역의 꼭짓점 {self.point_count}개를 클릭하거나, '
            '모두 끝났으면 Ctrl-C로 종료하세요.')


def main() -> None:
    """Capture polygon groups until the operator stops the process."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True, dest='map_yaml')
    parser.add_argument('--output', required=True)
    parser.add_argument('--zone-id', required=True)
    parser.add_argument('--points', type=int, default=4)
    parser.add_argument('--margin', type=float, default=0.55)
    parser.add_argument('--force', action='store_true')
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])

    map_yaml = Path(os.path.expandvars(os.path.expanduser(
        args.map_yaml))).resolve()
    output = Path(os.path.expandvars(os.path.expanduser(
        args.output))).resolve()
    if not map_yaml.is_file():
        parser.error(f'map YAML not found: {map_yaml}')
    if output.exists() and not args.force:
        parser.error(f'output exists; use --force to replace it: {output}')
    if args.points < 3:
        parser.error('--points must be at least 3')
    if args.margin < MINIMUM_SAFETY_MARGIN_M:
        parser.error(
            f'--margin must be at least {MINIMUM_SAFETY_MARGIN_M}')

    rclpy.init(args=sys.argv)
    node = ZoneCapture(
        map_yaml, output, args.zone_id, args.points, args.margin)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

"""Display saved service assets without issuing navigation or velocity commands."""

import argparse
import hashlib
import json
import math
from pathlib import Path

from geometry_msgs.msg import Point
from jdamr_cube_navigation.keepout_mask import _map_metadata
from jdamr_cube_navigation.service_destinations import load_registry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray


def marker(identifier, kind, x, y, color, frame='map'):
    """Construct a display-only marker in the registered map frame."""
    item = Marker()
    item.header.frame_id = frame
    item.ns = 'service_destinations'
    item.id = identifier
    item.type = kind
    item.action = Marker.ADD
    item.pose.position.x, item.pose.position.y = float(x), float(y)
    item.pose.orientation.w = 1.0
    item.color.r, item.color.g, item.color.b, item.color.a = color
    return item


def build_markers(registry, annotation, selected='table_01'):
    """Keep approximate table regions distinct from taught docking poses."""
    if annotation['source_map_sha256'] != registry['map']['image_sha256']:
        raise ValueError('table annotation belongs to a different map')
    frame = registry['frame_id']
    # RViz can outlive this publisher; remove poses from the previous registry.
    clear = Marker(action=Marker.DELETEALL, type=Marker.CUBE_LIST)
    clear.header.frame_id = frame
    clear.ns = 'service_destinations'
    result = [clear]
    for number in (1, 2):
        table_id = f'table_{number:02d}'
        region = annotation['markers'][f'table_{number}']
        x, y = region['approximate_map_xy_m']
        color = (0.15, 0.5, 1.0, 0.45) if number == 1 else (0.1, 0.8, 0.3, 0.45)
        area = marker(number * 10, Marker.CYLINDER, x, y, color, frame)
        area.scale.x = area.scale.y = 0.45
        area.scale.z = 0.015
        area.pose.position.z = 0.025
        label = marker(number * 10 + 1, Marker.TEXT_VIEW_FACING,
                       x, y + 0.35, (0.1, 0.15, 0.22, 1.0), frame)
        label.pose.position.z, label.scale.z = 0.2, 0.14
        label.text = f'TABLE_{number}' + ('_SELECTED' if selected == table_id else '')
        label.text += '\nAREA_ONLY'
        result.extend([area, label])
        table = next((t for t in registry['tables'] if t['table_id'] == table_id), None)
        if table:
            label.text = label.text.replace('AREA_ONLY', 'DOCK_POSE_TAUGHT')
            for index, pose in enumerate(table['service_poses']):
                result.append(pose_arrow(number * 10 + 2 + index, pose, color, frame))
    home = registry.get('home')
    if home:
        result.append(pose_arrow(100, home, (1.0, 0.65, 0.0, 1.0), frame))
        label = marker(101, Marker.TEXT_VIEW_FACING, home['x_m'], home['y_m'] - 0.4,
                       (0.4, 0.25, 0.0, 1.0), frame)
        label.pose.position.z, label.scale.z = 0.2, 0.14
        label.text = 'HOME_' + home.get('parking_direction', 'forward').upper()
        result.append(label)
    return MarkerArray(markers=result)


def pose_arrow(identifier, pose, color, frame):
    """Show the final chassis heading, not its reverse travel direction."""
    arrow = marker(identifier, Marker.ARROW, pose['x_m'], pose['y_m'], color, frame)
    arrow.color.a = 1.0
    arrow.pose.position.z = 0.08
    arrow.pose.orientation.z = math.sin(pose['yaw_rad'] / 2.0)
    arrow.pose.orientation.w = math.cos(pose['yaw_rad'] / 2.0)
    arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.5, 0.09, 0.09
    return arrow


def build_keepout(registry):
    """Render occupied mask cells in red without publishing a control map."""
    metadata = _map_metadata(Path(registry['keepout']['yaml_path']))
    item = marker(0, Marker.CUBE_LIST, 0, 0, (1.0, 0.1, 0.12, 0.3),
                  registry['frame_id'])
    item.ns = 'saved_keepout_reference'
    resolution = metadata['resolution']
    item.scale.x = item.scale.y = resolution
    item.scale.z = 0.01
    width, height = metadata['width'], metadata['height']
    for index, pixel in enumerate(metadata['pixels']):
        if pixel != 0:
            continue
        row, col = divmod(index, width)
        item.points.append(Point(
            x=metadata['origin'][0] + (col + 0.5) * resolution,
            y=metadata['origin'][1] + (height - row - 0.5) * resolution, z=0.01))
    return MarkerArray(markers=[item])


def main():
    """Publish latched display markers; never create a motion interface."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--registry', type=Path, required=True)
    parser.add_argument('--annotation', type=Path, required=True)
    parser.add_argument('--selected', choices=('table_01', 'table_02'), default='table_01')
    args = parser.parse_args()
    registry = load_registry(args.registry)
    annotation = json.loads(args.annotation.read_text())
    if hashlib.sha256(Path(registry['map']['yaml_path']).read_bytes()).hexdigest() != (
            registry['map']['yaml_sha256']):
        raise ValueError('map changed')
    destinations = build_markers(registry, annotation, args.selected)
    keepout = build_keepout(registry)
    rclpy.init()
    node = Node('service_visualization')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publishers = [node.create_publisher(MarkerArray, topic, qos) for topic in (
        '/service_visualization/destinations', '/service_visualization/saved_keepout')]
    publishers[0].publish(destinations)
    publishers[1].publish(keepout)
    node.get_logger().info(
        f'DISPLAY ONLY: {len(destinations.markers)} destination markers, '
        f'{len(keepout.markers[0].points)} saved keepout cells; no motion commands')
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

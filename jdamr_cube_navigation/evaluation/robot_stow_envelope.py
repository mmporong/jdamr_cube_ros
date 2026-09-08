"""Measure a fixed robot travel-pose envelope from URDF collision geometry."""

from __future__ import annotations

import hashlib
import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Iterable


Matrix = tuple[tuple[float, float, float, float], ...]


def _vector(value: str | None, default: str = '0 0 0') -> tuple[float, ...]:
    return tuple(float(item) for item in (value or default).split())


def _multiply(left: Matrix, right: Matrix) -> Matrix:
    return tuple(tuple(
        sum(left[row][index] * right[index][column]
            for index in range(4))
        for column in range(4)) for row in range(4))


def _origin(element: ET.Element | None) -> Matrix:
    xyz = _vector(element.get('xyz') if element is not None else None)
    roll, pitch, yaw = _vector(
        element.get('rpy') if element is not None else None)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )
    return tuple(
        tuple(rotation[row][column] if column < 3 else xyz[row]
              for column in range(4))
        for row in range(3)) + ((0.0, 0.0, 0.0, 1.0),)


def _axis_rotation(axis: tuple[float, ...], angle: float) -> Matrix:
    x_axis, y_axis, z_axis = axis
    norm = math.sqrt(x_axis ** 2 + y_axis ** 2 + z_axis ** 2)
    if norm <= 0.0:
        raise ValueError('joint axis must be nonzero')
    x_axis, y_axis, z_axis = (
        x_axis / norm, y_axis / norm, z_axis / norm)
    cosine, sine = math.cos(angle), math.sin(angle)
    complement = 1.0 - cosine
    rotation = (
        (x_axis * x_axis * complement + cosine,
         x_axis * y_axis * complement - z_axis * sine,
         x_axis * z_axis * complement + y_axis * sine),
        (y_axis * x_axis * complement + z_axis * sine,
         y_axis * y_axis * complement + cosine,
         y_axis * z_axis * complement - x_axis * sine),
        (z_axis * x_axis * complement - y_axis * sine,
         z_axis * y_axis * complement + x_axis * sine,
         z_axis * z_axis * complement + cosine),
    )
    return tuple(
        tuple(rotation[row][column] if column < 3 else 0.0
              for column in range(4))
        for row in range(3)) + ((0.0, 0.0, 0.0, 1.0),)


def _transform(matrix: Matrix, point: tuple[float, ...]) -> tuple[float, ...]:
    homogeneous = (*point, 1.0)
    return tuple(sum(matrix[row][column] * homogeneous[column]
                     for column in range(4)) for row in range(3))


def _mesh_vertices(path: Path) -> Iterable[tuple[float, float, float]]:
    data = path.read_bytes()
    if len(data) >= 84:
        triangle_count = struct.unpack_from('<I', data, 80)[0]
        if 84 + triangle_count * 50 == len(data):
            for triangle in range(triangle_count):
                vertices_offset = 84 + triangle * 50 + 12
                for vertex in range(3):
                    yield struct.unpack_from(
                        '<fff', data, vertices_offset + vertex * 12)
            return
    found = False
    for line in data.decode('ascii', errors='ignore').splitlines():
        fields = line.strip().split()
        if len(fields) == 4 and fields[0] == 'vertex':
            found = True
            yield tuple(float(value) for value in fields[1:])
    if not found:
        raise ValueError(f'unsupported or empty STL: {path}')


def _geometry_points(
        geometry: ET.Element, mesh_root: Path,
) -> Iterable[tuple[float, float, float]]:
    mesh = geometry.find('mesh')
    box = geometry.find('box')
    if mesh is not None:
        filename = mesh.get('filename', '')
        marker = 'package://jdamr_cube_description/meshes/'
        if not filename.startswith(marker):
            raise ValueError(f'unsupported mesh URI: {filename}')
        scale = _vector(mesh.get('scale'), '1 1 1')
        path = mesh_root / filename.removeprefix(marker)
        for vertex in _mesh_vertices(path):
            yield tuple(vertex[index] * scale[index] for index in range(3))
        return
    if box is not None:
        size = _vector(box.get('size'))
        for x_sign in (-1.0, 1.0):
            for y_sign in (-1.0, 1.0):
                for z_sign in (-1.0, 1.0):
                    yield (x_sign * size[0] / 2.0,
                           y_sign * size[1] / 2.0,
                           z_sign * size[2] / 2.0)
        return
    raise ValueError('only mesh and box arm collision geometry is supported')


def _initial_positions(robot: ET.Element) -> dict[str, float]:
    positions: dict[str, float] = {}
    for control in robot.findall('ros2_control'):
        for joint in control.findall('joint'):
            initial = joint.find(
                "state_interface[@name='position']/"
                "param[@name='initial_value']")
            if initial is not None and initial.text is not None:
                positions[joint.get('name', '')] = float(initial.text)
    return positions


def collision_envelope(
        urdf_path: Path, mesh_root: Path, *, root_frame: str,
        link_prefix: str,
) -> dict:
    """Return exact collision bounds at ros2_control initial positions."""
    robot = ET.parse(urdf_path).getroot()
    initial_positions = _initial_positions(robot)
    identity: Matrix = (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )
    children: dict[str, list[tuple[str, Matrix]]] = {}
    for joint in robot.findall('joint'):
        parent = joint.find('parent').get('link')
        child = joint.find('child').get('link')
        transform = _origin(joint.find('origin'))
        if joint.get('type') in {'revolute', 'continuous'}:
            axis = _vector(joint.find('axis').get('xyz'))
            transform = _multiply(transform, _axis_rotation(
                axis, initial_positions.get(joint.get('name'), 0.0)))
        children.setdefault(parent, []).append((child, transform))
    transforms = {root_frame: identity}
    pending = [root_frame]
    while pending:
        parent = pending.pop()
        for child, transform in children.get(parent, []):
            transforms[child] = _multiply(transforms[parent], transform)
            pending.append(child)

    bounds = {
        'front_m': -math.inf, 'rear_m': math.inf,
        'left_m': -math.inf, 'right_m': math.inf,
        'top_m': -math.inf, 'bottom_m': math.inf,
    }
    witnesses: dict[str, dict] = {}
    point_count = 0
    link_count = 0
    for link in robot.findall('link'):
        link_name = link.get('name', '')
        if (not link_name.startswith(link_prefix)
                or link_name not in transforms):
            continue
        collisions = link.findall('collision')
        if not collisions:
            continue
        link_count += 1
        for index, collision in enumerate(collisions):
            collision_name = collision.get('name', f'collision_{index}')
            transform = _multiply(
                transforms[link_name], _origin(collision.find('origin')))
            geometry = collision.find('geometry')
            if geometry is None:
                raise ValueError(f'missing collision geometry: {link_name}')
            for local_point in _geometry_points(geometry, mesh_root):
                x_m, y_m, z_m = _transform(transform, local_point)
                point_count += 1
                values = {
                    'front_m': x_m, 'rear_m': x_m,
                    'left_m': y_m, 'right_m': y_m,
                    'top_m': z_m, 'bottom_m': z_m,
                }
                comparisons = {
                    'front_m': x_m > bounds['front_m'],
                    'rear_m': x_m < bounds['rear_m'],
                    'left_m': y_m > bounds['left_m'],
                    'right_m': y_m < bounds['right_m'],
                    'top_m': z_m > bounds['top_m'],
                    'bottom_m': z_m < bounds['bottom_m'],
                }
                for name, changed in comparisons.items():
                    if changed:
                        bounds[name] = values[name]
                        witnesses[name] = {
                            'link': link_name,
                            'collision': collision_name,
                            'point_m': [x_m, y_m, z_m],
                        }
    if point_count == 0 or not all(math.isfinite(value)
                                   for value in bounds.values()):
        raise ValueError(f'no collision points matched prefix {link_prefix!r}')
    digest = hashlib.sha256(urdf_path.read_bytes()).hexdigest()
    return {
        'source': {
            'path': str(urdf_path.resolve()),
            'sha256': digest,
        },
        'root_frame': root_frame,
        'link_prefix': link_prefix,
        'joint_positions_rad': {
            name: value for name, value in sorted(initial_positions.items())
            if name.startswith(link_prefix)},
        'bounds_m': {
            **bounds,
            'half_width_m': max(abs(bounds['left_m']),
                                abs(bounds['right_m'])),
        },
        'witnesses': witnesses,
        'collision_link_count': link_count,
        'collision_point_count': point_count,
        'method': (
            'URDF joint transforms at ros2_control initial positions; '
            'all matching collision mesh vertices and box corners'),
    }

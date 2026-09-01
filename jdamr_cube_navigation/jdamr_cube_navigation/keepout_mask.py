"""Build and validate Nav2 keepout masks from map-frame polygons."""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
from typing import Iterable, Sequence

import yaml


MINIMUM_SAFETY_MARGIN_M = 0.35
FREE_PIXEL = 254
KEEPOUT_PIXEL = 0


def _expanded_path(value: str, base: Path | None = None) -> Path:
    expanded = Path(os.path.expandvars(os.path.expanduser(value)))
    if not expanded.is_absolute() and base is not None:
        expanded = base / expanded
    return expanded.resolve()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _read_pgm(path: Path) -> tuple[int, int, bytes]:
    data = path.read_bytes()
    tokens = []
    index = 0
    whitespace = b' \t\r\n\f\v'
    while len(tokens) < 4:
        while index < len(data) and data[index] in whitespace:
            index += 1
        if index < len(data) and data[index] == ord('#'):
            newline = data.find(b'\n', index)
            if newline < 0:
                raise ValueError(f'invalid PGM comment in {path}')
            index = newline + 1
            continue
        start = index
        while index < len(data) and data[index] not in whitespace + b'#':
            index += 1
        if start == index:
            raise ValueError(f'invalid PGM header in {path}')
        tokens.append(data[start:index])

    if tokens[0] != b'P5':
        raise ValueError(f'only binary P5 PGM is supported: {path}')
    width, height, max_value = (int(value) for value in tokens[1:])
    if max_value != 255:
        raise ValueError(f'PGM max value must be 255: {path}')
    if data[index:index + 2] == b'\r\n':
        index += 2
    elif index < len(data) and data[index] in whitespace:
        index += 1
    pixels = data[index:index + width * height]
    if len(pixels) != width * height:
        raise ValueError(f'PGM pixel count does not match header: {path}')
    return width, height, pixels


def _write_pgm(path: Path, width: int, height: int, pixels: bytes) -> None:
    header = f'P5\n{width} {height}\n255\n'.encode('ascii')
    path.write_bytes(header + pixels)


def _map_metadata(map_yaml: Path) -> dict:
    with map_yaml.open(encoding='utf-8') as stream:
        metadata = yaml.safe_load(stream)
    required = {'image', 'resolution', 'origin'}
    if not isinstance(metadata, dict) or not required <= metadata.keys():
        raise ValueError(f'invalid map YAML: {map_yaml}')
    image = _expanded_path(str(metadata['image']), map_yaml.parent)
    width, height, pixels = _read_pgm(image)
    origin = [float(value) for value in metadata['origin']]
    if len(origin) != 3 or not math.isclose(origin[2], 0.0, abs_tol=1e-9):
        raise ValueError('rotated map origins are not supported')
    return {
        'yaml': map_yaml,
        'image': image,
        'width': width,
        'height': height,
        'pixels': pixels,
        'resolution': float(metadata['resolution']),
        'origin': origin,
    }


def _point_in_polygon(
        x_value: float, y_value: float,
        polygon: Sequence[Sequence[float]]) -> bool:
    inside = False
    previous = polygon[-1]
    for current in polygon:
        x_current, y_current = current
        x_previous, y_previous = previous
        crosses = (y_current > y_value) != (y_previous > y_value)
        if crosses:
            x_intersection = (
                (x_previous - x_current) * (y_value - y_current) /
                (y_previous - y_current) + x_current)
            if x_value < x_intersection:
                inside = not inside
        previous = current
    return inside


def _distance_squared_to_segment(
        x_value: float, y_value: float,
        start: Sequence[float], end: Sequence[float]) -> float:
    x_start, y_start = start
    x_end, y_end = end
    delta_x = x_end - x_start
    delta_y = y_end - y_start
    length_squared = delta_x * delta_x + delta_y * delta_y
    if length_squared == 0.0:
        return (x_value - x_start) ** 2 + (y_value - y_start) ** 2
    ratio = (
        (x_value - x_start) * delta_x +
        (y_value - y_start) * delta_y) / length_squared
    ratio = min(1.0, max(0.0, ratio))
    nearest_x = x_start + ratio * delta_x
    nearest_y = y_start + ratio * delta_y
    return (x_value - nearest_x) ** 2 + (y_value - nearest_y) ** 2


def _near_polygon(
        x_value: float, y_value: float,
        polygon: Sequence[Sequence[float]], margin: float) -> bool:
    if _point_in_polygon(x_value, y_value, polygon):
        return True
    margin_squared = margin * margin
    segments: Iterable[tuple[Sequence[float], Sequence[float]]] = zip(
        polygon, polygon[1:] + polygon[:1])
    return any(
        _distance_squared_to_segment(x_value, y_value, start, end) <=
        margin_squared
        for start, end in segments)


def _enabled_polygons(config: dict) -> list[tuple[str, list[list[float]]]]:
    zones = config.get('zones', [])
    if not isinstance(zones, list):
        raise ValueError('zones must be a list')
    polygons = []
    for zone in zones:
        if not zone.get('enabled', True):
            continue
        zone_id = str(zone.get('id', '')).strip()
        polygon = zone.get('polygon')
        if not zone_id or not isinstance(polygon, list) or len(polygon) < 3:
            raise ValueError('each enabled zone needs an id and 3+ polygon points')
        points = [[float(point[0]), float(point[1])] for point in polygon]
        polygons.append((zone_id, points))
    if not polygons:
        raise ValueError('at least one enabled keepout zone is required')
    return polygons


def validate_mask(
        mask_yaml: Path, source_map_yaml: Path | None = None) -> dict:
    """Validate geometry alignment and non-empty keepout content."""
    mask = _map_metadata(mask_yaml.resolve())
    occupied_cells = sum(pixel < 128 for pixel in mask['pixels'])
    if occupied_cells == 0:
        raise ValueError('keepout mask contains no blocked cells')
    result = {
        'mask_yaml': str(mask['yaml']),
        'mask_sha256': _sha256(mask['image']),
        'width': mask['width'],
        'height': mask['height'],
        'resolution': mask['resolution'],
        'origin': mask['origin'],
        'keepout_cells': occupied_cells,
    }
    if source_map_yaml is not None:
        source = _map_metadata(source_map_yaml.resolve())
        for key in ('width', 'height', 'resolution', 'origin'):
            if mask[key] != source[key]:
                raise ValueError(f'keepout mask {key} does not match source map')
        result['source_map_yaml'] = str(source['yaml'])
        result['source_map_sha256'] = _sha256(source['image'])
    return result


def build_mask(
        zones_yaml: Path, output_prefix: Path,
        overwrite: bool = False) -> dict:
    """Rasterize enabled map-frame polygons into an aligned Nav2 mask."""
    zones_yaml = zones_yaml.resolve()
    with zones_yaml.open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict) or config.get('schema_version') != 1:
        raise ValueError('keepout zone schema_version must be 1')
    source_path = _expanded_path(str(config.get('map_yaml', '')), zones_yaml.parent)
    source = _map_metadata(source_path)
    margin = float(config.get('safety_margin_m', 0.0))
    if margin < MINIMUM_SAFETY_MARGIN_M:
        raise ValueError(
            f'safety_margin_m must be at least {MINIMUM_SAFETY_MARGIN_M}')
    polygons = _enabled_polygons(config)

    output_prefix = _expanded_path(str(output_prefix))
    output_image = output_prefix.with_suffix('.pgm')
    output_yaml = output_prefix.with_suffix('.yaml')
    output_report = output_prefix.with_suffix('.json')
    outputs = (output_image, output_yaml, output_report)
    if not overwrite and any(path.exists() for path in outputs):
        raise FileExistsError('output exists; use --force to replace it')
    output_prefix.parent.mkdir(parents=True, exist_ok=True)

    width = source['width']
    height = source['height']
    resolution = source['resolution']
    origin_x, origin_y, _origin_yaw = source['origin']
    pixels = bytearray([FREE_PIXEL]) * (width * height)
    for row in range(height):
        map_y = origin_y + (height - row - 0.5) * resolution
        for column in range(width):
            map_x = origin_x + (column + 0.5) * resolution
            if any(
                    _near_polygon(map_x, map_y, polygon, margin)
                    for _zone_id, polygon in polygons):
                pixels[row * width + column] = KEEPOUT_PIXEL

    if KEEPOUT_PIXEL not in pixels:
        raise ValueError('enabled zones do not overlap the source map')
    _write_pgm(output_image, width, height, bytes(pixels))
    mask_metadata = {
        'image': output_image.name,
        'mode': 'trinary',
        'resolution': resolution,
        'origin': source['origin'],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
    }
    output_yaml.write_text(
        yaml.safe_dump(mask_metadata, sort_keys=False), encoding='utf-8')
    report = validate_mask(output_yaml, source_path)
    report.update({
        'zones_yaml': str(zones_yaml),
        'zones': [zone_id for zone_id, _polygon in polygons],
        'safety_margin_m': margin,
    })
    output_report.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    return report


def main() -> None:
    """Run the keepout mask build or validation command."""
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest='command', required=True)
    build = subparsers.add_parser('build', help='build mask from zone polygons')
    build.add_argument('--zones', required=True, type=Path)
    build.add_argument('--output-prefix', required=True, type=Path)
    build.add_argument('--force', action='store_true')
    validate = subparsers.add_parser('validate', help='validate mask alignment')
    validate.add_argument('--mask', required=True, type=Path)
    validate.add_argument('--map', required=True, type=Path)
    args = parser.parse_args()
    if args.command == 'build':
        result = build_mask(args.zones, args.output_prefix, args.force)
    else:
        result = validate_mask(args.mask, args.map)
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()

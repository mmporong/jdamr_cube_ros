#!/usr/bin/env python3
"""Register two occupancy maps and reproject a Keepout mask as a candidate."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml


def sha256(path: Path) -> str:
    """Return a file SHA-256 digest."""
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def portable_path(path: Path) -> str:
    """Render paths below the artifact root through ``$HOME``."""
    resolved = path.resolve()
    roots = (
        (Path.home() / 'jdamr_artifacts').resolve(),
        (Path.home() / 'maps').resolve(),
    )
    labels = ('$HOME/jdamr_artifacts', '$HOME/maps')
    for root, label in zip(roots, labels):
        try:
            return f'{label}/{resolved.relative_to(root)}'
        except ValueError:
            continue
    return str(resolved)


def load_grid(yaml_path: Path) -> tuple[dict[str, Any], Path, np.ndarray]:
    """Load one axis-aligned map YAML and grayscale image."""
    metadata = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
    if not isinstance(metadata, dict):
        raise ValueError(f'invalid map YAML: {yaml_path}')
    origin = metadata.get('origin')
    resolution = float(metadata.get('resolution', 0.0))
    if (not isinstance(origin, list) or len(origin) != 3
            or not math.isfinite(resolution) or resolution <= 0.0
            or any(not math.isfinite(float(value)) for value in origin)):
        raise ValueError(f'invalid map geometry: {yaml_path}')
    if abs(float(origin[2])) > 1e-9:
        raise ValueError('only axis-aligned map origins are supported')
    image_path = (yaml_path.parent / metadata['image']).resolve()
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError(f'cannot load map image: {image_path}')
    return metadata, image_path, image


def occupied_mask(metadata: dict[str, Any], image: np.ndarray) -> np.ndarray:
    """Return occupied cells using the map-server trinary threshold."""
    negate = int(metadata.get('negate', 0))
    threshold = float(metadata.get('occupied_thresh', 0.65))
    occupancy = image.astype(np.float64) / 255.0
    if negate == 0:
        occupancy = 1.0 - occupancy
    return occupancy > threshold


def validate_source_grids(
        map_metadata: dict[str, Any], map_shape: tuple[int, int],
        mask_metadata: dict[str, Any], mask_shape: tuple[int, int]) -> None:
    """Reject a source mask that is not on its source map grid."""
    if (map_shape != mask_shape
            or map_metadata['origin'] != mask_metadata['origin']
            or float(map_metadata['resolution'])
            != float(mask_metadata['resolution'])):
        raise ValueError('source map and Keepout mask grids differ')


def cell_centres(metadata: dict[str, Any], shape: tuple[int, int],
                 rows: np.ndarray, columns: np.ndarray) -> np.ndarray:
    """Convert image rows and columns to map coordinates."""
    height, _ = shape
    resolution = float(metadata['resolution'])
    origin_x, origin_y, _ = (float(value) for value in metadata['origin'])
    return np.column_stack((
        origin_x + (columns + 0.5) * resolution,
        origin_y + (height - rows - 0.5) * resolution,
    ))


def transform_points(points: np.ndarray, yaw_rad: float,
                     translation_xy_m: tuple[float, float]) -> np.ndarray:
    """Apply the source-map to target-map rigid transform."""
    cosine = math.cos(yaw_rad)
    sine = math.sin(yaw_rad)
    tx_m, ty_m = translation_xy_m
    return np.column_stack((
        cosine * points[:, 0] - sine * points[:, 1] + tx_m,
        sine * points[:, 0] + cosine * points[:, 1] + ty_m,
    ))


def inverse_transform_points(points: np.ndarray, yaw_rad: float,
                             translation_xy_m: tuple[float, float]) -> np.ndarray:
    """Apply the inverse source-map to target-map transform."""
    tx_m, ty_m = translation_xy_m
    shifted = points - np.asarray([tx_m, ty_m])
    cosine = math.cos(yaw_rad)
    sine = math.sin(yaw_rad)
    return np.column_stack((
        cosine * shifted[:, 0] + sine * shifted[:, 1],
        -sine * shifted[:, 0] + cosine * shifted[:, 1],
    ))


def sample_distance(points: np.ndarray, metadata: dict[str, Any],
                    shape: tuple[int, int], distance_m: np.ndarray,
                    truncation_m: float = 0.5) -> float:
    """Return a truncated distance-transform score with an outside penalty."""
    height, width = shape
    resolution = float(metadata['resolution'])
    origin_x, origin_y, _ = (float(value) for value in metadata['origin'])
    columns = np.rint(
        (points[:, 0] - origin_x) / resolution - 0.5).astype(int)
    rows = height - 1 - np.rint(
        (points[:, 1] - origin_y) / resolution - 0.5).astype(int)
    inside = ((rows >= 0) & (rows < height)
              & (columns >= 0) & (columns < width))
    distances = np.full(len(points), truncation_m * 1.5)
    distances[inside] = distance_m[rows[inside], columns[inside]]
    return float(
        np.mean(np.minimum(distances, truncation_m))
        + 0.5 * (1.0 - np.mean(inside)))


def grid_indices(points: np.ndarray, metadata: dict[str, Any],
                 shape: tuple[int, int]) -> tuple[np.ndarray, np.ndarray]:
    """Return in-bounds image indices for map-coordinate points."""
    height, width = shape
    resolution = float(metadata['resolution'])
    origin_x, origin_y, _ = (float(value) for value in metadata['origin'])
    columns = np.rint(
        (points[:, 0] - origin_x) / resolution - 0.5).astype(int)
    rows = height - 1 - np.rint(
        (points[:, 1] - origin_y) / resolution - 0.5).astype(int)
    inside = ((rows >= 0) & (rows < height)
              & (columns >= 0) & (columns < width))
    return rows[inside], columns[inside]


def registration_context(
        source_metadata: dict[str, Any], source_image: np.ndarray,
        target_metadata: dict[str, Any], target_image: np.ndarray,
        decimation: int = 4) -> dict[str, Any]:
    """Build occupied point clouds and distance fields for registration."""
    source_occupied = occupied_mask(source_metadata, source_image)
    target_occupied = occupied_mask(target_metadata, target_image)
    source_rows, source_columns = np.nonzero(source_occupied)
    target_rows, target_columns = np.nonzero(target_occupied)
    if len(source_rows) < 10 or len(target_rows) < 10:
        raise ValueError('maps do not contain enough occupied cells')
    source_points = cell_centres(
        source_metadata, source_image.shape, source_rows, source_columns)
    target_points = cell_centres(
        target_metadata, target_image.shape, target_rows, target_columns)
    source_distance = cv2.distanceTransform(
        (~source_occupied).astype(np.uint8), cv2.DIST_L2, 3)
    target_distance = cv2.distanceTransform(
        (~target_occupied).astype(np.uint8), cv2.DIST_L2, 3)
    source_distance *= float(source_metadata['resolution'])
    target_distance *= float(target_metadata['resolution'])
    return {
        'source_points': source_points[::decimation],
        'target_points': target_points[::decimation],
        'source_distance_m': source_distance,
        'target_distance_m': target_distance,
    }


def registration_score(context: dict[str, Any],
                       source_metadata: dict[str, Any],
                       source_shape: tuple[int, int],
                       target_metadata: dict[str, Any],
                       target_shape: tuple[int, int],
                       yaw_rad: float, tx_m: float, ty_m: float) -> float:
    """Return the symmetric truncated Chamfer score in metres."""
    translation = (tx_m, ty_m)
    forward = transform_points(
        context['source_points'], yaw_rad, translation)
    backward = inverse_transform_points(
        context['target_points'], yaw_rad, translation)
    forward_score = sample_distance(
        forward, target_metadata, target_shape,
        context['target_distance_m'])
    backward_score = sample_distance(
        backward, source_metadata, source_shape,
        context['source_distance_m'])
    return 0.5 * (forward_score + backward_score)


def grid_search(context: dict[str, Any], source_metadata: dict[str, Any],
                source_shape: tuple[int, int],
                target_metadata: dict[str, Any],
                target_shape: tuple[int, int]) -> dict[str, float]:
    """Run deterministic coarse-to-fine SE(2) map registration."""
    centre = (0.0, 0.0, 0.0)
    stages = (
        ((math.radians(5.0), 2.0, 2.0), (21, 21, 21)),
        ((math.radians(0.6), 0.25, 0.25), (25, 26, 26)),
        ((math.radians(0.08), 0.04, 0.04), (17, 17, 17)),
    )
    best_score = math.inf
    for spans, counts in stages:
        best = None
        stage_best_score = math.inf
        for yaw_rad in np.linspace(
                centre[0] - spans[0], centre[0] + spans[0], counts[0]):
            for tx_m in np.linspace(
                    centre[1] - spans[1], centre[1] + spans[1], counts[1]):
                for ty_m in np.linspace(
                        centre[2] - spans[2], centre[2] + spans[2], counts[2]):
                    score = registration_score(
                        context, source_metadata, source_shape,
                        target_metadata, target_shape,
                        float(yaw_rad), float(tx_m), float(ty_m))
                    if score < stage_best_score:
                        stage_best_score = score
                        best = (float(yaw_rad), float(tx_m), float(ty_m))
        if best is None:
            raise RuntimeError('registration search produced no candidate')
        centre = best
        best_score = stage_best_score
    return {
        'yaw_rad': centre[0],
        'yaw_deg': math.degrees(centre[0]),
        'translation_x_m': centre[1],
        'translation_y_m': centre[2],
        'score_m': best_score,
    }


def reproject_mask(source_metadata: dict[str, Any], source_mask: np.ndarray,
                   target_metadata: dict[str, Any],
                   target_shape: tuple[int, int], yaw_rad: float,
                   translation_xy_m: tuple[float, float]) -> np.ndarray:
    """Sample a source mask at every target cell using inverse mapping."""
    height, width = target_shape
    rows, columns = np.indices((height, width))
    target_points = cell_centres(
        target_metadata, target_shape, rows.ravel(), columns.ravel())
    source_points = inverse_transform_points(
        target_points, yaw_rad, translation_xy_m)
    source_height, source_width = source_mask.shape
    resolution = float(source_metadata['resolution'])
    origin_x, origin_y, _ = (
        float(value) for value in source_metadata['origin'])
    source_columns = np.rint(
        (source_points[:, 0] - origin_x) / resolution - 0.5).astype(int)
    source_rows = source_height - 1 - np.rint(
        (source_points[:, 1] - origin_y) / resolution - 0.5).astype(int)
    inside = ((source_rows >= 0) & (source_rows < source_height)
              & (source_columns >= 0) & (source_columns < source_width))
    free_value = int(np.max(source_mask))
    output = np.full(height * width, free_value, dtype=np.uint8)
    output[inside] = source_mask[
        source_rows[inside], source_columns[inside]]
    return output.reshape(target_shape)


def main(argv: list[str] | None = None) -> int:
    """Generate a candidate mask and registration evidence."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--source-map', type=Path, required=True)
    parser.add_argument('--source-mask', type=Path, required=True)
    parser.add_argument('--target-map', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args(argv)

    source_meta, source_image_path, source_image = load_grid(args.source_map)
    mask_meta, source_mask_path, source_mask = load_grid(args.source_mask)
    target_meta, target_image_path, target_image = load_grid(args.target_map)
    validate_source_grids(
        source_meta, source_image.shape, mask_meta, source_mask.shape)
    if abs(float(source_meta['resolution'])
           - float(target_meta['resolution'])) > 1e-9:
        raise ValueError('source and target map resolutions differ')

    context = registration_context(
        source_meta, source_image, target_meta, target_image)
    identity_score = registration_score(
        context, source_meta, source_image.shape,
        target_meta, target_image.shape, 0.0, 0.0, 0.0)
    transform = grid_search(
        context, source_meta, source_image.shape,
        target_meta, target_image.shape)
    candidate_mask = reproject_mask(
        mask_meta, source_mask, target_meta, target_image.shape,
        transform['yaw_rad'],
        (transform['translation_x_m'], transform['translation_y_m']))

    args.output_dir.mkdir(parents=True, exist_ok=True)
    stem = 'new_base_revisit_20260916_keepout_candidate'
    mask_path = args.output_dir / f'{stem}.pgm'
    yaml_path = args.output_dir / f'{stem}.yaml'
    overlay_path = args.output_dir / f'{stem}_overlay.png'
    registration_overlay_path = (
        args.output_dir / f'{stem}_registration_overlay.png')
    report_path = args.output_dir / f'{stem}_registration.json'
    if not cv2.imwrite(str(mask_path), candidate_mask):
        raise RuntimeError(f'failed to write {mask_path}')
    yaml_record = {
        'image': mask_path.name,
        'mode': 'trinary',
        'resolution': float(target_meta['resolution']),
        'origin': [float(value) for value in target_meta['origin']],
        'negate': int(mask_meta.get('negate', 0)),
        'occupied_thresh': float(mask_meta.get('occupied_thresh', 0.65)),
        'free_thresh': float(mask_meta.get('free_thresh', 0.196)),
    }
    yaml_path.write_text(
        yaml.safe_dump(yaml_record, sort_keys=False), encoding='utf-8')
    overlay = cv2.cvtColor(target_image, cv2.COLOR_GRAY2BGR)
    overlay[candidate_mask < 128] = (0, 0, 255)
    if not cv2.imwrite(str(overlay_path), overlay):
        raise RuntimeError(f'failed to write {overlay_path}')
    registration_overlay = cv2.cvtColor(
        target_image, cv2.COLOR_GRAY2BGR)
    source_occupied = occupied_mask(source_meta, source_image)
    source_rows, source_columns = np.nonzero(source_occupied)
    source_points = cell_centres(
        source_meta, source_image.shape, source_rows, source_columns)
    transformed_source = transform_points(
        source_points, transform['yaw_rad'],
        (transform['translation_x_m'], transform['translation_y_m']))
    rows, columns = grid_indices(
        transformed_source, target_meta, target_image.shape)
    registration_overlay[rows, columns] = (255, 80, 0)
    if not cv2.imwrite(
            str(registration_overlay_path), registration_overlay):
        raise RuntimeError(f'failed to write {registration_overlay_path}')

    report = {
        'schema_version': 1,
        'status': 'CANDIDATE_REQUIRES_MANUAL_REVIEW_AND_NAV_TEST',
        'source_map': {
            'yaml': portable_path(args.source_map),
            'yaml_sha256': sha256(args.source_map),
            'image_sha256': sha256(source_image_path),
        },
        'source_mask': {
            'yaml': portable_path(args.source_mask),
            'yaml_sha256': sha256(args.source_mask),
            'image_sha256': sha256(source_mask_path),
            'occupied_cells': int(np.count_nonzero(source_mask < 128)),
            'connected_components': int(
                cv2.connectedComponents((source_mask < 128).astype(
                    np.uint8))[0] - 1),
        },
        'target_map': {
            'yaml': portable_path(args.target_map),
            'yaml_sha256': sha256(args.target_map),
            'image_sha256': sha256(target_image_path),
        },
        'registration': {
            **transform,
            'identity_score_m': identity_score,
            'score_improvement_ratio': identity_score / transform['score_m'],
            'method': 'symmetric truncated Chamfer, deterministic grid search',
        },
        'candidate_mask': {
            'yaml': portable_path(yaml_path),
            'yaml_sha256': sha256(yaml_path),
            'image_sha256': sha256(mask_path),
            'overlay_sha256': sha256(overlay_path),
            'registration_overlay_sha256': sha256(
                registration_overlay_path),
            'occupied_cells': int(np.count_nonzero(candidate_mask < 128)),
            'occupied_area_m2': float(
                np.count_nonzero(candidate_mask < 128)
                * float(target_meta['resolution']) ** 2),
            'connected_components': int(
                cv2.connectedComponents((candidate_mask < 128).astype(
                    np.uint8))[0] - 1),
            'grid_shape_cells': [
                int(target_image.shape[1]), int(target_image.shape[0])],
            'target_grid_match': True,
        },
        'limitations': [
            'The registration uses occupied map geometry, not surveyed landmarks.',
            'The target map contains faint duplicate walls.',
            'The candidate must pass manual landmark review and a no-motion Nav2 load check.',
            'The operational map and mask are unchanged.',
        ],
    }
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

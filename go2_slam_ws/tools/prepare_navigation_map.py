#!/usr/bin/env python3
"""Convert a saved GO2-W LIO map into conservative navigation artifacts.

The source NPZ is never modified.  Outputs contain a levelled 2-D occupancy
map, an inflated planning map and a 2.5-D PCD obstacle layer for local planners.
"""

import argparse
import json
import math
import os
from datetime import datetime
from pathlib import Path

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree


def disk(radius_cells):
    radius_cells = max(0, int(radius_cells))
    y, x = np.ogrid[-radius_cells:radius_cells + 1,
                    -radius_cells:radius_cells + 1]
    return x * x + y * y <= radius_cells * radius_cells


def fit_floor_plane(points, threshold=0.06, iterations=900, seed=20260812):
    """RANSAC fit of z = ax + by + c, constrained to a near-horizontal floor."""
    points = np.asarray(points, dtype=np.float64)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < 100:
        raise ValueError('not enough finite ground points to estimate the floor')

    rng = np.random.RandomState(seed)
    design = np.column_stack((points[:, :2], np.ones(len(points))))
    best = None
    best_count = 0
    best_median = float('inf')
    for _ in range(iterations):
        ids = rng.choice(len(points), 3, replace=False)
        sample = design[ids]
        if abs(np.linalg.det(sample)) < 1e-5:
            continue
        coeff = np.linalg.solve(sample, points[ids, 2])
        if math.hypot(coeff[0], coeff[1]) > math.tan(math.radians(5.0)):
            continue
        residual = np.abs(points[:, 2] - design.dot(coeff))
        inliers = residual <= threshold
        count = int(inliers.sum())
        median = float(np.median(residual[inliers])) if count else float('inf')
        if count > best_count or (count == best_count and median < best_median):
            best = inliers
            best_count = count
            best_median = median
    if best is None or best_count < 100:
        raise ValueError('could not find a stable horizontal floor plane')

    coeff = None
    inliers = best
    for _ in range(3):
        coeff = np.linalg.lstsq(design[inliers], points[inliers, 2], rcond=None)[0]
        residual = np.abs(points[:, 2] - design.dot(coeff))
        inliers = residual <= threshold
    rms = float(np.sqrt(np.mean(np.square(residual[inliers]))))
    return coeff, inliers, rms


def level_points(points, plane):
    result = np.asarray(points, dtype=np.float32).copy()
    result[:, 2] -= (plane[0] * result[:, 0] +
                     plane[1] * result[:, 1] + plane[2])
    return result


def filter_xy_outliers(points, radius, min_neighbors):
    if len(points) <= min_neighbors:
        return np.zeros(len(points), dtype=bool)
    tree = cKDTree(points[:, :2])
    # The nearest result includes the query point itself.
    distances = tree.query(points[:, :2], k=min_neighbors + 1)[0]
    if distances.ndim == 1:
        distances = distances[:, None]
    return distances[:, -1] <= radius


def filter_obstacle_components(seed, ix, iy, point_z, min_cells,
                               min_points, min_vertical_span,
                               vertical_ix=None, vertical_iy=None,
                               vertical_z=None):
    """Reject weak isolated obstacle evidence before safety inflation.

    A component is retained when it has a sufficiently large XY footprint,
    repeated point evidence, or a tall vertical return.  This keeps walls,
    compact repeatedly-observed objects and poles while removing single-scan
    speckle that would otherwise become a 60 cm-wide disk after inflation.
    """
    labels, component_count = ndimage.label(
        seed, structure=np.ones((3, 3), dtype=np.uint8))
    if not component_count:
        return seed, {
            'input_components': 0,
            'kept_components': 0,
            'removed_components': 0,
            'input_seed_cells': 0,
            'kept_seed_cells': 0,
            'kept_points': 0,
        }

    component_cells = np.bincount(
        labels.ravel(), minlength=component_count + 1)
    point_labels = labels[iy, ix]
    component_points = np.bincount(
        point_labels, minlength=component_count + 1)
    # Occupancy is seeded only from returns at collision height.  A wall can
    # nevertheless have very few returns in that narrow band while remaining
    # unmistakably vertical in the full body-height cloud.  Use those taller
    # returns only to validate an existing seed component; they never create a
    # new occupied cell on their own (so ceilings/overhangs stay passable).
    if vertical_ix is None or vertical_iy is None or vertical_z is None:
        vertical_labels = point_labels
        vertical_values = point_z
    else:
        vertical_labels = labels[vertical_iy, vertical_ix]
        supported = vertical_labels > 0
        vertical_labels = vertical_labels[supported]
        vertical_values = vertical_z[supported]
    min_z = np.full(component_count + 1, np.inf, dtype=np.float64)
    max_z = np.full(component_count + 1, -np.inf, dtype=np.float64)
    np.minimum.at(min_z, vertical_labels, vertical_values)
    np.maximum.at(max_z, vertical_labels, vertical_values)
    vertical_span = max_z - min_z

    keep = ((component_cells >= max(1, min_cells)) |
            (component_points >= max(1, min_points)) |
            (vertical_span >= max(0.0, min_vertical_span)))
    keep[0] = False
    filtered = keep[labels]
    return filtered, {
        'input_components': int(component_count),
        'kept_components': int(keep.sum()),
        'removed_components': int(component_count - keep.sum()),
        'input_seed_cells': int(seed.sum()),
        'kept_seed_cells': int(filtered.sum()),
        'kept_points': int(component_points[keep].sum()),
    }


def exact_row_membership(points, reference):
    """Return which float rows occur verbatim in *reference*.

    The mapper stores ``ground`` as a subset of ``map``.  Matching the original
    float32 rows lets the post-filter distinguish saved ground returns from
    independently observed obstacle returns without relying on another global
    height threshold.
    """
    points = np.ascontiguousarray(points)
    reference = np.ascontiguousarray(reference)
    if (points.ndim != 2 or reference.ndim != 2 or
            points.shape[1] != reference.shape[1] or
            points.dtype != reference.dtype):
        return np.zeros(len(points), dtype=bool)
    row_dtype = np.dtype((np.void, points.dtype.itemsize * points.shape[1]))
    point_rows = points.view(row_dtype).ravel()
    reference_rows = reference.view(row_dtype).ravel()
    return np.isin(point_rows, reference_rows)


def filter_dilated_speckles(occupied, support_ix, support_iy,
                            max_cells, min_support_points):
    """Remove only small obstacle islands lacking non-ground support.

    This runs after the thin-wall dilation, so ``max_cells`` describes the
    physical footprint that will be inflated for robot clearance.  Walls and
    large objects are retained by area; compact real objects are retained when
    multiple source points were not classified as ground by the mapper.
    """
    labels, component_count = ndimage.label(
        occupied, structure=np.ones((3, 3), dtype=np.uint8))
    if not component_count or max_cells <= 0:
        return occupied, {
            'input_components': int(component_count),
            'removed_components': 0,
            'removed_cells': 0,
        }

    component_cells = np.bincount(
        labels.ravel(), minlength=component_count + 1)
    if len(support_ix):
        support_labels = labels[support_iy, support_ix]
        support_points = np.bincount(
            support_labels, minlength=component_count + 1)
    else:
        support_points = np.zeros(component_count + 1, dtype=np.int64)
    remove = ((component_cells <= max(1, int(max_cells))) &
              (support_points < max(1, int(min_support_points))))
    remove[0] = False
    filtered = occupied & ~remove[labels]
    return filtered, {
        'input_components': int(component_count),
        'removed_components': int(remove.sum()),
        'removed_cells': int(occupied.sum() - filtered.sum()),
        'max_cells': int(max_cells),
        'max_area_m2': None,
        'min_nonground_support_points': int(min_support_points),
    }


def save_pgm(path, values):
    values = np.asarray(values, dtype=np.uint8)
    header = 'P5\n%d %d\n255\n' % (values.shape[1], values.shape[0])
    with open(path, 'wb') as stream:
        stream.write(header.encode('ascii'))
        stream.write(values.tobytes())


def save_yaml(path, image_name, resolution, origin):
    text = (
        'image: %s\n'
        'mode: trinary\n'
        'resolution: %.6f\n'
        'origin: [%.6f, %.6f, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.196\n'
    ) % (image_name, resolution, origin[0], origin[1])
    with open(path, 'w', encoding='utf-8') as stream:
        stream.write(text)


def save_pcd(path, points):
    points = np.asarray(points, dtype=np.float32)
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\n'
        'FIELDS x y z\n'
        'SIZE 4 4 4\n'
        'TYPE F F F\n'
        'COUNT 1 1 1\n'
        'WIDTH %d\n'
        'HEIGHT 1\n'
        'VIEWPOINT 0 0 0 1 0 0 0\n'
        'POINTS %d\n'
        'DATA ascii\n'
    ) % (len(points), len(points))
    with open(path, 'w', encoding='ascii') as stream:
        stream.write(header)
        np.savetxt(stream, points, fmt='%.5f %.5f %.5f')


def free_space_connectivity(free, x0, y0, resolution, start_x, start_y):
    """Summarize free space using the planner's exact connectivity rule."""
    labels, component_count = ndimage.label(
        free, structure=np.array(
            [[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=np.uint8))
    sizes = np.bincount(labels.ravel())[1:]
    start_ix = int(math.floor((start_x - x0) / resolution))
    start_iy = int(math.floor((start_y - y0) / resolution))
    start_size = 0
    if 0 <= start_ix < free.shape[1] and 0 <= start_iy < free.shape[0]:
        start_label = int(labels[start_iy, start_ix])
        if start_label:
            start_size = int(sizes[start_label - 1])
    largest = int(sizes.max()) if len(sizes) else 0
    return {
        'components': int(component_count),
        'largest_component_cells': largest,
        'start_component_cells': start_size,
        'start_in_largest_component': bool(start_size and start_size == largest),
    }


def latest_npz(maps_dir):
    files = sorted(Path(maps_dir).glob('map_*.npz'), key=lambda p: p.stat().st_mtime)
    if not files:
        raise FileNotFoundError('no map_*.npz found in %s' % maps_dir)
    return files[-1]


def build(args):
    source = Path(args.input).resolve() if args.input else latest_npz(args.maps_dir).resolve()
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    prefix = args.prefix or (source.stem + '_nav')
    if (not prefix or prefix != os.path.basename(prefix) or
            any(char not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for char in prefix)):
        raise ValueError('output prefix contains unsupported characters')

    with np.load(str(source)) as archive:
        if 'map' not in archive or 'ground' not in archive:
            raise ValueError('source NPZ must contain map and ground arrays')
        map_points = np.asarray(archive['map'], dtype=np.float32)
        ground_points = np.asarray(archive['ground'], dtype=np.float32)
    map_points = map_points[np.isfinite(map_points).all(axis=1)]
    ground_points = ground_points[np.isfinite(ground_points).all(axis=1)]

    plane, ground_inliers, floor_rms = fit_floor_plane(
        ground_points, threshold=args.floor_threshold)
    level_map = level_points(map_points, plane)
    level_ground = level_points(ground_points, plane)
    floor_points = level_ground[ground_inliers]
    map_is_saved_ground = exact_row_membership(map_points, ground_points)
    ground_is_in_map = exact_row_membership(ground_points, map_points)
    ground_match_ratio = (float(ground_is_in_map.mean())
                          if len(ground_is_in_map) else 0.0)

    # The dominant floor footprint is a safer map boundary than extrema from
    # long-range lidar returns.  Percentiles also reject a few RANSAC edge hits.
    x0, y0 = np.percentile(floor_points[:, :2], args.bound_percentile, axis=0)
    x1, y1 = np.percentile(floor_points[:, :2], 100.0 - args.bound_percentile, axis=0)
    x0 -= args.padding
    y0 -= args.padding
    x1 += args.padding
    y1 += args.padding

    obstacle_band = ((level_map[:, 2] >= args.obstacle_min_z) &
                     (level_map[:, 2] <= args.obstacle_max_z))
    inside = ((level_map[:, 0] >= x0) & (level_map[:, 0] <= x1) &
              (level_map[:, 1] >= y0) & (level_map[:, 1] <= y1))
    obstacle_source_mask = obstacle_band & inside
    obstacle_points = level_map[obstacle_source_mask]
    obstacle_is_saved_ground = map_is_saved_ground[obstacle_source_mask]
    neighbor_mask = filter_xy_outliers(
        obstacle_points, args.outlier_radius, args.min_neighbors)
    obstacle_points = obstacle_points[neighbor_mask]
    obstacle_is_saved_ground = obstacle_is_saved_ground[neighbor_mask]

    # Full vertical evidence protects sparse walls from the component noise
    # filter.  It is deliberately not projected into occupancy: only a return
    # already present in the collision-height band can seed an obstacle.
    vertical_evidence_mask = (
        (level_map[:, 2] >= args.obstacle_min_z) &
        (level_map[:, 2] <= args.vertical_evidence_max_z) & inside)
    vertical_evidence = level_map[vertical_evidence_mask]
    vertical_neighbor_mask = filter_xy_outliers(
        vertical_evidence, args.outlier_radius, args.min_neighbors)
    vertical_evidence = vertical_evidence[vertical_neighbor_mask]

    resolution = args.resolution
    width = max(4, int(math.ceil((x1 - x0) / resolution)))
    height = max(4, int(math.ceil((y1 - y0) / resolution)))

    def to_cells(points):
        ix = np.floor((points[:, 0] - x0) / resolution).astype(np.int64)
        iy = np.floor((points[:, 1] - y0) / resolution).astype(np.int64)
        valid = ((ix >= 0) & (ix < width) & (iy >= 0) & (iy < height))
        return ix[valid], iy[valid], valid

    occupied = np.zeros((height, width), dtype=bool)
    ix, iy, valid = to_cells(obstacle_points)
    vertical_ix, vertical_iy, vertical_valid = to_cells(vertical_evidence)
    occupied[iy, ix] = True
    occupied, component_stats = filter_obstacle_components(
        occupied, ix, iy, obstacle_points[valid, 2],
        args.min_component_cells, args.min_component_points,
        args.min_component_height,
        vertical_ix, vertical_iy, vertical_evidence[vertical_valid, 2])
    wall_cells = int(math.ceil(args.wall_thickness / resolution))
    if wall_cells:
        occupied = ndimage.binary_dilation(occupied, structure=disk(wall_cells))

    # Only enable semantic speckle removal when the mapper's ground cloud can
    # be verified as a subset of the full cloud.  On maps from another source,
    # falling back to the earlier evidence filter is safer than guessing.
    speckle_enabled = ground_match_ratio >= 0.95
    support_mask = (~obstacle_is_saved_ground[valid]
                    if speckle_enabled else np.ones(len(ix), dtype=bool))
    occupied, speckle_stats = filter_dilated_speckles(
        occupied, ix[support_mask], iy[support_mask],
        args.max_speckle_cells if speckle_enabled else 0,
        args.min_nonground_support_points)
    if 'max_area_m2' in speckle_stats:
        speckle_stats['max_area_m2'] = float(
            args.max_speckle_cells * resolution * resolution)
    speckle_stats['enabled'] = bool(speckle_enabled)

    clear_start = np.zeros_like(occupied)
    if args.clear_start_radius > 0.0:
        grid_y, grid_x = np.ogrid[:height, :width]
        cell_x = x0 + (grid_x + 0.5) * resolution
        cell_y = y0 + (grid_y + 0.5) * resolution
        clear_start = ((cell_x - args.clear_start_x) ** 2 +
                       (cell_y - args.clear_start_y) ** 2 <=
                       args.clear_start_radius ** 2)
        occupied[clear_start] = False

    free = np.zeros_like(occupied)
    ix, iy, _ = to_cells(floor_points)
    free[iy, ix] = True
    free_cells = int(math.ceil(args.floor_fill_radius / resolution))
    if free_cells:
        free = ndimage.binary_dilation(free, structure=disk(free_cells))
    free[clear_start] = True
    free &= ~occupied

    inflated_cells = int(math.ceil(args.inflation_radius / resolution))
    inflated = ndimage.binary_dilation(occupied, structure=disk(inflated_cells))
    # clear-start is an explicitly verified free region.  Dilation used to
    # paint its boundary back inward and trap the robot in a circular island.
    inflated[clear_start] = False
    # Optional permissive mode for maps whose floor evidence is incomplete:
    # every cell inside the generated map boundary is considered traversable
    # unless obstacle evidence (or its safety inflation) explicitly blocks it.
    # Keep this explicit instead of silently changing the conservative CLI
    # default, because genuinely unobserved space can otherwise be hazardous.
    raw_free = ~occupied if args.unknown_as_free else free
    inflated_free = ~inflated if args.unknown_as_free else (free & ~inflated)
    raw_connectivity = free_space_connectivity(
        raw_free, x0, y0, resolution, args.clear_start_x, args.clear_start_y)
    inflated_connectivity = free_space_connectivity(
        inflated_free, x0, y0, resolution,
        args.clear_start_x, args.clear_start_y)

    # ROS map PGM convention: black occupied, white free, gray unknown.  PGM
    # row zero is the maximum-y row, hence the vertical flip before saving.
    raw_image = np.full((height, width), 205, dtype=np.uint8)
    raw_image[raw_free] = 254
    raw_image[occupied] = 0
    inflated_image = np.full((height, width), 205, dtype=np.uint8)
    inflated_image[inflated_free] = 254
    inflated_image[inflated] = 0

    raw_pgm = output_dir / (prefix + '.pgm')
    raw_yaml = output_dir / (prefix + '.yaml')
    inflated_pgm = output_dir / (prefix + '_inflated.pgm')
    inflated_yaml = output_dir / (prefix + '_inflated.yaml')
    pcd_path = output_dir / (prefix + '_obstacles.pcd')
    metadata_path = output_dir / (prefix + '.json')
    save_pgm(str(raw_pgm), raw_image[::-1])
    save_yaml(str(raw_yaml), raw_pgm.name, resolution, (x0, y0))
    save_pgm(str(inflated_pgm), inflated_image[::-1])
    save_yaml(str(inflated_yaml), inflated_pgm.name, resolution, (x0, y0))

    # One point at the robot collision-check height per raw occupied cell gives
    # SCAN-Planner a deterministic 2.5-D layer without duplicating wall points.
    oy, ox = np.nonzero(occupied)
    pcd_points = np.column_stack((
        x0 + (ox + 0.5) * resolution,
        y0 + (oy + 0.5) * resolution,
        np.full(len(ox), args.collision_z),
    ))
    save_pcd(str(pcd_path), pcd_points)

    tilt = math.degrees(math.atan(math.hypot(plane[0], plane[1])))
    metadata = {
        'source': str(source),
        'created_at': datetime.now().astimezone().isoformat(),
        'source_points': int(len(map_points)),
        'source_ground_points': int(len(ground_points)),
        'saved_ground_exact_match_ratio': ground_match_ratio,
        'floor_inliers': int(ground_inliers.sum()),
        'floor_plane_z_ax_by_c': [float(v) for v in plane],
        'floor_tilt_deg': float(tilt),
        'floor_rms_m': floor_rms,
        'obstacle_band_m': [args.obstacle_min_z, args.obstacle_max_z],
        'vertical_validation_band_m': [
            args.obstacle_min_z, args.vertical_evidence_max_z],
        'filtered_obstacle_points': int(len(obstacle_points)),
        'vertical_validation_points': int(len(vertical_evidence)),
        'evidence_obstacle_points': component_stats['kept_points'],
        'obstacle_component_filter': {
            'min_cells': args.min_component_cells,
            'min_points': args.min_component_points,
            'min_vertical_span_m': args.min_component_height,
            **component_stats,
        },
        'post_dilation_speckle_filter': speckle_stats,
        'occupied_cells': int(occupied.sum()),
        'inflated_cells': int(inflated.sum()),
        'floor_evidence_cells': int(free.sum()),
        'free_cells': int(raw_free.sum()),
        'inflated_free_cells': int(inflated_free.sum()),
        'unknown_as_free': bool(args.unknown_as_free),
        'resolution_m': resolution,
        'origin_xy': [float(x0), float(y0)],
        'size_cells': [width, height],
        'inflation_radius_m': args.inflation_radius,
        'collision_z_m': args.collision_z,
        'cleared_start_xy_radius_m': [
            args.clear_start_x, args.clear_start_y, args.clear_start_radius],
        'clear_start_applied_after_inflation': True,
        'free_connectivity': {
            'raw': raw_connectivity,
            'inflated': inflated_connectivity,
        },
        'static_map_to_odom': [0.0, 0.0, 0.0],
        'static_map_to_odom_note': 'identity dry-run; calibrate before actuator enable',
        'artifacts': {
            'map_yaml': str(raw_yaml),
            'inflated_map_yaml': str(inflated_yaml),
            'obstacle_pcd': str(pcd_path),
        },
    }
    with open(metadata_path, 'w', encoding='utf-8') as stream:
        json.dump(metadata, stream, indent=2, ensure_ascii=False)
        stream.write('\n')

    print(json.dumps(metadata, indent=2, ensure_ascii=False))
    return metadata


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', nargs='?', help='saved map NPZ; default is latest map_*.npz')
    parser.add_argument('--maps-dir', default='/home/unitree/go2_slam_ws/maps')
    parser.add_argument('--output-dir', default='/home/unitree/go2_slam_ws/maps/navigation')
    parser.add_argument('--prefix', help='override output filename stem')
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--obstacle-min-z', type=float, default=0.08)
    parser.add_argument('--obstacle-max-z', type=float, default=0.85)
    parser.add_argument(
        '--vertical-evidence-max-z', type=float, default=2.20,
        help='use returns up to this height to validate sparse wall components')
    parser.add_argument('--collision-z', type=float, default=0.60)
    parser.add_argument('--clear-start-x', type=float, default=0.0)
    parser.add_argument('--clear-start-y', type=float, default=0.0)
    parser.add_argument('--clear-start-radius', type=float, default=0.0)
    parser.add_argument('--floor-threshold', type=float, default=0.06)
    parser.add_argument('--floor-fill-radius', type=float, default=0.20)
    parser.add_argument(
        '--unknown-as-free', action='store_true',
        help='treat every non-obstacle cell inside the map boundary as free')
    parser.add_argument('--wall-thickness', type=float, default=0.05)
    # Go2-W is about 0.43 m wide.  A 0.23 m radius keeps 15 mm clearance on
    # each side and matches the SCAN double-cylinder collision model.
    parser.add_argument('--inflation-radius', type=float, default=0.23)
    parser.add_argument('--outlier-radius', type=float, default=0.25)
    parser.add_argument('--min-neighbors', type=int, default=2)
    parser.add_argument(
        '--min-component-cells', type=int, default=8,
        help='keep an obstacle component with at least this many grid cells')
    parser.add_argument(
        '--min-component-points', type=int, default=12,
        help='keep a compact obstacle component with this many source points')
    parser.add_argument(
        '--min-component-height', type=float, default=0.70,
        help='keep a thin vertical obstacle with at least this z span in metres')
    parser.add_argument(
        '--max-speckle-cells', type=int, default=30,
        help='remove a dilated obstacle island no larger than this cell count')
    parser.add_argument(
        '--min-nonground-support-points', type=int, default=3,
        help='preserve a small island with at least this many non-ground returns')
    parser.add_argument('--bound-percentile', type=float, default=0.1)
    parser.add_argument('--padding', type=float, default=1.0)
    return parser.parse_args()


if __name__ == '__main__':
    build(parse_args())

"""Coarse-to-fine 2D/3D registration of live LIO clouds to a saved map.

The robot remains stationary during registration.  We estimate only planar
SE(2), because roll/pitch and floor height come from LIO/IMU and the navigation
controller is planar.  A cheap 2D distance-field search supplies the global
basin; height-aware ICP then refines and validates the result.
"""

import math

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def voxel_downsample(points, size):
    if not len(points):
        return points
    keys = np.floor(points / float(size)).astype(np.int32)
    _, indices = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(indices)]


def consensus_cloud(clouds, floor_z, z_min, z_max, voxel_size=0.08):
    """Keep voxels observed in several independent stationary scan frames."""
    frames = []
    for raw in clouds:
        points = np.asarray(raw, dtype=np.float64)
        points = points[np.isfinite(points).all(axis=1)].copy()
        points[:, 2] -= floor_z
        points = points[(points[:, 2] >= z_min) & (points[:, 2] <= z_max)]
        if len(points):
            frames.append(voxel_downsample(points, voxel_size))
    if not frames:
        return np.empty((0, 3), dtype=np.float64)
    if len(frames) == 1:
        return frames[0]
    points = np.concatenate(frames)
    keys = np.floor(points / voxel_size).astype(np.int32)
    unique, inverse = np.unique(keys, axis=0, return_inverse=True)
    del unique
    counts = np.bincount(inverse)
    sums = np.zeros((len(counts), 3), dtype=np.float64)
    np.add.at(sums, inverse, points)
    required = max(2, int(math.ceil(len(frames) * 0.35)))
    keep = counts >= required
    return sums[keep] / counts[keep, None]


class AutoMapRegistration:
    """Reusable static-map index for one-shot live-cloud registration."""

    Z_MIN = 0.20
    Z_MAX = 1.80
    GRID_RESOLUTION = 0.10
    LIO_WORLD_Z_OFFSET = 0.53

    def __init__(self, static_points):
        target = np.asarray(static_points, dtype=np.float64)
        target = target[np.isfinite(target).all(axis=1)]
        target = target[(target[:, 2] >= self.Z_MIN) &
                        (target[:, 2] <= self.Z_MAX)]
        target = voxel_downsample(target, 0.08)
        if len(target) < 500:
            raise ValueError('static map has too few registration features')
        self.target = target
        self.target_tree = cKDTree(target)

        margin = 1.0
        minimum = target[:, :2].min(axis=0) - margin
        maximum = target[:, :2].max(axis=0) + margin
        size = np.ceil((maximum - minimum) / self.GRID_RESOLUTION).astype(int) + 1
        occupied = np.zeros((int(size[1]), int(size[0])), dtype=np.bool_)
        cells = np.floor(
            (target[:, :2] - minimum) / self.GRID_RESOLUTION).astype(np.int32)
        occupied[cells[:, 1], cells[:, 0]] = True
        self.grid_origin = minimum
        self.distance_field = ndimage.distance_transform_edt(
            ~occupied) * self.GRID_RESOLUTION

    def _distance_score(self, relative_points, map_x, map_y, map_yaw):
        c, s = math.cos(map_yaw), math.sin(map_yaw)
        x = c * relative_points[:, 0] - s * relative_points[:, 1] + map_x
        y = s * relative_points[:, 0] + c * relative_points[:, 1] + map_y
        cells = np.floor((np.column_stack((x, y)) - self.grid_origin) /
                         self.GRID_RESOLUTION).astype(np.int32)
        height, width = self.distance_field.shape
        valid = ((cells[:, 0] >= 0) & (cells[:, 0] < width) &
                 (cells[:, 1] >= 0) & (cells[:, 1] < height))
        distances = np.full(len(cells), 1.0, dtype=np.float64)
        distances[valid] = self.distance_field[cells[valid, 1], cells[valid, 0]]
        clipped_mean = float(np.mean(np.minimum(distances, 0.60)))
        inlier_ratio = float(np.mean(distances <= 0.25))
        return clipped_mean - 0.05 * inlier_ratio, inlier_ratio

    @staticmethod
    def _axis_values(center, radius, step):
        count = int(round(2.0 * radius / step))
        return center + np.linspace(-radius, radius, count + 1)

    def _grid_search(self, relative_points, rough_x, rough_y, rough_yaw):
        # An optional rough heading narrows the yaw basin.  With no heading we
        # deliberately search 360 degrees, so the operator only needs to click
        # approximately where the robot is.
        if rough_yaw is None:
            yaw_values = np.arange(-math.pi, math.pi, math.radians(10.0))
        else:
            yaw_values = self._axis_values(
                rough_yaw, math.radians(45.0), math.radians(5.0))
        x_values = self._axis_values(rough_x, 2.5, 0.5)
        y_values = self._axis_values(rough_y, 2.5, 0.5)
        candidates = []
        for yaw in yaw_values:
            yaw = wrap_angle(float(yaw))
            for x in x_values:
                for y in y_values:
                    score, inliers = self._distance_score(
                        relative_points, float(x), float(y), yaw)
                    candidates.append((score, -inliers, float(x), float(y), yaw))
        candidates.sort()

        # Refine several distinct coarse basins so a single grid quantization
        # choice cannot decide the final pose.
        seeds = []
        for candidate in candidates:
            if all(math.hypot(candidate[2] - seed[2], candidate[3] - seed[3]) > 0.6 or
                   abs(wrap_angle(candidate[4] - seed[4])) > math.radians(12.0)
                   for seed in seeds):
                seeds.append(candidate)
            if len(seeds) >= 5:
                break
        fine = []
        for seed in seeds:
            for yaw in self._axis_values(seed[4], math.radians(6.0), math.radians(2.0)):
                yaw = wrap_angle(float(yaw))
                for x in self._axis_values(seed[2], 0.35, 0.10):
                    for y in self._axis_values(seed[3], 0.35, 0.10):
                        score, inliers = self._distance_score(
                            relative_points, float(x), float(y), yaw)
                        fine.append((score, -inliers, float(x), float(y), yaw))
        fine.sort()
        return fine

    @staticmethod
    def _rigid_fit_2d(source, target):
        source_center = source.mean(axis=0)
        target_center = target.mean(axis=0)
        covariance = (source - source_center).T.dot(target - target_center)
        u, _, vt = np.linalg.svd(covariance)
        rotation = vt.T.dot(u.T)
        if np.linalg.det(rotation) < 0:
            vt[-1, :] *= -1
            rotation = vt.T.dot(u.T)
        translation = target_center - rotation.dot(source_center)
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        return rotation, translation, yaw

    def _icp(self, relative_points, pose):
        x, y, yaw = pose
        thresholds = (0.65, 0.50, 0.40, 0.32, 0.28, 0.25,
                      0.25, 0.25, 0.25, 0.25)
        for threshold in thresholds:
            c, s = math.cos(yaw), math.sin(yaw)
            mapped_xy = np.column_stack((
                c * relative_points[:, 0] - s * relative_points[:, 1] + x,
                s * relative_points[:, 0] + c * relative_points[:, 1] + y))
            mapped = np.column_stack((mapped_xy, relative_points[:, 2]))
            distances, indices = self.target_tree.query(
                mapped, k=1, distance_upper_bound=threshold)
            valid = np.isfinite(distances)
            if int(valid.sum()) < 120:
                break
            # Trim the worst tail to reduce people and transient-object impact.
            limit = min(threshold, float(np.percentile(distances[valid], 85.0)))
            valid &= distances <= limit
            if int(valid.sum()) < 100:
                break
            rotation, translation, delta_yaw = self._rigid_fit_2d(
                mapped_xy[valid], self.target[indices[valid], :2])
            updated = rotation.dot(np.array([x, y])) + translation
            x, y = float(updated[0]), float(updated[1])
            yaw = wrap_angle(yaw + delta_yaw)
            if math.hypot(float(translation[0]), float(translation[1])) < 0.002 and \
                    abs(delta_yaw) < math.radians(0.05):
                break
        return x, y, yaw

    def _quality(self, relative_points, pose):
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        mapped = np.column_stack((
            c * relative_points[:, 0] - s * relative_points[:, 1] + x,
            s * relative_points[:, 0] + c * relative_points[:, 1] + y,
            relative_points[:, 2]))
        distances, _ = self.target_tree.query(mapped, k=1)
        inlier_20 = distances <= 0.20
        inlier_30 = distances <= 0.30
        trimmed = distances[inlier_30]
        rmse = float(math.sqrt(np.mean(np.square(trimmed)))) if len(trimmed) else math.inf
        return {
            'inlier_ratio_20cm': float(np.mean(inlier_20)),
            'inlier_ratio_30cm': float(np.mean(inlier_30)),
            'rmse_m': rmse,
            'median_m': float(np.median(distances)),
            'matched_points': int(inlier_30.sum()),
            'source_points': int(len(relative_points)),
        }

    def register(self, clouds, odom_pose, rough_x, rough_y, rough_yaw=None):
        if not clouds:
            return False, '未采集到实时配准点云', None
        # cloud_registered retains LIO's raw map z, whereas body_pose includes
        # the adapter's +0.53 m world offset.  Recover the live floor height and
        # level it exactly like the saved static map.
        # The adapter's body origin is roughly base_to_lidar_z (0.0908 m)
        # below the lidar pose; after the configured +0.53 m world offset a
        # level floor therefore appears at body_z - (0.53 - 0.0908).
        live_floor_z = float(odom_pose['z']) - (self.LIO_WORLD_Z_OFFSET - 0.0908)
        source = consensus_cloud(
            clouds, live_floor_z, self.Z_MIN, self.Z_MAX)
        if len(source) < 350:
            return False, '多帧静态一致点过少，无法可靠配准', None

        def relative_to_body(points):
            relative_points = points.copy()
            dx = points[:, 0] - float(odom_pose['x'])
            dy = points[:, 1] - float(odom_pose['y'])
            # Express the cloud in the live body-heading basis.  The search yaw
            # is consequently the robot's yaw in map coordinates.
            odom_yaw = float(odom_pose['yaw'])
            c, s = math.cos(odom_yaw), math.sin(odom_yaw)
            relative_points[:, 0] = c * dx + s * dy
            relative_points[:, 1] = -s * dx + c * dy
            return relative_points

        relative = relative_to_body(source)
        search_points = relative[::max(1, len(relative) // 1200)]
        fine = self._grid_search(
            search_points, float(rough_x), float(rough_y), rough_yaw)
        if not fine:
            return False, '自动配准未找到候选位姿', None
        best_grid = fine[0]
        pose = self._icp(relative, (best_grid[2], best_grid[3], best_grid[4]))

        # Cross-check the solution on two independent temporal subsets.  This
        # detects moving people, sparse one-sided views and corridor-axis drift
        # even when the aggregate ICP residual happens to look good.
        subset_poses = []
        subset_qualities = []
        for subset in (clouds[::2], clouds[1::2]):
            subset_source = consensus_cloud(
                subset, live_floor_z, self.Z_MIN, self.Z_MAX)
            if len(subset_source) < 220:
                continue
            subset_relative = relative_to_body(subset_source)
            subset_pose = self._icp(subset_relative, pose)
            subset_poses.append(subset_pose)
            subset_qualities.append(self._quality(subset_relative, subset_pose))
        if len(subset_poses) == 2:
            consistency_position = math.hypot(
                subset_poses[0][0] - subset_poses[1][0],
                subset_poses[0][1] - subset_poses[1][1])
            consistency_yaw = abs(math.degrees(wrap_angle(
                subset_poses[0][2] - subset_poses[1][2])))
            poses = [pose] + subset_poses
            pose = (
                float(np.median([value[0] for value in poses])),
                float(np.median([value[1] for value in poses])),
                math.atan2(np.mean([math.sin(value[2]) for value in poses]),
                           np.mean([math.cos(value[2]) for value in poses])),
            )
            pose = self._icp(relative, pose)
        else:
            consistency_position = math.inf
            consistency_yaw = math.inf
        quality = self._quality(relative, pose)

        # Alternative basin score, excluding ordinary neighboring samples of
        # the same solution.  A small margin means the building geometry is
        # locally repetitive and cannot uniquely establish the pose.
        alternative = next((candidate for candidate in fine[1:]
                            if math.hypot(candidate[2] - pose[0], candidate[3] - pose[1]) > 0.8 or
                            abs(wrap_angle(candidate[4] - pose[2])) > math.radians(15.0)), None)
        margin = (float(alternative[0] - best_grid[0])
                  if alternative is not None else 1.0)
        correction = math.hypot(pose[0] - rough_x, pose[1] - rough_y)
        yaw_correction = (None if rough_yaw is None else
                          abs(math.degrees(wrap_angle(pose[2] - rough_yaw))))
        quality.update({
            'ambiguity_margin': margin,
            'rough_position_correction_m': correction,
            'rough_yaw_correction_deg': yaw_correction,
            'captured_frames': len(clouds),
            'temporal_consistency_m': consistency_position,
            'temporal_consistency_deg': consistency_yaw,
            'temporal_subsets': len(subset_poses),
        })

        failures = []
        if quality['matched_points'] < 220:
            failures.append('有效匹配点过少')
        if quality['inlier_ratio_30cm'] < 0.42:
            failures.append('30 cm 重合率过低')
        if quality['inlier_ratio_20cm'] < 0.28:
            failures.append('20 cm 重合率过低')
        if not math.isfinite(quality['rmse_m']) or quality['rmse_m'] > 0.19:
            failures.append('配准残差过大')
        if correction > 2.8:
            failures.append('粗选位置偏差超过 2.8 m')
        if rough_yaw is not None and yaw_correction > 50.0:
            failures.append('粗选朝向偏差过大')
        if margin < 0.010:
            failures.append('存在相似的重复结构，解不唯一')
        if len(subset_poses) != 2:
            failures.append('独立时间子集特征不足')
        elif consistency_position > 0.18 or consistency_yaw > 3.0:
            failures.append('独立时间子集解不一致')

        result = {
            'x': float(pose[0]),
            'y': float(pose[1]),
            'yaw': float(pose[2]),
            'yaw_deg': math.degrees(pose[2]),
            'quality': quality,
        }
        if failures:
            result['failures'] = failures
            return False, '自动配准未通过质量检查：' + '；'.join(failures), result
        return True, '自动精配准通过质量检查', result

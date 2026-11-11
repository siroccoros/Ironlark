"""Local scan matching with an IMU prior; consumes sensors, never simulator truth.

The maze walls constrain x, y and yaw. A tracked wall top edge constrains height. This is intentionally
conservative: a corridor with an unobservable along-wall translation reports unhealthy instead
of manufacturing a precise position. Webots reports nominal per-return times, but the pinned
renderer gives one geometric snapshot at the latest return time, as verified by raycasting moving
scans against the evaluator's maze. The estimate uses that measured snapshot time.
"""
import math
from collections import deque

import numpy as np
from scipy.spatial import cKDTree


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def planar_points(cloud, offset=(0.0, 0.0, 0.083)):
    """Wall returns in the lidar frame, decimated into 12 cm cells."""
    points = np.asarray(cloud, dtype=np.float64)
    good = np.isfinite(points).all(axis=1)
    points = points[good] + np.asarray(offset)
    radius = np.linalg.norm(points[:, :2], axis=1)
    points = points[(points[:, 2] > -0.25) & (points[:, 2] < 1.15) &
                    (radius > 0.45) & (radius < 16.0), :2]
    if len(points) == 0:
        return points
    cells = np.floor(points / 0.12).astype(np.int32)
    _, indices = np.unique(cells, axis=0, return_index=True)
    return points[np.sort(indices)]


def wall_top_feature(cloud, offset=(0.0, 0.0, 0.083)):
    """High return boundary of nearby walls, measured relative to the lidar.

    In the generated maze the same top edge remains visible as the vehicle changes height.
    The value is learned from the first scan, not read from the world's geometry.
    """
    points = np.asarray(cloud, dtype=np.float64)
    points = points[np.isfinite(points).all(axis=1)] + np.asarray(offset)
    radius = np.linalg.norm(points[:, :2], axis=1)
    z = points[(radius > 0.5) & (radius < 4.0) & (points[:, 2] > 0), 2]
    return float(np.quantile(z, 0.999)) if len(z) >= 500 else None


def normals(points):
    tree = cKDTree(points)
    _, neighbors = tree.query(points, k=min(8, len(points)))
    near = points[neighbors]
    centered = near - near.mean(axis=1, keepdims=True)
    covariance = np.einsum("nki,nkj->nij", centered, centered)
    eigenvalues, vectors = np.linalg.eigh(covariance)
    valid = (eigenvalues[:, 1] > 0.02) & (eigenvalues[:, 0] < 0.2 * eigenvalues[:, 1])
    return tree, vectors[:, :, 0], valid


class ScanOdometry:
    def __init__(self, lidar_offset=(0.0, 0.0, 0.083), use_acquisition_time=True):
        self.offset = lidar_offset
        self.use_acquisition_time = use_acquisition_time
        self.pose = np.zeros(3)  # x, y, yaw in an arbitrary local ENU frame
        self.velocity = np.zeros(3)
        self.height = 0.0
        self.vertical_velocity = 0.0
        self.vertical_variance = 0.01
        self.wall_top_origin = None
        self.last_wall_height = None
        self.accel_baseline = 9.81
        self.rest_samples = 0
        self.airborne = False
        self.motion_samples = 0
        self.roll = self.pitch = 0.0
        self.last_gyro_time = None
        self.imu_history = deque(maxlen=128)
        self.scan_time = None
        self.scan_duration = 0.0
        self.acquisition_delay = 0.0
        self.timing_valid = False
        self.previous = None
        self.reference_pose = np.zeros(3)
        self.covariance = np.diag([1.0, 1.0, 1.0])
        self.health = False
        self.reason = "waiting for scans"
        self.correspondences = 0

    def imu(self, time_s, gyro_z, gyro_x=0.0, gyro_y=0.0, accel=(0.0, 0.0, 9.81)):
        if not all(math.isfinite(v) for v in (gyro_x, gyro_y, gyro_z, *accel)):
            return
        if self.last_gyro_time is not None:
            dt = time_s - self.last_gyro_time
            if 0 < dt <= 0.05:
                self.pose[2] = wrap(self.pose[2] + gyro_z * dt)
                self.roll += gyro_x * dt
                self.pitch += gyro_y * dt
                vertical_specific_force = (-math.sin(self.pitch) * accel[0]
                                           + math.sin(self.roll) * math.cos(self.pitch) * accel[1]
                                           + math.cos(self.roll) * math.cos(self.pitch) * accel[2])
                if not self.airborne:
                    if abs(vertical_specific_force - self.accel_baseline) > 0.2:
                        self.motion_samples += 1
                    else:
                        self.motion_samples = 0
                        self.accel_baseline = ((self.accel_baseline * self.rest_samples + vertical_specific_force)
                                               / (self.rest_samples + 1))
                        self.rest_samples += 1
                    if self.motion_samples >= 3:
                        self.airborne = True
                if self.airborne:
                    acceleration = vertical_specific_force - self.accel_baseline
                    self.height += self.vertical_velocity * dt + 0.5 * acceleration * dt * dt
                    self.vertical_velocity += acceleration * dt
                    self.velocity[2] = self.vertical_velocity
                    self.vertical_variance += (0.03 * dt * dt) ** 2 + (0.002 * dt) ** 2
        self.last_gyro_time = time_s
        self.imu_history.append((time_s, self.pose[2], self.roll, self.pitch))

    def prepare_scan(self, stamp, cloud):
        """Use the latest measured return time for Webots' one spatial snapshot.

        Its nominal point times span ~100 ms, ending ~98 ms before publication, but moving
        scans raycast accurately at the last return time. The first scan's zero times are
        unusable. No per-point motion is invented.
        """
        cloud = np.asarray(cloud, dtype=np.float64)
        self.scan_duration = self.acquisition_delay = 0.0
        self.timing_valid = False
        if cloud.shape[1] != 4 or not self.use_acquisition_time:
            return stamp, cloud[:, :3]
        times = cloud[:, 3]
        valid = np.isfinite(times) & (times >= stamp - 0.25) & (times <= stamp + 0.005)
        if valid.sum() < 0.9 * len(times):
            return stamp, cloud[:, :3]
        first, last = float(np.min(times[valid])), float(np.max(times[valid]))
        if not 0.05 <= last - first <= 0.15:
            return stamp, cloud[:, :3]
        self.scan_duration = last - first
        self.acquisition_delay = stamp - last
        self.timing_valid = True
        return last, cloud[:, :3]

    def scan(self, time_s, cloud):
        time_s, cloud = self.prepare_scan(time_s, cloud)
        # IMU messages are processed through publication time. Match at the measured
        # snapshot time, then carry the estimate forward with the intervening gyro turn.
        yaw_forward = roll_forward = pitch_forward = 0.0
        if self.timing_valid and self.imu_history:
            acquired = min(self.imu_history, key=lambda sample: abs(sample[0] - time_s))
            latest = self.imu_history[-1]
            yaw_forward = wrap(latest[1] - acquired[1])
            roll_forward = latest[2] - acquired[2]
            pitch_forward = latest[3] - acquired[3]
            self.pose[2] = wrap(self.pose[2] - yaw_forward)
            self.roll -= roll_forward
            self.pitch -= pitch_forward

        def finish():
            state = self.state(time_s)
            self.pose[2] = wrap(self.pose[2] + yaw_forward)
            self.roll += roll_forward
            self.pitch += pitch_forward
            return state

        # The scan matcher uses a yaw-only local map. Undo measured body roll/pitch so a
        # pitched vehicle does not turn a wall top into a false altitude change.
        if abs(self.roll) + abs(self.pitch) > 1e-5:
            cloud = cloud.copy()
            finite = np.isfinite(cloud).all(axis=1)
            x, y, z = cloud[finite, 0].copy(), cloud[finite, 1].copy(), cloud[finite, 2].copy()
            cr, sr = math.cos(self.roll), math.sin(self.roll)
            cp, sp = math.cos(self.pitch), math.sin(self.pitch)
            y1, z1 = cr * y - sr * z, sr * y + cr * z
            cloud[finite, 0] = cp * x + sp * z1
            cloud[finite, 1] = y1
            cloud[finite, 2] = -sp * x + cp * z1
        current = planar_points(cloud, self.offset)
        top = wall_top_feature(cloud, self.offset)
        if top is not None:
            if self.wall_top_origin is None:
                self.wall_top_origin = top
            measured_height = self.wall_top_origin - top
            if self.last_wall_height is not None and self.scan_time is not None and time_s > self.scan_time:
                measured_velocity = (measured_height - self.last_wall_height) / (time_s - self.scan_time)
                self.vertical_velocity = 0.6 * self.vertical_velocity + 0.4 * measured_velocity
                self.velocity[2] = self.vertical_velocity
            self.height = measured_height
            self.vertical_variance = 0.02 ** 2
            self.last_wall_height = measured_height
        if self.previous is None:
            self.previous = current
            self.reference_pose = self.pose.copy()
            self.scan_time = time_s
            self.reason = "first scan"
            return finish()
        dt = time_s - self.scan_time
        if dt <= 0 or dt > 0.3 or len(current) < 80 or len(self.previous) < 80:
            self.health, self.reason = False, "missing scan or too few wall returns"
            self.previous, self.scan_time = current, time_s
            self.reference_pose = self.pose.copy()
            return finish()

        # Register current points to the previous scan in its local frame. The gyro gives the
        # yaw change; the line fit corrects it. No IMU orientation or truth pose is consumed.
        reference = self.previous
        tree, normal, valid = normals(reference)
        relative_xy = self.pose[:2] - self.reference_pose[:2]
        c0, s0 = math.cos(self.reference_pose[2]), math.sin(self.reference_pose[2])
        relative_xy = np.array([[c0, s0], [-s0, c0]]) @ relative_xy
        change = np.array([*relative_xy, wrap(self.pose[2] - self.reference_pose[2])])
        matches = 0
        fit_hessian = fit_residual = None
        for _ in range(6):
            c, s = math.cos(change[2]), math.sin(change[2])
            rotated = current @ np.array([[c, s], [-s, c]])
            placed = rotated + change[:2]
            distance, index = tree.query(placed)
            keep = (distance < 0.7) & valid[index]
            if keep.sum() < 50:
                break
            p, q, n = placed[keep], reference[index[keep]], normal[index[keep]]
            residual = np.sum(n * (p - q), axis=1)
            keep2 = np.abs(residual) < 0.25
            p, n, residual = p[keep2], n[keep2], residual[keep2]
            if len(p) < 50:
                break
            matches = len(p)
            lever = p - change[:2]
            jacobian = np.column_stack((n[:, 0], n[:, 1], -n[:, 0] * lever[:, 1] + n[:, 1] * lever[:, 0]))
            hessian = jacobian.T @ jacobian
            fit_hessian, fit_residual = hessian, residual
            step = np.linalg.lstsq(hessian + np.diag([0.01, 0.01, 0.1]), -jacobian.T @ residual, rcond=None)[0]
            change += np.clip(step, [-0.2, -0.2, -0.06], [0.2, 0.2, 0.06])
            if np.linalg.norm(step) < 1e-4:
                break
        self.correspondences = matches
        if self.correspondences >= 50:
            # Smallest translation eigenvalue exposes a straight corridor's free direction.
            eig = np.linalg.eigvalsh(fit_hessian[:2, :2])
            rms = float(np.sqrt(np.mean(fit_residual ** 2)))
            observable = eig[0] >= 12 and eig[0] / eig[1] >= 0.025
            self.health = bool(observable and rms < 0.12 and np.linalg.norm(change[:2]) < 1.2 and top is not None)
            self.reason = ("ok" if self.health else "wall top unavailable" if top is None
                           else "corridor degeneracy" if not observable else "scan mismatch")
            sigma2 = max(rms ** 2, 0.01 ** 2)
            self.covariance = sigma2 * np.linalg.pinv(fit_hessian + np.eye(3) * 1e-6)
            if not observable:
                self.covariance[:2, :2] += np.eye(2) * 4.0
            old_xy = self.pose[:2].copy()
            angle = self.reference_pose[2]
            self.pose[:2] = self.reference_pose[:2] + np.array([[math.cos(angle), -math.sin(angle)],
                                       [math.sin(angle), math.cos(angle)]]) @ change[:2]
            self.pose[2] = wrap(self.reference_pose[2] + change[2])
            self.velocity[:2] = (self.pose[:2] - old_xy) / dt
        else:
            self.health, self.reason = False, "too few scan correspondences"
            self.covariance += np.eye(3) * dt
        if np.linalg.norm(self.pose[:2] - self.reference_pose[:2]) > 0.7 or abs(wrap(self.pose[2] - self.reference_pose[2])) > 0.25:
            self.previous = current
            self.reference_pose = self.pose.copy()
        self.scan_time = time_s
        return finish()

    def state(self, time_s):
        return {"time": time_s, "position": [float(self.pose[0]), float(self.pose[1]), self.height],
                "yaw": float(self.pose[2]), "velocity": self.velocity.tolist(),
                "covariance_z": self.vertical_variance,
                "covariance_xy_yaw": self.covariance.tolist(), "healthy": self.health,
                "reason": self.reason, "correspondences": self.correspondences,
                "scan_duration_s": self.scan_duration, "acquisition_delay_s": self.acquisition_delay,
                "timing_valid": self.timing_valid}

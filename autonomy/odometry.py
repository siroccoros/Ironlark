"""IMU-aided 6-DoF LiDAR odometry: gyro propagation + hybrid point-to-plane/point-to-point ICP.

Consumes strictly noisy onboard gyro, accelerometer, and LiDAR point clouds in base_link
(never ground truth and never Webots' noiseless InertialUnit orientation). This is a compact
KISS-ICP-style scan matcher with a gyro rotation prior and stationary bias/tilt calibration,
not a tightly-coupled preintegrated LIO filter.
"""
from collections import deque
import math
import threading
import numpy as np
from scipy.spatial import cKDTree


def rodrigues(rotvec):
    """SO(3) exponential map: 3-vector -> 3x3 rotation matrix."""
    angle = float(np.linalg.norm(rotvec))
    if angle < 1e-10:
        return np.eye(3)
    k = rotvec / angle
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


def matrix_to_quaternion(R):
    """3x3 rotation matrix -> unit quaternion (x, y, z, w)."""
    tr = float(R[0, 0] + R[1, 1] + R[2, 2])
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        q = ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        q = (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s)
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        q = ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s)
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        q = ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s)
    n = math.sqrt(sum(c * c for c in q)) or 1.0
    return tuple(c / n for c in q)


def voxel_downsample(points, voxel_size):
    """Fast vectorized voxel-grid downsampling returning one point per occupied voxel."""
    if len(points) == 0:
        return points
    grid = np.floor(points / voxel_size).astype(np.int64)
    # Shift to non-negative and pack (gx, gy, gz) into a single int64 key.
    shifted = grid - grid.min(axis=0)
    dims = shifted.max(axis=0) + 1
    keys = shifted[:, 0] + dims[0] * (shifted[:, 1] + dims[1] * shifted[:, 2])
    _, first = np.unique(keys, return_index=True)
    return points[first]


class LidarInertialOdometry:
    """Rolling-submap 6-DoF point-to-plane + point-to-point ICP fused with IMU propagation."""

    def __init__(self, initial_position=(0.0, 0.0, 0.09), scan_voxel=0.45, map_voxel=0.50,
                 max_range=38.0, min_range=0.45, submap_radius=34.0, max_scan_pts=900):
        self.p = np.array(initial_position, dtype=np.float64)
        self.R = np.eye(3, dtype=np.float64)
        self.v = np.zeros(3, dtype=np.float64)
        self.scan_voxel = scan_voxel
        self.map_voxel = map_voxel
        self.min_range = min_range
        self.max_range = max_range
        self.submap_radius = submap_radius
        self.max_scan_pts = max_scan_pts

        self._last_scan_t = None
        self._imu_lock = threading.Lock()
        self._imu_buf = deque(maxlen=2000)  # (stamp_s, gyro, accel) for exact [t_prev, t_cur] integration
        self._gyro_bias = np.zeros(3, dtype=np.float64)
        self._bias_sum = np.zeros(3, dtype=np.float64)
        self._bias_count = 0
        self._submap = {}  # (ix, iy, iz) -> [sum_xyz, count]
        self._map_pts = None
        self._map_normals = None
        self._map_planar = None
        self._map_tree = None
        self._map_dirty = False
        self._last_kf_p = self.p.copy()
        self._last_kf_R = self.R.copy()

        self.scan_count = 0
        self.healthy = True
        self.last_rmse = 0.0
        self.last_inliers = 0
        self.last_min_eig = 0.0

    def on_imu(self, stamp_s, gyro, accel):
        """Buffer raw noisy 3-axis gyro/accel for timestamp-synchronized propagation at scan time.

        Never reads Webots' noiseless InertialUnit orientation quaternion. Estimates static gyro
        bias while stationary on the pad and lets 6-DoF point-to-plane ICP correct in-flight drift.
        """
        gx, gy, gz = float(gyro[0]), float(gyro[1]), float(gyro[2])
        ax, ay, az = float(accel[0]), float(accel[1]), float(accel[2])
        omega_raw = np.array((gx, gy, gz), dtype=np.float64)
        a = np.array((ax, ay, az), dtype=np.float64)
        with self._imu_lock:
            self._imu_buf.append((float(stamp_s), omega_raw, a))
            if self._bias_count < 2500:
                v = self.v
                v2 = float(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
                g2 = gx * gx + gy * gy + gz * gz
                a_norm = math.sqrt(ax * ax + ay * ay + az * az)
                if v2 < 0.0225 and g2 < 0.0025 and abs(a_norm - 9.81) < 0.35:
                    self._bias_sum += omega_raw
                    self._bias_count += 1
                    if self._bias_count >= 250:
                        self._gyro_bias = self._bias_sum / self._bias_count

    def _propagate_rotation(self, t0, t1):
        """Integrate buffered gyro strictly across (t0, t1]; align tilt to gravity only when stationary."""
        with self._imu_lock:
            samples = [s for s in self._imu_buf if t0 < s[0] <= t1 + 1e-4]
            bias = self._gyro_bias.copy()
            while self._imu_buf and self._imu_buf[0][0] < t1 - 2.0:
                self._imu_buf.popleft()
        R = self.R.copy()
        prev_t = t0
        for t, omega_raw, a in samples:
            dt = t - prev_t
            prev_t = t
            if not (0.0 < dt < 0.1):
                continue
            R = R @ rodrigues((omega_raw - bias) * dt)
            a_norm = float(np.linalg.norm(a))
            if (np.linalg.norm(self.v) < 0.15 and np.linalg.norm(omega_raw) < 0.05
                    and abs(a_norm - 9.81) < 0.35):
                up_est = R @ (a / a_norm)
                err = np.cross(up_est, np.array([0.0, 0.0, 1.0]))
                err[2] = 0.0
                R = rodrigues(0.05 * err) @ R
        return R

    def predict(self, t_from, t_to):
        """The last scan's pose (stamped t_from) carried forward to t_to on its velocity and the gyro."""
        return self.p + self.v * (t_to - t_from), self._propagate_rotation(t_from, t_to)

    def _insert_submap(self, world_pts):
        keys = np.floor(world_pts / self.map_voxel).astype(np.int64)
        for key, pt in zip(map(tuple, keys), world_pts):
            entry = self._submap.get(key)
            if entry is None:
                self._submap[key] = [pt.copy(), 1]
            elif entry[1] < 16:
                entry[0] += pt
                entry[1] += 1
        # Prune voxels beyond submap_radius so the rolling submap stays bounded on long tours
        # without dropping mid-range walls/trees that keep inlier_ratio high.
        if len(self._submap) > 2800:
            r2 = self.submap_radius ** 2
            cx, cy, cz = self.p
            self._submap = {
                k: v for k, v in self._submap.items()
                if ((v[0][0] / v[1] - cx) ** 2 + (v[0][1] / v[1] - cy) ** 2 + (v[0][2] / v[1] - cz) ** 2) <= r2
            }
        self._map_dirty = True

    def _rebuild_tree(self):
        if not self._map_dirty and self._map_tree is not None:
            return
        pts = np.array([v[0] / v[1] for v in self._submap.values()], dtype=np.float64)
        tree = cKDTree(pts, compact_nodes=False, balanced_tree=False)
        k = min(6, len(pts))
        if k >= 4:
            _, nbrs = tree.query(pts, k=k, workers=2)
            local = pts[nbrs]  # (M, k, 3)
            centered = local - local.mean(axis=1, keepdims=True)
            cov = np.einsum("mki,mkj->mij", centered, centered) / k
            eigvals, eigvecs = np.linalg.eigh(cov)  # ascending eigenvalues
            normals = eigvecs[:, :, 0]
            # Planarity: smallest eigenvalue much smaller than middle eigenvalue.
            planar = (eigvals[:, 1] > 1e-4) & (eigvals[:, 0] / (eigvals[:, 1] + 1e-9) < 0.25)
        else:
            normals = np.zeros_like(pts)
            planar = np.zeros(len(pts), dtype=bool)
        self._map_pts = pts
        self._map_normals = normals
        self._map_planar = planar
        self._map_tree = tree
        self._map_dirty = False

    def on_lidar(self, stamp_s, points_body):
        """Register a body-frame LiDAR scan against the rolling submap; return (p, R, v, healthy)."""
        pts = np.asarray(points_body, dtype=np.float64)
        if pts.ndim != 2 or pts.shape[1] < 3 or len(pts) < 20:
            self.healthy = False
            return self.p.copy(), self.R.copy(), self.v.copy(), False
        pts = pts[:, :3]
        finite = np.all(np.isfinite(pts), axis=1)
        pts = pts[finite]
        r = np.linalg.norm(pts, axis=1)
        pts = pts[(r >= self.min_range) & (r <= self.max_range)]
        if len(pts) < 30:
            self.healthy = False
            return self.p.copy(), self.R.copy(), self.v.copy(), False

        R_prior = self.R.copy() if self._last_scan_t is None else self._propagate_rotation(self._last_scan_t, stamp_s)
        scan = voxel_downsample(pts, self.scan_voxel)
        if len(scan) > self.max_scan_pts:
            # Preserve vertical structure (trees, rocks, cave walls) so flat ground never crowds out XY/yaw.
            z_approx = (scan @ R_prior.T)[:, 2] + self.p[2]
            above = scan[z_approx > 0.3]
            ground = scan[z_approx <= 0.3]
            if len(above) > 600:
                above = above[::max(1, len(above) // 600)][:600]
            g_budget = max(150, self.max_scan_pts - len(above))
            if len(ground) > g_budget:
                ground = ground[::max(1, len(ground) // g_budget)][:g_budget]
            scan = np.vstack([above, ground]) if len(above) and len(ground) else (above if len(above) else ground)
        if self.scan_count == 0:
            self.R = R_prior
            self._insert_submap(scan @ self.R.T + self.p)
            self._last_scan_t = stamp_s
            self.scan_count = 1
            self.healthy = True
            return self.p.copy(), self.R.copy(), self.v.copy(), True

        raw_dt = stamp_s - self._last_scan_t
        dt = max(1e-3, min(0.5, raw_dt))
        self._last_scan_t = stamp_s

        # Predict pose from constant velocity + timestamp-synchronized gyro propagation.
        p_pred = self.p + self.v * dt
        R_est = R_prior.copy()
        p_est = p_pred.copy()

        self._rebuild_tree()
        max_corr = 1.8 if raw_dt > 0.25 else 1.2
        huber_delta = 0.25
        inliers = 0
        rmse = 0.0
        min_eig = 0.0
        xy_min_eig = 0.0

        for it in range(8):
            world = scan @ R_est.T + p_est
            limit = max_corr if it < 3 else 0.55
            dists, idx = self._map_tree.query(world, distance_upper_bound=limit)
            valid = idx < len(self._map_pts)
            inliers = int(valid.sum())
            if inliers < 25:
                break
            P = world[valid]
            M = self._map_pts[idx[valid]]
            N = self._map_normals[idx[valid]]
            is_planar = self._map_planar[idx[valid]]
            c = P - p_est  # centered points for well-conditioned 6-DoF Hessian
            diff = P - M

            H = np.zeros((6, 6), dtype=np.float64)
            g = np.zeros(6, dtype=np.float64)

            # Point-to-plane terms on planar surfaces (floor, walls, ceiling).
            if np.any(is_planar):
                Np, cp, dp = N[is_planar], c[is_planar], diff[is_planar]
                res_p = np.einsum("ij,ij->i", Np, dp)
                w_p = np.minimum(1.0, huber_delta / (np.abs(res_p) + 1e-6))
                J_p = np.hstack([np.cross(cp, Np), Np])  # (Kp, 6)
                Jw = J_p * w_p[:, None]
                H += Jw.T @ J_p
                g += Jw.T @ res_p

            # Point-to-point terms on non-planar features (tree trunks, corners, boulders).
            non_planar = ~is_planar
            if np.any(non_planar):
                cn, dn = c[non_planar], diff[non_planar]
                norms = np.linalg.norm(dn, axis=1)
                w_n = 0.35 * np.minimum(1.0, huber_delta / (norms + 1e-6))
                # Vectorized J^T W J for J_i = [-[c_i]_x, I]:
                # [c]_x^T [c]_x = ||c||^2 I - c c^T
                w_sum = float(w_n.sum())
                c_w = cn * w_n[:, None]
                c_norm2 = np.sum(cn * cn, axis=1)
                H_rr = np.eye(3) * float(np.dot(w_n, c_norm2)) - c_w.T @ cn
                cw_sum = c_w.sum(axis=0)
                # J_r = -[c]_x => J_r^T = +[c]_x, so H_rt = sum w_i [c_i]_x and g_r = sum w_i (c_i x d_i).
                H_rt = np.array([[0.0, -cw_sum[2], cw_sum[1]],
                                 [cw_sum[2], 0.0, -cw_sum[0]],
                                 [-cw_sum[1], cw_sum[0], 0.0]])
                H[:3, :3] += H_rr
                H[:3, 3:] += H_rt
                H[3:, :3] += H_rt.T
                H[3:, 3:] += np.eye(3) * w_sum
                # g_r = sum w_i * (c_i x d_i), g_t = sum w_i * d_i
                g[:3] += np.cross(cn, dn).T @ w_n
                g[3:] += dn.T @ w_n

            # Adaptive LM damping: strongly regularize Z when few horizontal surfaces are visible
            # (Mid-360's -7 deg lower FOV can lose the ground plane above ~4 m outdoors).
            z_reg = 1.0 if H[5, 5] > 15.0 else 40.0
            H_reg = H + np.diag([0.8, 0.8, 0.35, 0.2, 0.2, z_reg])
            dx = -np.linalg.solve(H_reg, g)
            dx[5] = float(np.clip(dx[5], -0.4, 0.4))
            dR = rodrigues(dx[:3])
            R_est = dR @ R_est
            p_est = p_est + dx[3:]
            rmse = float(np.sqrt(np.mean(np.sum(diff * diff, axis=1))))
            min_eig = float(np.linalg.eigvalsh(H[3:, 3:])[0])
            xy_min_eig = float(np.linalg.eigvalsh(H[3:5, 3:5])[0])
            if np.linalg.norm(dx[:3]) < 1e-4 and np.linalg.norm(dx[3:]) < 1e-3:
                break

        step_err = float(np.linalg.norm(p_est - p_pred))
        dz_err = abs(float(p_est[2] - p_pred[2]))
        inlier_ratio = inliers / max(1, len(scan))
        # Guard against flat-ground degenerate locks (xy_min_eig) and long scan blackouts (raw_dt).
        ok = (raw_dt <= 0.85 and inliers >= 25 and inlier_ratio >= 0.30 and rmse <= 0.65
              and step_err <= 1.6 and dz_err <= max(0.6, 4.5 * dt)
              and min_eig >= 1.0 and xy_min_eig >= 2.5)
        self.last_inliers = inliers
        self.last_rmse = rmse
        self.last_min_eig = min_eig
        self.healthy = ok

        if ok:
            v_meas = (p_est - self.p) / dt
            v_meas[2] = float(np.clip(v_meas[2], -1.8, 1.8))
            self.v = 0.45 * self.v + 0.55 * v_meas
            self.p = p_est
            self.R = R_est
            # Keyframe insertion: keeps the submap crisp without bloating it while hovering.
            moved = float(np.linalg.norm(self.p - self._last_kf_p))
            rot_cos = float(np.clip((np.trace(self.R.T @ self._last_kf_R) - 1.0) * 0.5, -1.0, 1.0))
            turned = math.degrees(math.acos(rot_cos))
            if moved > 0.65 or turned > 8.0 or (self.scan_count < 20 and self.scan_count % 5 == 0):
                self._insert_submap(scan @ self.R.T + self.p)
                self._last_kf_p = self.p.copy()
                self._last_kf_R = self.R.copy()
        else:
            # Degraded scan: coast conservatively on prediction and report unhealthy.
            self.p = p_pred
            self.R = R_prior.copy()
            self.v *= 0.8

        self.scan_count += 1
        return self.p.copy(), self.R.copy(), self.v.copy(), self.healthy

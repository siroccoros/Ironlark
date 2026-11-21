"""Smooth time-parameterized trajectories over the planner's A* paths.

A trajectory is a C2 cubic B-spline through the A* polyline, corners rounded and the rounding shrunk
wherever the curve would enter an inflated obstacle, timed by a jerk-limited speed profile along its
arc length. Speed is bounded by the cruise cap, the curvature (lateral acceleration), the clearance
to mapped obstacles and the distance to unmapped space; the profile starts from the state of the
trajectory it replaces and ends at rest. Yaw follows the tangent, rate- and acceleration-limited.
sample(t) gives position, velocity, acceleration, yaw and yaw rate for GUIDED setpoints.
"""
import math
import time
from dataclasses import dataclass

import numpy as np
from scipy.ndimage import distance_transform_edt

DT = 0.02  # s: profile step, also the setpoint period
DS = 0.05  # m: geometry resolution along the arc


@dataclass
class Limits:
    v_max: float = 2.1  # m/s cruise cap
    a_acc: float = 1.2  # m/s^2 tangential, speeding up
    a_brake: float = 1.6  # m/s^2 tangential, slowing down
    a_lat: float = 1.0  # m/s^2 from curvature
    jerk: float = 2.8  # m/s^3 total: 0.8 of it along the path, 0.3 to each of the two lateral terms
    yaw_rate: float = math.radians(22.0)  # rad/s
    yaw_acc: float = math.radians(55.0)  # rad/s^2
    v_unknown: float = 1.10  # m/s where the path enters unmapped space
    clear_v0: float = 1.25  # m/s at the inflation boundary ...
    clear_gain: float = 0.70  # ... plus this per metre of further clearance
    corner_r: float = 2.5  # m: largest corner rounding


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def sqrt_controller(err, p, limit):
    """ArduPilot's square-root controller: linear near zero, constant-deceleration far away."""
    if limit <= 0.0 or p <= 0.0:
        return p * err
    lin = limit / (p * p)
    if abs(err) <= lin:
        return p * err
    return math.copysign(math.sqrt(2.0 * limit * (abs(err) - 0.5 * lin)), err)


# --- geometry ------------------------------------------------------------------------------------

def control_points(poly, radii):
    """B-spline control points: each interior vertex becomes (V - r u_in, V, V + r u_out), or a tripled
    vertex when its radius is 0 (the curve then follows the polyline exactly and stops at the corner)."""
    ctrl = [poly[0]]
    for i in range(1, len(poly) - 1):
        a, v, b = poly[i - 1], poly[i], poly[i + 1]
        u_in, u_out = v - a, b - v
        l_in, l_out = float(np.linalg.norm(u_in)), float(np.linalg.norm(u_out))
        if l_in < 1e-6 or l_out < 1e-6:
            continue
        u_in, u_out = u_in / l_in, u_out / l_out
        r = min(radii[i], 0.45 * l_in, 0.45 * l_out)
        if r <= 0.02:
            ctrl.extend([v, v, v])
        elif float(np.dot(u_in, u_out)) > math.cos(math.radians(3.0)):
            ctrl.append(v)
        else:
            ctrl.extend([v - r * u_in, v, v + r * u_out])
    ctrl.append(poly[-1])
    return np.asarray(ctrl, dtype=np.float64)


_B = np.array([[-1, 3, -3, 1], [3, -6, 3, 0], [-3, 0, 3, 0], [1, 4, 1, 0]], dtype=np.float64) / 6.0


def _spline_points(ctrl):
    """Fast point-only evaluation of the clamped uniform cubic B-spline for collision checks."""
    C = np.vstack([ctrl[:1], ctrl[:1], ctrl, ctrl[-1:], ctrl[-1:]])
    pts = []
    for i in range(len(C) - 3):
        seg = C[i:i + 4]
        span = float(np.max(np.linalg.norm(seg[1:] - seg[:-1], axis=1)))
        if span < 1e-9:
            continue
        n = int(min(400, max(4, math.ceil(1.5 * span / DS))))
        u = np.linspace(0.0, 1.0, n, endpoint=False)
        U = np.stack([u ** 3, u ** 2, u, np.ones_like(u)], axis=1)
        pts.append(U @ (_B @ seg))
    if not pts:
        return ctrl[-1:].copy()
    pts.append(C[-1:])
    return np.vstack(pts)


def geometry(ctrl):
    """Clamped uniform cubic B-spline resampled every DS metres: (points, tangents, curvature vectors, s)."""
    C = np.vstack([ctrl[:1], ctrl[:1], ctrl, ctrl[-1:], ctrl[-1:]])
    pts, d1, d2 = [], [], []
    for i in range(len(C) - 3):
        seg = C[i:i + 4]
        span = float(np.max(np.linalg.norm(seg[1:] - seg[:-1], axis=1)))
        if span < 1e-9:
            continue
        n = int(min(800, max(6, math.ceil(3.0 * span / DS))))
        u = np.linspace(0.0, 1.0, n, endpoint=False)
        U = np.stack([u ** 3, u ** 2, u, np.ones_like(u)], axis=1)
        dU = np.stack([3 * u ** 2, 2 * u, np.ones_like(u), np.zeros_like(u)], axis=1)
        ddU = np.stack([6 * u, 2 * np.ones_like(u), np.zeros_like(u), np.zeros_like(u)], axis=1)
        coef = _B @ seg
        pts.append(U @ coef)
        d1.append(dU @ coef)
        d2.append(ddU @ coef)
    if not pts:
        return _point(ctrl[-1])
    pts.append(C[-1:])
    d1.append(np.zeros((1, 3)))
    d2.append(np.zeros((1, 3)))
    P, D1, D2 = np.vstack(pts), np.vstack(d1), np.vstack(d2)
    step = np.linalg.norm(np.diff(P, axis=0), axis=1)
    keep = np.concatenate([[True], step > 1e-9])
    P, D1, D2 = P[keep], D1[keep], D2[keep]
    s_raw = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
    if s_raw[-1] < 1e-6:
        return _point(P[-1])
    speed = np.linalg.norm(D1, axis=1)
    # Tangent and curvature vector from the analytic derivatives; where the parametric speed vanishes
    # (clamped ends, tripled corners) take the chord direction and an effectively infinite curvature.
    T = np.where(speed[:, None] > 1e-3, D1 / np.maximum(speed, 1e-12)[:, None], 0.0)
    chord = np.gradient(P, s_raw, axis=0)
    chord /= np.maximum(np.linalg.norm(chord, axis=1), 1e-12)[:, None]
    T = np.where(speed[:, None] > 1e-3, T, chord)
    acc_perp = D2 - np.sum(D2 * T, axis=1)[:, None] * T
    K = np.where(speed[:, None] > 1e-2, acc_perp / np.maximum(speed, 1e-12)[:, None] ** 2, 0.0)
    k_mag = np.linalg.norm(K, axis=1)
    K = np.where((k_mag > 8.0)[:, None], K * (8.0 / np.maximum(k_mag, 1e-12))[:, None], K)
    s = np.arange(0.0, s_raw[-1], DS)
    s = np.append(s, s_raw[-1]) if s_raw[-1] - s[-1] > 1e-6 else s
    out_p = np.stack([np.interp(s, s_raw, P[:, k]) for k in range(3)], axis=1)
    out_t = np.stack([np.interp(s, s_raw, T[:, k]) for k in range(3)], axis=1)
    out_t /= np.maximum(np.linalg.norm(out_t, axis=1), 1e-12)[:, None]
    out_k = np.stack([np.interp(s, s_raw, K[:, k]) for k in range(3)], axis=1)
    # A tripled corner is a cusp (the tangent turns on the spot): slow to rest at that single vertex.
    kappa_cusp = np.zeros(len(s))
    for idx in np.flatnonzero(speed < 1e-6):
        if 0 < idx < len(P) - 1:
            kappa_cusp[int(np.argmin(np.abs(s - s_raw[idx])))] = 1e3
    return out_p, out_t, out_k, s, kappa_cusp


def _point(p):
    return (np.asarray(p, dtype=np.float64).reshape(1, 3), np.array([[1.0, 0.0, 0.0]]), np.zeros((1, 3)),
            np.zeros(1), np.zeros(1))


# --- map queries ---------------------------------------------------------------------------------

class MapView:
    """What a trajectory needs from one occupancy snapshot: collisions, clearance, unmapped space."""

    def __init__(self, occ_map, band=(1.5, 3.25)):
        self.map = occ_map
        self.inflated = occ_map.inflated_obstacles()
        self.free = occ_map.free
        self._band = band
        self._clear2d = None  # computed lazily only when building/retiming a trajectory

    @property
    def clear2d(self):
        if self._clear2d is None:
            occ_map = self.map
            z0 = max(0, int((self._band[0] - occ_map.origin[2]) / occ_map.resolution))
            z1 = min(occ_map.shape[2], int((self._band[1] - occ_map.origin[2]) / occ_map.resolution) + 1)
            occ2d = np.any(occ_map.occupied[:, :, z0:z1], axis=2)
            if occ2d.any():
                self._clear2d = distance_transform_edt(~occ2d) * occ_map.resolution - 0.5 * occ_map.resolution
            else:
                self._clear2d = np.full(occ2d.shape, 99.0)
        return self._clear2d

    def _idx(self, pts):
        idx = self.map.world_to_idx(pts)
        ok = self.map.in_bounds(idx)
        return np.clip(idx, 0, np.array(self.map.shape) - 1), ok

    def blocked(self, pts, allow_unknown=True):
        """Per point: inside an inflated obstacle, outside the map, or (if not allowed) unmapped."""
        idx, ok = self._idx(pts)
        bad = ~ok | self.inflated[idx[:, 0], idx[:, 1], idx[:, 2]]
        if not allow_unknown:
            bad |= ~self.free[idx[:, 0], idx[:, 1], idx[:, 2]]
        return bad

    def clearance(self, pts):
        idx, ok = self._idx(pts)
        return np.where(ok, self.clear2d[idx[:, 0], idx[:, 1]], 0.0)

    def known_free(self, pts):
        idx, ok = self._idx(pts)
        return ok & self.free[idx[:, 0], idx[:, 1], idx[:, 2]]


def first_collision(view, pts, allow_unknown=True):
    """Index of the first blocked point after the start has left any obstacle margin it began in."""
    bad = view.blocked(pts, allow_unknown)
    if not bad.any():
        return None
    start = int(np.argmin(bad)) if bad[0] else 0  # escaping a margin the drone already sits in is allowed
    if bad[0] and not (~bad).any():
        return 0
    hits = np.flatnonzero(bad[start:])
    return int(start + hits[0]) if len(hits) else None


# --- timing --------------------------------------------------------------------------------------

def speed_limits(view, P, T, K, s, kappa_cusp, lim, allow_unknown=True):
    """Speed bound per geometry sample; also whether unmapped space limits it."""
    kappa_smooth = np.linalg.norm(K, axis=1)
    kappa = np.maximum(kappa_smooth, kappa_cusp)
    v = np.full(len(s), lim.v_max)
    v = np.minimum(v, np.sqrt(lim.a_lat / np.maximum(kappa, 1e-6)))
    if len(s) > 2:
        # Lateral jerk v^3 dkappa/ds: bound it on smooth curvature only (never smear a cusp's 1e3
        # impulse across neighbours, which would force a multi-second 0.05 m/s crawl).
        dk = np.abs(np.gradient(kappa_smooth, s))
        dk = np.maximum(dk, np.convolve(dk, np.ones(5) / 5.0, mode="same"))
        v = np.minimum(v, np.cbrt(0.3 * lim.jerk / np.maximum(dk, 1e-6)))
    if view is not None:
        v = np.minimum(v, lim.clear_v0 + lim.clear_gain * np.maximum(0.0, view.clearance(P) - 1.05))
    horizon_limited = False
    if view is not None and allow_unknown:
        known = view.known_free(P)
        known[s < 1.0] = True
        unknown = np.flatnonzero(~known)
        if len(unknown):
            s_u = s[unknown[0]]
            ahead = s < s_u
            cap = np.sqrt(lim.v_unknown ** 2 + 2.0 * 0.75 * lim.a_brake * np.maximum(0.0, s_u - s))
            horizon_limited = bool(np.any(cap[ahead] < v[ahead] - 1e-6))
            v = np.where(ahead, np.minimum(v, cap), np.minimum(v, lim.v_unknown))
    return np.maximum(v, 0.05), horizon_limited


def speed_profile(s_grid, v_lim, v0, a0, lim, kappa=None):
    """Jerk-limited motion along the arc from (v0, a0) to rest at its end, never far above v_lim.

    In a curve, changing speed changes the centripetal acceleration too (lateral jerk 2 kappa v a),
    so speeding up there is gentler; slowing down keeps the rate the envelope was built with.
    Returns arrays (s, v, a) every DT."""
    s_end = float(s_grid[-1])
    if s_end < 1e-4:
        return np.zeros(1), np.zeros(1), np.zeros(1)
    ab = 0.75 * lim.a_brake
    env = v_lim.astype(np.float64).copy()
    env[-1] = 0.0
    ds = np.diff(s_grid)
    for k in range(len(env) - 2, -1, -1):
        env[k] = min(env[k], math.sqrt(env[k + 1] ** 2 + 2.0 * ab * ds[k]))
    j = 0.8 * lim.jerk
    kv = j / lim.a_brake
    # Final approach: v = sqrt(2 ab d) far out, v = kp d inside ab / kp^2 (~0.25 m); deceleration is
    # continuous across the switch, then decays smoothly to rest without a multi-second 1 cm/s creep.
    kp = 2.2

    # Fast O(1) lookup on uniform DS-spaced arc grids (avoids ~3000 np.interp calls per build).
    n_grid = len(s_grid)
    last_idx = n_grid - 1
    uniform = n_grid >= 2 and abs(float(s_grid[1] - s_grid[0]) - DS) < 1e-6 and (
        n_grid <= 2 or abs(float(s_grid[-2] - s_grid[0]) - (n_grid - 2) * DS) < 1e-4
    )
    env_list = env.tolist()
    s_list = s_grid.tolist()
    kap_list = kappa.tolist() if kappa is not None else None
    inv_ds = 1.0 / DS

    def _lookup(arr, x):
        if x <= 0.0:
            return arr[0]
        if x >= s_end:
            return arr[last_idx]
        if uniform:
            i = int(x * inv_ds)
            if i >= last_idx:
                i = last_idx - 1
            s0 = s_list[i]
            ds_i = s_list[i + 1] - s0
            u = (x - s0) / ds_i if ds_i > 1e-12 else 0.0
            return arr[i] + u * (arr[i + 1] - arr[i])
        return float(np.interp(x, s_grid, arr))

    s, v, a = 0.0, max(0.0, v0), a0
    S, V, A = [s], [v], [a]
    for step in range(int(600.0 / DT)):
        if step > 0 and step % 150 == 0:
            time.sleep(0.001)  # yield GIL on long cross-world trajectories so LIO/IMU never starve
        rem = s_end - s
        if rem <= 1.2e-2 and v <= 2.5e-2:
            break
        look = v * (max(a, 0.0) + lim.a_brake) / j
        vt = min(_lookup(env_list, s), _lookup(env_list, s + look), max(0.0, sqrt_controller(rem, kp, ab)))
        e = vt - v
        a_t = sqrt_controller(e, kv, j)
        turn = 2.0 * (_lookup(kap_list, s) if kap_list is not None else 0.0) * v
        a_up = min(lim.a_acc, 0.3 * lim.jerk / turn) if turn > 1e-6 else lim.a_acc
        a_down = max(ab, min(lim.a_brake, 0.3 * lim.jerk / turn)) if turn > 1e-6 else lim.a_brake
        a_t = max(-a_down, min(a_up, a_t))
        if abs(a_t) * DT > abs(e) and e != 0.0:
            a_t = e / DT  # never cross the target speed within one step
        a += max(-j * DT, min(j * DT, a_t - a))
        v_new = v + a * DT
        if v_new < 0.0:
            v_new, a = 0.0, 0.0
        s = min(s_end, s + 0.5 * (v + v_new) * DT)
        v = v_new
        S.append(s)
        V.append(v)
        A.append(a)
    S[-1], V[-1], A[-1] = s_end, 0.0, 0.0
    return np.asarray(S), np.asarray(V), np.asarray(A)


def yaw_profile(heading, yaw0, rate0, lim, hold_after):
    """Yaw following per-step target headings with rate/acceleration limits; extended until settled."""
    ky = lim.yaw_acc / max(lim.yaw_rate, 1e-6)
    yaw, w = yaw0, rate0
    Y, W = [], []
    k = 0
    target = heading[0]
    while True:
        if k > 0 and k % 150 == 0:
            time.sleep(0.001)
        if k < len(heading):
            if k < hold_after:
                target = heading[k]
        elif abs(_wrap(target - yaw)) < math.radians(0.3) and abs(w) < math.radians(0.5):
            break
        if k > len(heading) + int(30.0 / DT):
            break
        Y.append(yaw)
        W.append(w)
        e = _wrap(target - yaw)
        w_t = max(-lim.yaw_rate, min(lim.yaw_rate, sqrt_controller(e, ky, lim.yaw_acc)))
        w += max(-lim.yaw_acc * DT, min(lim.yaw_acc * DT, w_t - w))
        yaw = _wrap(yaw + w * DT)
        k += 1
    Y.append(target if abs(_wrap(target - yaw)) < math.radians(0.3) else yaw)
    W.append(0.0)
    return np.asarray(Y), np.asarray(W)


# --- trajectory ----------------------------------------------------------------------------------

class Trajectory:
    """Geometry plus timing from t0 (sim seconds); holds its last state after the end."""

    def __init__(self, t0, P, T, K, s, ps, pv, pa, yaw, yaw_rate, goal=None, horizon_limited=False, poly=None,
                 kc=None):
        self.t0 = t0
        self.P, self.T, self.K, self.s = P, T, K, s
        self.kc = np.zeros(len(s)) if kc is None else kc
        self.ps, self.pv, self.pa = ps, pv, pa
        self.yaw, self.yaw_rate = yaw, yaw_rate
        self.goal = P[-1].copy() if goal is None else np.asarray(goal, dtype=np.float64)
        self.horizon_limited = horizon_limited
        self.poly = poly  # the polyline it was built over

    @property
    def t_motion_end(self):
        return self.t0 + (len(self.ps) - 1) * DT

    @property
    def t_end(self):
        return self.t0 + (max(len(self.ps), len(self.yaw)) - 1) * DT

    def _along(self, t):
        x = (t - self.t0) / DT
        n = len(self.ps)
        if x <= 0.0:
            return self.ps[0], self.pv[0], self.pa[0]
        if x >= n - 1:
            return self.ps[-1], 0.0, 0.0
        i = int(x)
        u = x - i
        return (self.ps[i] + u * (self.ps[i + 1] - self.ps[i]), self.pv[i] + u * (self.pv[i + 1] - self.pv[i]),
                self.pa[i] + u * (self.pa[i + 1] - self.pa[i]))

    def _geo(self, s):
        if len(self.s) == 1:
            return self.P[0], self.T[0], self.K[0]
        p = np.array([np.interp(s, self.s, self.P[:, k]) for k in range(3)])
        tan = np.array([np.interp(s, self.s, self.T[:, k]) for k in range(3)])
        tan /= max(1e-12, float(np.linalg.norm(tan)))
        kv = np.array([np.interp(s, self.s, self.K[:, k]) for k in range(3)])
        return p, tan, kv

    def sample(self, t):
        """(position, velocity, acceleration, yaw, yaw_rate) at sim time t."""
        s, v, a = self._along(t)
        p, tan, kv = self._geo(s)
        vel = tan * v
        acc = tan * a + kv * v * v
        x = (t - self.t0) / DT
        m = len(self.yaw)
        if x <= 0.0:
            yaw, rate = self.yaw[0], self.yaw_rate[0]
        elif x >= m - 1:
            yaw, rate = self.yaw[-1], 0.0
        else:
            i = int(x)
            u = x - i
            yaw = _wrap(self.yaw[i] + u * _wrap(self.yaw[i + 1] - self.yaw[i]))
            rate = self.yaw_rate[i] + u * (self.yaw_rate[i + 1] - self.yaw_rate[i])
        return p, vel, acc, yaw, rate

    def arc_at(self, t):
        return self._along(t)

    def remaining_points(self, t, spacing=0.25):
        s0 = self._along(t)[0]
        ss = np.arange(s0, self.s[-1] + 1e-9, spacing)
        if len(ss) == 0 or len(self.s) == 1:
            return self.P[-1:].copy()
        return np.stack([np.interp(ss, self.s, self.P[:, k]) for k in range(3)], axis=1)

    def remaining_length(self, t):
        return float(self.s[-1] - self._along(t)[0])


def rest(t0, p, yaw):
    """Hold position p and heading yaw."""
    p = np.asarray(p, dtype=np.float64).reshape(1, 3)
    return Trajectory(t0, p, np.array([[math.cos(yaw), math.sin(yaw), 0.0]]), np.zeros((1, 3)), np.zeros(1),
                      np.zeros(1), np.zeros(1), np.zeros(1), np.array([_wrap(yaw)]), np.zeros(1))


def _timed(t0, P, T, K, s, kappa_cusp, view, lim, start, allow_unknown, goal=None, poly=None,
           check_feasible=False):
    _, v0, a0, yaw0, rate0 = start
    vt0 = float(np.dot(v0, T[0])) if len(s) > 1 else 0.0
    at0 = float(np.dot(a0, T[0])) if len(s) > 1 else 0.0
    v_lim, horizon = speed_limits(view, P, T, K, s, kappa_cusp, lim, allow_unknown)
    if check_feasible and len(s) > 1 and vt0 > 0.25:
        # Backward feasibility check against curvature limits only: refuse curves where entering
        # at vt0 cannot decelerate to a tight downstream corner (which would blow up K*v^2), while
        # still allowing smooth straight-line deceleration across a v_max step-down (e.g. 2.1->1.55).
        ab = 0.75 * lim.a_brake
        kappa_all = np.maximum(np.linalg.norm(K, axis=1), kappa_cusp)
        v_curv = np.sqrt(1.35 * lim.a_lat / np.maximum(kappa_all, 1e-6))
        env0 = float(v_curv[-1])
        ds = np.diff(s)
        for k in range(len(s) - 2, -1, -1):
            env0 = min(float(v_curv[k]), math.sqrt(env0 * env0 + 2.0 * ab * float(ds[k])))
        if vt0 > env0 + 0.20:
            return None
    ps, pv, pa = speed_profile(s, v_lim, max(0.0, vt0), at0, lim, np.linalg.norm(K, axis=1))
    heading = np.array([math.atan2(T[i, 1], T[i, 0]) for i in np.searchsorted(s, ps).clip(0, len(s) - 1)])
    flat = np.hypot(T[:, 0], T[:, 1])
    hold_after = len(ps)
    near_end = np.flatnonzero(s[-1] - ps < 0.5)
    if len(near_end):
        hold_after = int(near_end[0])
    if len(s) == 1 or flat.max() < 0.3:
        heading = np.array([yaw0])
        hold_after = 0
    target_hold = heading[min(len(heading) - 1, max(0, hold_after - 1))] if hold_after > 0 else yaw0
    heading = np.where(np.arange(len(heading)) < hold_after, heading, target_hold)
    yaw, yaw_rate = yaw_profile(heading, yaw0, rate0, lim, hold_after=len(heading))
    return Trajectory(t0, P, T, K, s, ps, pv, pa, yaw, yaw_rate, goal=goal, horizon_limited=horizon, poly=poly,
                      kc=kappa_cusp)


def _rest_of(traj, t_r, s_stop=None):
    """traj's geometry from its arc position at t_r (to s_stop), restarting at arc length 0."""
    s_r = traj.arc_at(t_r)[0]
    s_stop = traj.s[-1] if s_stop is None else s_stop
    idx = np.flatnonzero((traj.s > s_r) & (traj.s <= s_stop))
    if len(idx) == 0 or traj.s[-1] - s_r < 1e-3:
        return None
    p_r, tan_r, k_r = traj._geo(s_r)
    P = np.vstack([p_r, traj.P[idx]])
    T = np.vstack([tan_r, traj.T[idx]])
    K = np.vstack([k_r, traj.K[idx]])
    s = np.concatenate([[0.0], traj.s[idx] - s_r])
    kc = np.concatenate([[0.0], traj.kc[idx]])
    return P, T, K, s, kc


def retime(traj, t_r, view, lim, allow_unknown=True):
    """The same curve from traj's state at t_r, timed again (new map, speed cap or estimator)."""
    start = traj.sample(t_r)
    geo = _rest_of(traj, t_r)
    if geo is None:
        return rest(t_r, start[0], start[3])  # at (or past) the end of the curve: already at rest
    return _timed(t_r, *geo, view, lim, start, allow_unknown, goal=traj.goal, poly=traj.poly)


def build(poly, start, view, lim, t0, allow_unknown=True, goal=None):
    """Trajectory from start state (p, v, a, yaw, yaw_rate) along polyline poly (poly[0] = start p).

    Keeps the start velocity's direction with a short lead-in, rounds corners, shrinks any rounding
    whose curve meets an inflated obstacle (down to following the polyline exactly), and times it.
    Returns None when no curve over this polyline is clear."""
    p0, v0 = np.asarray(start[0], dtype=np.float64), np.asarray(start[1], dtype=np.float64)
    pts = [p0] + [np.asarray(q, dtype=np.float64) for q in poly[1:]]
    pts = [q for i, q in enumerate(pts) if i == 0 or float(np.linalg.norm(q - pts[i - 1])) > 1e-3]
    if len(pts) < 2:
        return rest(t0, p0, start[3])
    speed0 = float(np.linalg.norm(v0[:2]))
    lead_options = [None]
    u = None
    if speed0 > 0.15:
        u = v0 / max(1e-9, float(np.linalg.norm(v0)))
        first = pts[1] - p0
        first_norm = float(np.linalg.norm(first))
        if first_norm > 1e-6 and float(np.dot(u, first / first_norm)) < 0.0:
            # Goal is >90 deg behind current velocity: brake to rest first instead of stitching a 180-deg hairpin.
            return None
        L_des = float(np.clip(0.8 * speed0, 0.3, 2.5))
        lead_options = [min(L_des * f, 0.5 * first_norm) for f in (1.0, 0.6, 0.35)]
        lead_options = [L for L in lead_options if L > 0.05] or [None]
    for lead in lead_options:
        poly2 = list(pts)
        if lead is not None:
            poly2 = [p0, p0 + u * lead] + pts[1:]
        radii = [lim.corner_r] * len(poly2)
        pushed = [0] * len(poly2)
        for it in range(36):
            if it > 0 and it % 2 == 0:
                time.sleep(0.002)  # yield GIL so 250 Hz ImuNode and 10 Hz LiDAR never stall
            ctrl = control_points(poly2, radii)
            P_chk = _spline_points(ctrl) if view is not None else None
            hit = first_collision(view, P_chk, allow_unknown) if view is not None else None
            if hit is None:
                P, T, K, s, kc = geometry(ctrl)
                if view is not None:
                    hit2 = first_collision(view, P, allow_unknown)
                    if hit2 is not None:
                        hit, P_chk = hit2, P
            if hit is None:
                # If moving at speed, refuse a curve whose initial tangent breaks velocity continuity.
                if speed0 > 0.35 and u is not None and float(np.dot(u, T[0])) < 0.85:
                    break
                tr = _timed(t0, P, T, K, s, kc, view, lim, start, allow_unknown, goal=goal, poly=pts,
                            check_feasible=(view is not None))
                if tr is not None:
                    return tr
                break
            # First try pushing the offending corner outward into free space so wide smooth arcs
            # clear obstacles at cruise speed instead of shrinking to a slow hairpin; shrink only
            # when the outer side is constrained (narrow doorway / corridor).
            if it == 34:
                poly2 = ([p0, p0 + u * lead] + pts[1:]) if lead is not None else list(pts)
                if not any(r > 0.0 for r in radii[1:-1]):
                    break
                radii = [0.0] * len(poly2)
                continue
            q = P_chk[hit]
            order = sorted(range(1, len(poly2) - 1), key=lambda i: float(np.linalg.norm(poly2[i] - q)))
            adjusted = False
            for i in order:
                if pushed[i] < 2 and radii[i] >= 1.0 and float(np.linalg.norm(poly2[i] - q)) < 3.5:
                    a_pt, v_pt, b_pt = poly2[i - 1], poly2[i], poly2[i + 1]
                    u_in = v_pt - a_pt
                    u_out = b_pt - v_pt
                    l_in, l_out = float(np.linalg.norm(u_in)), float(np.linalg.norm(u_out))
                    if l_in > 0.8 and l_out > 0.8:
                        inward = (u_out / l_out) - (u_in / l_in)
                        inward[2] = 0.0
                        in_n = float(np.linalg.norm(inward))
                        if in_n > 0.15:
                            cand_v = v_pt - (inward / in_n) * 0.45
                            seg_in = np.linspace(a_pt, cand_v, max(3, int(l_in / 0.15) + 2))
                            seg_out = np.linspace(cand_v, b_pt, max(3, int(l_out / 0.15) + 2))
                            if (not view.blocked(seg_in, allow_unknown).any()
                                    and not view.blocked(seg_out, allow_unknown).any()):
                                poly2[i] = cand_v
                                pushed[i] += 1
                                adjusted = True
                                break
            if adjusted:
                continue
            for i in order:
                if radii[i] > 0.0:
                    radii[i] = radii[i] * 0.45 if radii[i] > 0.25 else 0.0
                    break
            else:
                break
    return None


def brake(traj, t_r, lim):
    """Stop as soon as the limits allow along traj's own curve, starting from its state at t_r."""
    s_r, v_r, a_r = traj.arc_at(t_r)
    _, _, _, yaw, rate = traj.sample(t_r)
    if len(traj.s) == 1 or v_r < 1e-3:
        return rest(t_r, traj.sample(t_r)[0], yaw)
    # Stopping distance under the limits, by simulation.
    s, v, a = 0.0, v_r, a_r
    for _ in range(int(20.0 / DT)):
        a += max(-0.8 * lim.jerk * DT, min(0.8 * lim.jerk * DT, -lim.a_brake - a))
        v_new = max(0.0, v + a * DT)
        s += 0.5 * (v + v_new) * DT
        v = v_new
        if v <= 0.0:
            break
    geo = _rest_of(traj, t_r, min(traj.s[-1], s_r + 1.15 * s + 0.05))
    if geo is None or len(geo[3]) < 2:
        return rest(t_r, traj.sample(t_r)[0], yaw)
    P, T, K, s_new, kc = geo
    ps, pv, pa = speed_profile(s_new, np.full(len(s_new), v_r + 1.0), v_r, a_r, lim)
    stop_yaw = _wrap(yaw + rate * abs(rate) / (2.0 * lim.yaw_acc))  # where the turn rate can wind down
    y, w = yaw_profile(np.full(len(ps), stop_yaw), yaw, rate, lim, hold_after=len(ps))
    return Trajectory(t_r, P, T, K, s_new, ps, pv, pa, y, w, kc=kc)

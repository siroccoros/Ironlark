"""Collision-aware 3D A* path planner with line-of-sight shortcutting on OccupancyMap."""
import heapq
import math
import time
import numpy as np


# 18-connected 3D neighbor steps (all 6 face + 12 edge moves; crisp and fast).
_NEIGHBORS = [
    (dx, dy, dz, math.sqrt(dx * dx + dy * dy + dz * dz))
    for dx in (-1, 0, 1)
    for dy in (-1, 0, 1)
    for dz in (-1, 0, 1)
    if (dx, dy, dz) != (0, 0, 0) and (abs(dx) + abs(dy) + abs(dz) <= 2)
]


def _nearest_safe(occ_map, idx, inflated, z_min, z_max, allow_unknown, max_r=6):
    """Snap a start/goal voxel that landed inside inflation to the nearest traversable cell."""
    sx, sy, sz = occ_map.shape
    free = occ_map.free
    x0, y0, z0 = int(idx[0]), int(idx[1]), int(np.clip(idx[2], z_min, z_max))
    for r in range(max_r + 1):
        best = None
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                for dz in range(-min(r, 2), min(r, 2) + 1):
                    x, y, z = x0 + dx, y0 + dy, z0 + dz
                    if not (0 <= x < sx and 0 <= y < sy and z_min <= z <= z_max):
                        continue
                    if inflated[x, y, z]:
                        continue
                    if not allow_unknown and not free[x, y, z]:
                        continue
                    d2 = dx * dx + dy * dy + dz * dz
                    if best is None or d2 < best[0]:
                        best = (d2, (x, y, z))
        if best is not None:
            return best[1]
    return None


def line_is_clear(occ_map, a, b, allow_unknown=True):
    """True when the segment a -> b stays clear of inflated obstacles (and unknown if forbidden)."""
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    d = b - a
    dist = float(np.linalg.norm(d))
    # Combine exact grid-plane intersections (so no voxel corner is ever stepped over) with
    # dense samples (<= 5 cm) and tiny boundary offsets so trajectory.DS sampling always agrees.
    ts = [0.0, 1.0]
    res = occ_map.resolution
    for axis in range(3):
        if abs(d[axis]) > 1e-9:
            lo = min(a[axis], b[axis])
            hi = max(a[axis], b[axis])
            k0 = int(math.floor((lo - occ_map.origin[axis]) / res))
            k1 = int(math.ceil((hi - occ_map.origin[axis]) / res))
            planes = occ_map.origin[axis] + np.arange(k0, k1 + 1, dtype=np.float64) * res
            cross = (planes - a[axis]) / d[axis]
            cross = cross[(cross > 0.0) & (cross < 1.0)]
            if len(cross):
                ts.extend(cross.tolist())
    ts_arr = np.sort(np.asarray(ts, dtype=np.float64))
    mids = 0.5 * (ts_arr[:-1] + ts_arr[1:])
    t_all = np.concatenate([mids, np.clip(ts_arr - 1e-5, 0.0, 1.0), np.clip(ts_arr + 1e-5, 0.0, 1.0)])
    pts = a[None, :] + t_all[:, None] * d[None, :]
    idx = occ_map.world_to_idx(pts)
    if not np.all(occ_map.in_bounds(idx)):
        return False
    inflated = occ_map.inflated_obstacles()
    if np.any(inflated[idx[:, 0], idx[:, 1], idx[:, 2]]):
        return False
    if not allow_unknown and not np.all(occ_map.free[idx[:, 0], idx[:, 1], idx[:, 2]]):
        return False
    return True


def shortcut_path(occ_map, waypoints, allow_unknown=True):
    """Greedy line-of-sight path pruning to eliminate staircase voxel zig-zags."""
    if len(waypoints) <= 2:
        return list(waypoints)
    out = [waypoints[0]]
    i = 0
    checks = 0
    while i < len(waypoints) - 1:
        j = len(waypoints) - 1
        while j > i + 1 and not line_is_clear(occ_map, waypoints[i], waypoints[j], allow_unknown):
            j -= 1
            checks += 1
            if checks % 16 == 0:
                time.sleep(0.001)
        if j == i + 1 and not line_is_clear(occ_map, waypoints[i], waypoints[j], allow_unknown):
            a, b = waypoints[i], waypoints[j]
            for mid in (
                np.array([b[0], a[1], a[2]]),
                np.array([a[0], b[1], a[2]]),
                np.array([a[0], a[1], b[2]]),
                np.array([b[0], b[1], a[2]]),
                np.array([b[0], a[1], b[2]]),
                np.array([a[0], b[1], b[2]]),
            ):
                if (line_is_clear(occ_map, a, mid, allow_unknown)
                        and line_is_clear(occ_map, mid, b, allow_unknown)):
                    out.append(mid)
                    break
        out.append(waypoints[j])
        i = j
    # Elastic string-pulling: melt 0.5 m voxel staircases around circular obstacles into clean
    # chords while preserving strict line_is_clear safety on every segment, then re-shortcut.
    if len(out) >= 3:
        pts = [np.asarray(p, dtype=np.float64).copy() for p in out]
        for _ in range(10):
            moved = False
            for k in range(1, len(pts) - 1):
                mid = 0.5 * (pts[k - 1] + pts[k + 1])
                cand = 0.5 * pts[k] + 0.5 * mid
                if (line_is_clear(occ_map, pts[k - 1], cand, allow_unknown)
                        and line_is_clear(occ_map, cand, pts[k + 1], allow_unknown)):
                    if float(np.linalg.norm(cand - pts[k])) > 0.02:
                        moved = True
                    pts[k] = cand
            if not moved:
                break
        # Prune redundant intermediate points created by smoothing when direct line is now clear.
        cleaned = [pts[0]]
        idx_c = 0
        while idx_c < len(pts) - 1:
            j = len(pts) - 1
            while j > idx_c + 1 and not line_is_clear(occ_map, pts[idx_c], pts[j], allow_unknown):
                j -= 1
            cleaned.append(pts[j])
            idx_c = j
        out = cleaned
    return out


def plan_path(occ_map, start, goal, z_bounds=(1.2, 3.6), allow_unknown=True, unknown_penalty=1.35,
              max_expansions=7500):
    """Find a collision-free 3D path from start to goal on occ_map's inflated obstacle grid.

    Returns a list of world-frame waypoints [p0, p1, ..., goal], or None if unreachable.
    """
    inflated = occ_map.inflated_obstacles()
    free = occ_map.free
    sx, sy, sz = occ_map.shape
    z_min = max(1, int(math.floor((z_bounds[0] - occ_map.origin[2]) / occ_map.resolution)))
    z_max = min(sz - 2, max(z_min, int(math.floor((z_bounds[1] - 1e-6 - occ_map.origin[2]) / occ_map.resolution))))

    s_idx = _nearest_safe(occ_map, occ_map.world_to_idx(start), inflated, z_min, z_max, allow_unknown=True)
    g_idx = _nearest_safe(occ_map, occ_map.world_to_idx(goal), inflated, z_min, z_max, allow_unknown=allow_unknown)
    if s_idx is None or g_idx is None:
        return None
    if s_idx == g_idx:
        # Always target the snapped safe voxel, never a raw goal buried inside an obstacle.
        return [np.asarray(start, dtype=np.float64), occ_map.idx_to_world(g_idx)]

    gx, gy, gz = g_idx
    g_score = {s_idx: 0.0}
    came_from = {}
    heap = [(math.dist(s_idx, g_idx), 0.0, s_idx)]
    expanded = 0

    while heap and expanded < max_expansions:
        _, g_cur, cur = heapq.heappop(heap)
        if g_cur > g_score.get(cur, 1e18):
            continue
        if cur == g_idx:
            cells = [cur]
            while cur in came_from:
                cur = came_from[cur]
                cells.append(cur)
            cells.reverse()
            raw = [np.asarray(start, dtype=np.float64)] + [occ_map.idx_to_world(c) for c in cells]
            return shortcut_path(occ_map, raw, allow_unknown=allow_unknown)
        expanded += 1
        if expanded % 150 == 0:
            time.sleep(0.002)  # yield GIL so 250 Hz ImuNode and 10 Hz LiDAR never starve on long replans
        cx, cy, cz = cur
        for dx, dy, dz, step in _NEIGHBORS:
            nx, ny, nz = cx + dx, cy + dy, cz + dz
            if not (0 <= nx < sx and 0 <= ny < sy and z_min <= nz <= z_max):
                continue
            if inflated[nx, ny, nz]:
                continue
            # Prevent diagonal moves from squeezing through zero-width pinholes between inflated voxels.
            if dx != 0 and dy != 0 and inflated[nx, cy, cz] and inflated[cx, ny, cz]:
                continue
            if dx != 0 and dz != 0 and inflated[nx, cy, cz] and inflated[cx, cy, nz]:
                continue
            if dy != 0 and dz != 0 and inflated[cx, ny, cz] and inflated[cx, cy, nz]:
                continue
            is_free = bool(free[nx, ny, nz])
            if not allow_unknown and not is_free:
                continue
            # Prefer horizontal travel at cruise altitude and penalize traversing unknown space.
            cost = step * (1.0 if is_free else unknown_penalty) + (0.25 if dz != 0 else 0.0)
            ng = g_cur + cost
            nxt = (nx, ny, nz)
            if ng < g_score.get(nxt, 1e18):
                g_score[nxt] = ng
                came_from[nxt] = cur
                h = math.sqrt((nx - gx) ** 2 + (ny - gy) ** 2 + (nz - gz) ** 2)
                heapq.heappush(heap, (ng + 1.35 * h, ng, nxt))
    return None

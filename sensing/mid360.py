"""The Livox Mid-360's scan, and casting it into a rendered range panorama. Plain numpy: no Webots.

The pattern (config/sensors.json lidar.pattern) was fitted to a real unit's recording by
tools/fit_mid360_pattern.py: a slot every slot_ns from time 0, lines 0-3 in turn; the head spins at
spin_hz; line 0's elevation nods, a Fourier series of nod_period_s; line a sits a times the fan from it.
Spin and nod share no period, so the pattern never repeats.

The panorama is the Webots Lidar's range image (cylindrical, 360 degrees): column j looks at azimuth
pi - (j + 0.5) * 2 pi / columns, row i at elevation fov / 2 - i * fov / (rows - 1) (WbLidar::updatePointCloud).
A ray cast from where the sensor was at its own instant meets the surface the panorama saw from where it
was when rendered: the nearest pixel's point and the plane through it and its neighbours, which corrects
the sensor's motion between the two exactly on flat surfaces. Across a depth edge (a neighbour more
than EDGE of the range away) there is no plane: the range is the point's along the ray.
"""
import numpy as np

EDGE = 0.1


def slots(pattern, start_ns, end_ns):
    """Indices of the slots fired in [start_ns, end_ns)."""
    slot = pattern["slot_ns"]
    return np.arange(-(-start_ns // slot), -(-end_ns // slot), dtype=np.int64)


def directions(pattern, k):
    """Firing time (ns), line and unit direction (sensor frame: x forward, y left, z up) of slots k."""
    time_ns = k * pattern["slot_ns"]
    line = k % pattern["lines"]
    t = time_ns * 1e-9
    azimuth = 2 * np.pi * pattern["spin_hz"] * t
    nod = pattern["nod_deg"]
    phase = 2 * np.pi * t / pattern["nod_period_s"]
    elevation = nod[0] + sum(nod[2 * m - 1] * np.cos(m * phase) + nod[2 * m] * np.sin(m * phase)
                             for m in range(1, (len(nod) - 1) // 2 + 1))
    e0, e1, e2 = pattern["fan_elevation_deg"]
    z1, z2 = pattern["fan_azimuth_deg"]
    cos, sin = np.cos(azimuth), np.sin(azimuth)
    elevation = np.radians(elevation + line * (e0 + e1 * cos + e2 * sin))
    azimuth = azimuth + np.radians(line * (z1 * sin + z2 * cos))
    return time_ns, line, unit(azimuth, elevation)


def unit(azimuth, elevation):
    c = np.cos(elevation)
    return np.stack([c * np.cos(azimuth), c * np.sin(azimuth), np.sin(elevation)], axis=-1)


def perturb(rays, sigma, rng):
    """Rays turned by a random angle, each of two perpendicular components normal with deviation sigma (rad)."""
    helper = np.where(np.abs(rays[:, 2:3]) < 0.9, [[0.0, 0.0, 1.0]], [[1.0, 0.0, 0.0]])
    a = np.cross(rays, helper)
    a /= np.linalg.norm(a, axis=1, keepdims=True)
    b = np.cross(rays, a)
    turned = rays + a * rng.normal(0.0, sigma, (len(rays), 1)) + b * rng.normal(0.0, sigma, (len(rays), 1))
    return turned / np.linalg.norm(turned, axis=1, keepdims=True)


def cast(image, vertical_fov, origins, rays):
    """Range along each ray from its origin (both in the panorama's frame) to the surface the panorama saw,
    and the cosine of its incidence there (1 across an edge); NaN where the pixel saw nothing."""
    rows, columns = image.shape
    d_azimuth, d_elevation = 2 * np.pi / columns, vertical_fov / (rows - 1)

    def pixel_point(i, j):
        return image[i, j][:, None] * unit(np.pi - (j + 0.5) * d_azimuth, vertical_fov / 2 - i * d_elevation)

    azimuth = np.arctan2(rays[:, 1], rays[:, 0])
    elevation = np.arcsin(np.clip(rays[:, 2], -1.0, 1.0))
    j = np.rint((np.pi - azimuth) / d_azimuth - 0.5).astype(np.int64) % columns
    i = np.clip(np.rint((vertical_fov / 2 - elevation) / d_elevation).astype(np.int64), 0, rows - 1)
    below = np.where(i + 1 < rows, i + 1, i - 1)
    r = image[i, j]
    with np.errstate(invalid="ignore", divide="ignore"):  # pixels that saw nothing are infinite
        point, side, down = pixel_point(i, j), pixel_point(i, (j + 1) % columns), pixel_point(below, j)
        normal = np.cross(side - point, down - point)
        normal /= np.linalg.norm(normal, axis=1, keepdims=True)
        smooth = ((np.abs(image[i, (j + 1) % columns] - r) < EDGE * r) & (np.abs(image[below, j] - r) < EDGE * r)
                  & np.all(np.isfinite(normal), axis=1))
        facing = np.einsum("ij,ij->i", normal, rays)
        along = np.einsum("ij,ij->i", point - origins, rays)
        planar = np.einsum("ij,ij->i", normal, point - origins) / facing
        smooth &= np.isfinite(planar) & (np.abs(planar - along) < EDGE * r)
    distance = np.where(smooth, planar, along)
    cosine = np.where(smooth, np.abs(facing), 1.0)
    distance[~np.isfinite(r)] = np.nan
    return distance, cosine


def rotations(q):
    """Rotation matrices of quaternions (x, y, z, w), shape (n, 4) to (n, 3, 3)."""
    x, y, z, w = q.T
    return np.stack([np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
                     np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
                     np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1)], -2)


POINT = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("layer", "<i4"), ("time", "<f4")])


class Sweep:
    """The Mid-360 swept over renders (config/sensors.json lidar): fed the body's ground truth pose every step
    and each render half a render period after it was taken (when the poses of every slot it serves are known),
    it returns the frames completed: (start in ns, points). A render at T serves the slots fired in
    [T - half, T + half), each cast from the sensor's pose at its own instant (interpolated between steps) at
    its direction turned by the angular noise. A return needs a pixel that saw something and, every surface
    taken as Lambertian at the configured reflectivity, enough light back: the range at most max_range times
    sqrt(reflectivity * cos(incidence) / 0.1) (the datasheet's 40 m at 10%), and no farther than the render.
    Range noise is added after; points are the nominal direction times the range, in the sensor frame at their
    instant, their time in seconds after the frame's start, their layer the line. Frames start every frame
    period; the first one whole is the first output."""

    def __init__(self, lidar, step_ms):
        self.pattern = lidar["pattern"]
        self.lidar = lidar
        self.frame_ns = round(1000 / lidar["rate_hz"]) * 1_000_000
        self.half_ns = round(500 / lidar["render_hz"]) * 1_000_000
        self.mount_t = np.array(lidar["translation"], float)
        x, y, z, angle = lidar["rotation"]
        axis = np.array([x, y, z], float) / (np.linalg.norm([x, y, z]) or 1.0)
        self.mount_q = np.array([*axis * np.sin(angle / 2), np.cos(angle / 2)])
        self.rng = np.random.default_rng(lidar["seed"])
        self.poses = []  # (time ns, sensor position, sensor quaternion), every step, the last two render periods
        self.keep = 2 * (2 * self.half_ns // (step_ms * 1_000_000)) + 2
        self.pending, self.first = {}, None

    def pose(self, time_ns, position, quaternion):
        """The body's pose (world ENU, quaternion x, y, z, w) at a step."""
        q = np.asarray(quaternion, float)
        sensor_q = quaternion_multiply(q, self.mount_q)
        self.poses.append((time_ns, np.asarray(position, float) + rotations(q[None])[0] @ self.mount_t, sensor_q))
        del self.poses[:-self.keep]

    def _at(self, time_ns):
        times = np.array([p[0] for p in self.poses])
        if not (len(times) and times[0] <= time_ns.min() and time_ns.max() <= times[-1]):
            raise RuntimeError(f"no sensor pose for slots at {time_ns.min()}-{time_ns.max()} ns")
        i = np.clip(np.searchsorted(times, time_ns, side="right") - 1, 0, len(times) - 2)
        f = ((time_ns - times[i]) / (times[i + 1] - times[i]))[:, None]
        positions = np.array([p[1] for p in self.poses])
        quaternions = np.array([p[2] for p in self.poses])
        a, b = quaternions[i], quaternions[i + 1]
        b = np.where(np.sum(a * b, axis=1, keepdims=True) < 0, -b, b)
        q = a * (1 - f) + b * f  # within a step the turn is small: normalised lerp is slerp to 1e-7 rad
        return positions[i] * (1 - f) + positions[i + 1] * f, q / np.linalg.norm(q, axis=1, keepdims=True)

    def render(self, render_ns, image):
        """Cast the slots a render serves; the frames completed, oldest first."""
        lidar = self.lidar
        start, end = render_ns - self.half_ns, render_ns + self.half_ns
        self.first = start if self.first is None else self.first
        time_ns, line, nominal = directions(self.pattern, slots(self.pattern, start, end))
        positions, quaternions = self._at(time_ns)
        (render_position,), (render_q,) = self._at(np.array([render_ns]))
        render_r = rotations(render_q[None])[0]
        cast_rays = perturb(nominal, lidar["angular_noise"], self.rng)
        rays = np.einsum("nij,nj->ni", rotations(quaternions), cast_rays) @ render_r
        distance, cosine = cast(image, lidar["vertical_field_of_view"], (positions - render_position) @ render_r, rays)
        reach = np.minimum(lidar["max_range"] * np.sqrt(lidar["reflectivity"] * cosine / 0.1), lidar["max_range"])
        with np.errstate(invalid="ignore"):
            hit = (distance >= lidar["min_range"]) & (distance <= reach)
        distance = distance[hit] + self.rng.normal(0.0, lidar["noise"], hit.sum())
        points = np.empty(hit.sum(), POINT)
        for axis, name in enumerate("xyz"):
            points[name] = distance * nominal[hit, axis]
        points["layer"] = line[hit]
        frame = time_ns[hit] // self.frame_ns * self.frame_ns
        points["time"] = (time_ns[hit] - frame) * 1e-9
        for f in np.unique(time_ns // self.frame_ns * self.frame_ns):  # a frame may see nothing: still a frame
            self.pending.setdefault(int(f), []).append(points[frame == f])
        done = sorted(f for f in self.pending if f + self.frame_ns <= end)
        return [(f, np.concatenate(self.pending.pop(f))) for f in done if f >= self.first]


def quaternion_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz])

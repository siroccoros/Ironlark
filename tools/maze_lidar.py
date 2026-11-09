"""The LiDAR against a generated maze's truth.json: every ray, hit or not, against the walls and ground the world holds.

For scans taken while hovering (tools/check_recording.py picks them), each ray's direction comes from its place in the
scan: points are layer-major, top layer first, `layers` of `horizontal_resolution` columns. A layer's elevation is the
configured one (config/sensors.json: render_layers spread over the symmetric vertical field, the top `layers` kept);
the columns' azimuth is fitted to the returns (an offset and a sense of rotation). Both are checked against where the
returns actually lie, so a misplaced layer (as Webots' tiltAngle once did) fails rather than hides.

Each ray is cast from the LiDAR's pose (ground truth and the static transforms) against truth.json's wall boxes and its
ground, then judged:
- placed: a return within TOLERANCE_M of the surface truth puts on its ray (measured along the surface's normal);
- off: a return elsewhere, or where truth has nothing within range;
- missing: no return where truth has a surface within range (a wall that does not reflect);
- self: a return nearer than the drone's reach, the drone itself, counted apart;
- edge: a ray whose neighbour one layer or one column away meets another surface over JUMP_M nearer or farther, or
  nothing: its footprint holds a depth edge, so either side is a true return. Counted apart, neither placed nor
  missing. Webots' LiDAR shows why: below a wall's top edge it returns nothing for 89% of the rays within 0.25 degrees,
  47% at 0.5-0.75 and 9% at 2-3 (seed 7, 1.5 m hover, 18 scans), the band of one layer (1.9 degrees).
The range error on surfaces seen near face-on is the rig's noise, within NOISE_BAND of its configured value.

numpy only: it runs in the ROS 2 container (numpy 1.21) and in the project's environment.
"""
import math

import numpy as np

POINT = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("layer", "<i4"), ("time", "<f4")])  # a Webots LidarPoint
TOLERANCE_M = 0.1  # 5 x the rig's 2 cm range noise
SELF_M = 0.35  # the drone's collision box reaches 0.33 m from its centre
JUMP_M = 0.3  # a range step between neighbouring rays on different surfaces: a depth edge
FACE_ON = 0.7  # |cos| of the angle between a ray and the surface's normal, for the noise measurement
NOISE_BAND = (0.5, 1.5)
LIMITS = {"placed": 0.99, "missing": 0.01, "elevation_error_deg": 0.3, "azimuth_error_deg": 0.3, "returns": 1000}


def elevations(lidar):
    """Each kept layer's elevation in radians, top first."""
    fov, rendered = lidar["vertical_field_of_view"], lidar["render_layers"]
    return fov / 2 - np.arange(lidar["layers"]) * fov / (rendered - 1)


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def fit_azimuth(points, columns):
    """(sense, offset, RMS residual in radians) of azimuth = offset + sense * 2 pi column / columns, over the finite
    returns of scans of the same rig."""
    finite = np.isfinite(points["x"])
    column = (np.nonzero(finite)[1] % columns).astype(float)
    azimuth = np.arctan2(points["y"][finite], points["x"][finite]).astype(float)
    best = None
    for sense in (1, -1):
        residual = azimuth - sense * 2 * math.pi * column / columns
        offset = float(np.angle(np.mean(np.exp(1j * residual))))
        rms = float(np.sqrt(np.mean(wrap(residual - offset) ** 2)))
        if best is None or rms < best[2]:
            best = (sense, offset, rms)
    return best


def rotation(q):
    """The rotation matrix of a quaternion (x, y, z, w)."""
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def boxes(truth):
    """truth.json's surfaces as axis-aligned boxes: (low corners, high corners, wall index or -1 for the ground)."""
    low, high, index = [], [], []
    for i, wall in enumerate(truth["walls"]):
        centre, size = np.array(wall["centre"]), np.array(wall["size"])
        low.append(centre - size / 2)
        high.append(centre + size / 2)
        index.append(i)
    ground = truth["ground"]
    low.append([*ground["min"], ground["top_z"] - ground["thickness_m"]])
    high.append([*ground["max"], ground["top_z"]])
    index.append(-1)
    return np.array(low), np.array(high), np.array(index)


def edges(expected, box):
    """Which rays (layers x columns, the grid padded by one layer above and below) hold a depth edge in their footprint:
    a neighbour one layer or one column away (columns wrap around) meets nothing where they meet something, or the
    reverse, or another surface over JUMP_M away. Returns the unpadded layers x columns."""
    centre, centre_box = expected[1:-1], box[1:-1]
    found = np.zeros(centre.shape, dtype=bool)
    for other, other_box in ((expected[:-2], box[:-2]), (expected[2:], box[2:]),
                             (np.roll(centre, 1, axis=1), np.roll(centre_box, 1, axis=1)),
                             (np.roll(centre, -1, axis=1), np.roll(centre_box, -1, axis=1))):
        a, b = np.isfinite(centre), np.isfinite(other)
        with np.errstate(invalid="ignore"):
            found |= (a != b) | (a & b & (centre_box != other_box) & (np.abs(centre - other) > JUMP_M))
    return found


def cast(origin, directions, low, high):
    """For rays from one origin along unit directions (N x 3): the range to the nearest box (inf for none), the box's
    index, and |cos| of the angle between the ray and the face it meets."""
    with np.errstate(divide="ignore", invalid="ignore"):
        inverse = 1.0 / directions
    best = np.full(len(directions), np.inf)
    which = np.full(len(directions), -1)
    cosine = np.zeros(len(directions))
    for b in range(len(low)):
        with np.errstate(invalid="ignore"):
            t1, t2 = (low[b] - origin) * inverse, (high[b] - origin) * inverse
        near, far = np.fmin(t1, t2), np.fmax(t1, t2)
        enter, leave = np.nanmax(near, axis=1), np.nanmin(far, axis=1)
        hit = (enter <= leave) & (enter > 0) & (enter < best)
        axis = np.nanargmax(near, axis=1)
        best[hit], which[hit] = enter[hit], b
        cosine[hit] = np.abs(directions[hit, axis[hit]])
    return best, which, cosine


def check(scans, lidar_link, truth, lidar):
    """The report on scans [(base pose ((x, y, z), (qx, qy, qz, qw)), points: an array of POINT, layer-major)] against
    truth (truth.json), with lidar_link the LiDAR's static transform from the base and lidar the rig's configuration."""
    columns, layers = lidar["horizontal_resolution"], lidar["layers"]
    if not scans:
        return {"scans": 0, "problems": ["no hovering scans to check"]}
    points = np.stack([p[: layers * columns] for _, p in scans]).reshape(len(scans), layers, columns)
    # The returns that place the rays: finite (arctan2(inf, inf) is 45 degrees, not missing), not the drone itself.
    with np.errstate(invalid="ignore"):
        usable = np.sqrt(points["x"].astype(float) ** 2 + points["y"] ** 2 + points["z"] ** 2) >= SELF_M
    usable &= np.isfinite(points["x"]) & np.isfinite(points["y"]) & np.isfinite(points["z"])
    calibration = points.copy()
    for c in "xyz":
        calibration[c][~usable] = np.nan
    sense, offset, azimuth_rms = fit_azimuth(calibration.reshape(len(scans), -1), columns)
    x, y, z = (calibration[c].astype(float) for c in "xyz")
    measured_elevation = np.arctan2(z, np.hypot(x, y))
    configured = elevations(lidar)
    layer_error = [abs(float(np.nanmedian(measured_elevation[:, k])) - configured[k])
                   for k in range(layers) if np.isfinite(measured_elevation[:, k]).sum() >= 10]

    azimuth = offset + sense * 2 * math.pi * np.arange(columns) / columns
    spacing = configured[0] - configured[1]
    padded = np.concatenate([[configured[0] + spacing], configured, [configured[-1] - spacing]])  # edges' neighbours
    el, az = np.meshgrid(padded, azimuth, indexing="ij")
    rays = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], axis=-1).reshape(-1, 3)
    inner = slice(columns, (layers + 1) * columns)  # the rays the LiDAR has
    low, high, index = boxes(truth)
    lidar_t, lidar_q = np.array(lidar_link[0]), rotation(lidar_link[1])
    counts = {"returns": 0, "self": 0, "edge": 0, "placed": 0, "off": 0, "expected": 0, "missing": 0}
    face_on, walls_seen = [], np.zeros(len(truth["walls"]), dtype=int)
    for (base_t, base_q), scan in zip((pose for pose, _ in scans), points):
        base = rotation(base_q)
        origin = np.array(base_t) + base @ lidar_t
        expected, box, cosine = cast(origin, rays @ (base @ lidar_q).T, low, high)
        expected[(expected < lidar["min_range"]) | (expected > lidar["max_range"])] = np.inf
        edge = edges(expected.reshape(layers + 2, columns), box.reshape(layers + 2, columns)).reshape(-1)
        expected, box, cosine = expected[inner], box[inner], cosine[inner]
        counts["edge"] += int(edge.sum())
        flat = scan.reshape(-1)
        xyz = np.stack([flat["x"], flat["y"], flat["z"]], axis=-1).astype(float)
        returned = np.all(np.isfinite(xyz), axis=1)
        distance = np.where(returned, np.linalg.norm(np.where(returned[:, None], xyz, 0.0), axis=1), np.inf)
        own = returned & (distance < SELF_M)
        counts["self"] += int(own.sum())
        considered = returned & ~own & ~edge
        counts["returns"] += int(considered.sum())
        with np.errstate(invalid="ignore"):
            error = (distance - expected) * cosine
        placed = considered & np.isfinite(expected) & (np.abs(error) <= TOLERANCE_M)
        counts["placed"] += int(placed.sum())
        counts["off"] += int((considered & ~placed).sum())
        reachable = np.isfinite(expected) & (expected < lidar["max_range"] - 0.5) & ~edge
        counts["expected"] += int(reachable.sum())
        counts["missing"] += int((reachable & ~returned).sum())
        on = placed & (cosine >= FACE_ON)
        face_on.extend((distance[on] - expected[on]).tolist())
        hits = index[box[placed]]
        walls_seen += np.bincount(hits[hits >= 0], minlength=len(walls_seen))

    def share(part, whole):
        return round(part / whole, 5) if whole else None

    residual = float(np.sqrt(np.mean(np.square(face_on)))) if face_on else None
    report = {
        "scans": len(scans),
        **counts,
        "edge_share": share(counts["edge"], len(scans) * layers * columns),
        "placed_share": share(counts["placed"], counts["returns"]),
        "missing_share": share(counts["missing"], counts["expected"]),
        "range_rms_m": None if residual is None else round(residual, 4),
        "residual_over_noise": None if residual is None else round(residual / lidar["noise"], 3),
        "azimuth": {"sense": sense, "offset_deg": round(math.degrees(offset), 3),
                    "rms_deg": round(math.degrees(azimuth_rms), 4)},
        "elevation_error_deg": round(math.degrees(max(layer_error)), 4) if layer_error else None,
        "layers_checked": len(layer_error),
        "walls_seen": int((walls_seen >= 20).sum()),
        "walls": len(truth["walls"]),
    }
    report["problems"] = problems(report)
    return report


def problems(report):
    """What is wrong with a report, none if the LiDAR sees the maze truth.json describes."""
    found = []
    if report["returns"] < LIMITS["returns"]:
        found.append(f"{report['returns']} returns in {report['scans']} scans, fewer than {LIMITS['returns']}")
        return found
    if report["elevation_error_deg"] is None or report["elevation_error_deg"] > LIMITS["elevation_error_deg"]:
        found.append(f"the LiDAR's layers lie up to {report['elevation_error_deg']} deg from their configured angles")
    if report["azimuth"]["rms_deg"] > LIMITS["azimuth_error_deg"]:
        found.append(f"the LiDAR's columns scatter {report['azimuth']['rms_deg']} deg around evenly spaced azimuths")
    if report["placed_share"] < LIMITS["placed"]:
        found.append(f"{report['placed_share']:.2%} of the returns lie on a surface truth.json places on their ray "
                     f"({report['off']} off)")
    if report["missing_share"] is not None and report["missing_share"] > LIMITS["missing"]:
        found.append(f"{report['missing_share']:.2%} of the rays meeting a surface in range returned nothing "
                     f"({report['missing']} missing)")
    if report["residual_over_noise"] is None or not NOISE_BAND[0] <= report["residual_over_noise"] <= NOISE_BAND[1]:
        found.append(f"the range error on face-on surfaces is {report['residual_over_noise']}x the configured noise")
    return found

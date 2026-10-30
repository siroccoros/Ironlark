"""Check a recorded flight's sensors: the bag complete, the IMU at rest, the calibration wall in the empty world and the
surfaces in the forest.

Runs in the ROS 2 container on a run folder holding a bag (./ironlark check --record):

    ./ironlark check-recording runs/<run>

First, completeness: every topic is in the bag, each periodic one with a message every period
(config/sensors.json; /clock every physics step) and none missing between its first and last,
starting and ending within one period of the clock, and so is ground truth (truth.bin beside the bag,
every step, as many records as the stream's account wrote), over a flight that leaves the ground, climbs past 2.5 m and lands. Then the IMU
while the drone stands still before takeoff (ground truth speed under 1 cm/s, from 1 s on): it reads
gravity's push, up in the world by ground truth's attitude, and no rotation, each within its part's
tolerances (config/sensors.json), with its noise, and a level attitude. Every LiDAR frame: its points' times
within it and their lines the pattern's. In the empty world, the sensors against the calibration wall: for LiDAR
frames taken while hovering it places the returns in the world (ground truth pose at each point's instant and
the static transforms), fits the wall (face at x = 5.9 m, normal -x) and checks their distance, orientation and
scatter (the rig's range noise, and its angular noise along the ray), then projects them into the camera frame
taken at the frame's start: wall returns fall on the wall's red, and pixels seeing the ground in front of it are
not red. Frames taken while turning (the flight's half turn) scatter about the wall's face as little placed
point by point, and at least TIMING times as much placed all at the frame's start: the points' times are their
own. In the forest, the returns of those frames, placed alike, against the surfaces the flown world draws (its meshes at
every instance, tools/scene_surfaces.py): next to none lie farther from them than OFF_SURFACE times the rig's spread.
Writes sensor_check.json beside the bag; exits nonzero on failure.
"""
import json
import math
import re
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from sensing import frames, mid360, protocol  # noqa: E402

import numpy as np  # noqa: E402

WALL_X = 5.9  # the calibration wall's face (worlds/flight_foundation.wbt)
WALL = {"x": (5.0, 7.0), "y": (-2.2, 2.2), "z": (0.05, 3.0)}
LIMITS = {"distance_error_m": 0.05, "normal_error_deg": 2.0, "camera_agreement": 0.95, "background_red": 0.05,
          "off_surfaces": 0.001}
# The forest: returns sampled, and a return's distance from the nearest surface past which it lies off them, over the
# rig's spread. Edges blended into points between two surfaces put 4.6% past it
# (runs/20261030-040600-731000), none since (runs/20261030-044454-742139).
FOREST_SAMPLE, OFF_SURFACE = 20_000, 4.0
NOISE_BAND = (0.5, 1.5)  # measured spread over the rig's configured noise (config/sensors.json)
TIMING = 2.0  # turning at the Iris's 1.3 rad/s: 2.7 measured (runs/20261030-025342-691544)
TURNING_RAD_S, STILL_RAD_S = 1.0, 0.05
GRAVITY = 9.81
TOPICS = ["/clock", "/imu/data", "/lidar/points", "/camera/image_raw", "/camera/camera_info", "/tf_static"]
TRUTH = "truth.bin"  # not a topic: ground truth stays off the ROS graph
GROUND_M, CLIMB_M = 0.2, 2.5  # on the ground; past it, the flight climbed (tests/flight_smoke.py's bounds)


def periods(step_ms, config):
    """Each periodic topic's period in nanoseconds: the clock and ground truth every physics step
    (controllers/flight_bridge/sensor_stream.py)."""
    ms = {"/clock": step_ms, TRUTH: step_ms}
    for topics, sensor in ((("/imu/data",), "imu"),
                           (("/lidar/points",), "lidar"), (("/camera/image_raw", "/camera/camera_info"), "camera")):
        for topic in topics:
            ms[topic] = round(1000 / config[sensor]["rate_hz"])
    return {topic: value * 1_000_000 for topic, value in ms.items()}


def completeness(stamps, periods, heights, lags=None):
    """Problems with the recording's completeness, none if it is complete: stamps holds each topic's
    message stamps (ns) in bag order, heights the ground truth heights in order, lags how long after its stamp a
    topic's message can first be sent (ns; a LiDAR frame once whole)."""
    problems = []
    for topic in dict.fromkeys([*TOPICS, *periods]):
        if not stamps.get(topic):
            problems.append(f"{topic}: no messages")
    clock = stamps.get("/clock")
    for topic, period in periods.items():
        series = stamps.get(topic)
        if not series or not clock:
            continue
        steps = [b - a for a, b in zip(series, series[1:])]
        if any(step != period for step in steps):
            missing = sum(step // period - 1 for step in steps if step > period)
            problems.append(f"{topic}: {missing} missing and {sum(step < period for step in steps)} "
                            f"out of order or repeated, at {period // 1_000_000} ms")
        lag = (lags or {}).get(topic, 0)
        if not 0 <= series[0] - clock[0] < period or not 0 <= clock[-1] - series[-1] < period + lag:
            problems.append(f"{topic}: covers {series[0] / 1e9:.3f}-{series[-1] / 1e9:.3f} s, "
                            f"the clock {clock[0] / 1e9:.3f}-{clock[-1] / 1e9:.3f} s")
    if heights and not (heights[0] <= GROUND_M and max(heights) >= CLIMB_M and heights[-1] <= GROUND_M):
        problems.append(f"the flight is not all there: heights {heights[0]:.2f}, peak {max(heights):.2f}, "
                        f"last {heights[-1]:.2f} m")
    return problems


def imu_problems(samples, config):
    """Problems with the IMU at rest, none if it reads as its part may (config/sensors.json): gravity's push and no
    rotation, each off by no more than the part's bias, scale and cross-axis tolerances allow, spread by its noise,
    and the drone level. Samples are (acceleration, angular velocity, orientation) while the drone stood still, the
    orientation ground truth's for the IMU frame in the world (ENU), a quaternion (x, y, z, w)."""
    if len(samples) < 250:
        return [f"the IMU at rest: {len(samples)} samples, fewer than a second's"]
    imu, problems = config["imu"], []

    def mean(values):
        return sum(values) / len(values)

    def spread(values):
        centre = mean(values)
        return math.sqrt(sum((v - centre) ** 2 for v in values) / len(values))

    # At rest an accelerometer reads the ground's push, up in the world, and a gyro nothing.
    ideal = {"acceleration": [mean([rotate(inverse(q), (0.0, 0.0, GRAVITY))[k] for _, _, q in samples]) for k in range(3)],
             "angular velocity": [0.0, 0.0, 0.0]}
    for name, index, part, unit in (("acceleration", 0, imu["accelerometer"], "m/s^2"),
                                    ("angular velocity", 1, imu["gyro"], "rad/s")):
        for k, axis in enumerate("xyz"):
            values = [sample[index][k] for sample in samples]
            truth = ideal[name]
            allowed = (part["bias"] + part["scale"] * abs(truth[k])
                       + part["cross_axis"] * sum(abs(v) for j, v in enumerate(truth) if j != k)
                       + 4 * part["noise"] / math.sqrt(len(values)))
            if abs(off := mean(values) - truth[k]) > allowed:
                problems.append(f"the IMU's {name} {axis} at rest is {off:+.4f} {unit} off, beyond the part's "
                                f"{allowed:.4f}")
            if not NOISE_BAND[0] <= (ratio := spread(values) / part["noise"]) <= NOISE_BAND[1]:
                problems.append(f"the IMU's {name} {axis} at rest spreads {ratio:.2f}x its configured noise")
    tilt = max(math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (q[0] ** 2 + q[1] ** 2))))) for _, _, q in samples)
    if tilt > 2.0:
        problems.append(f"the IMU at rest is tilted {tilt:.1f} deg")
    return problems


def fit_wall(points):
    """The plane x = a + b y + c z through the wall's returns: its distance at their centre, its normal's angle from
    -x in degrees, and the returns' RMS distance from it in metres."""
    n = len(points)
    sy, sz = sum(p[1] for p in points), sum(p[2] for p in points)
    m = [[n, sy, sz], [sy, sum(p[1] ** 2 for p in points), sum(p[1] * p[2] for p in points)],
         [sz, sum(p[1] * p[2] for p in points), sum(p[2] ** 2 for p in points)]]
    v = [sum(p[0] for p in points), sum(p[0] * p[1] for p in points), sum(p[0] * p[2] for p in points)]
    a, b, c = solve3(m, v)
    angle = math.degrees(math.acos(1.0 / math.sqrt(1.0 + b * b + c * c)))
    distance = a + b * sy / n + c * sz / n
    residual = math.sqrt(sum((p[0] - (a + b * p[1] + c * p[2])) ** 2 for p in points) / n)
    return distance, angle, residual


def lags(config):
    """A LiDAR frame goes out once whole, half a render period after its end (sensing/mid360.py)."""
    lidar = config["lidar"]
    return {"/lidar/points": round(1000 / lidar["rate_hz"] + 500 / lidar["render_hz"]) * 1_000_000}


def frame_problems(points, config):
    """Problems with one LiDAR frame's points (structured: layer, time), none if they are the pattern's."""
    lidar = config["lidar"]
    if len(points) and not (0 <= points["time"].min() and points["time"].max() < 1 / lidar["rate_hz"]):
        return [f"/lidar/points: times {points['time'].min():.4f}-{points['time'].max():.4f} s, beyond the frame"]
    if len(points) and not set(points["layer"].tolist()) <= set(range(lidar["pattern"]["lines"])):
        return [f"/lidar/points: lines {sorted(set(points['layer'].tolist()))}"]
    return []


def forest_report(run, config, poses, frames_, place):
    """FOREST_SAMPLE returns of the frames (drawn alike every run), placed at their instants, against the surfaces
    the flown world draws (tools/scene_surfaces.py): their distances to the nearest, and those over the spread the
    rig's noise gives a return (its range noise, and its angular noise across the ray)."""
    import scene_surfaces

    lidar = config["lidar"]
    worlds = run / "worlds"
    wbt = next(worlds.glob("*.wbt"))
    folder = worlds if (worlds / "meshes").exists() else ROOT / "worlds" / wbt.stem  # its link may lead off the container
    world = np.concatenate([place(start, points)[0] for start, points in frames_])
    ranges = np.concatenate([np.sqrt(p["x"].astype(float) ** 2 + p["y"] ** 2 + p["z"] ** 2) for _, p in frames_])
    pick = np.random.default_rng(0).permutation(len(world))[:FOREST_SAMPLE]
    surfaces = scene_surfaces.Surfaces(wbt, [poses[start][0] for start, _ in frames_], lidar["max_range"], folder)
    distance = surfaces.distance(world[pick])
    over = distance / np.hypot(lidar["noise"], lidar["angular_noise"] * ranges[pick])
    return {"frames": len(frames_), "returns": len(world), "sampled": len(pick),
            "distance_m": {f"p{q}": round(float(np.percentile(distance, q)), 4) for q in (50, 90, 99, 99.9)},
            "over_spread": {f"p{q}": round(float(np.percentile(over, q)), 3) for q in (50, 90, 99, 99.9)},
            "off_surfaces": round(float(np.mean(over > OFF_SURFACE)), 5),
            "off_every_surface": round(float(np.mean(distance >= scene_surfaces.RADIUS)), 5)}


def forest_problems(report):
    """Problems with the forest's returns, none if they lie on the surfaces the world draws, within the rig's noise."""
    problems = []
    if report["frames"] < 10 or report["sampled"] < FOREST_SAMPLE:
        problems.append(f"{report['frames']} LiDAR frames hovering or turning, {report['sampled']} returns sampled: too few")
    elif report["off_surfaces"] > LIMITS["off_surfaces"]:
        problems.append(f"{report['off_surfaces']:.2%} of the LiDAR's returns lie off the forest's surfaces "
                        f"(beyond {OFF_SURFACE} times the rig's spread)")
    return problems


def wall_problems(report):
    """Problems with the sensors against the calibration wall, none if the LiDAR places it where it stands with the
    rig's range noise, and the camera sees its red where the LiDAR does, and the ground in front of it not red."""
    problems = []
    if report["distance_error_m"] > LIMITS["distance_error_m"]:
        problems.append(f"the LiDAR places the wall {report['distance_error_m']} m off")
    if report["normal_error_deg"] > LIMITS["normal_error_deg"]:
        problems.append(f"the LiDAR turns the wall {report['normal_error_deg']} deg")
    if not NOISE_BAND[0] <= report["residual_over_noise"] <= NOISE_BAND[1]:
        problems.append(f"the wall's returns scatter {report['residual_over_noise']}x the configured noise")
    turning = report["turning"]
    if turning["frames"] < 3:
        problems.append(f"{turning['frames']} LiDAR frames taken turning, fewer than 3")
    elif not NOISE_BAND[0] <= turning["residual_over_noise"] <= NOISE_BAND[1]:
        problems.append(f"turning, the wall's returns scatter {turning['residual_over_noise']}x the configured noise")
    elif turning["at_frame_start_over_noise"] < TIMING * turning["residual_over_noise"]:
        problems.append(f"turning, the wall's returns placed at their frame's start scatter only "
                        f"{turning['at_frame_start_over_noise']}x the configured noise: the points' times are not theirs")
    if report["camera_points"] < 100:
        problems.append(f"{report['camera_points']} wall returns in the camera's view, fewer than 100")
    elif report["camera_agreement"] < LIMITS["camera_agreement"]:
        problems.append(f"{report['camera_agreement']:.1%} of the wall's returns land on red in the camera")
    if report["ground_points"] < 100:
        problems.append(f"{report['ground_points']} pixels of the ground in front of the wall in view, fewer than 100")
    elif report["background_red"] > LIMITS["background_red"]:
        problems.append(f"{report['background_red']:.1%} of the ground in front of the wall looks red")
    return problems


def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def rotate(q, v):
    x, y, z, w = q
    r = q_mul(q_mul(q, (*v, 0.0)), (-x, -y, -z, w))
    return r[:3]


def inverse(q):
    return (-q[0], -q[1], -q[2], q[3])


def add(a, b):
    return tuple(p + q for p, q in zip(a, b))


def sub(a, b):
    return tuple(p - q for p, q in zip(a, b))


def solve3(m, v):
    """Solve a 3 x 3 linear system (Cramer)."""
    def det(a):
        return (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
                + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))
    d = det(m)
    out = []
    for k in range(3):
        mk = [row[:] for row in m]
        for i in range(3):
            mk[i][k] = v[i]
        out.append(det(mk) / d)
    return out


def main():
    from bag_reader import open_bag
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    run = Path(sys.argv[1])
    world = next((run / "worlds").glob("*.wbt")).read_text()
    step_ms = int(re.search(r"basicTimeStep (\d+)", world).group(1))
    config = json.loads((ROOT / "config/sensors.json").read_text())
    expected = periods(step_ms, config)
    calibration_wall = json.loads((run / "manifest.json").read_text())["world"] == "empty"
    truth = protocol.read_truth(run / TRUTH) if (run / TRUTH).exists() else []
    poses = {t: (position, orientation, math.sqrt(sum(v * v for v in velocity)))
             for t, position, orientation, velocity in truth}
    heights = [position[2] for _, position, _, _ in truth]
    # Takeoff: the first instant the drone leaves the ground. The IMU at rest is read before it.
    takeoff = next((t for t, position, _, _ in truth if position[2] > GROUND_M), math.inf)
    reader = open_bag(run / "bag")
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    static, info = {}, None
    images, stamps, at_rest = {}, {TRUTH: [t for t, _, _, _ in truth]}, []

    def ns(stamp):
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def yaw(q):
        x, y, z, w = q
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def motion(stamp):
        """At a LiDAR frame's start: 'hovering' (above 2 m, still), 'turning' (above 2 m) or None."""
        here, later = poses.get(stamp), poses.get(stamp + FRAME_NS)
        if not (here and later) or here[0][2] <= 2.0:
            return None
        rate = abs(math.remainder(yaw(later[1]) - yaw(here[1]), 2 * math.pi)) / (FRAME_NS * 1e-9)
        if rate > TURNING_RAD_S:
            return "turning"
        return "hovering" if here[2] < 0.3 and rate < STILL_RAD_S else None

    FRAME_NS = round(1000 / config["lidar"]["rate_hz"]) * 1_000_000
    scans, frame_issues, hovering_stamps = {"hovering": [], "turning": []}, [], set()
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == "/tf_static":
            stamps.setdefault(topic, []).append(None)  # counted; its transforms carry the stamps
        else:
            # Every other message starts with its stamp: a CDR header (4 bytes), then seconds and nanoseconds.
            sec, nanosec = struct.unpack_from("<iI", data, 4)
            stamps.setdefault(topic, []).append(sec * 1_000_000_000 + nanosec)
        stamp = stamps[topic][-1]
        pose = poses.get(stamp) if topic != "/tf_static" else None
        # Stamps only, but for the IMU while the drone stands still before takeoff, LiDAR frames and images at the
        # first hovering frames' instants.
        if topic == "/clock" or (topic == "/imu/data" and not (
                pose and 1e9 <= stamp < takeoff and pose[2] < 0.01)) or (
                topic == "/camera/camera_info" and info is not None) or (
                topic == "/camera/image_raw" and not (
                    stamp % FRAME_NS == 0 and motion(stamp) == "hovering" and len(images) < 20)):
            continue
        msg = deserialize_message(data, get_message(types[topic]))
        if topic == "/tf_static":
            for t in msg.transforms:
                r, tr = t.transform.rotation, t.transform.translation
                static[t.child_frame_id] = ((tr.x, tr.y, tr.z), (r.x, r.y, r.z, r.w))
        elif topic == "/camera/camera_info":
            info = msg
        elif topic == "/imu/data":
            a, g = msg.linear_acceleration, msg.angular_velocity
            at_rest.append(((a.x, a.y, a.z), (g.x, g.y, g.z), pose[1]))  # the body's attitude: the mount follows
        elif topic == "/lidar/points":
            points = np.frombuffer(bytes(msg.data), mid360.POINT)
            frame_issues += frame_problems(points, config)
            kind = motion(ns(msg.header.stamp))
            if kind and len(scans[kind]) < 20:
                scans[kind].append((ns(msg.header.stamp), points))
        elif topic == "/camera/image_raw":
            images[ns(msg.header.stamp)] = msg

    # The IMU frame's attitude: the body's, turned by the mount the bag publishes.
    imu_mount = static.get("imu_link", (None, (0.0, 0.0, 0.0, 1.0)))[1]
    at_rest = [(a, g, frames.quaternion_multiply(q, imu_mount)) for a, g, q in at_rest]
    written = json.loads((run / "sensing.json").read_text()).get("truth_records")
    problems = (completeness(stamps, expected, heights, lags(config)) or imu_problems(at_rest, config)
                + sorted(set(frame_issues))[:5])
    if written != len(truth):
        problems.insert(0, f"{TRUTH}: {len(truth)} records, the stream wrote {written}")
    if problems:
        report = {"complete": not completeness(stamps, expected, heights, lags(config)),
                  "imu_samples_at_rest": len(at_rest), "problems": problems, "passed": False}
        (run / "sensor_check.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2))
        sys.exit(1)
    lidar_t, lidar_q = static["lidar_link"]
    camera_t, camera_q = static["camera_link"]
    optical_q = static["camera_optical_frame"][1]
    fx, cx, fy, cy = info.k[0], info.k[2], info.k[4], info.k[5]
    truth_ns = np.array([t for t, _, _, _ in truth])
    truth_p = np.array([p for _, p, _, _ in truth])
    truth_q = np.array([q for _, _, q, _ in truth])
    mount_r = mid360.rotations(np.array([lidar_q]))[0]

    def place(start, points, own_instant=True):
        """Points in the world (rows x, y, z) and their rays' directions, with the body's pose at each point's
        instant (or all at the frame's start)."""
        t = start + np.round(points["time"].astype(float) * 1e9 * own_instant).astype(np.int64)
        i = np.clip(np.searchsorted(truth_ns, t, side="right") - 1, 0, len(truth_ns) - 2)
        f = ((t - truth_ns[i]) / (truth_ns[i + 1] - truth_ns[i]))[:, None]
        a, b = truth_q[i], truth_q[i + 1]
        b = np.where(np.sum(a * b, axis=1, keepdims=True) < 0, -b, b)
        q = a * (1 - f) + b * f
        body = mid360.rotations(q / np.linalg.norm(q, axis=1, keepdims=True))
        local = np.column_stack([points["x"], points["y"], points["z"]]).astype(float)
        sensor = local @ mount_r.T + np.array(lidar_t)
        origin = truth_p[i] * (1 - f) + truth_p[i + 1] * f
        world = origin + np.einsum("nij,nj->ni", body, sensor)
        rays = np.einsum("nij,nj->ni", body, local @ mount_r.T)
        return world, rays / np.linalg.norm(rays, axis=1, keepdims=True)

    if not calibration_wall:  # the forest: its returns against the surfaces the world draws
        report = {"complete": True,
                  "messages": {topic: len(stamps.get(topic, ())) for topic in [*TOPICS, TRUTH]},
                  "imu_samples_at_rest": len(at_rest),
                  "forest": forest_report(run, config, poses, [*scans["hovering"], *scans["turning"]], place)}
        report["problems"] = forest_problems(report["forest"])
        report["passed"] = not report["problems"]
        (run / "sensor_check.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2))
        sys.exit(0 if report["passed"] else 1)

    def on_wall(world):
        return np.all([(world[:, k] >= lo) & (world[:, k] <= hi)
                       for k, (lo, hi) in enumerate((WALL["x"], WALL["y"], WALL["z"]))], axis=0)

    def scatter(frames_):
        """Wall returns (placed at their instants), their fit and scatter about it over the scatter the rig's noise
        gives them, and over that noise their scatter about the wall's face itself, placed at their instants and all
        at their frame's start (a frame sees the wall within a few milliseconds of its nod: a turn then tilts the
        returns of one frame together, which a fitted plane would take up)."""
        lidar = config["lidar"]
        walls, starts, expected = [], [], []
        for start, points in frames_:
            world, rays = place(start, points)
            keep = on_wall(world)
            ranges = np.sqrt(points["x"].astype(float) ** 2 + points["y"] ** 2 + points["z"] ** 2)[keep]
            # Normal to the wall, range noise along the ray; angular noise moves the hit along the ray by r tan(i).
            facing = np.abs(rays[keep, 0])
            expected.append((lidar["noise"] * facing) ** 2
                            + (lidar["angular_noise"] * ranges * np.sqrt(1 - facing ** 2)) ** 2)
            walls.append(world[keep])
            starts.append(place(start, points[keep], own_instant=False)[0])
        wall = np.concatenate(walls) if walls else np.empty((0, 3))
        if len(wall) < 100:
            return wall, None, None, None, None
        noise = math.sqrt(np.mean(np.concatenate(expected)))
        fit = fit_wall(wall.tolist())
        about_face = [round(math.sqrt(np.mean((w[:, 0] - WALL_X) ** 2)) / noise, 3) for w in (wall, np.concatenate(starts))]
        return wall, fit, round(fit[2] / noise, 3), *about_face

    def red(world, base_t, base_q, image):
        """Whether the camera saw red where a world point lies, or None outside its view."""
        # World to base, to camera_link, to the optical frame (x right, y down, z forward).
        link = sub(rotate(inverse(base_q), sub(world, base_t)), camera_t)
        ox, oy, oz = rotate(inverse(optical_q), rotate(inverse(camera_q), link))
        if oz <= 0.1:
            return None
        u, v = int(fx * ox / oz + cx), int(fy * oy / oz + cy)
        if not (0 <= u < image.width and 0 <= v < image.height):
            return None
        b, g, r = image.data[v * image.step + 4 * u: v * image.step + 4 * u + 3]
        return r > 60 and r > 1.6 * g and r > 1.6 * b

    def ground(base_t, base_q, image):
        """Whether the camera saw red, for a grid of pixels whose rays meet the green ground in front of the wall."""
        origin = add(rotate(base_q, camera_t), base_t)
        colours = []
        for v in range(0, image.height, 16):
            for u in range(0, image.width, 16):
                ray = rotate(base_q, rotate(camera_q, rotate(optical_q, ((u - cx) / fx, (v - cy) / fy, 1.0))))
                if ray[2] >= 0:
                    continue
                x, y, _ = add(origin, tuple(c * -origin[2] / ray[2] for c in ray))
                if 0.5 < x < WALL["x"][0] - 0.2 and abs(y) < 3.0:
                    b, g, r = image.data[v * image.step + 4 * u: v * image.step + 4 * u + 3]
                    colours.append(r > 60 and r > 1.6 * g and r > 1.6 * b)
        return colours

    wall, fit, over_noise, _, _ = scatter(scans["hovering"])
    if fit is None:
        raise SystemExit(f"Too few wall returns ({len(wall)}) in {len(scans['hovering'])} hovering frames.")
    distance, angle, residual = fit
    seen = {"wall": [], "ground": []}
    for start, points in scans["hovering"]:
        image = images.get(start)
        if image is None:
            continue
        base_t, base_q = poses[start][:2]
        seen["ground"] += ground(base_t, base_q, image)
        world, _ = place(start, points)
        for point in world[on_wall(world)].tolist():
            if (colour := red(tuple(point), base_t, base_q, image)) is not None:
                seen["wall"].append(colour)
    _, _, _, turning, at_start = scatter(scans["turning"])

    def share(colours):
        return round(sum(colours) / len(colours), 4) if colours else None

    report = {
        "complete": True,
        "messages": {topic: len(stamps.get(topic, ())) for topic in [*TOPICS, TRUTH]},
        "imu_samples_at_rest": len(at_rest),
        "hovering_frames": len(scans["hovering"]),
        "wall_returns": len(wall),
        "wall_distance_m": round(distance, 4),
        "distance_error_m": round(abs(distance - WALL_X), 4),
        "normal_error_deg": round(angle, 3),
        "plane_residual_m": round(residual, 4),
        "residual_over_noise": over_noise,
        "turning": {"frames": len(scans["turning"]), "residual_over_noise": turning or 0.0,
                    "at_frame_start_over_noise": at_start or 0.0},
        "camera_points": len(seen["wall"]),
        "camera_agreement": share(seen["wall"]),
        "ground_points": len(seen["ground"]),
        "background_red": share(seen["ground"]),
    }
    report["problems"] = wall_problems(report)
    report["passed"] = not report["problems"]
    (run / "sensor_check.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    sys.exit(0 if report["passed"] else 1)


if __name__ == "__main__":
    main()

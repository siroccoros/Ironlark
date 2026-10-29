"""Check a recorded flight's sensors: the bag complete, the IMU at rest, and in the empty world the calibration wall.

Runs in the ROS 2 container on a run folder holding a bag (./ironlark check --record):

    ./ironlark check-recording runs/<run>

First, completeness: every topic is in the bag, each periodic one with a message every period
(config/sensors.json; /clock every physics step) and none missing between its first and last,
starting and ending within one period of the clock, and so is ground truth (truth.bin beside the bag,
every step, as many records as the stream's account wrote), over a flight that leaves the ground, climbs past 2.5 m and lands. Then the IMU
while the drone stands still before takeoff (ground truth speed under 1 cm/s, from 1 s on): it reads
gravity, pointing up in the world by ground truth's attitude, no rotation and a level attitude, with
the rig's configured noise. In the empty world, the sensors against the calibration wall: for LiDAR scans
taken while hovering it places the returns in the world (ground truth pose and the static
transforms), fits the wall (face at x = 5.9 m, normal -x) and checks their distance, orientation
and scatter (the rig's range noise), then projects them into the camera frame taken at the same
instant: wall returns fall on the wall's red, and pixels seeing the ground in front of it are not red. Writes
sensor_check.json beside the bag; exits nonzero on failure.
"""
import json
import math
import re
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from sensing import frames, protocol  # noqa: E402

WALL_X = 5.9  # the calibration wall's face (worlds/flight_foundation.wbt)
WALL = {"x": (5.0, 7.0), "y": (-2.2, 2.2), "z": (0.05, 3.0)}
LIMITS = {"distance_error_m": 0.05, "normal_error_deg": 2.0, "camera_agreement": 0.95, "background_red": 0.05}
NOISE_BAND = (0.5, 1.5)  # measured spread over the rig's configured noise (config/sensors.json)
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


def completeness(stamps, periods, heights):
    """Problems with the recording's completeness, none if it is complete: stamps holds each topic's
    message stamps (ns) in bag order, heights the ground truth heights in order."""
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
        if not 0 <= series[0] - clock[0] < period or not 0 <= clock[-1] - series[-1] < period:
            problems.append(f"{topic}: covers {series[0] / 1e9:.3f}-{series[-1] / 1e9:.3f} s, "
                            f"the clock {clock[0] / 1e9:.3f}-{clock[-1] / 1e9:.3f} s")
    if heights and not (heights[0] <= GROUND_M and max(heights) >= CLIMB_M and heights[-1] <= GROUND_M):
        problems.append(f"the flight is not all there: heights {heights[0]:.2f}, peak {max(heights):.2f}, "
                        f"last {heights[-1]:.2f} m")
    return problems


def imu_problems(samples, config):
    """Problems with the IMU at rest, none if it reads gravity where its attitude puts it, no rotation, level, and
    the configured noise: samples are (acceleration, angular velocity, orientation) while the drone stood still, the
    orientation ground truth's for the IMU frame in the world (ENU), a quaternion (x, y, z, w)."""
    if len(samples) < 250:
        return [f"the IMU at rest: {len(samples)} samples, fewer than a second's"]
    imu, problems = config["imu"], []

    def mean(values):
        return sum(values) / len(values)

    def spread(values):
        centre = mean(values)
        return math.sqrt(sum((v - centre) ** 2 for v in values) / len(values))

    gravity = mean([math.sqrt(sum(c * c for c in accel)) for accel, _, _ in samples])
    if abs(gravity - GRAVITY) > 0.05:
        problems.append(f"the IMU at rest reads {gravity:.3f} m/s^2 of gravity")
    # At rest an accelerometer reads the ground's push, up in the world: (0, 0, g) in the IMU frame when level.
    read = [mean([accel[k] for accel, _, _ in samples]) for k in range(3)]
    up = [mean([rotate(inverse(q), (0.0, 0.0, 1.0))[k] for _, _, q in samples]) for k in range(3)]
    cosine = sum(r * u for r, u in zip(read, up)) / (math.sqrt(sum(r * r for r in read) * sum(u * u for u in up)) or 1)
    off = math.degrees(math.acos(max(-1.0, min(1.0, cosine))))
    if off > 0.5:
        problems.append(f"the IMU at rest reads gravity {off:.1f} deg from where its attitude puts it")
    rotation = [mean([gyro[k] for _, gyro, _ in samples]) for k in range(3)]
    if max(map(abs, rotation)) > 0.001:
        problems.append(f"the IMU at rest turns at {max(map(abs, rotation)):.4f} rad/s")
    tilt = max(math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (q[0] ** 2 + q[1] ** 2))))) for _, _, q in samples)
    if tilt > 2.0:
        problems.append(f"the IMU at rest is tilted {tilt:.1f} deg")
    for name, index, noise in (("acceleration", 0, imu["accelerometer_noise"]),
                               ("angular velocity", 1, imu["gyro_noise"])):
        for k, axis in enumerate("xyz"):
            ratio = spread([sample[index][k] for sample in samples]) / noise
            if not NOISE_BAND[0] <= ratio <= NOISE_BAND[1]:
                problems.append(f"the IMU's {name} {axis} at rest spreads {ratio:.2f}x its configured noise")
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


def wall_problems(report):
    """Problems with the sensors against the calibration wall, none if the LiDAR places it where it stands with the
    rig's range noise, and the camera sees its red where the LiDAR does, and the ground in front of it not red."""
    problems = []
    if report["distance_error_m"] > LIMITS["distance_error_m"]:
        problems.append(f"the LiDAR places the wall {report['distance_error_m']} m off")
    if report["normal_error_deg"] > LIMITS["normal_error_deg"]:
        problems.append(f"the LiDAR turns the wall {report['normal_error_deg']} deg")
    if not NOISE_BAND[0] <= report["residual_over_noise"] <= NOISE_BAND[1]:
        problems.append(f"the wall's returns scatter {report['residual_over_noise']}x the configured range noise")
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
    scans, images, stamps, at_rest = [], {}, {TRUTH: [t for t, _, _, _ in truth]}, []

    def ns(stamp):
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == "/tf_static":
            stamps.setdefault(topic, []).append(None)  # counted; its transforms carry the stamps
        else:
            # Every other message starts with its stamp: a CDR header (4 bytes), then seconds and nanoseconds.
            sec, nanosec = struct.unpack_from("<iI", data, 4)
            stamps.setdefault(topic, []).append(sec * 1_000_000_000 + nanosec)
        # Stamps only, but for the IMU while the drone stands still before takeoff and images at a scan's instant.
        pose = poses.get(stamps[topic][-1]) if topic != "/tf_static" else None
        if topic == "/clock" or (topic == "/imu/data" and not (
                pose and 1e9 <= stamps[topic][-1] < takeoff and pose[2] < 0.01)) or (
                topic == "/camera/image_raw" and stamps[topic][-1] not in {s[0] for s in scans}):
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
        elif topic == "/lidar/points" and pose and pose[0][2] > 2.0 and pose[2] < 0.3 and len(scans) < 20:
            scans.append((ns(msg.header.stamp), pose[:2], msg))
        elif topic == "/camera/image_raw":
            images[ns(msg.header.stamp)] = msg

    # The IMU frame's attitude: the body's, turned by the mount the bag publishes.
    imu_mount = static.get("imu_link", (None, (0.0, 0.0, 0.0, 1.0)))[1]
    at_rest = [(a, g, frames.quaternion_multiply(q, imu_mount)) for a, g, q in at_rest]
    written = json.loads((run / "sensing.json").read_text()).get("truth_records")
    problems = completeness(stamps, expected, heights) or imu_problems(at_rest, config)
    if written != len(truth):
        problems.insert(0, f"{TRUTH}: {len(truth)} records, the stream wrote {written}")
    if problems or not calibration_wall:  # the forest has no calibration wall
        report = {"complete": not completeness(stamps, expected, heights), "imu_samples_at_rest": len(at_rest),
                  "problems": problems, "passed": not problems}
        (run / "sensor_check.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2))
        sys.exit(1 if problems else 0)
    lidar_t, lidar_q = static["lidar_link"]
    camera_t, camera_q = static["camera_link"]
    optical_q = static["camera_optical_frame"][1]
    fx, cx, fy, cy = info.k[0], info.k[2], info.k[4], info.k[5]
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

    wall, seen = [], {"wall": [], "ground": []}
    for time_ns, (base_t, base_q), msg in scans:
        image = images.get(time_ns)
        if image is not None:
            seen["ground"] += ground(base_t, base_q, image)
        for i in range(msg.width):
            x, y, z, _, _ = struct.unpack_from("<fffif", msg.data, i * msg.point_step)
            if not all(math.isfinite(c) for c in (x, y, z)):
                continue
            world = add(rotate(base_q, add(rotate(lidar_q, (x, y, z)), lidar_t)), base_t)
            if not all(lo <= c <= hi for c, (lo, hi) in zip(world, (WALL["x"], WALL["y"], WALL["z"]))):
                continue
            wall.append(world)
            if image is not None and (colour := red(world, base_t, base_q, image)) is not None:
                seen["wall"].append(colour)
    if len(wall) < 100:
        raise SystemExit(f"Too few wall returns ({len(wall)}) in {len(scans)} hovering scans.")
    distance, angle, residual = fit_wall(wall)
    def share(colours):
        return round(sum(colours) / len(colours), 4) if colours else None

    report = {
        "complete": True,
        "messages": {topic: len(stamps.get(topic, ())) for topic in [*TOPICS, TRUTH]},
        "imu_samples_at_rest": len(at_rest),
        "hovering_scans": len(scans),
        "wall_returns": len(wall),
        "wall_distance_m": round(distance, 4),
        "distance_error_m": round(abs(distance - WALL_X), 4),
        "normal_error_deg": round(angle, 3),
        "plane_residual_m": round(residual, 4),
        "residual_over_noise": round(residual / config["lidar"]["noise"], 3),
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

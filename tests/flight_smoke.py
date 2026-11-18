"""Execute and evaluate the simulated takeoff/hover/land integration gate."""

import argparse
from collections import deque
import json
import math
from pathlib import Path
import sys
import time

from pymavlink import mavutil

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from sensing import frames  # noqa: E402
from tools.evaluate_batch import escape_index  # noqa: E402


EXPLORATION_LIMIT_S = 1500
UNHEALTHY_CHECKS_BEFORE_LAND = 4  # consecutive 0.5 s health checks
# Crossing an exit gate, only the outer wall is left behind the drone: odometry turns degenerate
# before the explorer can commit (it commits only on a healthy estimate). Seed 135227 lost 6 s
# there (runs/20261115-163215-13755, degenerate from 170 deg open); inside nine batch mazes no
# uncommitted reading exceeded 120 deg on either radius. The grace is bounded per flight.
OPENING_GRACE_SECTOR_DEG = 150
OPENING_GRACE_CHECKS = 16  # 8 s of IMU coasting, at most, over the whole flight
DRONE_REACH_M = 0.33  # half the diagonal of the drone's 0.47 m collision box: nearer a wall, it can touch it


def clearance(truth, walls):
    """The drone's least horizontal distance from a wall's footprint while below the wall's top, in metres; None
    without walls. Walls are truth.json's boxes (tools/generate_maze.py), read only after the flight."""
    nearest = None
    for sample in truth:
        x, y, z = sample["position"]
        for wall in walls:
            (cx, cy, cz), (sx, sy, sz) = wall["centre"], wall["size"]
            if z > cz + sz / 2:
                continue
            d = math.hypot(max(abs(x - cx) - sx / 2, 0.0), max(abs(y - cy) - sy / 2, 0.0))
            nearest = d if nearest is None else min(nearest, d)
    return nearest


def limit_xy_acceleration(previous, target, dt, acceleration=0.2):
    """Limit change in a horizontal velocity command to metres per second squared."""
    change = math.dist(previous, target)
    limit = acceleration * max(0.0, dt)
    if change <= limit:
        return target
    if limit == 0:
        return previous
    return tuple(old + (new - old) * limit / change for old, new in zip(previous, target))


def judge(truth, autopilot_seconds, altitude=3.0, walls=(), external_nav_route=False, route_points=(),
          escape_route=False):
    """The flight as Webots measured it (flight_observer's ground truth, every 100 ms of simulated time), never as the
    autopilot reported it: the trajectory covers the flight without a gap, and shows a takeoff, a continuous 5-second
    hover within 0.5 m of the altitude, a landing back on the ground (z = 0 in every world), no drift and, among
    walls, never nearer one than the drone's reach."""
    if not truth:
        raise RuntimeError("No independent Webots trajectory was recorded")
    times = [sample["time"] for sample in truth]
    gaps = [(a, b) for a, b in zip(times, times[1:]) if b - a > 0.1 + 1e-6]
    if times[0] > 0.2 or gaps:
        first = f", the first from {gaps[0][0]:.1f} to {gaps[0][1]:.1f} s" if gaps else ""
        raise RuntimeError(f"The Webots trajectory does not cover the flight: starts at {times[0]:.1f} s, "
                           f"{len(gaps)} gaps{first}")
    # The streams are sampled independently at 10 Hz. A one-second allowance
    # accommodates startup/sampling offsets while catching backend clock jumps.
    clock_difference = abs(autopilot_seconds - times[-1])
    if clock_difference > 1:
        raise RuntimeError(f"Autopilot/Webots clock mismatch: {clock_difference:.2f} s")
    heights = [sample["position"][2] for sample in truth]
    peak = max(heights)
    low, high = altitude - 0.5, altitude + 0.5
    hover, entered = 0.0, None  # the longest stay in the band, and when the current one began
    for at, height in zip(times, heights):
        if not low <= height <= high:
            entered = None
            continue
        entered = at if entered is None else entered
        hover = max(hover, at - entered)
    start_xy = truth[0]["position"][:2]
    drift = max(math.hypot(sample["position"][0] - start_xy[0],
                           sample["position"][1] - start_xy[1]) for sample in truth)
    if not low <= peak <= altitude + 1:
        raise RuntimeError(f"Physical takeoff height out of bounds: {peak:.2f} m")
    if hover < 5:
        raise RuntimeError(f"No physical 5-second hover at {low:g}-{high:g} m: the longest lasted {hover:.1f} s")
    if not -0.2 <= heights[-1] <= 0.2:
        raise RuntimeError(f"Drone did not physically land: final height {heights[-1]:.2f} m")
    if not route_points and not escape_route and drift > 1.5:
        raise RuntimeError(f"Excessive horizontal drift: {drift:.2f} m")
    nearest = clearance(truth, walls)
    if nearest is not None and nearest < DRONE_REACH_M:
        raise RuntimeError(f"The drone came within {nearest:.2f} m of a wall, inside its {DRONE_REACH_M} m reach")
    if external_nav_route:
        forward = max(sample["position"][0] - start_xy[0] for sample in truth)
        lateral = max(abs(sample["position"][1] - start_xy[1]) for sample in truth)
        final_drift = math.hypot(truth[-1]["position"][0] - start_xy[0],
                                 truth[-1]["position"][1] - start_xy[1])
        if forward < 0.35 or lateral > 0.4 or final_drift > 0.6:
            raise RuntimeError(f"External-navigation route missed physically: forward {forward:.2f} m, "
                               f"lateral {lateral:.2f} m, final drift {final_drift:.2f} m")
    if route_points:
        cursor = 0
        for i, point in enumerate(route_points, 1):
            reached = next((j for j in range(cursor, len(truth))
                            if math.hypot(truth[j]["position"][0] - point[0],
                                          truth[j]["position"][1] - point[1]) < 0.4), None)
            if reached is None:
                raise RuntimeError(f"Physical odometry route missed waypoint {i}: {point}")
            cursor = reached
        if math.hypot(truth[-1]["position"][0] - route_points[-1][0],
                      truth[-1]["position"][1] - route_points[-1][1]) > 0.6:
            raise RuntimeError("Physical odometry route landed away from its final waypoint")
    measured = {"peak_height_m": peak, "hover_s": round(hover, 1), "final_height_m": heights[-1],
                "max_horizontal_drift_m": drift, "clock_difference_s": clock_difference}
    if nearest is not None:
        measured["min_wall_clearance_m"] = round(nearest, 3)
    if external_nav_route:
        measured.update({"route_forward_m": round(forward, 3), "route_lateral_m": round(lateral, 3),
                         "route_final_drift_m": round(final_drift, 3)})
    if route_points:
        measured["route_length_m"] = round(sum(math.hypot(b["position"][0] - a["position"][0],
                                                  b["position"][1] - a["position"][1])
                                                 for a, b in zip(truth, truth[1:])), 3)
        measured["route_waypoints_reached"] = len(route_points)
    return measured


class Flight:
    def __init__(self, output, altitude=3.0, external_nav=False, odometry_route=False, gps_denied=False,
                 explore=False, inject_link_loss=False, inject_unhealthy_odometry=False):
        self.output = output
        self.altitude = altitude
        self.external_nav = external_nav
        self.route_points = (json.loads((ROOT / "config/odometry_route_seed7.json").read_text())
                             ["waypoints_webots_xy_m"] if odometry_route or (gps_denied and not explore
                                                                            and not inject_link_loss
                                                                            and not inject_unhealthy_odometry) else ())
        self.gps_denied = gps_denied
        self.explore = explore
        self.inject_link_loss = inject_link_loss
        self.inject_unhealthy_odometry = inject_unhealthy_odometry
        self.gps_fix_seen = False
        self.connection = None
        self.messages = {}
        self.events = []
        self.log = output.with_name("telemetry.jsonl").open("w", buffering=1)
        self.started = time.monotonic()
        self.last_heartbeat = 0
        self.sim_time = None  # autopilot boot time in seconds; tracks simulated time
        self.armed = False
        self.health_check_time = None
        self.monitor_odometry = True
        self.stale_odometry_checks = 0
        self.unhealthy_odometry_checks = 0
        self.opening_grace_checks = 0
        self.odometry_unhealthy = False

    def event(self, state):
        print(state, flush=True)
        self.events.append({"state": state, "wall_seconds": time.monotonic() - self.started})
        # Presentation consumes this state; it cannot send commands back.
        status = self.output.with_name("flight-state.json")
        temporary = status.with_suffix(".tmp")
        temporary.write_text(json.dumps({"state": state}) + "\n")
        temporary.replace(status)

    def poll(self):
        now = time.monotonic()
        if now - self.last_heartbeat >= 1:
            self.connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.last_heartbeat = now
        message = self.connection.recv_match(blocking=True, timeout=0.2)
        if message and message.get_type() != "BAD_DATA":
            if message.get_type() == "GPS_RAW_INT" and message.fix_type >= 3:
                self.gps_fix_seen = True
            if message.get_type() == "HEARTBEAT" and message.get_srcSystem() == self.connection.target_system:
                self.armed = bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.messages[message.get_type()] = message
            boot = getattr(message, "time_boot_ms", None)
            if boot is not None:
                self.sim_time = boot / 1000
            self.log.write(json.dumps(message.to_dict()) + "\n")
            if message.get_type() == "STATUSTEXT":
                print(f"  ArduPilot: {message.text}", flush=True)
            if self.gps_denied and self.armed and self.monitor_odometry:
                self.check_odometry_health()
        return message

    def check_odometry_health(self):
        if self.sim_time is None or (self.health_check_time is not None
                                     and self.sim_time - self.health_check_time < 0.5):
            return
        self.health_check_time = self.sim_time
        path = self.output.with_name("odometry.jsonl")
        if not path.exists():
            raise RuntimeError("GPS-denied odometry stream disappeared")
        with path.open() as handle:
            recent = [json.loads(line) for line in deque(handle, maxlen=5)]
        age = self.sim_time - recent[-1]["time"] if recent else math.inf
        self.stale_odometry_checks = self.stale_odometry_checks + 1 if age > 1.0 else 0
        if age > 2.5 and self.stale_odometry_checks >= 3:
            raise RuntimeError("GPS-denied odometry stream became stale")
        self.odometry_unhealthy = len(recent) == 5 and not any(item["healthy"] for item in recent)
        # Beyond the opening there may be no walls to match. EKF3 can coast
        # briefly while the already committed exit maneuver completes.
        if self.odometry_unhealthy:
            target = self.output.with_name("navigation_target.json")
            if target.exists():
                nav = json.loads(target.read_text())
                if (self.sim_time - nav["time"] < 2.0 and nav.get("exit_commitment")
                        and nav.get("open_sector_deg", 0) >= 220
                        and nav.get("exit_commitment_age_s") is not None
                        and nav["exit_commitment_age_s"] <= 20):
                    self.unhealthy_odometry_checks = 0
                    return
                if (self.sim_time - nav["time"] < 2.0
                        and recent[-1]["reason"] == "corridor degeneracy"
                        and nav.get("open_sector_deg", 0) >= OPENING_GRACE_SECTOR_DEG
                        and (nav.get("open_sector_6m_deg") or 0) >= OPENING_GRACE_SECTOR_DEG
                        and self.opening_grace_checks < OPENING_GRACE_CHECKS):
                    self.opening_grace_checks += 1
                    self.unhealthy_odometry_checks = 0
                    return
        self.unhealthy_odometry_checks = (self.unhealthy_odometry_checks + 1
                                          if self.odometry_unhealthy else 0)
        # Away from the exit, in-flight unhealthy streaks lasted at most 0.4 s; without a pose,
        # EKF3 coasts on the IMU and the drone drifts quadratically (an injected loss drifted
        # 0.45 m in 5.6 s), so land after about 2 s.
        if self.unhealthy_odometry_checks >= UNHEALTHY_CHECKS_BEFORE_LAND:
            raise RuntimeError("GPS-denied odometry became unhealthy: " + recent[-1]["reason"])

    def fail_safe_land(self):
        """Keep the simulation alive until LAND disarms when a mission fails airborne."""
        if not self.armed or self.connection is None:
            return
        self.monitor_odometry = False
        self.event("FAILSAFE_LANDING")
        try:
            self.connection.mav.command_long_send(self.connection.target_system,
                self.connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 9, 0, 0, 0, 0, 0)
            self.wait("failsafe landing and disarm", lambda m: m.get_type() == "HEARTBEAT"
                      and not m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED, 60)
            self.event("FAILSAFE_LANDED")
        except Exception as exc:
            self.event(f"FAILSAFE_LANDING_FAILED: {exc}")

    def expired(self, sim_start, wall_start, timeout):
        """Timeouts run on simulated time, so a slow renderer cannot fail a
        correct flight. A wall-clock backstop still catches a stalled simulator."""
        if self.sim_time is not None and sim_start is not None and self.sim_time - sim_start > timeout:
            return True
        return time.monotonic() - wall_start > max(4 * timeout, 60)

    def wait(self, description, predicate, timeout=30):
        sim_start, wall_start = self.sim_time, time.monotonic()
        while not self.expired(sim_start, wall_start, timeout):
            message = self.poll()
            if sim_start is None:
                sim_start = self.sim_time
            if message and predicate(message):
                return message
        raise RuntimeError(f"Timeout: {description}")

    def command(self, command, *params):
        self.connection.mav.command_long_send(
            self.connection.target_system, self.connection.target_component,
            command, 0, *(list(params) + [0] * (7 - len(params))))
        ack = self.wait(f"command acknowledgement {command}",
                        lambda m: m.get_type() == "COMMAND_ACK" and m.command == command, 10)
        if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            # The explanatory STATUSTEXT can arrive after the acknowledgement.
            deadline = time.monotonic() + 0.5
            while time.monotonic() < deadline:
                self.poll()
            status = self.messages.get("STATUSTEXT")
            detail = f": {status.text}" if status else ""
            raise RuntimeError(f"Command {command} rejected with result {ack.result}{detail}")

    def mode(self, value):
        self.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                     mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, value)
        self.wait(f"mode {value}", lambda m: m.get_type() == "HEARTBEAT" and m.custom_mode == value)

    def connect(self):
        self.event("CONNECTING")
        deadline = time.monotonic() + 45
        while time.monotonic() < deadline:
            try:
                self.connection = mavutil.mavlink_connection("tcp:127.0.0.1:5760",
                                                             source_system=255, retries=0)
                break
            except OSError:
                time.sleep(0.5)
        if self.connection is None:
            raise RuntimeError("SITL did not expose MAVLink on loopback port 5760")
        heartbeat = self.wait("autopilot heartbeat", lambda m: m.get_type() == "HEARTBEAT", 45)
        self.connection.target_system = heartbeat.get_srcSystem()
        self.connection.target_component = heartbeat.get_srcComponent()
        self.connection.mav.request_data_stream_send(
            self.connection.target_system, self.connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        self.connection.mav.param_request_read_send(self.connection.target_system,
            self.connection.target_component, b"AHRS_EKF_TYPE", -1)
        estimator = self.wait("EKF3 configuration", lambda m: m.get_type() == "PARAM_VALUE"
                              and m.param_id == "AHRS_EKF_TYPE")
        if estimator.param_value != 3:
            raise RuntimeError("Expected EKF3; refusing to validate a ground-truth-driven autopilot")

    def execute(self):
        self.connect()
        self.event("WAITING_FOR_POSITION")
        if self.gps_denied:
            self.connection.mav.param_request_read_send(self.connection.target_system,
                self.connection.target_component, b"SIM_GPS1_ENABLE", -1)
            disabled = self.wait("GPS disabled parameter", lambda m: m.get_type() == "PARAM_VALUE"
                                 and m.param_id == "SIM_GPS1_ENABLE", 10)
            if disabled.param_value != 0:
                raise RuntimeError("GPS-denied check requires SIM_GPS1_ENABLE=0")
        else:
            self.wait("GPS fix", lambda m: m.get_type() == "GPS_RAW_INT" and m.fix_type >= 3, 60)
        # Require the EKF to have an absolute horizontal position estimate.
        self.wait("EKF position", lambda m: m.get_type() == "EKF_STATUS_REPORT"
                  and m.flags & 16 and not m.flags & 128, 60)
        if self.gps_denied:
            # A fixed geographic origin gives EKF3 a local NED frame; it is metadata,
            # never a position measurement or evaluator pose. Send after EKF startup.
            self.connection.mav.set_gps_global_origin_send(self.connection.target_system,
                                                            319500000, 359300000, 750000)
            self.wait("EKF geographic origin", lambda m: m.get_type() == "GLOBAL_POSITION_INT"
                      and m.lat != 0 and m.lon != 0, 10)
        self.wait("local position", lambda m: m.get_type() == "LOCAL_POSITION_NED")
        if self.gps_denied:
            # ExternalNav initializes early; SITL's separate accelerometer
            # consistency check still needs its stationary calibration period.
            self.wait("stationary IMU calibration", lambda m: self.sim_time is not None
                      and self.sim_time >= 42.0, 50)
        self.mode(4)  # GUIDED
        self.event("ARMING")
        self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        self.wait("armed", lambda m: m.get_type() == "HEARTBEAT"
                  and m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        self.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, self.altitude)
        self.event("TAKING_OFF")
        low, high = self.altitude - 0.5, self.altitude + 0.5
        self.wait(f"{self.altitude:g} metre altitude", lambda m: m.get_type() == "GLOBAL_POSITION_INT"
                  and (self.altitude - 0.3) * 1000 <= m.relative_alt <= (self.altitude + 0.4) * 1000, 45)
        self.event("HOVERING")
        # The autopilot's altitude decides when to land; judge() measures the hover on Webots' trajectory.
        start = None
        hovered = False
        sim_start, wall_start = self.sim_time, time.monotonic()
        while not self.expired(sim_start, wall_start, 25):
            message = self.poll()
            if not message or message.get_type() != "GLOBAL_POSITION_INT":
                continue
            altitude = message.relative_alt / 1000
            if not low <= altitude <= high:
                start = None
                continue
            if start is None:
                start = message.time_boot_ms
            if message.time_boot_ms - start >= 5000:
                hovered = True
                break
        if not hovered:
            raise RuntimeError("Could not sustain a 5-second hover within 0.5 m of target altitude")
        if self.external_nav:
            self.transition_external_nav()
        if self.inject_link_loss:
            self.test_link_loss()
        if self.inject_unhealthy_odometry:
            detection = self.test_unhealthy_odometry()
        if self.explore:
            self.fly_explore()
        if self.route_points:
            self.fly_odometry_route()
        if not self.inject_link_loss and not self.inject_unhealthy_odometry:
            self.event("LANDING")
            # Exploration can finish in open ground with too few LiDAR features
            # for scan matching. Guidance is over; LAND owns the descent now.
            self.monitor_odometry = False
            self.mode(9)  # LAND
            self.wait("landed and disarmed", lambda m: m.get_type() == "HEARTBEAT"
                      and not m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED, 60)
        self.event("EVALUATING")
        if self.gps_denied and self.gps_fix_seen:
            raise RuntimeError("GPS fix appeared during GPS-denied flight")
        # Ground truth is read only after the mission, never to command the vehicle.
        truth = [json.loads(line) for line in self.output.with_name("ground_truth.jsonl").read_text().splitlines()]
        maze = self.output.with_name("truth.json")  # a generated world's geometry, evaluator only
        maze_truth = json.loads(maze.read_text()) if maze.exists() else None
        walls = maze_truth["walls"] if maze_truth else ()
        autopilot_time = self.messages.get("SYSTEM_TIME")
        if autopilot_time is None:
            raise RuntimeError("No autopilot clock was recorded")
        measurements = judge(truth, autopilot_time.time_boot_ms / 1000, self.altitude, walls,
                             self.external_nav, self.route_points, self.explore)
        if self.inject_unhealthy_odometry:
            measurements["unhealthy_detection_s"] = detection
        if self.explore:
            index = escape_index(truth, maze_truth)
            measurements["escaped"] = index is not None
            if index is None:
                raise RuntimeError("Exploration did not leave the maze through its exit")
            measurements["escape_time_s"] = round(truth[index]["time"] - truth[0]["time"], 2)
        self.event("COMPLETED")
        return measurements

    def test_unhealthy_odometry(self):
        """Have the odometer mark its estimates unhealthy in flight; the health monitor must land.

        The odometer keeps publishing (the stream is not stale) but reports every estimate
        unhealthy and sends EKF3 quality -1. Returns simulated seconds from injection to LAND."""
        trigger = self.output.with_name("inject-unhealthy-odometry")
        trigger.write_text(json.dumps({"time": self.sim_time}) + "\n")
        self.event("UNHEALTHY_ODOMETRY_INJECTED")
        injected, wall_start = self.sim_time, time.monotonic()
        try:
            while not self.expired(injected, wall_start, 30):
                message = self.poll()
                if message and message.get_type() == "LOCAL_POSITION_NED":
                    self.send_route_velocity(0, 0, 0)
        except RuntimeError as exc:
            if "odometry became unhealthy" not in str(exc):
                raise
            detection = round(self.sim_time - injected, 2)
            self.event(f"UNHEALTHY_ODOMETRY_DETECTED after {detection:.1f} s")
            self.fail_safe_land()
            if self.events[-1]["state"] != "FAILSAFE_LANDED":
                raise RuntimeError("Unhealthy-odometry failsafe did not land and disarm")
            return detection
        raise RuntimeError("Injected unhealthy odometry was not detected within 30 simulated seconds")

    def test_link_loss(self):
        """Drop the GCS TCP connection in flight; Copter's configured failsafe must land."""
        self.event("LINK_LOSS_INJECTED")
        self.connection.close()
        self.connection = None
        time.sleep(15)  # the simulation is real-time or slower; Copter needs 5 simulated seconds
        self.connect()
        self.wait("GCS failsafe landing and disarm", lambda m: m.get_type() == "HEARTBEAT"
                  and m.custom_mode == 9 and not m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED, 30)
        self.event("LINK_FAILSAFE_LANDED")

    def fly_explore(self):
        """Follow sensor-map frontiers without opening the world's evaluator truth."""
        start = self.wait("exploration origin", lambda m: m.get_type() == "LOCAL_POSITION_NED", 5)
        self.event("EXPLORING")
        sim_start, wall_start = self.sim_time, time.monotonic()
        last_target = self.sim_time
        stale_planner_checks = 0
        overspeed_checks = 0
        commanded_vx = commanded_vy = 0.0
        last_command_time = self.sim_time

        def hold():
            nonlocal commanded_vx, commanded_vy, last_command_time
            self.send_route_velocity(0, 0, 0)
            commanded_vx = commanded_vy = 0.0
            last_command_time = self.sim_time

        target_file = self.output.with_name("navigation_target.json")
        # Both escapes of the first batch took 823-855 s at 0.25 m/s, so 900 s decided outcomes.
        # 1500 simulated s is ~1900 wall s at 0.8x real time, inside the 3600 s flight backstop.
        while not self.expired(sim_start, wall_start, EXPLORATION_LIMIT_S):
            m = self.poll()
            if not m or m.get_type() != "LOCAL_POSITION_NED":
                continue
            if not self.armed:
                raise RuntimeError("Exploration vehicle disarmed in flight")
            overspeed_checks = overspeed_checks + 1 if math.hypot(m.vx, m.vy) > 0.75 else 0
            if overspeed_checks:
                hold()
                if overspeed_checks >= 5:
                    raise RuntimeError("Exploration horizontal speed exceeded safety limit")
                continue
            if not target_file.exists():
                hold()
                continue
            nav = json.loads(target_file.read_text())
            age = self.sim_time - nav["time"]
            stale_planner_checks = stale_planner_checks + 1 if age > 2.0 else 0
            if age > 2.0:
                hold()
                if age > 5.0 and stale_planner_checks >= 5:
                    raise RuntimeError("Exploration map and planner became stale")
                continue
            if nav["state"] == "escaped":
                hold()
                self.event("EXIT_DETECTED")
                return
            waypoint = nav["waypoint_xy_m"]
            if waypoint is None:
                hold()
                if self.sim_time - last_target > 15:
                    raise RuntimeError("Exploration found no reachable frontier")
                continue
            last_target = self.sim_time
            dx, dy, _ = frames.webots_to_sitl_local((*waypoint, 0.0))
            ex, ey = start.x + dx - m.x, start.y + dy - m.y
            distance = math.hypot(ex, ey)
            speed = min(0.25 if nav.get("odometry_healthy", True) else 0.12, 0.5 * distance)
            vx, vy = (speed * ex / distance, speed * ey / distance) if distance > 0.02 else (0, 0)
            vx, vy = limit_xy_acceleration((commanded_vx, commanded_vy), (vx, vy),
                                           self.sim_time - last_command_time)
            commanded_vx, commanded_vy = vx, vy
            last_command_time = self.sim_time
            vz = max(-0.15, min(0.15, 0.5 * (start.z - m.z)))
            self.send_route_velocity(vx, vy, vz)
        raise RuntimeError(f"Exploration exceeded {EXPLORATION_LIMIT_S} simulated seconds")

    def transition_external_nav(self):
        """Switch EKF3 GPS -> sensor odometry -> GPS while airborne; move 0.5 m on source 2."""
        odometry = self.output.with_name("odometry.jsonl")
        if not odometry.exists():
            raise RuntimeError("External odometry did not start")
        recent = [json.loads(line) for line in odometry.read_text().splitlines()[-5:]]
        if (len(recent) < 5 or not all(item["healthy"] for item in recent)
                or self.sim_time is None or abs(recent[-1]["time"] - self.sim_time) > 1.0):
            raise RuntimeError("External odometry is unhealthy: " + (recent[-1]["reason"] if recent else "no scans"))
        self.event("EXTERNAL_NAV")
        self.command(42007, 2)  # MAV_CMD_SET_EKF_SOURCE_SET
        initial = self.wait("external navigation position", lambda m: m.get_type() == "LOCAL_POSITION_NED", 5)
        start, wall_start = self.sim_time, time.monotonic()
        while self.sim_time is None or self.sim_time - start < 5:
            message = self.poll()
            if message and message.get_type() == "EKF_STATUS_REPORT" and (not message.flags & 16 or message.flags & 128):
                raise RuntimeError("EKF position became unhealthy after external-navigation switch")
            if self.expired(start, wall_start, 15):
                raise RuntimeError("External-navigation hover timed out")
        self.event("EXTERNAL_NAV_ROUTE")
        for target_x in (initial.x + 0.5, initial.x):
            self.connection.mav.set_position_target_local_ned_send(
                int(self.sim_time * 1000), self.connection.target_system, self.connection.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0x0DF8,
                target_x, initial.y, initial.z, 0, 0, 0, 0, 0, 0, 0, 0)
            self.wait("external navigation route", lambda m: m.get_type() == "LOCAL_POSITION_NED"
                      and abs(m.x - target_x) < 0.12 and abs(m.y - initial.y) < 0.18, 15)
        self.command(42007, 1)
        self.event("GPS_RESTORED")

    def fly_odometry_route(self):
        """GPS-guided data collection fixture; no generated maze truth enters the mission."""
        start = self.wait("route start position", lambda m: m.get_type() == "LOCAL_POSITION_NED", 5)
        for i, (x, y) in enumerate(self.route_points, 1):
            self.event(f"ODOMETRY_ROUTE_{i}")
            dx, dy, _ = frames.webots_to_sitl_local((x, y, 0.0))
            target_x, target_y = start.x + dx, start.y + dy
            sim_start, wall_start = self.sim_time, time.monotonic()
            while not self.expired(sim_start, wall_start, 45):
                m = self.poll()
                if not m or m.get_type() != "LOCAL_POSITION_NED":
                    continue
                ex, ey = target_x - m.x, target_y - m.y
                distance = math.hypot(ex, ey)
                if distance < 0.12 and math.hypot(m.vx, m.vy) < 0.2:
                    self.send_route_velocity(0.0, 0.0, 0.0)
                    break
                speed = min(0.6, 0.55 * distance)
                vx, vy = (speed * ex / distance, speed * ey / distance) if distance > 0 else (0, 0)
                vz = max(-0.15, min(0.15, 0.5 * (start.z - m.z)))
                self.send_route_velocity(vx, vy, vz)
            else:
                raise RuntimeError(f"Timeout: route waypoint {i}")

    def send_route_velocity(self, vx, vy, vz):
        # Guided velocity target, refreshed at each LOCAL_POSITION_NED update (~10 Hz).
        self.connection.mav.set_position_target_local_ned_send(
            int(self.sim_time * 1000), self.connection.target_system, self.connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0x0DC7,
            0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)

    def close(self):
        self.log.close()
        if self.connection:
            self.connection.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--altitude", type=float, default=3.0, help="hover altitude, m (3 in the empty world)")
    parser.add_argument("--external-nav", action="store_true")
    parser.add_argument("--odometry-route", action="store_true")
    parser.add_argument("--gps-denied", action="store_true")
    parser.add_argument("--explore", action="store_true")
    parser.add_argument("--inject-link-loss", action="store_true")
    parser.add_argument("--inject-unhealthy-odometry", action="store_true")
    args = parser.parse_args()
    flight = Flight(args.output, args.altitude, args.external_nav, args.odometry_route, args.gps_denied,
                    args.explore, args.inject_link_loss, args.inject_unhealthy_odometry)
    report = {"passed": False, "estimator": "EKF3",
              "position_source": ("sensor odometry, GPS disabled" if args.gps_denied else
                                  "GPS and sensor odometry transition" if args.external_nav else "simulated GPS")}
    try:
        report["measurements"] = flight.execute()
        report["passed"] = True
    except Exception as exc:
        report["error"] = str(exc)
        flight.event(f"FAILED: {exc}")
        flight.fail_safe_land()
    finally:
        report["events"] = flight.events
        args.output.write_text(json.dumps(report, indent=2) + "\n")
        flight.close()
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())

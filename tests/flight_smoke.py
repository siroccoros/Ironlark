"""Execute and evaluate the simulated takeoff/hover/land integration gate."""

import argparse
import json
import math
from pathlib import Path
import sys
import time

from pymavlink import mavutil

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from sensing import frames  # noqa: E402


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


def judge(truth, autopilot_seconds, altitude=3.0, walls=(), external_nav_route=False, route_points=()):
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
    if not route_points and drift > 1.5:
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
    def __init__(self, output, altitude=3.0, external_nav=False, odometry_route=False):
        self.output = output
        self.altitude = altitude
        self.external_nav = external_nav
        self.route_points = (json.loads((ROOT / "config/odometry_route_seed7.json").read_text())
                             ["waypoints_webots_xy_m"] if odometry_route else ())
        self.connection = None
        self.messages = {}
        self.events = []
        self.log = output.with_name("telemetry.jsonl").open("w", buffering=1)
        self.started = time.monotonic()
        self.last_heartbeat = 0
        self.sim_time = None  # autopilot boot time in seconds; tracks simulated time

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
            self.messages[message.get_type()] = message
            boot = getattr(message, "time_boot_ms", None)
            if boot is not None:
                self.sim_time = boot / 1000
            self.log.write(json.dumps(message.to_dict()) + "\n")
            if message.get_type() == "STATUSTEXT":
                print(f"  ArduPilot: {message.text}", flush=True)
        return message

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
        self.wait("GPS fix", lambda m: m.get_type() == "GPS_RAW_INT" and m.fix_type >= 3, 60)
        # Require the EKF to have an absolute horizontal position estimate.
        self.wait("EKF position", lambda m: m.get_type() == "EKF_STATUS_REPORT"
                  and m.flags & 16 and not m.flags & 128, 60)
        self.wait("local position", lambda m: m.get_type() == "LOCAL_POSITION_NED")
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
        if self.route_points:
            self.fly_odometry_route()
        self.event("LANDING")
        self.mode(9)  # LAND
        self.wait("landed and disarmed", lambda m: m.get_type() == "HEARTBEAT"
                  and not m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED, 60)
        self.event("EVALUATING")
        # Ground truth is read only after the mission, never to command the vehicle.
        truth = [json.loads(line) for line in self.output.with_name("ground_truth.jsonl").read_text().splitlines()]
        maze = self.output.with_name("truth.json")  # a generated world's geometry, evaluator only
        walls = json.loads(maze.read_text())["walls"] if maze.exists() else ()
        autopilot_time = self.messages.get("SYSTEM_TIME")
        if autopilot_time is None:
            raise RuntimeError("No autopilot clock was recorded")
        measurements = judge(truth, autopilot_time.time_boot_ms / 1000, self.altitude, walls,
                             self.external_nav, self.route_points)
        self.event("COMPLETED")
        return measurements

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
    args = parser.parse_args()
    flight = Flight(args.output, args.altitude, args.external_nav, args.odometry_route)
    report = {"passed": False, "estimator": "EKF3",
              "position_source": "GPS and sensor odometry transition" if args.external_nav else "simulated GPS"}
    try:
        report["measurements"] = flight.execute()
        report["passed"] = True
    except Exception as exc:
        report["error"] = str(exc)
        flight.event(f"FAILED: {exc}")
        # The enclosing launcher tears down this software-only simulation on failure.
    finally:
        report["events"] = flight.events
        args.output.write_text(json.dumps(report, indent=2) + "\n")
        flight.close()
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())

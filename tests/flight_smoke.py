"""Execute and evaluate the simulated takeoff/hover/land integration gate."""

import argparse
import json
import math
from pathlib import Path
import statistics
import struct
import sys
import time

from pymavlink import mavutil


def judge(truth, autopilot_seconds):
    """The flight as Webots measured it (flight_observer's ground truth, every 100 ms of simulated time), never as the
    autopilot reported it: the trajectory covers the flight without a gap, and shows a takeoff, a continuous 5-second
    hover at 2.5-3.5 m, a landing back on the ground (z = 0 in both worlds) and no drift."""
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
    hover, entered = 0.0, None  # the longest stay in the band, and when the current one began
    for at, height in zip(times, heights):
        if not 2.5 <= height <= 3.5:
            entered = None
            continue
        entered = at if entered is None else entered
        hover = max(hover, at - entered)
    start_xy = truth[0]["position"][:2]
    drift = max(math.hypot(sample["position"][0] - start_xy[0],
                           sample["position"][1] - start_xy[1]) for sample in truth)
    if not 2.5 <= peak <= 4:
        raise RuntimeError(f"Physical takeoff height out of bounds: {peak:.2f} m")
    if hover < 5:
        raise RuntimeError(f"No physical 5-second hover at 2.5-3.5 m: the longest lasted {hover:.1f} s")
    if not -0.2 <= heights[-1] <= 0.2:
        raise RuntimeError(f"Drone did not physically land: final height {heights[-1]:.2f} m")
    if drift > 1.5:
        raise RuntimeError(f"Excessive horizontal drift: {drift:.2f} m")
    return {"peak_height_m": peak, "hover_s": round(hover, 1), "final_height_m": heights[-1],
            "max_horizontal_drift_m": drift, "clock_difference_s": clock_difference}


# SITL's record of each IMU's every sample (SIM_GYR_FILE_RW, SIM_ACC_FILE_RW: x, y, z as float32), with the
# parameters of config/flight.parm that give its datasheet's noise density and the gyro's offset.
IMUS = {"gyro0": ("SIM_GYR1_NSD", "SIM_GYR1_BIAS", "INS_GYROFFS"),
        "gyro1": ("SIM_GYR2_NSD", "SIM_GYR2_BIAS", "INS_GYR2OFFS"),
        "accel0": ("SIM_ACC1_NSD", None, None), "accel1": ("SIM_ACC2_NSD", None, None)}
DENSITY_TOLERANCE = 0.05  # over ~35 s on the pad a density reads within 2%; SITL's uniform floor, without
                          # docker/sitl-imu-noise.patch, 4 to 20 times lower
OFFSET_TOLERANCE_DPS = 0.1  # the boot calibration's own bound (AP_InertialSensor's GYRO_INIT_MAX_DIFF_DPS)


def flight_parameters(path):
    values = {}
    for line in Path(path).read_text().splitlines():
        fields = line.split("#")[0].split()
        if len(fields) == 2:
            values[fields[0]] = float(fields[1])
    return values


def judge_imus(samples, seconds, parameters, offsets):
    """The IMUs as the drone waited on its pad, seconds of simulated time: samples, each sensor's rows (x, y, z),
    every sample it made. White noise of density d, sampled n times over T, has a standard deviation of
    d * sqrt(n / 2T): each sensor's against its datasheet's (parameters, config/flight.parm). The offsets the boot
    calibration measured (INS_GYROFFS_X ...: offsets) against those SITL adds."""
    measured = {}
    for sensor, (density, bias, offset) in IMUS.items():
        rows = samples[sensor]
        if len(rows) < 5000:  # fewer give no density to 2%
            raise RuntimeError(f"{sensor}: {len(rows)} samples over {seconds:.1f} s on the pad")
        spread = [statistics.pstdev(axis) * math.sqrt(2 * seconds / len(rows)) / parameters[density]
                  for axis in zip(*rows)]
        if any(abs(ratio - 1) > DENSITY_TOLERANCE for ratio in spread):
            raise RuntimeError(f"{sensor}: noise density {', '.join(f'{r:.3f}' for r in spread)} times its datasheet's")
        measured[sensor] = {"density_over_datasheet": [round(r, 3) for r in spread]}
        if bias:
            error = max(abs(math.degrees(offsets[f"{offset}_{axis}"] - parameters[f"{bias}_{axis}"])) for axis in "XYZ")
            if error > OFFSET_TOLERANCE_DPS:
                raise RuntimeError(f"{sensor}: the boot calibration's offset is {error:.3f} deg/s off SITL's")
            measured[sensor]["offset_error_dps"] = round(error, 4)
    return {"rest_s": round(seconds, 1), **measured}


class Flight:
    def __init__(self, output):
        self.output = output
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

    def parameter(self, name, value=None):
        """A parameter's value, set first to value if given."""
        if value is None:
            self.connection.mav.param_request_read_send(self.connection.target_system,
                self.connection.target_component, name.encode(), -1)
        else:
            self.connection.mav.param_set_send(self.connection.target_system, self.connection.target_component,
                name.encode(), value, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        return self.wait(f"parameter {name}", lambda m: m.get_type() == "PARAM_VALUE" and m.param_id == name
                         and (value is None or m.param_value == value)).param_value

    def record_imus(self, on):
        """SITL's record of the IMUs' samples (judge_imus), from the time on until off."""
        for name in ("SIM_GYR_FILE_RW", "SIM_ACC_FILE_RW"):
            self.parameter(name, 2 if on else 0)  # 2: write each sample to /tmp/<sensor><instance>.dat
        # The autopilot's clock just after: as late at the start as at the end.
        self.wait("autopilot clock", lambda m: getattr(m, "time_boot_ms", None) is not None)
        return self.sim_time

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
        resting = self.record_imus(True)
        self.event("WAITING_FOR_POSITION")
        self.wait("GPS fix", lambda m: m.get_type() == "GPS_RAW_INT" and m.fix_type >= 3, 60)
        # Require the EKF to have an absolute horizontal position estimate.
        self.wait("EKF position", lambda m: m.get_type() == "EKF_STATUS_REPORT"
                  and m.flags & 16 and not m.flags & 128, 60)
        self.wait("local position", lambda m: m.get_type() == "LOCAL_POSITION_NED")
        self.mode(4)  # GUIDED
        rest = self.record_imus(False) - resting
        self.event("ARMING")
        self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        self.wait("armed", lambda m: m.get_type() == "HEARTBEAT"
                  and m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        self.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 3)
        self.event("TAKING_OFF")
        self.wait("3 metre altitude", lambda m: m.get_type() == "GLOBAL_POSITION_INT"
                  and 2700 <= m.relative_alt <= 3400, 45)
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
            if not 2.5 <= altitude <= 3.5:
                start = None
                continue
            if start is None:
                start = message.time_boot_ms
            if message.time_boot_ms - start >= 5000:
                hovered = True
                break
        if not hovered:
            raise RuntimeError("Could not sustain a 5-second hover within 0.5 m of target altitude")
        # A half turn in place: the LiDAR's points, each at its own instant, are checked turning too.
        self.event("TURNING")
        self.command(mavutil.mavlink.MAV_CMD_CONDITION_YAW, 180, 120, 1, 1)
        start = self.wait("attitude", lambda m: m.get_type() == "ATTITUDE").yaw
        self.wait("half turn", lambda m: m.get_type() == "ATTITUDE"
                  and abs(math.remainder(m.yaw - start, 2 * math.pi)) > math.radians(175), 20)
        self.event("LANDING")
        self.mode(9)  # LAND
        self.wait("landed and disarmed", lambda m: m.get_type() == "HEARTBEAT"
                  and not m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED, 60)
        self.event("EVALUATING")
        # Ground truth is read only after the mission, never to command the vehicle.
        truth = [json.loads(line) for line in self.output.with_name("ground_truth.jsonl").read_text().splitlines()]
        autopilot_time = self.messages.get("SYSTEM_TIME")
        if autopilot_time is None:
            raise RuntimeError("No autopilot clock was recorded")
        measurements = judge(truth, autopilot_time.time_boot_ms / 1000)
        samples = {sensor: list(struct.iter_unpack("<3f", Path(f"/tmp/{sensor}.dat").read_bytes())) for sensor in IMUS}
        offsets = {f"{name}_{axis}": self.parameter(f"{name}_{axis}")
                   for name in ("INS_GYROFFS", "INS_GYR2OFFS") for axis in "XYZ"}
        measurements["imus"] = judge_imus(samples, rest, flight_parameters("/project/config/flight.parm"), offsets)
        self.event("COMPLETED")
        return measurements

    def close(self):
        self.log.close()
        if self.connection:
            self.connection.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    flight = Flight(args.output)
    report = {"passed": False, "estimator": "EKF3", "position_source": "simulated GPS"}
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

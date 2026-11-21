"""MAVLink interface to ArduPilot SITL: EKF3 ExternalNav, GPS<->LIO switching, and GUIDED control."""
import bisect
import json
import math
import threading
import time
from collections import deque
from pathlib import Path

from pymavlink import mavutil

from sensing.frames import (
    quaternion_to_euler,
    sitl_ned_to_webots,
    webots_euler_to_sitl_ned,
    webots_to_sitl_ned,
)

MAV_CMD_SET_EKF_SOURCE_SET = 42007
MASK_PVA_YAW_RATE = 0x0000  # SET_POSITION_TARGET_LOCAL_NED: use position, velocity, acceleration, yaw, yaw rate


class MavlinkPilot:
    """Clean MAVLink flight interface talking exclusively to ArduPilot on TCP 5760."""

    def __init__(self, endpoint="tcp:127.0.0.1:5760", telemetry_path=None, state_path=None, pad_z=0.09):
        self.endpoint = endpoint
        self.pad_z = pad_z
        self.state_path = Path(state_path) if state_path else None
        self.log = Path(telemetry_path).open("w") if telemetry_path else None
        self._send_lock = threading.Lock()
        self._event_lock = threading.Lock()
        self.conn = None
        self.messages = {}
        self.events = []
        self.started = time.monotonic()
        self.last_heartbeat = 0.0
        self.sim_time = None

        self.armed = False
        self.custom_mode = None
        self.gps_fix = 0
        self.ekf_flags = 0
        self.ekf_local_webots = None
        self.ekf_vel_webots = None
        self.frame_offset = [0.0, 0.0, 0.0]  # EKF(GPS) - LIO offset tracked outdoors, frozen when GPS-denied
        self.ekf_source = 1  # 1 = GPS (SRC1), 2 = ExternalNav/LIO (SRC2)
        self.source_switches = []
        self.gps_lost_seen = False
        self.vision_sent = 0
        self._gps_good_since = None
        # EKF3 local positions by ArduPilot time, to align odometry with the EKF frame at the scan's time.
        self._ekf_track = deque(maxlen=120)
        self._clock_offsets = deque(maxlen=51)  # ArduPilot time - sensor time, sampled at each vision send
        # A reader thread owns the link once connected, so telemetry is fresh whatever the executive does.
        self._reader = None
        self._closing = False
        self._msg_cond = threading.Condition()
        self._recent = deque(maxlen=4000)
        self._seq = 0

    def event(self, state, detail=None):
        entry = {"state": state, "sim_time": self.sim_time, "wall_seconds": round(time.monotonic() - self.started, 2)}
        if detail:
            entry["detail"] = detail
        with self._event_lock:
            self.events.append(entry)
            print(f"[autonomy] {state}" + (f" ({detail})" if detail else ""), flush=True)
            if self.state_path:
                tmp = self.state_path.with_suffix(".tmp")
                tmp.write_text(json.dumps({"state": state}) + "\n")
                tmp.replace(self.state_path)

    def _heartbeat(self):
        now = time.monotonic()  # link keep-alive only (1 Hz GCS heartbeat), not a control decision
        if now - self.last_heartbeat >= 1.0:
            with self._send_lock:
                self.conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0
                )
            self.last_heartbeat = now

    def poll(self, timeout=0.02):
        """Read one message on the caller's thread (only before the reader thread starts)."""
        self._heartbeat()
        msg = self.conn.recv_match(blocking=timeout > 0, timeout=timeout if timeout > 0 else None)
        if not msg or msg.get_type() == "BAD_DATA":
            return None
        return self._handle(msg)

    def start_reader(self):
        if self._reader is None:
            self._reader = threading.Thread(target=self._read_loop, name="mavlink-reader", daemon=True)
            self._reader.start()

    def _read_loop(self):
        while not self._closing:
            try:
                self._heartbeat()
                msg = self.conn.recv_match(blocking=True, timeout=0.05)
            except Exception:
                if self._closing:
                    break
                time.sleep(0.01)
                continue
            if not msg or msg.get_type() == "BAD_DATA":
                continue
            # Drain any immediately queued MAVLink packets before notifying SetpointStreamer so
            # sim_time is always the freshest on the wire (never waking on a stale TCP backlog).
            batch = [msg]
            for _ in range(64):
                try:
                    nxt = self.conn.recv_match(blocking=False)
                except Exception:
                    break
                if not nxt:
                    break
                if nxt.get_type() != "BAD_DATA":
                    batch.append(nxt)
            for m in batch:
                self._handle(m)
            with self._msg_cond:
                for m in batch:
                    self._seq += 1
                    self._recent.append((self._seq, m))
                self._msg_cond.notify_all()

    def _handle(self, msg):
        kind = msg.get_type()
        self.messages[kind] = msg
        boot = getattr(msg, "time_boot_ms", None)
        if boot is not None:
            t = boot / 1000.0
            if self.sim_time is None or t > self.sim_time:
                self.sim_time = t
        if self.log:
            self.log.write(json.dumps(msg.to_dict()) + "\n")
        if kind == "HEARTBEAT":
            self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.custom_mode = msg.custom_mode
        elif kind == "GPS_RAW_INT":
            self.gps_fix = int(msg.fix_type)
            if self.gps_fix < 3 and self.armed:
                self.gps_lost_seen = True
                self._gps_good_since = None
            elif self.gps_fix >= 3 and self._gps_good_since is None:
                self._gps_good_since = self.sim_time
        elif kind == "EKF_STATUS_REPORT":
            self.ekf_flags = int(msg.flags)
        elif kind == "LOCAL_POSITION_NED":
            wx, wy, wz = sitl_ned_to_webots((msg.x, msg.y, msg.z))
            self.ekf_local_webots = (wx, wy, wz + self.pad_z)
            self.ekf_vel_webots = sitl_ned_to_webots((msg.vx, msg.vy, msg.vz))
            self._ekf_track.append((msg.time_boot_ms / 1000.0, self.ekf_local_webots))
        elif kind == "STATUSTEXT":
            print(f"  ArduPilot: {msg.text}", flush=True)
        return msg

    def drain(self, max_messages=300):
        if self._reader is not None:
            return  # the reader thread keeps state current
        for _ in range(max_messages):
            if self.poll(timeout=0.0) is None:
                break

    @property
    def closing(self):
        return self._closing

    def wait_message(self, timeout):
        """Block until the reader thread has received another message, or timeout seconds pass."""
        with self._msg_cond:
            self._msg_cond.wait(timeout)

    def cursor(self):
        """Position in the message stream; wait(since=...) sees only messages after it."""
        return self._seq

    def _next_messages(self, since):
        with self._msg_cond:
            self._msg_cond.wait_for(lambda: self._seq > since or self._closing, timeout=0.05)
            if self._recent and self._recent[0][0] > since + 1:
                since = self._recent[0][0] - 1  # older messages were dropped from the window
            return [m for k, m in self._recent if k > since], self._seq

    def wait(self, description, predicate, sim_timeout=45.0, tick=None, since=None):
        """Block until a message satisfies predicate; timeouts in sim time (wall clock only as a backstop)."""
        sim_start, wall_start = self.sim_time, time.monotonic()
        last_tick = None
        since = self._seq if since is None else since
        while True:
            if self._reader is not None:
                msgs, since = self._next_messages(since)
            else:
                msg = self.poll(timeout=0.0) or self.poll(timeout=0.03)
                msgs = [msg] if msg else []
            if sim_start is None:
                sim_start = self.sim_time
            for msg in msgs:
                if predicate(msg):
                    return msg
            now_sim = self.sim_time
            if tick and now_sim is not None and (last_tick is None or now_sim - last_tick >= 0.1):
                tick()
                last_tick = now_sim
            if now_sim is not None and sim_start is not None and now_sim - sim_start > sim_timeout:
                raise RuntimeError(f"Timeout ({sim_timeout}s sim): {description}")
            if time.monotonic() - wall_start > max(4.0 * sim_timeout, 90.0):
                raise RuntimeError(f"Timeout (wall): {description}")

    def command(self, cmd, *params, tick=None):
        padded = list(params) + [0.0] * (7 - len(params))
        since = self.cursor()
        with self._send_lock:
            self.conn.mav.command_long_send(
                self.conn.target_system, self.conn.target_component, cmd, 0, *padded
            )
        ack = self.wait(
            f"ACK {cmd}", lambda m: m.get_type() == "COMMAND_ACK" and m.command == cmd,
            sim_timeout=10.0, tick=tick, since=since,
        )
        if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise RuntimeError(f"MAVLink command {cmd} rejected (result={ack.result})")

    def connect(self):
        self.event("CONNECTING")
        deadline = time.monotonic() + 45.0
        while time.monotonic() < deadline:
            try:
                self.conn = mavutil.mavlink_connection(self.endpoint, source_system=255, retries=0)
                break
            except OSError:
                time.sleep(0.4)
        if self.conn is None:
            raise RuntimeError(f"SITL did not expose MAVLink on {self.endpoint}")
        hb = self.wait("heartbeat", lambda m: m.get_type() == "HEARTBEAT", sim_timeout=45.0)
        self.conn.target_system = hb.get_srcSystem()
        self.conn.target_component = hb.get_srcComponent()
        self.conn.mav.request_data_stream_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1,
        )
        self.conn.mav.param_request_read_send(
            self.conn.target_system, self.conn.target_component, b"AHRS_EKF_TYPE", -1
        )
        est = self.wait("AHRS_EKF_TYPE", lambda m: m.get_type() == "PARAM_VALUE" and m.param_id == "AHRS_EKF_TYPE")
        if int(round(est.param_value)) != 3:
            raise RuntimeError("Expected EKF3; refusing to run on ground-truth sim AHRS")
        self.start_reader()
        # EKF position at 50 Hz: the frame-offset alignment interpolates it at each scan's capture time.
        self.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                     mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20000)
        # The setpoint the autopilot holds, at 10 Hz in the telemetry log, for tracking analysis.
        self.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                     mavutil.mavlink.MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED, 100000)

    def wait_ready_and_takeoff(self, altitude_m=2.2, tick=None):
        self.event("WAITING_FOR_POSITION")
        self.wait("GPS 3D fix", lambda m: m.get_type() == "GPS_RAW_INT" and m.fix_type >= 3, 60.0, tick=tick)
        self.wait("EKF3 position", lambda m: m.get_type() == "EKF_STATUS_REPORT" and (m.flags & 16) and not (m.flags & 128),
                  60.0, tick=tick)
        self.wait("LOCAL_POSITION_NED", lambda m: m.get_type() == "LOCAL_POSITION_NED", 30.0, tick=tick)
        self.set_mode(4, tick=tick)  # GUIDED
        self.event("ARMING")
        for attempt in range(8):
            try:
                self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, tick=tick)
                break
            except RuntimeError:
                if attempt == 7:
                    raise
                retry_at = (self.sim_time or 0.0) + 1.0
                self.wait("arming retry", lambda m: (self.sim_time or 0.0) >= retry_at, 5.0, tick=tick)
        self.wait("armed", lambda m: m.get_type() == "HEARTBEAT"
                  and bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED), 15.0, tick=tick)
        self.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, altitude_m, tick=tick)
        self.event("TAKING_OFF")
        self.wait(
            f"{altitude_m:.1f} m altitude",
            lambda m: m.get_type() == "LOCAL_POSITION_NED"
            and abs((-m.z) - altitude_m) <= 0.25 and abs(m.vz) <= 0.45,
            45.0,
            tick=tick,
        )

    def set_mode(self, mode_id, tick=None):
        self.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                     mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, tick=tick)
        self.wait(f"mode {mode_id}", lambda m: m.get_type() == "HEARTBEAT" and m.custom_mode == mode_id,
                  15.0, tick=tick)

    def ekf_position_at(self, t_ap):
        """EKF3 local position (Webots frame) interpolated at ArduPilot time t_ap, or None if not covered."""
        track = list(self._ekf_track)
        if len(track) < 2 or not (track[0][0] <= t_ap <= track[-1][0]):
            return None
        i = bisect.bisect_left([t for t, _ in track], t_ap)
        (t0, a), (t1, b) = track[max(0, i - 1)], track[min(len(track) - 1, i)]
        u = (t_ap - t0) / (t1 - t0) if t1 > t0 else 0.0
        return tuple(a[k] + u * (b[k] - a[k]) for k in range(3))

    def note_sensor_time(self, sensor_now):
        """Sample the ArduPilot-minus-sensor clock offset (both clocks read at the same instant)."""
        if self.sim_time is not None:
            self._clock_offsets.append(self.sim_time - sensor_now)

    def send_vision_estimate(self, stamp_s, pos_webots, quat_xyzw, capture_s=None):
        """Stream 6-DoF LiDAR odometry pose to ArduPilot EKF3 External Navigation.

        usec carries the scan's own sensor-clock stamp: ArduPilot maps it to its clock with its jitter
        correction (GCS_MAVLINK::correct_offboard_timestamp_usec_to_ms, minimum transport lag over 100
        messages), so a scan processed late is still fused at its time; VISO_DELAY_MS adds the fixed
        rest (capture precedes the stamp by one LiDAR period, plus the minimum transport lag).

        While outdoors on GPS (SRC1), tracks the slow frame offset between EKF3's GPS frame and
        odometry so repeated GPS <-> ExternalNav transitions stay continuous with zero step jump. The
        EKF position is taken at the scan's capture time, so the offset carries no speed-dependent bias.
        """
        if self.conn is None:
            return
        if self.ekf_source == 1 and self.gps_fix >= 3 and self._clock_offsets:
            offset = sorted(self._clock_offsets)[len(self._clock_offsets) // 2]
            ekf = self.ekf_position_at((stamp_s if capture_s is None else capture_s) + offset)
            if ekf is not None:
                errs = [ekf[k] - float(pos_webots[k]) for k in range(3)]
                drift = math.hypot(errs[0] - self.frame_offset[0], errs[1] - self.frame_offset[1])
                if drift > 2.5:
                    # LIO lost lock and drifted far while outdoors on GPS: re-anchor frame_offset
                    # immediately so the occupancy map and next indoor transition stay GPS-aligned.
                    self.frame_offset = list(errs)
                else:
                    for k in range(3):
                        self.frame_offset[k] = 0.92 * self.frame_offset[k] + 0.08 * errs[k]
        aligned = (
            float(pos_webots[0]) + self.frame_offset[0],
            float(pos_webots[1]) + self.frame_offset[1],
            float(pos_webots[2]) + self.frame_offset[2],
        )
        usec = int(stamp_s * 1_000_000)
        nx, ny, nz = webots_to_sitl_ned((aligned[0], aligned[1], aligned[2] - self.pad_z))
        r, p, y = webots_euler_to_sitl_ned(quaternion_to_euler(quat_xyzw))
        with self._send_lock:
            self.conn.mav.vision_position_estimate_send(usec, nx, ny, nz, r, p, y)
        self.vision_sent += 1

    def set_ekf_source(self, source_set, reason=""):
        """Switch ArduPilot EKF3 live between Source Set 1 (GPS) and Source Set 2 (ExternalNav/LIO)."""
        if source_set == self.ekf_source or self.conn is None:
            return
        with self._send_lock:
            self.conn.mav.command_long_send(
                self.conn.target_system, self.conn.target_component,
                MAV_CMD_SET_EKF_SOURCE_SET, 0, float(source_set), 0, 0, 0, 0, 0, 0,
            )
        self.ekf_source = source_set
        self.source_switches.append({"sim_time": self.sim_time, "source": source_set, "reason": reason})
        label = "EXTERNAL_NAV (LIO)" if source_set == 2 else "GPS"
        self.event(f"EKF_SOURCE_{source_set}", f"{label}: {reason}")

    def update_gps_ekf_transition(self, lio_healthy):
        """React strictly to ArduPilot's own GPS_RAW_INT fix quality: switch to LIO on GPS loss, back on recovery."""
        if not self.armed:
            return
        if self.gps_fix < 3 and self.ekf_source == 1 and lio_healthy:
            self.set_ekf_source(2, "GPS fix lost")
        elif self.gps_fix >= 3 and self.ekf_source == 2:
            if not lio_healthy:
                self.set_ekf_source(1, "GPS immediate fallback (LIO degraded outdoors)")
            elif (self._gps_good_since is not None and self.sim_time is not None
                  and (self.sim_time - self._gps_good_since) >= 1.2):
                self.set_ekf_source(1, "GPS fix restored outdoors")

    def send_pva_target(self, pos_webots, vel_webots, acc_webots, yaw_webots, yaw_rate_webots):
        """GUIDED setpoint with every field used (type_mask 0): ArduPilot 4.7.1 tracks position with
        velocity and acceleration as feed-forward and yaw with yaw rate (GCS_MAVLink_Copter.cpp,
        handle_message_set_position_target_local_ned -> ModeGuided::set_pos_vel_accel_NED_m)."""
        nx, ny, nz = webots_to_sitl_ned((pos_webots[0], pos_webots[1], pos_webots[2] - self.pad_z))
        vx, vy, vz = webots_to_sitl_ned(vel_webots)
        ax, ay, az = webots_to_sitl_ned(acc_webots)
        with self._send_lock:
            self.conn.mav.set_position_target_local_ned_send(
                0, self.conn.target_system, self.conn.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, MASK_PVA_YAW_RATE,
                nx, ny, nz, vx, vy, vz, ax, ay, az, -float(yaw_webots), -float(yaw_rate_webots),
            )

    def yaw_webots(self):
        """Current heading in the Webots frame from ArduPilot's ATTITUDE (NED yaw is its negative)."""
        att = self.messages.get("ATTITUDE")
        return -float(att.yaw) if att is not None else 0.0

    def land_and_disarm(self, tick=None):
        self.event("LANDING")
        self.set_mode(9, tick=tick)  # LAND
        self.wait("disarmed", lambda m: m.get_type() == "HEARTBEAT"
                  and not bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED), 75.0, tick=tick)

    def close(self):
        self._closing = True
        if self._reader is not None:
            self._reader.join(timeout=1.0)
        if self.log:
            self.log.close()
        if self.conn:
            self.conn.close()

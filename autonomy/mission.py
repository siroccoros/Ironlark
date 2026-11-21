"""Ironlark ROS 2 Autonomy Node, Interruptible Task Executive & Supervisory GCS.

Subscribes ONLY to onboard sensor topics (/imu/data, /lidar/points, /camera/image_raw, /tf_static)
— never /ground_truth/* — and executes either the full autonomous outdoor-to-cave mission or live
supervisory tasks submitted through the browser Ground Control Station (http://127.0.0.1:8080).
"""
import argparse
import json
import math
import queue
import sys
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, Imu, PointCloud2
from tf2_msgs.msg import TFMessage

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from autonomy.gcs import occupancy_slice_2d, start_gcs_server  # noqa: E402
from autonomy.mapping import OccupancyMap  # noqa: E402
from autonomy.mavlink_interface import MavlinkPilot  # noqa: E402
from autonomy.odometry import LidarInertialOdometry, matrix_to_quaternion  # noqa: E402
from autonomy import trajectory as tj  # noqa: E402
from autonomy.planner import plan_path  # noqa: E402
from sensing.frames import quaternion_to_euler, rotate_vector  # noqa: E402

# Webots returns a LiDAR scan one sensor period after it was taken, stamped with the step it is read
# at (measured: odometry yaw lags truth by 0.100 s). The scan stamp minus this is its capture time.
LIDAR_CAPTURE_LEAD_S = 1.0 / json.loads((ROOT / "config/sensors.json").read_text())["lidar"]["rate_hz"]
# EKF3 fuses a measurement only inside its buffer horizon, the largest sensor delay: SITL's u-blox M8
# GPS lag 0.12 s, so 11 IMU samples of 12 ms. A pose older than that on arrival is fused as if newer
# (NavEKF3_core::writeExtNavData). With VISO_DELAY_MS 105 a pose may arrive this much later than the
# promptest one and still be fused at its own time; a later pose is carried forward to it instead.
VISION_LAG_ALLOWANCE_S = 0.015


class PerceptionNode(Node):
    """Runs real-time LIO, 3D occupancy mapping, and throttled camera preview from ROS 2 topics only."""

    def __init__(self, pilot, capture_camera=False):
        super().__init__("ironlark_autonomy")
        self.pilot = pilot
        self.lio_lock = threading.Lock()
        self.map_lock = threading.Lock()
        self.lio = LidarInertialOdometry(initial_position=(0.0, 0.0, pilot.pad_z))
        self.occ_map = OccupancyMap()
        self.lidar_tf = ((0.0, 0.0, 0.083), (0.0, 0.0, 0.0, 1.0))
        self.lio_log = []
        self.trail = []
        self._latest = (self.lio.p.copy(), self.lio.R.copy(), self.lio.v.copy(), True, 0)
        self._latest_stamp = 0.0
        self._imu_cond = threading.Condition()
        self._last_imu_stamp = -1.0
        self._imu_waiting = 0
        self._cam_cond = threading.Condition()
        self._cam_frame = None  # (seq, width, height, rgba_bytes)
        self._cam_seq = 0
        self._last_cam_wall = 0.0
        self._last_vision_stamp = -1.0
        # Timing evidence for the motion gate (sim seconds): IMU-sync waits and scan pipeline lags.
        self.timing = {"sync_wait_max_s": 0.0, "unsynced_scans": 0, "queue_lag_s": [], "send_lag_s": []}

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, "/tf_static", self._on_tf, latched)
        self.create_subscription(PointCloud2, "/lidar/points", self._on_lidar, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))

    def _on_tf(self, msg):
        for t in msg.transforms:
            if t.child_frame_id == "lidar_link":
                tr, r = t.transform.translation, t.transform.rotation
                self.lidar_tf = ((tr.x, tr.y, tr.z), (r.x, r.y, r.z, r.w))

    def push_imu(self, stamp, g, a):
        self.lio.on_imu(stamp, g, a)
        if stamp > self._last_imu_stamp:
            self._last_imu_stamp = stamp
            if self._imu_waiting > 0:
                with self._imu_cond:
                    self._imu_cond.notify_all()

    def push_camera(self, msg):
        if msg.encoding != "bgra8":
            return
        now = time.monotonic()
        if now - self._last_cam_wall < 0.095:
            return  # 10 FPS preview cuts interactive GIL/memory bandwidth in half without hurting GCS feel
        self._last_cam_wall = now
        bgra = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 4))
        rgba = bgra[::2, ::2, [2, 1, 0, 3]]
        h, w, _ = rgba.shape
        raw = rgba.tobytes()
        with self._cam_cond:
            self._cam_seq += 1
            self._cam_frame = (self._cam_seq, w, h, raw)
            self._cam_cond.notify_all()

    def _on_lidar(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        count = msg.width * msg.height
        if count == 0 or len(raw) < count * msg.point_step:
            return
        pts_lidar = np.ndarray(shape=(count, 3), dtype=np.float32, buffer=raw,
                               strides=(msg.point_step, 4)).astype(np.float64)
        t_off, q_off = self.lidar_tf
        if q_off != (0.0, 0.0, 0.0, 1.0):
            pts_body = np.array([rotate_vector(q_off, p) for p in pts_lidar]) + np.asarray(t_off)
        else:
            pts_body = pts_lidar + np.asarray(t_off)

        recv_lag = max(0.0, self._last_imu_stamp - stamp)
        wait_start = time.monotonic()
        if self._last_imu_stamp >= stamp - 0.01:
            synced = True
        else:
            with self._imu_cond:
                self._imu_waiting += 1
                try:
                    synced = self._imu_cond.wait_for(lambda: self._last_imu_stamp >= stamp - 0.01, timeout=1.5)
                finally:
                    self._imu_waiting -= 1
        self.timing["sync_wait_max_s"] = max(self.timing["sync_wait_max_s"], time.monotonic() - wait_start)
        if not synced and self.lio.scan_count > 0:
            self.timing["unsynced_scans"] += 1
            return
        with self.lio_lock:
            p, R, v, healthy = self.lio.on_lidar(stamp, pts_body)
            quat = matrix_to_quaternion(R)
            sensor_now = self._last_imu_stamp
            lead = max(0.0, sensor_now - stamp - VISION_LAG_ALLOWANCE_S)
            p_send, R_send = self.lio.predict(stamp, stamp + lead) if lead > 0.0 else (p, R)
            self._latest = (p.copy(), R.copy(), v.copy(), healthy, self.lio.scan_count)
            self._latest_stamp = stamp
            self.lio_log.append({
                "time": round(stamp, 3),
                "position": [round(float(c), 4) for c in p],
                "yaw_deg": round(math.degrees(quaternion_to_euler(quat)[2]), 2),
                "healthy": healthy,
                "inliers": self.lio.last_inliers,
                "rmse": round(self.lio.last_rmse, 4),
            })
            if not self.trail or math.hypot(p[0] - self.trail[-1][0], p[1] - self.trail[-1][1]) >= 0.4:
                self.trail.append([round(float(p[0]), 2), round(float(p[1]), 2)])
                if len(self.trail) > 400:
                    self.trail = self.trail[-400:]
            if not healthy and self.pilot.ekf_source == 1 and self.pilot.gps_fix >= 3:
                # Outdoors on solid 3D GPS: re-seed LIO's prior back onto the GPS-anchored EKF pose
                # in LIO's frame so any transient scan gap recovers ICP lock on the very next scan.
                ekf_cap = None
                if self.pilot._clock_offsets:
                    clk_off = sorted(self.pilot._clock_offsets)[len(self.pilot._clock_offsets) // 2]
                    ekf_cap = self.pilot.ekf_position_at(stamp - LIDAR_CAPTURE_LEAD_S + clk_off)
                if ekf_cap is None:
                    ekf_cap = self.pilot.ekf_local_webots
                if ekf_cap is not None:
                    self.lio.p = np.asarray(ekf_cap, dtype=np.float64) - np.asarray(self.pilot.frame_offset, dtype=np.float64)
                    if self.pilot.ekf_vel_webots is not None:
                        self.lio.v = np.asarray(self.pilot.ekf_vel_webots, dtype=np.float64)
        if healthy:
            # ArduPilot's jitter correction maps the stamp to its clock at the promptest pose's lag,
            # and VISO_DELAY_MS takes it back to the capture. A pose processed late goes out carried
            # forward and stamped to within VISION_LAG_ALLOWANCE_S of prompt, so the EKF fuses it at
            # its own time instead of at its buffer's end. Poses later than 0.45 s are dropped.
            send_lag = max(0.0, sensor_now - stamp)
            if len(self.timing["queue_lag_s"]) < 20000:
                self.timing["queue_lag_s"].append(round(recv_lag, 4))
                self.timing["send_lag_s"].append(round(send_lag, 4))
            # Long constant-velocity extrapolation (> 0.22 s) under deceleration/turning creates
            # 0.15-0.25 m ballistic error that spikes EKF3 innovation variance; never extrapolate
            # past 0.35 s (EKF3's 250 Hz IMU bridges a brief hiccup far cleaner than linear drag).
            since_vis = (sensor_now - self._last_vision_stamp) if self._last_vision_stamp > 0.0 else 9.0
            max_lag = 0.22 if since_vis < 0.6 else 0.35
            if send_lag <= max_lag:
                self.pilot.note_sensor_time(sensor_now)
                self.pilot.send_vision_estimate(stamp + lead, p_send, matrix_to_quaternion(R_send),
                                                capture_s=stamp + lead - LIDAR_CAPTURE_LEAD_S)
                self._last_vision_stamp = sensor_now
            p_nav = p + np.asarray(self.pilot.frame_offset, dtype=np.float64)
            # Outdoors on GPS, anchor map updates to time-aligned EKF3 when available so a transient
            # LIO glitch can never stamp ghost obstacles or carve free space through real boulders.
            if self.pilot.ekf_source == 1 and self.pilot.gps_fix >= 3 and self.pilot._clock_offsets:
                clk_off = sorted(self.pilot._clock_offsets)[len(self.pilot._clock_offsets) // 2]
                ekf_cap = self.pilot.ekf_position_at(stamp - LIDAR_CAPTURE_LEAD_S + clk_off)
                if ekf_cap is not None:
                    p_nav = np.asarray(ekf_cap, dtype=np.float64)
            sub = pts_body[::3]
            finite = np.all(np.isfinite(sub), axis=1)
            world_pts = sub[finite] @ R.T + p_nav
            with self.map_lock:
                self.occ_map.update(p_nav + np.asarray(t_off), world_pts, stride=4)

    def snapshot(self):
        return self._latest

    def nav_pos(self):
        """Position in the reconciled GPS/odometry navigation frame, propagated to latest IMU time."""
        p, _, v, healthy, _ = self._latest
        dt = max(0.0, min(0.25, self._last_imu_stamp - self._latest_stamp)) if healthy and self._latest_stamp > 0.0 else 0.0
        return p + v * dt + np.asarray(self.pilot.frame_offset, dtype=np.float64)

    def map_snapshot(self):
        """Clone the occupancy grid in ~0.1 ms so planning never blocks /lidar/points."""
        with self.map_lock:
            return self.occ_map.copy()


class ImuNode(Node):
    """Dedicated high-rate /imu/data subscriber so 250 Hz gyro buffering never waits behind LiDAR ICP."""

    def __init__(self, perception):
        super().__init__("ironlark_imu")
        self.perception = perception
        self.create_subscription(Imu, "/imu/data", self._on_imu, QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE))

    def _on_imu(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        g = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        a = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        self.perception.push_imu(stamp, g, a)


class CameraNode(Node):
    """Dedicated /camera/image_raw subscriber so 20 FPS browser streaming never stalls LiDAR odometry."""

    def __init__(self, perception):
        super().__init__("ironlark_camera")
        self.perception = perception
        self.create_subscription(Image, "/camera/image_raw", self.perception.push_camera,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))


PLAN_Z = (2.0, 2.6)  # m: cruise band the planner keeps paths in


class SetpointStreamer:
    """Streams the committed trajectory to GUIDED at 50 Hz of autopilot time, on its own thread.

    A replacement trajectory takes over at its own t0 (the time it was stitched at); until then the
    one it replaces keeps streaming, so position, velocity and acceleration stay continuous. Holding
    is a trajectory at rest: the setpoint keeps going out, there is no mode or mask switch."""

    def __init__(self, pilot):
        self.pilot = pilot
        self._lock = threading.Lock()
        self._active = None
        self._queue = []  # chronological pending trajectories sorted by t0
        self.enabled = False
        self.gaps = []  # autopilot seconds between consecutive setpoints
        threading.Thread(target=self._run, name="setpoints", daemon=True).start()

    def now(self):
        return self.pilot.sim_time or 0.0

    def start(self, traj):
        with self._lock:
            self._active, self._queue, self.enabled = traj, [], True

    def stop(self):
        self.enabled = False

    def commit(self, traj):
        with self._lock:
            # Keep any earlier pending trajectory (e.g. a brake starting before traj.t0) so
            # traj.sample(traj.t0) stitches cleanly onto whatever is actually streaming at traj.t0.
            self._queue = [q for q in self._queue if q.t0 < traj.t0 - 1e-6]
            self._queue.append(traj)

    def at(self, t):
        """The committed trajectory that governs time t."""
        with self._lock:
            cur = self._active
            for q in self._queue:
                if t >= q.t0:
                    cur = q
                else:
                    break
            return cur

    def _run(self):
        next_t, last = None, None
        while not self.pilot.closing:
            self.pilot.wait_message(0.02)
            now = self.pilot.sim_time
            if now is None or not self.enabled:
                next_t = last = None
                continue
            if next_t is not None and now < next_t:
                continue
            with self._lock:
                while self._queue and now >= self._queue[0].t0:
                    self._active = self._queue.pop(0)
                traj = self._active
            if traj is None:
                continue
            try:
                self.pilot.send_pva_target(*traj.sample(now))
            except Exception:
                continue  # link closing
            if last is not None and len(self.gaps) < 200000:
                self.gaps.append(round(now - last, 3))
            last = now
            next_t = next_t + tj.DT if next_t is not None and now - next_t < tj.DT else now + tj.DT


def _path_length(path):
    return sum(float(np.linalg.norm(np.asarray(b) - np.asarray(a))) for a, b in zip(path, path[1:]))


def _exact_goal(view, path, goal, allow_unknown):
    """End the path on the requested goal itself when clear and within the cruise altitude band."""
    goal = np.asarray(goal, dtype=np.float64)
    if not (PLAN_Z[0] <= goal[2] <= PLAN_Z[1]):
        return list(path)
    if float(view.clearance(goal[None, :])[0]) < 1.05:
        return list(path)
    a = np.asarray(path[-2] if len(path) > 1 else path[-1])
    seg = np.linspace(a, goal, max(2, int(np.linalg.norm(goal - a) / tj.DS) + 2))
    if not view.blocked(seg, allow_unknown).any():
        return list(path[:-1]) + [goal]
    return list(path)


class MissionExecutive:
    """Interruptible task manager serving both automated missions and live supervisory GCS commands."""

    def __init__(self, pilot, perception, brief):
        self.pilot = pilot
        self.perception = perception
        self.brief = brief
        self.cmd_queue = queue.Queue()
        self.preempt = threading.Event()
        self.streamer = SetpointStreamer(pilot)

        self.phase = "INIT"
        self.active_path = []
        self.active_goal = None
        self.active_frontiers = []
        self.active_explore_box = None
        self.frontiers_visited = 0
        self.free_volume_m3 = 0.0
        self._vol_cache = (0.0, 0.0)
        self._map_cache = (0.0, None)
        self._plan_s = deque([0.1], maxlen=20)  # recent planning durations (s)
        self.plan_s_max = 0.0
        self.nav_stats = {"plans": 0, "replan_blocked": 0, "replan_shorter": 0, "retime": 0, "brake": 0, "no_path": 0}

    @property
    def latency(self):
        """Stitch a replan this far ahead: planning finishes before the old trajectory reaches it."""
        return float(np.clip(1.5 * max(self._plan_s), 0.1, 0.85))

    def _note_plan_time(self, seconds):
        self._plan_s.append(seconds)
        self.plan_s_max = max(self.plan_s_max, seconds)

    def _limits(self, max_speed):
        # Tilt follows acceleration (tan(tilt) = a/g) and the Mid-360 sees 6.7 deg below its plane.
        # Indoors on odometry: hold tilt <= ~8-9 deg and speed <= 1.55 m/s where EKF3 SRC2 is proven.
        # Outdoors on GPS: smooth ~9-11 deg curves with 25 deg headroom for disturbance recovery.
        if self.pilot.ekf_source == 2:
            return tj.Limits(v_max=min(max_speed, 1.55), a_acc=0.95, a_brake=1.30, a_lat=0.80,
                             jerk=1.8, yaw_rate=math.radians(17.0), yaw_acc=math.radians(35.0))
        return tj.Limits(v_max=max_speed, a_acc=1.10, a_brake=1.45, a_lat=0.95,
                         jerk=2.0, yaw_rate=math.radians(19.0), yaw_acc=math.radians(40.0))

    def set_phase(self, name, detail=None):
        self.phase = name
        self.pilot.event(name, detail)

    def submit_command(self, cmd):
        kind = cmd.get("cmd")
        if kind not in ("goto", "explore", "mission", "hold", "rth", "land"):
            raise ValueError(f"Unknown supervisory command: {kind}")
        self.cmd_queue.put(cmd)
        self.preempt.set()
        self.pilot.event("OPERATOR_CMD", json.dumps(cmd))
        return {"ok": True, "accepted": kind}

    def gcs_state(self):
        _, R, v, healthy, scans = self.perception.snapshot()
        p = self.perception.nav_pos()
        _, _, yaw = quaternion_to_euler(matrix_to_quaternion(R))
        now = time.monotonic()
        if now - self._vol_cache[0] >= 1.0:
            bounds = {"x": [-41, 41], "y": [-41, 41], "z": [1.0, 3.5]}
            self._vol_cache = (now, round(self.perception.map_snapshot().free_volume_m3(bounds), 1))
        vol = self._vol_cache[1]
        return {
            "phase": self.phase,
            "armed": self.pilot.armed,
            "sim_time": self.pilot.sim_time,
            "pos": [round(float(c), 2) for c in p],
            "speed": round(float(np.linalg.norm(v)), 2),
            "yaw": round(float(yaw), 3),
            "gps_fix": self.pilot.gps_fix,
            "ekf_source": self.pilot.ekf_source,
            "lio_healthy": healthy,
            "lio_inliers": self.perception.lio.last_inliers,
            "lio_scans": scans,
            "free_volume_m3": vol,
            "frontiers_visited": self.frontiers_visited,
            "path": [[round(float(w[0]), 2), round(float(w[1]), 2)] for w in self.active_path],
            "trail": list(self.perception.trail),
            "frontiers": [[round(float(f[0]), 2), round(float(f[1]), 2)] for f in self.active_frontiers],
            "active_goal": [round(float(c), 2) for c in self.active_goal] if self.active_goal is not None else None,
            "explore_box": self.active_explore_box,
            "events": list(self.pilot.events)[-18:],
        }

    def gcs_map(self):
        now = time.monotonic()
        if self._map_cache[1] is None or now - self._map_cache[0] >= 0.85:
            self._map_cache = (now, occupancy_slice_2d(self.perception.map_snapshot()))
        return self._map_cache[1]

    def gcs_camera_frame(self, after_seq=0, timeout=0.15):
        with self.perception._cam_cond:
            self.perception._cam_cond.wait_for(
                lambda: self.perception._cam_seq > after_seq, timeout=timeout
            )
            return self.perception._cam_frame

    def _tick(self):
        _, _, _, healthy, _ = self.perception.snapshot()
        self.pilot.update_gps_ekf_transition(healthy)

    def ensure_airborne(self, altitude=2.2):
        if not self.pilot.armed:
            self.streamer.stop()
            self.pilot.wait_ready_and_takeoff(altitude_m=altitude, tick=self._tick)
        elif self.pilot.custom_mode != 4:
            self.pilot.set_mode(4, tick=self._tick)  # e.g. a Go-To while landing: back to GUIDED
        if not self.streamer.enabled:
            # Take over from the takeoff (or the landing) where the autopilot is, at rest.
            ekf = self.pilot.ekf_local_webots
            here = np.asarray(ekf if ekf is not None else self.perception.nav_pos(), dtype=np.float64)
            self.streamer.start(tj.rest(self.streamer.now(), here, self.pilot.yaw_webots()))

    def land(self):
        self.set_phase("LANDING")
        self.streamer.stop()
        self.pilot.land_and_disarm(tick=self._tick)

    def _stitch(self):
        """(time, state) a replan starts from: the committed trajectory's state one latency ahead."""
        t_r = self.streamer.now() + self.latency
        return t_r, self.streamer.at(t_r).sample(t_r)

    def _brake(self, max_speed):
        t_r = self.streamer.now() + self.latency
        cur = self.streamer.at(t_r)
        if cur is not None and t_r < cur.t_motion_end:
            self.streamer.commit(tj.brake(cur, t_r, self._limits(max_speed)))

    def _show(self, traj):
        self.active_path = [traj.P[i] for i in range(0, len(traj.P), 10)] + [traj.P[-1]]

    def fly_to_goal(self, goal, allow_unknown=True, tolerance=0.85, sim_timeout=75.0,
                    max_speed=2.1, abort_if_blocked=False, through=False):
        """Fly a smooth trajectory to goal, re-checking it against every new map and replanning.

        Ends at rest on the goal, or with through=True hands over once the goal is within stopping
        distance, so the next leg stitches on at speed instead of stopping."""
        goal = np.asarray(goal, dtype=np.float64)
        self.active_goal = goal
        traj = None  # this leg's committed trajectory; None while braking or before the first plan
        traj_relaxed = False
        view, view_scans = None, -1
        last_plan = last_retime = last_log = -999.0
        blocked_since = diverged_since = unhealthy_since = None
        best_dist = float("inf")
        best_dist_t = self.streamer.now()
        leg_brakes = 0
        stall_count = 0
        source = self.pilot.ekf_source
        sim_start = self.streamer.now()
        wall_start = time.monotonic()

        while True:
            if self.preempt.is_set():
                return "preempted"
            if not self.pilot.armed:
                raise RuntimeError("Autopilot disarmed mid-flight")
            _, _, _, healthy, scans = self.perception.snapshot()
            self.pilot.update_gps_ekf_transition(healthy)
            now = self.streamer.now()
            ekf = self.pilot.ekf_local_webots
            pos = (np.asarray(ekf, dtype=np.float64)
                   if (self.pilot.ekf_source == 1 and ekf is not None)
                   else self.perception.nav_pos())
            if now - sim_start > sim_timeout or time.monotonic() - wall_start > max(4.0 * sim_timeout, 120.0):
                self._brake(max_speed)
                return "failed"

            if abort_if_blocked:
                d_goal = math.hypot(pos[0] - goal[0], pos[1] - goal[1])
                if d_goal < best_dist - 0.6:
                    best_dist, best_dist_t = d_goal, now
                elif now - best_dist_t >= 5.5:
                    # No net progress toward an optional frontier (e.g., shadowed behind a crate/wall):
                    # abort decisively instead of pacing back and forth until sim_timeout.
                    self._brake(max_speed)
                    return "failed"

            if traj is not None:
                if through:
                    # Hand over before this leg's own stop begins: braking distance plus the time to
                    # turn acceleration around at the jerk limit, plus the next leg's planning latency.
                    _, v_now, _ = traj.arc_at(now)
                    lim0 = tj.Limits()
                    handoff = (v_now * v_now / (2.0 * 0.75 * lim0.a_brake)
                               + v_now * (lim0.a_brake / (0.8 * lim0.jerk) + self.latency))
                    if traj.remaining_length(now) <= max(tolerance, handoff):
                        self.pilot.event("ARRIVED", f"{traj.goal[0]:.2f},{traj.goal[1]:.2f},{traj.goal[2]:.2f}")
                        return "reached"
                elif now >= traj.t_motion_end and float(np.linalg.norm(pos - traj.goal)) <= tolerance:
                    self.pilot.event("ARRIVED", f"{traj.goal[0]:.2f},{traj.goal[1]:.2f},{traj.goal[2]:.2f}")
                    self.active_path = []
                    self.active_goal = None
                    return "reached"

            # Odometry lost while flying on it: stop and wait for it before planning again.
            if not healthy and self.pilot.ekf_source == 2:
                unhealthy_since = unhealthy_since if unhealthy_since is not None else now
                if now - unhealthy_since >= 0.3 and traj is not None:
                    self._brake(max_speed)
                    traj = None
                time.sleep(0.02)
                continue
            unhealthy_since = None

            # Anti-pinning: the autopilot cannot follow the reference (pressed against something).
            ekf = self.pilot.ekf_local_webots
            ekf_vel = self.pilot.ekf_vel_webots
            ref = self.streamer.at(now).sample(now)[0]
            ekf_speed = math.hypot(ekf_vel[0], ekf_vel[1]) if ekf_vel is not None else 0.0
            if ekf is not None and math.hypot(ekf[0] - ref[0], ekf[1] - ref[1]) > 0.9 and ekf_speed < 0.65:
                diverged_since = diverged_since if diverged_since is not None else now
                if now - diverged_since >= 1.5:
                    stall_count += 1
                    self.pilot.event("STALL_RECOVERY", f"count={stall_count}")
                    self.streamer.commit(tj.rest(now, np.asarray(ekf), self.pilot.yaw_webots()))
                    traj, diverged_since, last_plan = None, None, -999.0
                    if stall_count >= 2:
                        self.active_path = []
                        self.active_goal = None
                        return "failed"
            else:
                diverged_since = None

            new_scan = scans != view_scans
            if new_scan:
                view, view_scans = tj.MapView(self.perception.map_snapshot()), scans
            lim = self._limits(max_speed)
            chk_view = tj.MapView(view.map.relaxed_copy()) if (traj is not None and traj_relaxed and new_scan) else view
            hit_idx = (tj.first_collision(chk_view, traj.remaining_points(now), allow_unknown)
                       if (traj is not None and new_scan) else None)
            # Replan immediately for obstacles within 8 m; throttle distant (>8 m) replans to avoid
            # CPU/GIL storms on 60 m cross-world legs while still routing around newly mapped trees.
            blocked = hit_idx is not None and (hit_idx * 0.25 <= 8.0 or now - last_plan >= 1.5)
            periodic = traj is not None and now - last_plan >= 6.0
            retry = traj is None and (now - last_plan >= 0.45)

            if blocked or periodic or retry:
                t_r, start = self._stitch()
                t_wall = time.monotonic()
                plan_view = view
                used_relaxed = False
                path = plan_path(plan_view.map, start[0], goal, z_bounds=PLAN_Z, allow_unknown=allow_unknown,
                                 max_expansions=4500 if not allow_unknown else 7500)
                if path is None and not allow_unknown and not abort_if_blocked:
                    path = plan_path(plan_view.map, start[0], goal, z_bounds=PLAN_Z, allow_unknown=True,
                                     max_expansions=4500)
                if path is None and not abort_if_blocked:
                    # Escaping a tight corner/corridor sealed by full inflation: relax XY by 1 voxel (~1.25 m).
                    plan_view = tj.MapView(view.map.relaxed_copy())
                    path = plan_path(plan_view.map, start[0], goal, z_bounds=PLAN_Z, allow_unknown=True,
                                     max_expansions=7500)
                    used_relaxed = path is not None
                cand = None
                brk_prefix = None
                if path is not None:
                    path = _exact_goal(plan_view, path, goal, allow_unknown)
                    # Homotopy commitment: keep a clear trajectory unless the new route saves > 1.5 m.
                    if traj is None or blocked or _path_length(path) + 1.5 < traj.remaining_length(t_r):
                        cand = tj.build(path, start, plan_view, lim, t_r, allow_unknown, goal=path[-1])
                        if cand is None and float(np.linalg.norm(start[1][:2])) > 0.15:
                            # Moving away from the new route (e.g., >90 deg turnaround out of a room):
                            # pre-stitch a brake to rest plus the turnaround starting seamlessly at t_stop
                            # so the drone never hesitates or re-brakes in a retry loop.
                            cur_t = self.streamer.at(t_r)
                            if cur_t is not None and t_r < cur_t.t_motion_end:
                                brk_cand = tj.brake(cur_t, t_r, lim)
                                t_stop = brk_cand.t_motion_end
                                if t_stop <= self.streamer.now() + 3.0:
                                    stop_st = brk_cand.sample(t_stop)
                                    path2 = plan_path(plan_view.map, stop_st[0], goal, z_bounds=PLAN_Z,
                                                      allow_unknown=allow_unknown,
                                                      max_expansions=4500 if not allow_unknown else 7500)
                                    if path2 is not None:
                                        path2 = _exact_goal(plan_view, path2, goal, allow_unknown)
                                        cand = tj.build(path2, stop_st, plan_view, lim, t_stop, allow_unknown,
                                                        goal=path2[-1])
                                        if cand is not None:
                                            brk_prefix = brk_cand
                self._note_plan_time(time.monotonic() - t_wall)
                if (cand is not None and brk_prefix is None and self.streamer.now() > t_r - 0.02
                        and float(np.linalg.norm(start[1])) < 0.05):
                    # Starting from rest: shifting t0 forward stays C2-continuous with rest() and avoids a retry loop.
                    cand.t0 = self.streamer.now() + 0.06
                commit_t0 = brk_prefix.t0 if brk_prefix is not None else (cand.t0 if cand is not None else -1.0)
                if cand is not None and self.streamer.now() <= commit_t0 - 0.02:
                    last_plan = now
                    self.nav_stats["plans"] += 1
                    if blocked:
                        self.nav_stats["replan_blocked"] += 1
                    elif periodic:
                        self.nav_stats["replan_shorter"] += 1
                    if brk_prefix is not None:
                        self.streamer.commit(brk_prefix)
                    self.streamer.commit(cand)
                    traj, traj_relaxed, last_retime, blocked_since = cand, used_relaxed, now, None
                    self._show(cand)
                elif cand is not None:
                    # Planning overran its stitch horizon t_r: never jump onto a past t0; wait briefly with updated latency.
                    last_plan = now
                elif traj is None or blocked:
                    last_plan = now
                    leg_brakes += 1
                    self.nav_stats["no_path" if path is None else "brake"] += 1
                    self._brake(max_speed)
                    traj = None
                    self.active_path = []
                    blocked_since = blocked_since if blocked_since is not None else now
                    if abort_if_blocked and (now - blocked_since > 2.0 or leg_brakes >= 3):
                        return "failed"
                else:
                    last_plan = now
            elif traj is not None and ((traj.horizon_limited and new_scan and self.pilot.ekf_source == 1
                                        and now - last_retime >= 1.4)
                                       or self.pilot.ekf_source != source):
                # Same curve, new speeds: more of it is mapped now, or the estimator (and its cap) changed.
                t_r = self.streamer.now() + self.latency
                if self.streamer.at(t_r) is traj and t_r < traj.t_motion_end:
                    t_wall = time.monotonic()
                    cand = tj.retime(traj, t_r, view, lim, allow_unknown)
                    self._note_plan_time(time.monotonic() - t_wall)
                    if cand is not None and self.streamer.now() <= t_r - 0.02:
                        self.nav_stats["retime"] += 1
                        self.streamer.commit(cand)
                        traj = cand
                last_retime = now
            source = self.pilot.ekf_source

            if now - last_log >= 5.0:
                print(f"  [nav] t={now:.1f}s pos=({pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f}) "
                      f"goal=({goal[0]:.1f},{goal[1]:.1f},{goal[2]:.1f}) gps={self.pilot.gps_fix} "
                      f"src={self.pilot.ekf_source} lio={'ok' if healthy else 'BAD'} "
                      f"left={traj.remaining_length(now) if traj else -1:.1f}m {self.nav_stats}", flush=True)
                last_log = now
            time.sleep(0.02)

    def explore_region(self, bounds, max_frontiers=4, budget_s=75.0):
        """Autonomous 3D frontier exploration inside a world bounding box."""
        self.active_explore_box = bounds
        pos0 = self.perception.nav_pos()
        visited = [pos0.copy()]
        start_sim = self.pilot.sim_time or 0.0
        while len(visited) - 1 < max_frontiers and ((self.pilot.sim_time or 0.0) - start_sim) < budget_s:
            if self.preempt.is_set():
                return "preempted"
            pos = self.perception.nav_pos()
            snap = self.perception.map_snapshot()
            cands = snap.frontiers(bounds, pos, visited=visited, min_sep=3.2)
            self.active_frontiers = list(cands)
            target = None
            for cand in cands[:8]:
                if plan_path(snap, pos, cand, z_bounds=PLAN_Z, allow_unknown=False, max_expansions=3000) is not None:
                    target = cand
                    break
            if target is None:
                break
            self.set_phase("EXPLORE_FRONTIER", f"({target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f})")
            status = self.fly_to_goal(target, allow_unknown=False, tolerance=1.3, sim_timeout=28.0,
                                      max_speed=1.6, abort_if_blocked=True, through=True)
            if status == "preempted":
                return "preempted"
            visited.append(target)
            if status == "reached":
                self.frontiers_visited += 1
        self.free_volume_m3 = round(self.perception.map_snapshot().free_volume_m3(bounds), 1)
        self.active_frontiers = []
        self.active_explore_box = None
        return "done"

    def execute_full_mission(self):
        home = np.asarray(self.brief["home"], dtype=np.float64)
        approach = np.asarray(self.brief["entrance_approach"], dtype=np.float64)
        entry = np.asarray(self.brief["interior_entry"], dtype=np.float64)
        bounds = self.brief["explore_bounds"]

        self.ensure_airborne(altitude=float(home[2]))

        # Waypoints on the way somewhere are flown through; the drone stops only to hover or land.
        self.set_phase("OUTDOOR_TRANSIT")
        st = self.fly_to_goal(approach, allow_unknown=True, tolerance=1.1, sim_timeout=70.0, through=True)
        if st != "reached":
            return st if st == "preempted" else "Failed to reach cave entrance approach"

        self.set_phase("ENTERING_INTERIOR")
        st = self.fly_to_goal(entry, allow_unknown=True, tolerance=1.0, sim_timeout=45.0, max_speed=1.55, through=True)
        if st != "reached":
            return st if st == "preempted" else "Failed to enter cave interior"

        self.set_phase("EXPLORING_INTERIOR")
        st = self.explore_region(bounds, max_frontiers=4, budget_s=75.0)
        if st == "preempted":
            return "preempted"

        self.set_phase("EXITING_INTERIOR")
        st = self.fly_to_goal(entry, allow_unknown=True, tolerance=1.2, sim_timeout=45.0, max_speed=1.7, through=True)
        if st != "reached":
            return st if st == "preempted" else "Failed to reach cave exit staging point"
        st = self.fly_to_goal(approach, allow_unknown=True, tolerance=1.2, sim_timeout=45.0, max_speed=1.9, through=True)
        if st != "reached":
            return st if st == "preempted" else "Failed to exit cave outdoors"

        self.set_phase("RETURNING_HOME")
        st = self.fly_to_goal(home, allow_unknown=True, tolerance=0.6, sim_timeout=75.0)
        if st != "reached":
            return st if st == "preempted" else "Failed to return to home pad"
        if self.settle_over(home, timeout_s=4.5) == "preempted":
            return "preempted"
        self.land()
        self.set_phase("COMPLETED")
        return "completed"

    def settle_over(self, target, timeout_s=4.5):
        """Before landing: be on target and still (or give up waiting after timeout_s)."""
        target = np.asarray(target, dtype=np.float64)
        t_r, start = self._stitch()
        hopped = float(np.linalg.norm(start[0] - target)) > 0.35
        if float(np.linalg.norm(start[0] - target)) > 0.05 or float(np.linalg.norm(start[1])) > 0.05:
            hop = tj.build([start[0], target], start, None, tj.Limits(v_max=0.8), t_r, goal=target)
            if hop is not None:
                self.streamer.commit(hop)
        start_sim = self.streamer.now()
        while not self.preempt.is_set():
            _, _, v, healthy, _ = self.perception.snapshot()
            self.pilot.update_gps_ekf_transition(healthy)
            err = target - self.perception.nav_pos()
            now = self.streamer.now()
            if (math.hypot(err[0], err[1]) <= 0.30 and math.hypot(v[0], v[1]) <= 0.20) or now - start_sim >= timeout_s:
                if hopped:
                    self.pilot.event("ARRIVED", f"{target[0]:.2f},{target[1]:.2f},{target[2]:.2f}")
                return "settled"
            time.sleep(0.02)
        return "preempted"

    def hold_loop(self):
        """Hover: brake along the committed trajectory if it is still moving; its rest state keeps streaming."""
        self.active_path = []
        self.active_goal = None
        self.active_frontiers = []
        self.active_explore_box = None
        self.set_phase("HOLDING" if self.pilot.armed else "STANDBY")
        if self.pilot.armed and self.streamer.enabled:
            self._brake(2.1)
        while not self.preempt.is_set():
            self._tick()
            time.sleep(0.05)


def run_mission(run_dir, brief, interactive=False, standby=False, gcs_port=0):
    run = Path(run_dir)
    pilot = MavlinkPilot(
        endpoint="tcp:127.0.0.1:5760",
        telemetry_path=run / "telemetry.jsonl",
        state_path=run / "flight-state.json",
    )
    rclpy.init()
    perception = PerceptionNode(pilot, capture_camera=bool(gcs_port))
    imu_node = ImuNode(perception)
    cam_node = CameraNode(perception) if gcs_port else None
    executor = SingleThreadedExecutor()
    executor.add_node(perception)
    imu_executor = SingleThreadedExecutor()
    imu_executor.add_node(imu_node)
    cam_executor = None
    if cam_node is not None:
        cam_executor = SingleThreadedExecutor()
        cam_executor.add_node(cam_node)
    stop_spin = threading.Event()

    def _spin(exec_obj):
        while not stop_spin.is_set() and rclpy.ok():
            exec_obj.spin_once(timeout_sec=0.05)

    spin_thread = threading.Thread(target=_spin, args=(executor,), daemon=True)
    imu_thread = threading.Thread(target=_spin, args=(imu_executor,), daemon=True)
    cam_thread = threading.Thread(target=_spin, args=(cam_executor,), daemon=True) if cam_executor else None
    imu_thread.start()
    spin_thread.start()
    if cam_thread:
        cam_thread.start()
    exec_mgr = MissionExecutive(pilot, perception, brief)
    gcs_server = None

    report = {"passed": False, "mission": "outdoor-to-cave-exploration", "estimator": "EKF3"}

    def write_report():
        with perception.lio_lock:
            lio_trace = list(perception.lio_log)
            scan_count = perception.lio.scan_count
        (run / "lio_trajectory.jsonl").write_text("\n".join(json.dumps(r) for r in lio_trace) + "\n")
        report["frontiers_visited"] = exec_mgr.frontiers_visited
        report["interior_free_volume_m3"] = exec_mgr.free_volume_m3
        report["lio_scans"] = scan_count
        report["vision_messages_sent"] = pilot.vision_sent
        report["gps_lost_observed"] = pilot.gps_lost_seen
        report["ekf_source_switches"] = pilot.source_switches
        report["timing"] = dict(perception.timing)
        gaps = sorted(exec_mgr.streamer.gaps)
        if gaps:
            report["timing"]["setpoint_gap_p99_s"] = gaps[int(0.99 * (len(gaps) - 1))]
            report["timing"]["setpoint_gap_max_s"] = gaps[-1]
        report["timing"]["plan_s_max"] = round(exec_mgr.plan_s_max, 3)
        report["nav"] = dict(exec_mgr.nav_stats)
        report["events"] = list(pilot.events)
        tmp = run / "autonomy_report.json.tmp"
        tmp.write_text(json.dumps(report, indent=2) + "\n")
        tmp.replace(run / "autonomy_report.json")

    try:
        if gcs_port:
            gcs_server, url = start_gcs_server(exec_mgr, host="127.0.0.1", port=gcs_port)
            print(f"[gcs] Supervisory GCS live at {url}", flush=True)

        pilot.connect()
        try:
            pilot.wait("LiDAR scans", lambda m: perception.snapshot()[4] >= 2, 30.0)
        except RuntimeError:
            raise RuntimeError("No LiDAR scans received on /lidar/points before takeoff") from None

        if not interactive:
            outcome = exec_mgr.execute_full_mission()
            if outcome != "completed":
                raise RuntimeError(outcome)
            report["passed"] = True
        else:
            # Interactive supervisory loop: run full mission initially (unless --standby), then keep
            # serving click-to-Go-To, drag-to-Explore, Hold, RTH, Land, and Run Mission indefinitely.
            if not standby:
                exec_mgr.cmd_queue.put({"cmd": "mission"})
                exec_mgr.preempt.set()
            while True:
                if not exec_mgr.preempt.is_set() and exec_mgr.cmd_queue.empty():
                    exec_mgr.hold_loop()
                exec_mgr.preempt.clear()
                cmd = None
                while not exec_mgr.cmd_queue.empty():
                    cmd = exec_mgr.cmd_queue.get_nowait()  # latest operator command wins
                if cmd is None:
                    continue
                kind = cmd["cmd"]
                if kind == "mission":
                    res = exec_mgr.execute_full_mission()
                    if res == "completed":
                        report["passed"] = True
                        write_report()
                elif kind == "goto":
                    exec_mgr.ensure_airborne()
                    gx = float(np.clip(float(cmd["x"]), -40.5, 40.5))
                    gy = float(np.clip(float(cmd["y"]), -40.5, 40.5))
                    gz = float(np.clip(float(cmd.get("z", 2.2)), 1.6, 3.2))
                    target = [gx, gy, gz]
                    exec_mgr.set_phase("GOTO", f"({target[0]:.1f}, {target[1]:.1f})")
                    exec_mgr.fly_to_goal(target, allow_unknown=True, tolerance=0.75, sim_timeout=90.0)
                elif kind == "explore":
                    exec_mgr.ensure_airborne()
                    bounds = {"x": [float(cmd["x0"]), float(cmd["x1"])],
                              "y": [float(cmd["y0"]), float(cmd["y1"])], "z": [1.8, 2.7]}
                    center = [(bounds["x"][0] + bounds["x"][1]) / 2, (bounds["y"][0] + bounds["y"][1]) / 2, 2.2]
                    exec_mgr.set_phase("TRANSIT_TO_REGION")
                    st = exec_mgr.fly_to_goal(center, allow_unknown=True, tolerance=2.2, sim_timeout=80.0)
                    if st != "preempted":
                        exec_mgr.set_phase("EXPLORING_REGION")
                        exec_mgr.explore_region(bounds, max_frontiers=6, budget_s=90.0)
                elif kind == "rth":
                    if pilot.armed:
                        home = np.asarray(brief["home"], dtype=np.float64)
                        exec_mgr.set_phase("RETURNING_HOME")
                        st = exec_mgr.fly_to_goal(home, allow_unknown=True, tolerance=0.6, sim_timeout=90.0)
                        if st == "reached" and exec_mgr.settle_over(home, timeout_s=4.5) == "settled":
                            exec_mgr.land()
                            write_report()
                elif kind == "land":
                    if pilot.armed:
                        exec_mgr.land()
                        write_report()
                elif kind == "hold":
                    exec_mgr.hold_loop()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        report["error"] = str(exc)
        pilot.event("FAILED", str(exc))
    finally:
        if gcs_server:
            gcs_server.shutdown()
        write_report()
        pilot.close()
        stop_spin.set()
        spin_thread.join(timeout=2.0)
        imu_thread.join(timeout=2.0)
        if cam_thread:
            cam_thread.join(timeout=2.0)
        executor.remove_node(perception)
        executor.shutdown()
        imu_executor.remove_node(imu_node)
        imu_executor.shutdown()
        if cam_executor and cam_node:
            cam_executor.remove_node(cam_node)
            cam_executor.shutdown()
            cam_node.destroy_node()
        perception.destroy_node()
        imu_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if report["passed"] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--brief", type=Path, required=True)
    parser.add_argument("--interactive", action="store_true", help="keep serving GCS supervisory commands after tasks")
    parser.add_argument("--standby", action="store_true", help="start on pad waiting for GCS command instead of auto-running")
    parser.add_argument("--gcs-port", type=int, default=0, help="HTTP port for Supervisory GCS (0 = disabled)")
    args = parser.parse_args()
    brief = json.loads(args.brief.read_text())
    sys.exit(run_mission(args.run, brief, interactive=args.interactive, standby=args.standby, gcs_port=args.gcs_port))


if __name__ == "__main__":
    main()

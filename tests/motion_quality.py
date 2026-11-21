"""Motion quality and safety of one flight, measured from independent truth (pure Python, no numpy).

Inputs in a run folder: ground_truth.jsonl (Webots truth, 10 Hz), telemetry.jsonl (ArduPilot MAVLink),
world-obstacles.json (Webots collision geometry, written by flight_observer), autonomy_report.json
(events and timing the autonomy logged) and lio_trajectory.jsonl.

  measure(run) -> dict             used by mission_eval.py (result.json "motion")
  python3 tests/motion_quality.py RUN...                    print measurements
  python3 tests/motion_quality.py RUN... --gate BASELINE    judge runs against a baseline (see GATE)
"""
import argparse
import bisect
import json
import math
import sys
from pathlib import Path

AIRBORNE_Z = 0.5  # m: truth height above which a sample counts as flight


def _jsonl(path):
    path = Path(path)
    if not path.is_file():
        return []
    out = []
    for line in path.read_text().splitlines():
        if line.strip():
            try:
                out.append(json.loads(line))
            except ValueError:
                pass
    return out


def _pct(values, q):
    if not values:
        return None
    s = sorted(values)
    k = (len(s) - 1) * q / 100.0
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def _r(x, n=3):
    return None if x is None else round(x, n)


class Truth:
    def __init__(self, samples):
        self.t = [s["time"] for s in samples]
        self.p = [s["position"] for s in samples]
        self.v = [s["velocity"][:3] for s in samples]
        self.up = [s["orientation"][8] for s in samples]
        self.denied = [bool(s.get("gps_denied")) for s in samples]
        self.contacts = [(s["time"], s["contacts"]) for s in samples if s.get("contacts")]

    def at(self, t, series=None):
        series = self.p if series is None else series
        i = bisect.bisect_left(self.t, t)
        if i <= 0:
            return series[0]
        if i >= len(self.t):
            return series[-1]
        t0, t1 = self.t[i - 1], self.t[i]
        u = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        a, b = series[i - 1], series[i]
        return [a[k] + u * (b[k] - a[k]) for k in range(3)]

    def airborne(self, i):
        return self.p[i][2] > AIRBORNE_Z

    def airborne_at(self, t):
        return self.at(t)[2] > AIRBORNE_Z


# --- clearance -----------------------------------------------------------------------------------

def _dist_box(p, prim):
    t, R, size = prim["t"], prim["R"], prim["size"]
    d = [p[k] - t[k] for k in range(3)]
    local = [R[0][k] * d[0] + R[1][k] * d[1] + R[2][k] * d[2] for k in range(3)]  # R^T d
    q = [abs(local[k]) - 0.5 * size[k] for k in range(3)]
    outside = math.sqrt(sum(max(c, 0.0) ** 2 for c in q))
    return outside if outside > 0.0 else max(q)


def _dist_cyl(p, prim):
    t, R = prim["t"], prim["R"]
    d = [p[k] - t[k] for k in range(3)]
    local = [R[0][k] * d[0] + R[1][k] * d[1] + R[2][k] * d[2] for k in range(3)]
    radial = math.hypot(local[0], local[1]) - prim["radius"]
    axial = abs(local[2]) - 0.5 * prim["height"]
    if radial > 0.0 or axial > 0.0:
        return math.hypot(max(radial, 0.0), max(axial, 0.0))
    return max(radial, axial)


def _dist(p, prim):
    if prim["type"] == "box":
        return _dist_box(p, prim)
    if prim["type"] == "cyl":
        return _dist_cyl(p, prim)
    return math.dist(p, prim["t"]) - prim["radius"]


def _bound(prim):
    if prim["type"] == "box":
        return 0.5 * math.sqrt(sum(s * s for s in prim["size"]))
    if prim["type"] == "cyl":
        return math.hypot(prim["radius"], 0.5 * prim["height"])
    return prim["radius"]


def _top(prim):
    """Highest z of a primitive."""
    if prim["type"] == "box":
        return prim["t"][2] + 0.5 * sum(abs(prim["R"][2][k]) * prim["size"][k] for k in range(3))
    if prim["type"] == "cyl":
        c = abs(prim["R"][2][2])
        return prim["t"][2] + 0.5 * prim["height"] * c + prim["radius"] * math.sqrt(max(0.0, 1.0 - c * c))
    return prim["t"][2] + prim["radius"]


def clearance(truth, obstacles):
    """Minimum distance from the drone's centre to any collision primitive while airborne.

    Floors and ground slabs (tops below 0.25 m) are left out: takeoff and landing touch them."""
    prims = [(prim, _bound(prim)) for prim in obstacles.get("primitives", []) if _top(prim) >= 0.25]
    best = (float("inf"), None, None)
    near = 0
    for i, p in enumerate(truth.p):
        if not truth.airborne(i):
            continue
        local_best = float("inf")
        for prim, bound in prims:
            c = prim["t"]
            if abs(p[0] - c[0]) > bound + 3.0 or abs(p[1] - c[1]) > bound + 3.0:
                continue
            d = _dist(p, prim)
            if d < local_best:
                local_best = d
                if d < best[0]:
                    best = (d, prim.get("name"), truth.t[i])
        if local_best < 1.0:
            near += 1
    return {
        "min_clearance_m": _r(best[0]) if best[1] is not None else None,
        "min_clearance_obstacle": best[1],
        "min_clearance_time_s": _r(best[2], 1),
        "seconds_within_1m": round(near * 0.1, 1),
    }


# --- airframe ------------------------------------------------------------------------------------

def airframe(truth, telemetry):
    tilt = [math.degrees(math.acos(max(-1.0, min(1.0, u)))) for i, u in enumerate(truth.up) if truth.airborne(i)]
    # The LiDAR sees 6.7 deg below its plane: tilt past that blinds one side, so it is kept per zone.
    zone = {True: [], False: []}
    for i, u in enumerate(truth.up):
        if truth.airborne(i):
            zone[truth.denied[i]].append(math.degrees(math.acos(max(-1.0, min(1.0, u)))))
    rates = {"roll": [], "pitch": [], "yaw": []}
    for m in telemetry:
        if m.get("mavpackettype") == "ATTITUDE" and truth.airborne_at(m["time_boot_ms"] / 1000.0):
            rates["roll"].append(abs(math.degrees(m["rollspeed"])))
            rates["pitch"].append(abs(math.degrees(m["pitchspeed"])))
            rates["yaw"].append(abs(math.degrees(m["yawspeed"])))
    acc, jerk = [], []
    t, v = truth.t, truth.v
    a = [None] * len(t)
    for i in range(1, len(t) - 1):
        dt = t[i + 1] - t[i - 1]
        if 0.15 <= dt <= 0.25 and truth.airborne(i - 1) and truth.airborne(i + 1):
            a[i] = [(v[i + 1][k] - v[i - 1][k]) / dt for k in range(2)]
            acc.append(math.hypot(*a[i]))
    for i in range(2, len(t) - 2):
        if a[i - 1] is not None and a[i + 1] is not None:
            dt = t[i + 1] - t[i - 1]
            jerk.append(math.hypot(*[(a[i + 1][k] - a[i - 1][k]) / dt for k in range(2)]))
    return {
        "tilt_p95_deg": _r(_pct(tilt, 95), 2), "tilt_max_deg": _r(max(tilt, default=None), 2),
        "tilt_max_outdoor_deg": _r(max(zone[False], default=None), 2),
        "tilt_max_indoor_deg": _r(max(zone[True], default=None), 2),
        "roll_rate_p95_dps": _r(_pct(rates["roll"], 95), 2),
        "pitch_rate_p95_dps": _r(_pct(rates["pitch"], 95), 2),
        "yaw_rate_p95_dps": _r(_pct(rates["yaw"], 95), 2),
        "yaw_rate_max_dps": _r(max(rates["yaw"], default=None), 2),
        "accel_xy_p95": _r(_pct(acc, 95)), "accel_xy_max": _r(max(acc, default=None)),
        "jerk_xy_p95": _r(_pct(jerk, 95)), "jerk_xy_max": _r(max(jerk, default=None)),
        "min_upright_cos": _r(min(truth.up, default=None)),
    }


def motion(truth, stops):
    """Speed profile: how fast, and how often the drone slows hard mid-leg (stop-and-go)."""
    idx = [i for i in range(len(truth.t)) if truth.airborne(i)]
    if not idx:
        return {}
    speed = [math.hypot(truth.v[i][0], truth.v[i][1]) for i in idx]
    smooth = [sum(speed[max(0, k - 2):k + 3]) / len(speed[max(0, k - 2):k + 3]) for k in range(len(speed))]
    moving = [s for s in speed if s > 0.3]
    dips = 0
    for k in range(1, len(smooth) - 1):
        if not (smooth[k] <= smooth[k - 1] and smooth[k] < smooth[k + 1]):
            continue
        before = max(smooth[max(0, k - 60):k], default=0.0)
        after = max(smooth[k + 1:k + 61], default=0.0)
        ref = min(before, after)
        p = truth.p[idx[k]]
        if ref >= 0.8 and smooth[k] <= 0.5 * ref and all(math.dist(p[:2], g[:2]) > 2.0 for g in stops):
            dips += 1
    return {
        "airborne_s": round(truth.t[idx[-1]] - truth.t[idx[0]], 1),
        "speed_moving_mean": _r(sum(moving) / len(moving) if moving else None),
        "speed_p95": _r(_pct(speed, 95)), "speed_max": _r(max(speed)),
        "speed_dips": dips,
    }


# --- estimator -----------------------------------------------------------------------------------

def _ekf_samples(telemetry, pad_z=0.09):
    out, variance, last_t = [], [], None
    for m in telemetry:
        kind = m.get("mavpackettype")
        if "time_boot_ms" in m:
            last_t = m["time_boot_ms"] / 1000.0
        if kind == "LOCAL_POSITION_NED":
            out.append((m["time_boot_ms"] / 1000.0, (m["x"], -m["y"], -m["z"] + pad_z), (m["vx"], -m["vy"], -m["vz"])))
        elif kind == "EKF_STATUS_REPORT" and last_t is not None:
            variance.append((last_t, m["pos_horiz_variance"], m["velocity_variance"]))
    return out, variance


def _source_at(switches, t):
    src = 1
    for sw in switches:
        if sw["sim_time"] is not None and sw["sim_time"] <= t:
            src = sw["source"]
    return src


def _lag(samples, truth, series_index=1, lo=-0.1, hi=0.5):
    """Delay d minimising horizontal error between an estimate at t and truth at t - d (moving samples)."""
    moving = [s for s in samples if math.hypot(*truth.at(s[0], truth.v)[:2]) > 0.5]
    if len(moving) < 30:
        return None
    best = (float("inf"), None)
    steps = int(round((hi - lo) / 0.005))
    for k in range(steps + 1):
        d = lo + k * 0.005
        err = 0.0
        for s in moving:
            p = truth.at(s[0] - d)
            err += (s[series_index][0] - p[0]) ** 2 + (s[series_index][1] - p[1]) ** 2
        if err < best[0]:
            best = (err, d)
    return best[1]


def estimator(truth, telemetry, report, lio):
    ekf, variance = _ekf_samples(telemetry)
    switches = report.get("ekf_source_switches", [])
    out = {}
    for src in (1, 2):
        sel = [s for s in ekf if truth.airborne_at(s[0]) and _source_at(switches, s[0]) == src]
        perr = [math.dist(s[1][:2], truth.at(s[0])[:2]) for s in sel]
        verr = [math.dist(s[2][:2], truth.at(s[0], truth.v)[:2]) for s in sel]
        key = f"src{src}"
        out[f"{key}_ekf_pos_err_mean"] = _r(sum(perr) / len(perr) if perr else None)
        out[f"{key}_ekf_pos_err_p95"] = _r(_pct(perr, 95))
        out[f"{key}_ekf_pos_err_max"] = _r(max(perr, default=None))
        out[f"{key}_ekf_vel_err_rms"] = _r(math.sqrt(sum(e * e for e in verr) / len(verr)) if verr else None)
        out[f"{key}_ekf_lag_s"] = _r(_lag(sel, truth))
    var2 = [v[1] for v in variance if _source_at(switches, v[0]) == 2]
    out["ekf_pos_horiz_variance_max"] = _r(max((v[1] for v in variance), default=None), 4)
    out["src2_ekf_pos_horiz_variance_max"] = _r(max(var2, default=None), 4)
    lio_samples = [(s["time"], s["position"]) for s in lio if s.get("healthy") and truth.airborne_at(s["time"])]
    out["lio_lag_s"] = _r(_lag(lio_samples, truth, series_index=1))
    healthy = [bool(s.get("healthy")) for s in lio]
    streak = longest = 0
    for h in healthy:
        streak = 0 if h else streak + 1
        longest = max(longest, streak)
    out["lio_unhealthy_scans"] = healthy.count(False)
    out["lio_longest_unhealthy_scans"] = longest
    timing = report.get("timing") or {}
    for name in ("queue_lag_s", "send_lag_s"):
        vals = timing.get(name) or []
        out[f"{name[:-2]}_p50"] = _r(_pct(vals, 50))
        out[f"{name[:-2]}_p95"] = _r(_pct(vals, 95))
        out[f"{name[:-2]}_max"] = _r(max(vals, default=None))
    out["imu_sync_wait_max_s"] = _r(timing.get("sync_wait_max_s"))
    out["unsynced_scans"] = timing.get("unsynced_scans")
    return out


# --- arrivals ------------------------------------------------------------------------------------

PASS_EVENTS = ("EKF_SOURCE_1", "EKF_SOURCE_2", "STALL_RECOVERY")


def arrivals(truth, events):
    """Overshoot and back-and-forth at every arrival where the drone is meant to stop (hold or land)."""
    out = []
    for i, ev in enumerate(events):
        if ev.get("state") != "ARRIVED" or ev.get("sim_time") is None:
            continue
        nxt = next((e for e in events[i + 1:] if e.get("state") not in PASS_EVENTS), None)
        if nxt is None or nxt.get("state") not in ("HOLDING", "LANDING"):
            continue
        g = [float(c) for c in ev["detail"].split(",")]
        ta = ev["sim_time"]
        if nxt["state"] == "LANDING":
            t_end = nxt["sim_time"]
        else:
            later = [e["sim_time"] for e in events[i + 1:]
                     if e.get("state") == "OPERATOR_CMD" and e.get("sim_time") is not None]
            t_end = min(later[0] if later else truth.t[-1], ta + 20.0)
        p0 = truth.at(ta - 3.0)
        ax, ay = g[0] - p0[0], g[1] - p0[1]
        n = math.hypot(ax, ay)
        if n < 0.3:
            vx, vy = truth.at(ta - 1.0, truth.v)[:2]
            ax, ay, n = vx, vy, math.hypot(vx, vy)
        if n < 1e-3:
            continue
        ax, ay = ax / n, ay / n
        over, sign, reversals = 0.0, 0, 0
        for k, t in enumerate(truth.t):
            if t < ta - 5.0 or t > t_end:
                continue
            p, v = truth.p[k], truth.v[k]
            over = max(over, (p[0] - g[0]) * ax + (p[1] - g[1]) * ay)
            s = v[0] * ax + v[1] * ay
            if abs(s) > 0.10:
                cur = 1 if s > 0 else -1
                if sign and cur != sign:
                    reversals += 1
                sign = cur
        pe = truth.at(t_end)
        out.append({"time_s": round(ta, 1), "goal": [round(c, 2) for c in g[:2]], "end": nxt["state"],
                    "overshoot_m": round(over, 3), "reversals": reversals,
                    "final_error_m": round(math.hypot(pe[0] - g[0], pe[1] - g[1]), 3)})
    return out


def measure(run):
    run = Path(run)
    truth = Truth(_jsonl(run / "ground_truth.jsonl"))
    telemetry = _jsonl(run / "telemetry.jsonl")
    report_path = run / "autonomy_report.json"
    report = json.loads(report_path.read_text()) if report_path.is_file() else {}
    lio = _jsonl(run / "lio_trajectory.jsonl")
    obstacles_path = run / "world-obstacles.json"
    obstacles = json.loads(obstacles_path.read_text()) if obstacles_path.is_file() else {}
    events = report.get("events", [])
    arr = arrivals(truth, events)
    out = {}
    out.update(airframe(truth, telemetry))
    out.update(motion(truth, [a["goal"] for a in arr] + [truth.p[0][:2]]))
    out.update(estimator(truth, telemetry, report, lio))
    out.update(clearance(truth, obstacles) if obstacles else {"min_clearance_m": None})
    out["airborne_contacts"] = len(truth.contacts)
    out["stop_overshoot_max_m"] = _r(max((a["overshoot_m"] for a in arr), default=None))
    out["stop_reversals_max"] = max((a["reversals"] for a in arr), default=None)
    out["stop_final_error_max_m"] = _r(max((a["final_error_m"] for a in arr), default=None))
    out["stops"] = arr
    out["stall_recoveries"] = sum(1 for e in events if e.get("state") == "STALL_RECOVERY")
    result_path = run / "result.json"
    if result_path.is_file():
        out["mission_passed"] = int(bool(json.loads(result_path.read_text()).get("passed")))
    legs = _jsonl(run / "tour.jsonl")
    if legs:
        out["tour_legs_arrived"] = sum(1 for leg in legs if leg.get("arrived"))
        out["tour_landed"] = int(any(leg.get("landed") for leg in legs))
    return out


def merge(runs):
    """One label's repeats: the worst value of each safety metric, the mean of everything else."""
    out = {}
    for key in {k for r in runs for k in r if k != "stops"}:
        vals = [r[key] for r in runs if isinstance(r.get(key), (int, float)) and not isinstance(r.get(key), bool)]
        if not vals:
            out[key] = runs[0].get(key)
        elif key in GATE_SAFETY:
            out[key] = max(vals) if GATE_SAFETY[key][0] == "max" else min(vals)
        else:
            out[key] = _r(sum(vals) / len(vals))
    out["repeats"] = len(runs)
    return out


# --- gate ----------------------------------------------------------------------------------------
# Safety metrics may not get worse than the baseline's worst run by more than the stated slack;
# hard limits hold regardless. Smoothness metrics are reported against the baseline (lower is better).

GATE_SAFETY = {  # metric: (direction, slack); "max" = larger is worse, "min" = smaller is worse
    "airborne_contacts": ("max", 0),
    "min_clearance_m": ("min", 0.05),
    "min_upright_cos": ("min", 0.03),
    "src2_ekf_pos_err_p95": ("max", 0.03),
    "src2_ekf_pos_err_max": ("max", 0.05),
    "src2_ekf_vel_err_rms": ("max", 0.03),
    "src2_ekf_pos_horiz_variance_max": ("max", 0.02),
    "stop_overshoot_max_m": ("max", 0.05),
    "stop_reversals_max": ("max", 0),
    "stop_final_error_max_m": ("max", 0.05),
    "lio_longest_unhealthy_scans": ("max", 1),
    "unsynced_scans": ("max", 0),
    "stall_recoveries": ("max", 0),
    "mission_passed": ("min", 0),
    "tour_legs_arrived": ("min", 0),
    "tour_landed": ("min", 0),
}
GATE_SMOOTH = ("tilt_p95_deg", "roll_rate_p95_dps", "pitch_rate_p95_dps", "yaw_rate_p95_dps", "jerk_xy_p95",
               "speed_dips")
GATE_INFO = ("airborne_s", "speed_moving_mean", "speed_max", "tilt_max_deg", "tilt_max_outdoor_deg",
             "tilt_max_indoor_deg", "accel_xy_p95", "src2_ekf_lag_s", "lio_lag_s", "queue_lag_p95", "send_lag_p95", "imu_sync_wait_max_s", "lio_unhealthy_scans")


def gate(base, new):
    """Compare measurement dicts {label: measure()} against baseline dicts; returns (lines, failed)."""
    lines, failed = [], False
    for label, m in new.items():
        b = base.get(label)
        if b is None:
            lines.append(f"{label}: no baseline")
            continue
        for key, (direction, slack) in GATE_SAFETY.items():
            bv, nv = b.get(key), m.get(key)
            if bv is None or nv is None:
                if nv is None and bv is not None:
                    lines.append(f"FAIL {label} {key}: missing (baseline {bv})")
                    failed = True
                continue
            worse = nv > bv + slack if direction == "max" else nv < bv - slack
            if worse:
                failed = True
            lines.append(f"{'FAIL' if worse else 'ok  '} {label} {key}: {bv} -> {nv}")
        for key in GATE_SMOOTH + GATE_INFO:
            lines.append(f"     {label} {key}: {b.get(key)} -> {m.get(key)}")
    return lines, failed


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("runs", nargs="+", help="LABEL=RUN_DIR or RUN_DIR; repeats of a label are merged")
    parser.add_argument("--save", type=Path, help="write {label: measurements} JSON")
    parser.add_argument("--gate", type=Path, help="baseline JSON written by --save")
    args = parser.parse_args()
    grouped = {}
    for spec in args.runs:
        label, _, path = spec.rpartition("=")
        grouped.setdefault(label or Path(path).name, []).append(measure(path))
    results = {label: runs[0] if len(runs) == 1 else merge(runs) for label, runs in grouped.items()}
    if args.save:
        args.save.write_text(json.dumps(results, indent=2) + "\n")
    if args.gate:
        lines, failed = gate(json.loads(args.gate.read_text()), results)
        print("\n".join(lines))
        print("GATE: " + ("FAIL" if failed else "PASS"))
        return 1 if failed else 0
    print(json.dumps(results, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())

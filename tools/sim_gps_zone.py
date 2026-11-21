"""Simulator-side GPS denial zone driver (runs in SITL container on MAVLink serial1).

Watches gps-denied.json written by Webots' flight_observer supervisor and toggles SITL's
SIM_GPS1_ENABLE / SIM_GPS1_NUMSATS at the measurement layer. Strictly part of the simulated
environment — isolated on TCP 5762 so autonomy on TCP 5760 only sees real GPS_RAW_INT loss.
"""
import argparse
import json
import time
from pathlib import Path

from pymavlink import mavutil


def run_gps_injector(run_dir, endpoint="tcp:127.0.0.1:5762"):
    run = Path(run_dir)
    conn = None
    deadline = time.monotonic() + 60.0
    while time.monotonic() < deadline and not (run / "mission-done").exists():
        try:
            conn = mavutil.mavlink_connection(endpoint, source_system=254, retries=0)
            break
        except OSError:
            time.sleep(0.4)
    if conn is None:
        return
    hb = conn.wait_heartbeat(timeout=30)
    if hb is None:
        conn.close()
        return
    conn.target_system = hb.get_srcSystem()
    conn.target_component = hb.get_srcComponent()

    flag_path = run / "gps-denied.json"
    done_path = run / "mission-done"
    applied = None
    while not done_path.exists():
        conn.recv_match(blocking=False)
        if flag_path.is_file():
            try:
                denied = bool(json.loads(flag_path.read_text())["denied"])
            except (OSError, ValueError, KeyError):
                denied = applied
            if denied is not None and denied != applied:
                enable_val = 0.0 if denied else 1.0
                sats_val = 0.0 if denied else 10.0
                for pname, pval in ((b"SIM_GPS1_NUMSATS", sats_val), (b"SIM_GPS1_ENABLE", enable_val)):
                    conn.mav.param_set_send(
                        conn.target_system, conn.target_component,
                        pname, pval, mavutil.mavlink.MAV_PARAM_TYPE_INT8,
                    )
                print(f"[sim-env] GPS {'DENIED' if denied else 'RESTORED'} "
                      f"(SIM_GPS1_ENABLE={int(enable_val)}, NUMSATS={int(sats_val)})", flush=True)
                applied = denied
        time.sleep(0.05)
    conn.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    args = parser.parse_args()
    run_gps_injector(args.run_dir)


if __name__ == "__main__":
    main()

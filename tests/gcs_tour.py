"""Fly a multi-sector Go-To tour through the Supervisory GCS, the way an operator clicks it.

Start the world with `./ironlark run --standby`, then run this against its GCS. Each leg is a Go-To
into one of the four GPS-denied interiors (seed-7 layout); the tour ends with Return Home and landing.
The drone's own state and events are polled from /api/state; tour.jsonl in the run folder records
legs and outcomes. Exit status 0 only when every leg arrived and the drone landed.
"""
import argparse
import json
import sys
import time
import urllib.request
from pathlib import Path

LEGS = (
    ("east_bunker", 35.5, 0.5),
    ("north_deck", 2.6, 31.2),
    ("west_mine", -35.5, 0.5),
    ("south_warehouse", -6.5, -31.0),
)


def _get(url):
    with urllib.request.urlopen(url + "/api/state", timeout=5) as r:
        return json.loads(r.read())


def _post(url, cmd):
    req = urllib.request.Request(url + "/api/command", data=json.dumps(cmd).encode(),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.loads(r.read())


class Tour:
    def __init__(self, url, log):
        self.url = url
        self.log = log
        self.seen = set()
        self.events = []
        self.state = None

    def poll(self):
        self.state = _get(self.url)
        for ev in self.state.get("events", []):
            key = (ev.get("state"), ev.get("sim_time"), ev.get("wall_seconds"))
            if key not in self.seen:
                self.seen.add(key)
                self.events.append(ev)
        return self.state

    def record(self, entry):
        print(json.dumps(entry), flush=True)
        self.log.write(json.dumps(entry) + "\n")
        self.log.flush()

    def wait(self, done, wall_timeout):
        deadline = time.monotonic() + wall_timeout
        while time.monotonic() < deadline:
            try:
                st = self.poll()
            except OSError:
                time.sleep(0.5)
                continue
            if done(st):
                return st
            time.sleep(0.25)
        return None

    def leg(self, name, x, y, settle_s=4.0):
        mark = len(self.events)
        t0 = self.state.get("sim_time") or 0.0
        _post(self.url, {"cmd": "goto", "x": x, "y": y})
        # The executive leaves GOTO for HOLDING whether it arrived or gave up; ARRIVED tells which.
        self.wait(lambda st: st["phase"] == "GOTO", 120.0)
        st = self.wait(lambda st: st["phase"] in ("HOLDING", "STANDBY"), 240.0)
        arrived = any(e.get("state") == "ARRIVED" for e in self.events[mark:])
        t1 = (st or self.state).get("sim_time") or t0
        settle_until = t1 + settle_s
        self.wait(lambda s: (s.get("sim_time") or 0.0) >= settle_until, 30.0)
        self.record({"leg": name, "goal": [x, y], "arrived": arrived, "sim_start": t0, "sim_end": t1,
                     "pos_end": self.state.get("pos")})
        return arrived


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:8080")
    parser.add_argument("--run", type=Path, required=True, help="run folder of the live flight")
    args = parser.parse_args()
    with (args.run / "tour.jsonl").open("w") as log:
        tour = Tour(args.url, log)
        if tour.wait(lambda st: st["phase"] == "STANDBY" and st["lio_scans"] >= 2, 180.0) is None:
            print("GCS never reached STANDBY", file=sys.stderr)
            return 1
        ok = all([tour.leg(*leg) for leg in LEGS])
        _post(args.url, {"cmd": "rth"})
        st = tour.wait(lambda s: s["phase"] == "STANDBY" and not s["armed"], 300.0)
        landed = st is not None
        tour.record({"leg": "home", "landed": landed, "pos_end": tour.state.get("pos")})
        time.sleep(2.0)  # the executive writes autonomy_report.json right after landing
        return 0 if ok and landed else 1


if __name__ == "__main__":
    sys.exit(main())

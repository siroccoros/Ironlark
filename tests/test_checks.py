"""The checks' own verdicts: each refuses evidence that is missing or wrong."""
import importlib.util
import json
import subprocess
import sys
import unittest
from importlib.machinery import SourceFileLoader
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import check_recording  # noqa: E402
import check_replay  # noqa: E402

# flight_smoke runs in the SITL container beside pymavlink, which this environment lacks; judge() never uses it.
sys.modules.setdefault("pymavlink", mock.MagicMock())
sys.path.insert(0, str(ROOT / "tests"))
import flight_smoke  # noqa: E402
import run_resources  # noqa: E402

loader = SourceFileLoader("launcher", str(ROOT / "ironlark"))  # the launcher script, as a module
launcher = importlib.util.module_from_spec(importlib.util.spec_from_loader("launcher", loader))
loader.exec_module(launcher)


def bag(**extra):
    """Digests per topic for a small flight: every topic present, two messages each."""
    topics = {topic: [f"{topic}-{i}".encode() for i in range(2)] for topic in check_replay.TOPICS}
    topics.update(extra)
    return topics


class Replay(unittest.TestCase):
    def test_identical_bags_pass(self):
        self.assertTrue(check_replay.compare(bag(), bag())["passed"])

    def test_a_topic_missing_from_both_bags_fails(self):
        original, replayed = bag(), bag()
        del original["/lidar/points"], replayed["/lidar/points"]
        report = check_replay.compare(original, replayed)
        self.assertFalse(report["passed"])
        self.assertEqual(report["topics"]["/lidar/points"], {"recorded": 0, "replayed": 0, "equal": False})

    def test_a_message_lost_in_the_replay_fails_where_it_was_lost(self):
        report = check_replay.compare(bag(), bag(**{"/imu/data": [b"/imu/data-0"]}))
        self.assertFalse(report["passed"])
        self.assertEqual(report["topics"]["/imu/data"]["first_difference"], 1)

    def test_a_changed_payload_fails_although_the_count_matches(self):
        report = check_replay.compare(bag(), bag(**{"/camera/image_raw": [b"/camera/image_raw-0", b"other"]}))
        self.assertFalse(report["passed"])
        self.assertEqual(report["topics"]["/camera/image_raw"]["first_difference"], 1)


class Recording(unittest.TestCase):
    PERIODS = check_recording.periods(2, json.loads((ROOT / "config/sensors.json").read_text()))

    def stamps(self, seconds=2.0):
        """A complete recording: each topic every period, from the first clock tick to the last."""
        end = int(seconds * 1e9)
        out = {topic: list(range(period, end + 1, period)) for topic, period in self.PERIODS.items()}
        out["/tf_static"] = [None]
        return out

    def problems(self, stamps, heights=(0.05, 3.0, 0.05)):
        return check_recording.completeness(stamps, self.PERIODS, list(heights))

    def test_periods_come_from_the_rig_and_the_physics_step(self):
        ms = {topic: period // 1_000_000 for topic, period in self.PERIODS.items()}
        self.assertEqual(ms, {"/clock": 2, "/imu/data": 4, "/ground_truth/pose": 4, "/ground_truth/velocity": 4,
                              "/lidar/points": 100, "/camera/image_raw": 50, "/camera/camera_info": 50})

    def test_a_complete_flight_passes(self):
        self.assertEqual(self.problems(self.stamps()), [])

    def test_a_missing_message_is_counted(self):
        stamps = self.stamps()
        del stamps["/imu/data"][100:103]
        self.assertEqual(self.problems(stamps), ["/imu/data: 3 missing and 0 out of order or repeated, at 4 ms"])

    def test_a_repeated_message_fails(self):
        stamps = self.stamps()
        stamps["/camera/image_raw"].insert(5, stamps["/camera/image_raw"][5])
        self.assertIn("/camera/image_raw: 0 missing and 1 out of order or repeated, at 50 ms", self.problems(stamps))

    def test_a_topic_without_messages_fails(self):
        stamps = self.stamps()
        stamps["/tf_static"] = []
        self.assertEqual(self.problems(stamps), ["/tf_static: no messages"])

    def test_a_topic_that_starts_late_or_ends_early_fails(self):
        stamps = self.stamps()
        stamps["/lidar/points"] = stamps["/lidar/points"][2:]
        stamps["/camera/image_raw"] = stamps["/camera/image_raw"][:-2]
        problems = self.problems(stamps)
        self.assertTrue(any(p.startswith("/lidar/points: covers 0.300") for p in problems), problems)
        self.assertTrue(any(p.startswith("/camera/image_raw: covers") for p in problems), problems)

    def test_a_recording_without_the_whole_flight_fails(self):
        for heights in ((0.05, 1.0, 0.05), (3.0, 3.0, 0.05), (0.05, 3.0, 2.0)):
            self.assertEqual(len(self.problems(self.stamps(), heights)), 1, heights)


class Sensors(unittest.TestCase):
    """check_recording judges what the sensors measured, not only when."""
    CONFIG = json.loads((ROOT / "config/sensors.json").read_text())

    def at_rest(self, count=2500, gravity=9.81, turning=0.0, tilt=0.0, noise=1.0, up=None):
        """IMU samples of a drone standing still, tilted tilt degrees about x, with the rig's configured noise times
        noise; up: where in the IMU frame its accelerometer reads gravity instead of up in the world."""
        import math
        import random

        rng, imu = random.Random(0), self.CONFIG["imu"]
        a, g = imu["accelerometer_noise"] * noise, imu["gyro_noise"] * noise
        angle = math.radians(tilt)
        orientation = (math.sin(angle / 2), 0.0, 0.0, math.cos(angle / 2))
        up = up or (0.0, math.sin(angle), math.cos(angle))  # the world's z in a frame turned by angle about x
        return [(tuple(gravity * u + rng.gauss(0, a) for u in up),
                 (rng.gauss(0, g), rng.gauss(0, g), turning + rng.gauss(0, g)), orientation) for _ in range(count)]

    def test_an_imu_at_rest_passes(self):
        self.assertEqual(check_recording.imu_problems(self.at_rest(), self.CONFIG), [])
        self.assertEqual(check_recording.imu_problems(self.at_rest(tilt=1.5), self.CONFIG), [])

    def test_an_imu_that_misreads_rest_fails(self):
        for change, words in (({"gravity": 9.6}, "gravity"), ({"turning": 0.01}, "turns"), ({"tilt": 5.0}, "tilted"),
                              ({"up": (-1.0, 0.0, 0.0)}, "deg from where its orientation puts it"),
                              ({"up": (0.0, 0.0, -1.0)}, "180.0 deg from"),
                              ({"noise": 3.0}, "spreads 3.0"), ({"noise": 0.0}, "spreads 0.00"),
                              ({"count": 100}, "fewer than a second")):
            problems = check_recording.imu_problems(self.at_rest(**change), self.CONFIG)
            self.assertTrue(any(words in p for p in problems), (change, problems))

    def wall(self, scatter):
        """The wall's face at x = 5.9 m, 2 x 2 m of returns, each off by scatter() in x."""
        import random

        rng = random.Random(1)
        points = [(5.9 + scatter(rng), y / 10, 0.5 + z / 10) for y in range(-10, 10) for z in range(20)]
        distance, angle, residual = check_recording.fit_wall(points)
        return {"distance_error_m": abs(distance - 5.9), "normal_error_deg": angle,
                "residual_over_noise": residual / self.CONFIG["lidar"]["noise"], "camera_points": 400,
                "camera_agreement": 0.99, "ground_points": 3000, "background_red": 0.0}

    def test_a_wall_seen_with_the_rig_noise_passes(self):
        self.assertEqual(check_recording.wall_problems(self.wall(lambda rng: rng.gauss(0, 0.02))), [])

    def test_returns_scattered_around_the_right_plane_fail(self):
        report = self.wall(lambda rng: rng.uniform(-0.9, 0.9))
        self.assertLess(report["distance_error_m"], 0.05)  # the fit alone cannot tell
        self.assertTrue(any("scatter" in p for p in check_recording.wall_problems(report)))

    def test_a_camera_that_sees_everything_red_fails(self):
        report = self.wall(lambda rng: rng.gauss(0, 0.02)) | {"background_red": 1.0, "camera_agreement": 1.0}
        self.assertEqual(check_recording.wall_problems(report), ["100.0% of the ground in front of the wall looks red"])


class Stream(unittest.TestCase):
    """The launcher's reading of the sensor stream's account (sensing.json)."""

    def account(self, **change):
        return {"queued": 1000, "sent": 1000, "acknowledged": 1000, "dropped": 0, "failure": None} | change

    def test_a_stream_that_delivered_everything_passes(self):
        self.assertEqual(launcher.sensing_losses(self.account()), [])

    def test_each_loss_is_named(self):
        self.assertEqual(launcher.sensing_losses(self.account(dropped=3)), ["3 messages dropped on a full queue"])
        self.assertEqual(launcher.sensing_losses(self.account(acknowledged=998)),
                         ["the ROS bridge read 998 of 1000 messages"])
        self.assertEqual(launcher.sensing_losses(self.account(acknowledged=None)),
                         ["the ROS bridge never acknowledged the stream"])

    def test_a_lost_bridge_names_the_failure_and_what_it_stranded(self):
        failure = "lost the ROS bridge after 990 messages ([Errno 32] Broken pipe)"
        losses = launcher.sensing_losses(self.account(sent=990, acknowledged=None, failure=failure))
        self.assertEqual(losses, [failure, "10 queued messages never sent"])


class Flight(unittest.TestCase):
    """flight_smoke judges the flight on Webots' trajectory, never on the autopilot's telemetry."""

    def trajectory(self, hover_s=6.0, final=0.017, gap=None, start=0.1):
        """Every 100 ms: 10 s on the pad, a 3 s climb to 3 m, the hover, a 4 s descent, 5 s on the pad."""
        out, steps = [], round((10 + 3 + hover_s + 4 + 5) * 10)
        for i in range(steps):
            at = round(start + i / 10, 1)
            phase = at - 10
            height = (0.017 if phase < 0 else 0.017 + phase * 1.0 if phase < 3 else 3.0 if phase < 3 + hover_s
                      else 3.0 - (phase - 3 - hover_s) * 0.75 if phase < 7 + hover_s else final)
            if gap is None or not gap[0] <= at < gap[1]:
                out.append({"time": at, "position": [0.01 * phase if phase > 0 else 0.0, 0.0, height]})
        return out

    def judge(self, truth):
        return flight_smoke.judge(truth, truth[-1]["time"] + 0.05)

    def test_a_flight_passes(self):
        result = self.judge(self.trajectory())
        self.assertGreaterEqual(result["hover_s"], 6.0)
        self.assertAlmostEqual(result["peak_height_m"], 3.0)

    def test_landing_underground_fails(self):
        with self.assertRaisesRegex(RuntimeError, "did not physically land: final height -1.00"):
            self.judge(self.trajectory(final=-1.0))

    def test_a_hover_the_drone_did_not_hold_fails(self):
        with self.assertRaisesRegex(RuntimeError, "No physical 5-second hover"):
            self.judge(self.trajectory(hover_s=1.0))

    def test_a_trajectory_with_a_gap_or_a_late_start_fails(self):
        with self.assertRaisesRegex(RuntimeError, "1 gaps, the first from 13.9 to 15.0 s"):
            self.judge(self.trajectory(gap=(14.0, 15.0)))
        with self.assertRaisesRegex(RuntimeError, "starts at 2.0 s"):
            self.judge(self.trajectory(start=2.0))

    def test_clocks_apart_fail(self):
        truth = self.trajectory()
        with self.assertRaisesRegex(RuntimeError, "clock mismatch"):
            flight_smoke.judge(truth, truth[-1]["time"] + 2.0)


class Resources(unittest.TestCase):
    """run_resources measures real processes, and names what it could not measure."""

    def test_a_session_is_measured_whole(self):
        hold = subprocess.Popen([sys.executable, "-c", "x = bytearray(64 << 20); import time; time.sleep(5)"],
                                start_new_session=True)
        self.addCleanup(hold.wait)
        self.addCleanup(hold.kill)
        sampler = run_resources.Sampler(sessions={"webots": hold.pid}, containers={"sitl": "ironlark-no-such-container"})
        for _ in range(200):  # until the bytes are allocated
            sampler.sample()
            if sampler.peak["webots"] >= 64 << 20:
                break
        result = sampler.result()
        self.assertGreaterEqual(result["peak_rss_mib"]["webots"], 64)
        self.assertIn("sitl (never sampled)", result["missing"])

    def test_framebuffers_are_read_per_process(self):
        pmon = "# gpu pid type fb ccpm command\n# Idx # C/G MB MB name\n    0   1032  G  180   0  Hyprland\n    0  4321  G  -  -  x\n"
        self.assertEqual(run_resources.framebuffers(pmon), {1032: 180})


if __name__ == "__main__":
    unittest.main()

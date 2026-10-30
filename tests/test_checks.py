"""The checks' own verdicts: each refuses evidence that is missing or wrong."""
import contextlib
import importlib.util
import io
import json
import os
import subprocess
import sys
import tempfile
import unittest
from importlib.machinery import SourceFileLoader
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import check_recording  # noqa: E402
import check_replay  # noqa: E402
import compare_views  # noqa: E402
import frame_log  # noqa: E402

# flight_smoke runs in the SITL container beside pymavlink, which this environment lacks; judge() never uses it.
sys.modules.setdefault("pymavlink", mock.MagicMock())
sys.path.insert(0, str(ROOT / "tests"))
import flight_smoke  # noqa: E402
import profile_forest  # noqa: E402
import run_resources  # noqa: E402
from PIL import Image  # noqa: E402

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
    CONFIG = json.loads((ROOT / "config/sensors.json").read_text())
    PERIODS = check_recording.periods(2, CONFIG)

    def stamps(self, seconds=2.0):
        """A complete recording: each topic every period, from the first clock tick to the last."""
        end = int(seconds * 1e9)
        out = {topic: list(range(period, end + 1, period)) for topic, period in self.PERIODS.items()}
        out["/tf_static"] = [None]
        return out

    def problems(self, stamps, heights=(0.05, 3.0, 0.05)):
        return check_recording.completeness(stamps, self.PERIODS, list(heights), check_recording.lags(self.CONFIG))

    def test_periods_come_from_the_rig_and_the_physics_step(self):
        ms = {topic: period // 1_000_000 for topic, period in self.PERIODS.items()}
        self.assertEqual(ms, {"/clock": 2, "/imu/data": 4, "truth.bin": 2,
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

    def test_the_lidar_may_end_a_frame_and_half_a_render_before_the_clock(self):
        stamps = self.stamps()
        stamps["/lidar/points"] = stamps["/lidar/points"][:-1]  # its last frame was not whole when the clock stopped
        self.assertEqual(self.problems(stamps), [])
        stamps["/lidar/points"] = stamps["/lidar/points"][:-2]
        self.assertTrue(any(p.startswith("/lidar/points: covers") for p in self.problems(stamps)))

    def test_lidar_frames_hold_the_patterns_times_and_lines(self):
        import numpy as np
        from sensing import mid360
        points = np.zeros(4, mid360.POINT)
        points["layer"], points["time"] = [0, 1, 2, 3], [0.0, 0.01, 0.05, 0.0999]
        self.assertEqual(check_recording.frame_problems(points, self.CONFIG), [])
        points["time"][2] = 0.1
        self.assertIn("beyond the frame", check_recording.frame_problems(points, self.CONFIG)[0])
        points["time"][2], points["layer"][2] = 0.05, 4
        self.assertIn("lines [0, 1, 3, 4]", check_recording.frame_problems(points, self.CONFIG)[0])

    def test_a_recording_without_the_whole_flight_fails(self):
        for heights in ((0.05, 1.0, 0.05), (3.0, 3.0, 0.05), (0.05, 3.0, 2.0)):
            self.assertEqual(len(self.problems(self.stamps(), heights)), 1, heights)


class Sensors(unittest.TestCase):
    """check_recording judges what the sensors measured, not only when."""
    CONFIG = json.loads((ROOT / "config/sensors.json").read_text())

    def at_rest(self, count=2500, gravity=9.81, turning=0.0, tilt=0.0, noise=1.0, up=None, offset=0.0):
        """IMU samples of a drone standing still, tilted tilt degrees about x, with the rig's configured noise times
        noise; up: where in the IMU frame its accelerometer reads gravity instead of up in the world; offset: each
        axis's bias, as a share of the part's tolerance."""
        import math
        import random

        rng, imu = random.Random(0), self.CONFIG["imu"]
        a, g = imu["accelerometer"]["noise"] * noise, imu["gyro"]["noise"] * noise
        ab, gb = imu["accelerometer"]["bias"] * offset, imu["gyro"]["bias"] * offset
        angle = math.radians(tilt)
        orientation = (math.sin(angle / 2), 0.0, 0.0, math.cos(angle / 2))
        up = up or (0.0, math.sin(angle), math.cos(angle))  # the world's z in a frame turned by angle about x
        return [(tuple(gravity * u + ab + rng.gauss(0, a) for u in up),
                 (gb + rng.gauss(0, g), gb + rng.gauss(0, g), turning + gb + rng.gauss(0, g)), orientation)
                for _ in range(count)]

    def test_an_imu_at_rest_passes(self):
        for change in ({}, {"tilt": 1.5}, {"offset": 0.99}, {"offset": -0.99}):
            self.assertEqual(check_recording.imu_problems(self.at_rest(**change), self.CONFIG), [], change)

    def test_an_imu_that_misreads_rest_fails(self):
        for change, words in (({"gravity": 9.0}, "acceleration z at rest is -0.8"), ({"turning": 0.05}, "angular velocity z"),
                              ({"offset": 1.5}, "beyond the part's"), ({"tilt": 5.0}, "tilted"),
                              ({"up": (-1.0, 0.0, 0.0)}, "acceleration x at rest is -9.8"),
                              ({"up": (0.0, 0.0, -1.0)}, "acceleration z at rest is -19.6"),
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
                "camera_agreement": 0.99, "ground_points": 3000, "background_red": 0.0,
                "turning": {"frames": 10, "residual_over_noise": 1.1, "at_frame_start_over_noise": 40.0}}

    def test_a_wall_seen_with_the_rig_noise_passes(self):
        self.assertEqual(check_recording.wall_problems(self.wall(lambda rng: rng.gauss(0, 0.02))), [])

    def test_returns_scattered_around_the_right_plane_fail(self):
        report = self.wall(lambda rng: rng.uniform(-0.9, 0.9))
        self.assertLess(report["distance_error_m"], 0.05)  # the fit alone cannot tell
        self.assertTrue(any("scatter" in p for p in check_recording.wall_problems(report)))

    def test_turning_frames_must_place_each_point_at_its_own_instant(self):
        report = self.wall(lambda rng: rng.gauss(0, 0.02))
        for turning, words in (({"frames": 1}, "fewer than 3"), ({"residual_over_noise": 4.0}, "turning, the wall's"),
                               ({"at_frame_start_over_noise": 1.2}, "not theirs")):
            problems = check_recording.wall_problems(report | {"turning": report["turning"] | turning})
            self.assertTrue(any(words in p for p in problems), (turning, problems))

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



class Views(unittest.TestCase):
    """compare_views scores every reference view, or refuses."""

    def setUp(self):
        scratch = tempfile.TemporaryDirectory()
        self.addCleanup(scratch.cleanup)
        self.reference, self.webots = Path(scratch.name, "reference"), Path(scratch.name, "webots")
        self.reference.mkdir(), self.webots.mkdir()
        for name, shade in (("forest", 90), ("drone", 160)):
            Image.new("RGB", (32, 24), (shade, shade, shade)).save(self.reference / f"{name}.png")
            Image.new("RGB", (32, 24), (shade, shade + 20, shade)).save(self.webots / f"{name}.png")

    def test_every_view_is_scored(self):
        with contextlib.redirect_stdout(io.StringIO()):
            compare_views.main([str(self.reference), str(self.webots)])
        report = json.loads((self.webots / "compare.json").read_text())
        self.assertEqual(sorted(report), ["drone", "forest"])
        self.assertGreater(report["forest"]["channel_ratio"][1], 1.0)

    def test_a_view_without_a_webots_render_fails(self):
        (self.webots / "drone.png").unlink()
        with self.assertRaisesRegex(SystemExit, "No Webots render of drone"):
            compare_views.main([str(self.reference), str(self.webots)])

    def tiles(self, change):
        """Changed tiles between a 240 x 120 capture and a copy changed by change(pixels)."""
        import numpy as np

        pixels = np.full((120, 240, 3), 90, dtype=np.uint8)
        Image.fromarray(pixels).save(self.reference / "forest.png")
        Image.fromarray(change(pixels.copy())).save(self.webots / "forest.png")
        (self.reference / "drone.png").unlink(missing_ok=True), (self.webots / "drone.png").unlink(missing_ok=True)
        with contextlib.redirect_stdout(io.StringIO()):
            compare_views.main([str(self.reference), str(self.webots), "--tiles"])
        return json.loads((self.webots / "compare.json").read_text())["forest"]["changed_tiles"]

    def test_repeatable_captures_list_the_tiles_that_changed(self):
        def missing_object(pixels):  # 12 x 12 px: 4% of one 60 px tile
            pixels[70:82, 130:142] = 20
            return pixels

        self.assertEqual(self.tiles(lambda pixels: pixels), [])
        self.assertEqual(self.tiles(missing_object), [[1, 2]])
        self.assertEqual(len(self.tiles(lambda pixels: pixels + 1)), 8)  # 1/255 brighter: every tile

    def test_captures_of_different_sizes_are_refused(self):
        Image.new("RGB", (64, 48), (90, 90, 90)).save(self.webots / "forest.png")
        with self.assertRaisesRegex(SystemExit, "forest: the reference is \\(32, 24\\), Webots \\(64, 48\\)"):
            compare_views.main([str(self.reference), str(self.webots), "--tiles"])

    def test_no_references_fails(self):
        for path in self.reference.iterdir():
            path.unlink()
        with self.assertRaisesRegex(SystemExit, "No reference renders"):
            compare_views.main([str(self.reference), str(self.webots)])



class FrameLog(unittest.TestCase):
    """A frame log's interval holds only the frames, steps and rendered views inside it."""

    def test_an_interval_without_frames_or_steps_has_no_views(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory, "frames.log")
            # Two main-view frames 100 ms apart at 1.0 and 1.1 simulated seconds, each rendered once.
            log.write_text("F 1000000000 1000000500 1000001000 1000.000\n"
                           "G main 1000000100 cpu 4000.0 gpu 2000.0 opaque 3000.0 1500.0\n"
                           "F 1100000000 1100000500 1100001000 1100.000\n"
                           "G main 1100000100 cpu 4000.0 gpu 2000.0 opaque 3000.0 1500.0\n")
            records = frame_log.parse(log)
        inside = frame_log.summarize(*records, 0.5, 2.0)
        self.assertEqual(inside["frames"]["count"], 2)
        self.assertEqual(inside["views"]["main"]["gpu_ms"]["n"], 2)
        self.assertEqual(frame_log.summarize(*records, 5.0, 6.0), {"views": {}})



# Stands in for Webots and the display it needs: writes what the profile's controller and frame log would.
# FAKE_WEBOTS: complete; complete, then exit 3; no frame log (stock Webots); frame log cut short; exits at once.
FAKE_WEBOTS = """#!/usr/bin/env python3
import json, os, sys, time
mode = os.environ["FAKE_WEBOTS"]
scene = json.load(open(os.environ["IRONLARK_SCENE"]))
seconds = float(os.environ["IRONLARK_SECONDS"])
lines = ["F 1 2 3 0.000", "F 4 5 6 0.000"]  # frames shown while the world loads, at simulated time 0
views = []
for k, name in enumerate([*scene["cameras"], "moving"]):
    start = k * seconds
    views.append({"view": name, "sim_start": start, "sim_seconds": seconds, "wall_seconds": seconds,
                  "real_time_factor": 1.0})
    for step in range(int(seconds * 500)):  # a 2 ms physics step; a main-view frame every 50 steps, rendered
        sim = start + step / 500
        ns = int(sim * 1e9) + 1
        lines.append(f"S {ns} {ns + 100} {sim * 1000:.3f}")
        if step % 50 == 0:
            lines += [f"F {ns + 200} {ns + 300} {ns + 400} {sim * 1000:.3f}",
                      f"G main {ns + 250} cpu 4000.0 gpu 2000.0 opaque 3000.0 1500.0"]
if mode == "frame log cut short":
    lines = lines[:-200]
if mode != "no frame log":
    open(os.environ["IRONLARK_FRAME_LOG"], "w").write("\\n".join(lines[:2] if mode == "exits at once" else lines) + "\\n")
if mode != "exits at once":
    json.dump({"first_step_wall": time.time(), "step_ms": 2, "views": views}, open(os.environ["IRONLARK_REPORT"], "w"))
if mode == "complete":
    time.sleep(0.1)  # alive through the profile's first resource sample
sys.exit(3 if mode.endswith("exit 3") else 0)
"""


# Stands in for the GPU: the card's memory, and 100 MiB of framebuffer for every process (the sampler counts Webots').
FAKE_NVIDIA_SMI = """#!/usr/bin/env python3
import os, sys
if "pmon" in sys.argv:
    print("# gpu        pid   type     fb   ccpm    command")
    for pid in filter(str.isdigit, os.listdir("/proc")):
        print(f"    0 {pid:>10}     G    100      0    process")
else:
    print(800)
"""


class Profile(unittest.TestCase):
    """profile_forest reports every view, measured whole from its frame log, or fails."""

    def setUp(self):
        scratch = tempfile.TemporaryDirectory()
        self.addCleanup(scratch.cleanup)
        root = Path(scratch.name)
        (root / "bin").mkdir()
        (root / "bin/nvidia-smi").write_text(FAKE_NVIDIA_SMI)
        (root / "bin/nvidia-smi").chmod(0o755)
        path = mock.patch.dict(os.environ, {"PATH": f"{root / 'bin'}:{os.environ['PATH']}"})
        path.start()
        self.addCleanup(path.stop)
        world = root / "worlds/pine_forest"
        (world / "meshes").mkdir(parents=True)
        (world / "pine_forest.wbt").write_text('Robot { controller "flight_observer" }\n')
        (world / "scene.json").write_text(json.dumps({"cameras": {"forest": {}, "backlit": {}}}))
        self.home, self.out = root / "webots", root / "profile"
        self.home.mkdir()
        (self.home / "webots").write_text(FAKE_WEBOTS)
        (self.home / "webots").chmod(0o755)
        patch = mock.patch.object(profile_forest, "ROOT", root)
        patch.start()
        self.addCleanup(patch.stop)

    def profile(self, webots):
        with mock.patch.dict(os.environ, {"FAKE_WEBOTS": webots}), contextlib.redirect_stdout(io.StringIO()):
            profile_forest.main(["--out", str(self.out), "--webots-home", str(self.home), "--seconds", "2"])
        return json.loads((self.out / "profile.json").read_text())

    def test_a_complete_profile_passes(self):
        report = self.profile("complete")
        self.assertEqual([view["view"] for view in report["views"]], ["forest", "backlit", "moving"])
        self.assertTrue(all((view["frames"]["count"], view["steps"]) == (10, 500) for view in report["views"]))

    def test_an_earlier_profile_never_counts_for_this_one(self):
        self.profile("complete")
        with self.assertRaisesRegex(RuntimeError, "Incomplete profile: no load time; forest not measured"):
            self.profile("exits at once")
        self.profile("complete")
        with self.assertRaisesRegex(RuntimeError, "Incomplete profile: forest has no frames"):
            self.profile("no frame log")

    def test_a_frame_log_cut_short_fails(self):
        with self.assertRaisesRegex(RuntimeError, "Incomplete profile: the frame log holds 306 of moving's 500"):
            self.profile("frame log cut short")

    def test_a_view_of_one_second_or_less_is_refused(self):  # its first second is left out
        with self.assertRaises(SystemExit), contextlib.redirect_stderr(io.StringIO()):
            profile_forest.main(["--out", str(self.out), "--seconds", "1"])

    def test_a_profile_without_its_resources_measured_fails(self):
        with mock.patch.dict(os.environ, {"PATH": "/usr/bin:/bin"}):  # no nvidia-smi on the path: no GPU
            with self.assertRaisesRegex(RuntimeError, "no measure of Webots' GPU memory"):
                self.profile("complete")

    def test_webots_failing_after_a_complete_profile_fails(self):
        with self.assertRaisesRegex(RuntimeError, "Webots exited with 3"):
            self.profile("complete, then exit 3")

    def test_each_gap_is_named(self):
        def view(name, seconds=2.0, frames=10, passes=True, steps=500, renders=10):
            return {"view": name, "sim_seconds": seconds, "frames": {"count": frames} if frames else None,
                    "main_passes_ms": {"opaque": {}} if passes else None, "steps": steps,
                    "main_gpu_ms": {"n": renders}}

        report = {"load_seconds": 5.0, "step_ms": 2, "resources": {"missing": []},
                  "views": [view("forest", 1.5), view("high", frames=None), view("moving", passes=False),
                            view("drone", steps=499), view("dusk", renders=8)]}
        self.assertEqual(profile_forest.incomplete(report, ["forest", "backlit", "high", "moving", "drone", "dusk"], 2.0),
                         ["forest ran 1.5 of 2.0 simulated seconds", "backlit not measured",
                          "high has no frames or render passes in the frame log",
                          "moving has no frames or render passes in the frame log",
                          "the frame log holds 499 of drone's 500 physics steps",
                          "dusk has render passes for 8 of its 10 frames"])


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

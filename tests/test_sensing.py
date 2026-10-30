"""Frames and wire format of the sensor stream (sensing/)."""
import json
import math
import socket
import struct
import sys
import threading
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from sensing import frames, protocol  # noqa: E402


def rotate(q, v):
    x, y, z, w = q
    r = frames.quaternion_multiply(frames.quaternion_multiply(q, (*v, 0.0)), (-x, -y, -z, w))
    return r[:3]


class Frames(unittest.TestCase):
    def assertVector(self, a, b):
        for p, q in zip(a, b):
            self.assertAlmostEqual(p, q, places=6)

    def test_enu_to_ned(self):
        self.assertEqual(frames.enu_to_ned((1, 2, 3)), (2, 1, -3))
        self.assertVector(rotate(frames.ENU_TO_NED, (1, 0, 0)), (0, 1, 0))  # east
        self.assertVector(rotate(frames.ENU_TO_NED, (0, 0, 1)), (0, 0, -1))  # up

    def test_flu_to_frd(self):
        self.assertEqual(frames.flu_to_frd((1, 2, 3)), (1, -2, -3))

    def test_attitude(self):
        # Nose east, level: yaw 90 degrees in NED.
        x, y, z, w = frames.attitude_enu_flu_to_ned_frd((0, 0, 0, 1))
        self.assertAlmostEqual(math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))), 90.0)

    def test_camera_optical_frame(self):
        o = frames.CAMERA_LINK_TO_OPTICAL
        self.assertVector(rotate(o, (0, 0, 1)), (1, 0, 0))  # optical z: forward
        self.assertVector(rotate(o, (1, 0, 0)), (0, -1, 0))  # optical x: right
        self.assertVector(rotate(o, (0, 1, 0)), (0, 0, -1))  # optical y: down

    def test_config_rates_fit_the_physics_step(self):
        config = json.loads((ROOT / "config/sensors.json").read_text())
        for key in ("imu", "lidar", "camera"):
            self.assertEqual((1000 / config[key]["rate_hz"]) % 2, 0, key)  # 2 ms basicTimeStep
        self.assertEqual((1000 / config["lidar"]["render_hz"]) % 4, 0)  # served half a render period on


class Protocol(unittest.TestCase):
    def test_truth_reads_back_and_a_cut_record_fails(self):
        import tempfile
        records = [protocol.TRUTH.pack(2_000_000 * i, *range(10)) for i in range(1, 4)]
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "truth.bin"
            path.write_bytes(b"".join(records))
            truth = protocol.read_truth(path)
            self.assertEqual([t for t, *_ in truth], [2_000_000, 4_000_000, 6_000_000])
            self.assertEqual(truth[0][1:], ((0, 1, 2), (3, 4, 5, 6), (7, 8, 9)))
            path.write_bytes(b"".join(records)[:-1])
            with self.assertRaisesRegex(ValueError, "cut short"):
                protocol.read_truth(path)

    def test_round_trip(self):
        a, b = socket.socketpair()
        payload = protocol.COUNT.pack(2) + struct.pack("<fffif", 1, 2, 3, 4, 0) * 2
        a.sendall(protocol.pack(protocol.LIDAR, 123_456_789, payload) + protocol.pack(protocol.CLOCK, 5))
        self.assertEqual(protocol.read_message(b), (protocol.LIDAR, 123_456_789, payload))
        self.assertEqual(protocol.read_message(b), (protocol.CLOCK, 5, b""))
        a.close()
        with self.assertRaises(EOFError):
            protocol.read_message(b)
        b.close()

    def test_finish_returns_what_the_reader_acknowledges(self):
        controller, bridge = socket.socketpair()
        for stamp in range(3):
            controller.sendall(protocol.pack(protocol.CLOCK, stamp))

        def read_to_the_end():  # the bridge's side (ros/ironlark_bridge.py, serve)
            received = 0
            try:
                while True:
                    protocol.read_message(bridge)
                    received += 1
            except EOFError:
                bridge.sendall(protocol.ACK.pack(received))
            bridge.close()

        reader = threading.Thread(target=read_to_the_end)
        reader.start()
        self.assertEqual(protocol.finish(controller), 3)
        reader.join()
        controller.close()

    def test_finish_fails_when_the_reader_closes_without_answering(self):
        controller, bridge = socket.socketpair()
        bridge.close()
        with self.assertRaises((EOFError, OSError)):
            protocol.finish(controller)
        controller.close()


class Mid360(unittest.TestCase):
    LIDAR = json.loads((ROOT / "config/sensors.json").read_text())["lidar"]

    @classmethod
    def wall(cls, x=5.0, rows=None, columns=None):
        """A panorama (the Webots Lidar's layout) of the plane x = const in front of the sensor, nothing else."""
        import numpy as np
        rows, columns = rows or cls.LIDAR["render_layers"], columns or cls.LIDAR["horizontal_resolution"]
        fov = cls.LIDAR["vertical_field_of_view"]
        azimuth = math.pi - (np.arange(columns) + 0.5) * 2 * math.pi / columns
        elevation = fov / 2 - np.arange(rows) * fov / (rows - 1)
        ux = np.cos(elevation)[:, None] * np.cos(azimuth)[None, :]
        with np.errstate(divide="ignore"):
            r = np.where(ux > 0, x / ux, np.inf)
        return np.where(r < cls.LIDAR["max_range"], r, np.inf).astype(np.float32)

    def test_the_pattern_fires_four_lines_at_200_khz_over_its_field(self):
        import numpy as np
        from sensing import mid360
        pattern = self.LIDAR["pattern"]
        k = mid360.slots(pattern, 0, 100_000_000)
        time_ns, line, rays = mid360.directions(pattern, k)
        self.assertTrue(19_000 < len(k) < 21_000)
        self.assertTrue(np.all(np.diff(time_ns) == pattern["slot_ns"]))
        self.assertEqual(list(line[:8]), [0, 1, 2, 3, 0, 1, 2, 3])
        elevation = np.degrees(np.arcsin(rays[:, 2]))
        self.assertTrue(-12 < elevation.min() < -2 and 45 < elevation.max() < 55, (elevation.min(), elevation.max()))
        azimuth = np.degrees(np.arctan2(rays[:, 1], rays[:, 0]))
        self.assertGreater(len(np.unique(np.floor(azimuth))), 359)  # all around

    def test_a_ray_from_a_moved_sensor_meets_the_rendered_plane(self):
        import numpy as np
        from sensing import mid360
        rng = np.random.default_rng(1)
        rays = mid360.unit(rng.uniform(-0.8, 0.8, 500), rng.uniform(-0.6, 0.6, 500))
        origins = rng.uniform(-0.05, 0.05, (500, 3))
        distance, cosine = mid360.cast(self.wall(), self.LIDAR["vertical_field_of_view"], origins, rays)
        expected = (5.0 - origins[:, 0]) / rays[:, 0]
        self.assertLess(np.max(np.abs(distance - expected)), 2e-4)
        self.assertLess(np.max(np.abs(cosine - rays[:, 0])), 1e-3)
        # Away from the wall the panorama saw nothing.
        nothing, _ = mid360.cast(self.wall(), self.LIDAR["vertical_field_of_view"], origins[:1], -rays[:1])
        self.assertTrue(np.isnan(nothing[0]))

    def test_frames_come_whole_with_each_point_at_its_instant(self):
        import numpy as np
        from sensing import mid360
        lidar = {**self.LIDAR, "noise": 0.0, "angular_noise": 0.0}
        sweep = mid360.Sweep(lidar, 2)
        frames_out = []
        # Flying at the wall at 1 m/s, level; renders every 20 ms from 20 ms (from where the sensor then was), each
        # served 10 ms later.
        for step in range(1, 206):
            t_ns = step * 2_000_000
            sweep.pose(t_ns, (t_ns * 1e-9, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
            if step >= 15 and step % 10 == 5:
                render_ns = t_ns - 10_000_000
                frames_out += sweep.render(render_ns, self.wall(5.0 - render_ns * 1e-9))
        starts = [start for start, _ in frames_out]
        self.assertEqual(starts, list(range(100_000_000, 400_000_000, 100_000_000)))
        mount = np.array(lidar["translation"])
        for start, points in frames_out:
            self.assertTrue(np.all((points["time"] >= 0) & (points["time"] < 0.1)))
            self.assertEqual(set(points["layer"]), {0, 1, 2, 3})
            # On the wall, placed with the sensor's pose at each point's own instant, and nearer than 40 m.
            # Exactly but at the render's 40 m edge, where no neighbour gives a plane.
            off = np.abs(points["x"] - (5.0 - mount[0] - (start * 1e-9 + points["time"]))) > 1e-3
            ranges = np.sqrt(points["x"] ** 2 + points["y"] ** 2 + points["z"] ** 2)
            self.assertTrue(np.all(ranges[off] > 39.0) and off.mean() < 0.005, (off.sum(), ranges[off].min(initial=40)))
            self.assertTrue(np.all(ranges <= 40.0))
            self.assertGreater(len(points), 3000)


if __name__ == "__main__":
    unittest.main()

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


if __name__ == "__main__":
    unittest.main()

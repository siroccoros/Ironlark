"""Sample the drone's sensors on Webots time and stream them to the ROS 2 bridge.

Called after every physics step. Each sensor is read at the steps its period divides (config/
sensors.json), stamped with that step's Webots time, and queued; a sender thread writes the queue
to the bridge over TCP (sensing/protocol.py). A full queue drops messages and counts them: the
stream never stalls physics, and pausing Webots stops its clock.

The stream starts when the bridge connects and ends when the launcher asks (sensing-end in the run
folder, after the flight) or when it fails: an error while sampling (a sensor without data among
them), or a lost connection. Ending, it stops sampling, sends what it queued, has the bridge
acknowledge every message (protocol.finish) and writes its account, sensing.json in the run folder:
messages queued, sent, acknowledged and dropped, and the failure. A failure is written at once. The
launcher fails a run that lost any message.
"""
import ctypes
import json
import random
import os
import queue
import socket
import sys
import threading
import time
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from sensing import protocol  # noqa: E402


class SensorStream:
    def __init__(self, robot, config_path=ROOT / "config/sensors.json"):
        from controller.wb import wb

        self._wb = wb
        self.robot = robot
        self.config = json.loads(Path(config_path).read_text())
        self.step_ms = int(robot.getBasicTimeStep())
        self.run = Path(os.environ["IRONLARK_RUN_DIR"])
        self.queued = self.sent = self.dropped = 0
        self.acknowledged = self.failure = None
        self.started = self.ended = False
        self.lock = threading.Lock()  # queueing against ending: the counts stop when the stream ends

        def period(rate_hz):
            ms = int(round(1000.0 / rate_hz))
            if ms % self.step_ms:
                raise ValueError(f"{rate_hz} Hz is not a whole number of {self.step_ms} ms steps")
            return ms

        imu, lidar, camera = self.config["imu"], self.config["lidar"], self.config["camera"]
        # IMU noise: absolute, white, from a fixed seed so a run's sensor data repeats.
        self.noise = random.Random(imu.get("seed", 0))
        self.accel_noise, self.gyro_noise = imu["accelerometer_noise"], imu["gyro_noise"]
        self.imu_period = period(imu["rate_hz"])
        self.accelerometer = robot.getDevice(f"{imu['name']} accelerometer")
        self.gyro = robot.getDevice(f"{imu['name']} gyro")
        self.attitude = robot.getDevice(f"{imu['name']} inertial unit")
        for device in (self.accelerometer, self.gyro, self.attitude):
            device.enable(self.imu_period)
        self.lidar_period = period(lidar["rate_hz"])
        self.lidar = robot.getDevice(lidar["name"])
        self.lidar.enable(self.lidar_period)
        self.lidar.enablePointCloud()
        self.camera_period = period(camera["rate_hz"])
        self.camera = robot.getDevice(camera["name"])
        self.camera.enable(self.camera_period)
        # Ground truth: the Iris's noiseless GPS and inertial unit (enabled by the flight bridge).
        self.gps = robot.getDevice("gps")
        self.truth_attitude = robot.getDevice("inertial unit")

        stream = self.config["stream"]
        self.address = (stream["host"], stream["port"])
        self.queue = queue.Queue(maxsize=stream["queue"])
        threading.Thread(target=self._send, daemon=True).start()

    def _put(self, message):
        with self.lock:
            if self.ended:
                return
            try:
                self.queue.put_nowait(message)
                self.queued += 1
            except queue.Full:
                self.dropped += 1

    def _end(self, failure=None):
        with self.lock:
            self.ended = True
            if failure and not self.failure:
                self.failure = failure
                print(f"sensing failed: {failure}", flush=True)
                self._account()  # now: the sender may be stuck on the bridge

    def _account(self):
        """Write sensing.json, holding the lock."""
        account = {"queued": self.queued, "sent": self.sent, "acknowledged": self.acknowledged,
                   "dropped": self.dropped, "failure": self.failure}
        temporary = self.run / "sensing.json.tmp"
        temporary.write_text(json.dumps(account, indent=2) + "\n")
        temporary.replace(self.run / "sensing.json")

    def _send(self):
        connection = None
        while connection is None and not self.ended:
            try:
                connection = socket.create_connection(self.address, timeout=2.0)
            except OSError:
                time.sleep(1.0)  # the bridge is not up (yet): nothing is sampled meanwhile
        if connection is not None:
            connection.settimeout(None)  # a slow bridge holds the sender back; the queue absorbs it or drops
            connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.started = True
            try:
                while not (self.ended and self.queue.empty()):
                    try:
                        message = self.queue.get(timeout=0.1)
                    except queue.Empty:
                        continue
                    connection.sendall(message)
                    self.sent += 1
                self.acknowledged = protocol.finish(connection)
            except (OSError, EOFError) as error:
                self._end(f"lost the ROS bridge after {self.sent} messages ({error or 'closed'})")
            connection.close()
        with self.lock:
            self._account()

    def after_step(self):
        """Queue what this step measured, from the bridge's connection to the stream's end."""
        if self.ended:
            return
        now_ms = None
        try:
            now_ms = int(round(self.robot.getTime() * 1000))
            if now_ms % 100 == 0 and (self.run / "sensing-end").exists():
                self._end(None if self.started else "the ROS bridge never connected")
            elif self.started:
                self._sample(now_ms)
        except Exception as error:
            traceback.print_exc()
            at = "" if now_ms is None else f" at {now_ms / 1000:.3f} s"
            self._end(f"{type(error).__name__} while sampling{at}: {error}")

    def _sample(self, now_ms):
        stamp = now_ms * 1_000_000
        if now_ms % 5000 == 0:
            # Webots and wall time side by side: the launcher derives sensing's cost from them.
            print(f"sensing: {self.sent} sent, {self.dropped} dropped, {self.queue.qsize()} queued "
                  f"at {now_ms / 1000:.0f} s, wall {time.monotonic():.3f}", flush=True)
        self._put(protocol.pack(protocol.CLOCK, stamp))
        if now_ms % self.imu_period == 0:
            q = self.attitude.getQuaternion()
            gyro = [v + self.noise.gauss(0.0, self.gyro_noise) for v in self.gyro.getValues()]
            accel = [v + self.noise.gauss(0.0, self.accel_noise) for v in self.accelerometer.getValues()]
            values = (*q, *gyro, *accel)
            self._put(protocol.pack(protocol.IMU, stamp, protocol.VECTOR10.pack(*values)))
            truth = (*self.gps.getValues(), *self.truth_attitude.getQuaternion(), *self.gps.getSpeedVector())
            self._put(protocol.pack(protocol.TRUTH, stamp, protocol.VECTOR10.pack(*truth)))
        if now_ms % self.lidar_period == 0:
            # Layer-major points, top layer first: the configured layers are the first ones.
            count = min(self.lidar.getNumberOfPoints(),
                        self.config["lidar"]["layers"] * self.lidar.getHorizontalResolution())
            pointer = self._wb.wb_lidar_get_point_cloud(self.lidar._tag)
            if not (count and pointer):
                raise RuntimeError("the LiDAR returned no points")
            self._put(_message(protocol.LIDAR, stamp, protocol.COUNT.pack(count),
                               pointer, count * protocol.LIDAR_POINT_BYTES))
        if now_ms % self.camera_period == 0:
            width, height = self.camera.getWidth(), self.camera.getHeight()
            self._wb.wb_camera_get_image.restype = ctypes.c_void_p
            pointer = self._wb.wb_camera_get_image(self.camera._tag)
            if not pointer:
                raise RuntimeError("the camera returned no image")
            self._put(_message(protocol.IMAGE, stamp, protocol.SIZE.pack(width, height), pointer, 4 * width * height))


def _message(kind, stamp, prefix, pointer, size):
    """protocol.pack(kind, stamp, prefix + the size bytes at pointer), copying Webots' data once instead of three times."""
    start = protocol.HEADER.size + len(prefix)
    message = bytearray(start + size)
    protocol.HEADER.pack_into(message, 0, protocol.MAGIC, kind, stamp, len(prefix) + size)
    message[protocol.HEADER.size:start] = prefix
    ctypes.memmove((ctypes.c_char * size).from_buffer(message, start), pointer, size)
    return message


def enabled():
    return os.environ.get("IRONLARK_SENSING") == "1"

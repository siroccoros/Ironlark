"""Sample the drone's sensors on Webots time and stream them to the ROS 2 bridge.

Called after every physics step. Built before the first step: Webots refreshes a sensor one period
after enabling it, so each one then refreshes at the steps its period divides (config/sensors.json),
where it is read, stamped with that step's Webots time, and queued; a sender thread writes the queue
to the bridge over TCP (sensing/protocol.py). A full queue drops messages and counts them: the
stream never stalls physics, and pausing Webots stops its clock. A reading older than its stamp
fails the stream: the LiDAR by the time Webots labels its scan with, the IMU's gyro against the
Iris's, which refreshes every step.

The LiDAR's returns are rendered in one step, so each is stamped with the scan's time (offset 0);
rays that hit nothing within the range (or something closer than its minimum) are left out. Ground
truth (the Iris's noiseless GPS and inertial unit) goes to truth.bin in the run folder every step,
never to the bridge; the account counts its records.

The stream starts when the bridge connects and ends when the launcher asks (sensing-end in the run
folder, after the flight) or when it fails: an error while sampling (a sensor without data among
them), or a lost connection. Ending, it stops sampling, sends what it queued, has the bridge
acknowledge every message (protocol.finish) and writes its account, sensing.json in the run folder:
messages queued, sent, acknowledged and dropped, and the failure. A failure is written at once. The
launcher fails a run that lost any message.
"""
import ctypes
import json
import math
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

import numpy as np  # noqa: E402

LIDAR_POINT = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("layer", "<i4"), ("time", "<f4")])


class SensorStream:
    def __init__(self, robot, config_path=ROOT / "config/sensors.json"):
        from controller.wb import wb

        if robot.getTime() != 0:
            raise RuntimeError(f"sensors enabled at {robot.getTime():.3f} s, not before the first step")

        self._wb = wb
        self.robot = robot
        self.config = json.loads(Path(config_path).read_text())
        self.step_ms = int(robot.getBasicTimeStep())
        self.run = Path(os.environ["IRONLARK_RUN_DIR"])
        self.queued = self.sent = self.dropped = self.truth_records = 0
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
        for device in (self.accelerometer, self.gyro):
            device.enable(self.imu_period)
        self.lidar_period = period(lidar["rate_hz"])
        self.lidar = robot.getDevice(lidar["name"])
        self.lidar.enable(self.lidar_period)
        self.lidar.enablePointCloud()
        self.camera_period = period(camera["rate_hz"])
        self.camera = robot.getDevice(camera["name"])
        self.camera.enable(self.camera_period)
        # Ground truth and the freshness reference: the Iris's noiseless GPS, inertial unit and gyro, enabled by the
        # flight bridge every step.
        self.gps = robot.getDevice("gps")
        self.truth_attitude = robot.getDevice("inertial unit")
        self.iris_gyro = robot.getDevice("gyro")
        self.truth = (self.run / "truth.bin").open("wb")

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
            self.truth.close()
            if failure and not self.failure:
                self.failure = failure
                print(f"sensing failed: {failure}", flush=True)
                self._account()  # now: the sender may be stuck on the bridge

    def _account(self):
        """Write sensing.json, holding the lock."""
        account = {"queued": self.queued, "sent": self.sent, "acknowledged": self.acknowledged,
                   "dropped": self.dropped, "truth_records": self.truth_records, "failure": self.failure}
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
        try:  # called from C (lockstep.c), which cannot see a Python exception
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
        truth = (*self.gps.getValues(), *self.truth_attitude.getQuaternion(), *self.gps.getSpeedVector())
        with self.lock:
            if not self.ended:
                self.truth.write(protocol.TRUTH.pack(stamp, *truth))
                self.truth_records += 1
        if now_ms % self.imu_period == 0:
            gyro = self.gyro.getValues()
            # Both gyros sit on one rigid body: equal rates, whatever the mount, unless this one was read stale.
            if abs(math.hypot(*gyro) - math.hypot(*self.iris_gyro.getValues())) > 1e-9:
                raise RuntimeError("the IMU's gyro is older than its stamp")
            gyro = [v + self.noise.gauss(0.0, self.gyro_noise) for v in gyro]
            accel = [v + self.noise.gauss(0.0, self.accel_noise) for v in self.accelerometer.getValues()]
            self._put(protocol.pack(protocol.IMU, stamp, protocol.VECTOR6.pack(*gyro, *accel)))
        if now_ms % self.lidar_period == 0:
            # Layer-major points, top layer first: the configured layers are the first ones.
            count = min(self.lidar.getNumberOfPoints(),
                        self.config["lidar"]["layers"] * self.lidar.getHorizontalResolution())
            pointer = self._wb.wb_lidar_get_point_cloud(self.lidar._tag)
            if not (count and pointer):
                raise RuntimeError("the LiDAR returned no points")
            points = np.frombuffer(ctypes.string_at(pointer, count * protocol.LIDAR_POINT_BYTES), LIDAR_POINT)
            # Webots labels the scan from the time it rendered it, backwards over a sweep it did not make.
            age_ms = now_ms - 1000.0 * float(points["time"].max())
            if abs(age_ms) > 0.5:
                raise RuntimeError(f"the LiDAR's scan is {age_ms:.1f} ms older than its stamp")
            hits = points[np.isfinite(points["x"]) & np.isfinite(points["y"]) & np.isfinite(points["z"])]
            hits["time"] = 0.0
            self._put(protocol.pack(protocol.LIDAR, stamp, protocol.COUNT.pack(len(hits)) + hits.tobytes()))
        if now_ms % self.camera_period == 0:
            image = self.camera.getImage()  # raises without an image
            size = protocol.SIZE.pack(self.camera.getWidth(), self.camera.getHeight())
            self._put(protocol.pack(protocol.IMAGE, stamp, size + bytes(image)))


def enabled():
    return os.environ.get("IRONLARK_SENSING") == "1"

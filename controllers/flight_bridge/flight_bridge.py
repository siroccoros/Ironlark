"""Connect the drone (the Iris flight model) to the pinned upstream ArduPilot dynamics bridge."""

import ctypes
import os
from pathlib import Path
import select
import socket
import struct
import sys
import time

root = Path(__file__).resolve().parents[2]
upstream = root / ".cache/Webots_Python/controllers/ardupilot_vehicle_controller"
if not upstream.is_dir():
    raise SystemExit("Missing ArduPilot assets. Run ./ironlark setup first.")
sys.path.insert(0, str(upstream))

from webots_vehicle import WebotsArduVehicle

import sensor_stream


class IrisBridge(WebotsArduVehicle):
    _sensors = False

    def _after_step(self):
        """Sensing (IRONLARK_SENSING=1): sampled on the step just taken, in Webots time."""
        if self._sensors:
            self._sensors.after_step()

    def _handle_controls(self, command):
        # SITL encodes disabled PWM outputs as -1. The upstream generic bridge
        # treats that value as reverse thrust. Iris has unidirectional motors:
        # disabled outputs must stop them, including before arming and at landing.
        super()._handle_controls(tuple(max(0.0, value) for value in command))

    def _handle_sitl(self, sitl_address="127.0.0.1", port=9002):
        """Strict lockstep: one sensor packet per physics step, one control per step.

        Upstream sends sensor packets in a busy loop and steps once per received
        control packet. When a rendered frame outlasts SITL's 100 ms resend, the
        queued resends each advanced physics an extra step, so dynamics ran ahead
        of the autopilot and slow rendering destabilised flight.
        """
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("0.0.0.0", port))
        # Before the first step: sensors enabled at 0 s refresh on their periods' multiples (sensor_stream.py).
        self._sensors = sensor_stream.enabled() and sensor_stream.SensorStream(self.robot)
        print(f"Listening for ardupilot SITL (I{self._instance}) at 127.0.0.1:{port}")
        while not select.select([s], [], [], 0)[0]:
            if self.robot.step(self._timestep) == -1:
                s.close()
                self._webots_connected = False
                return
            self._after_step()
        print(f"Connected to ardupilot SITL (I{self._instance})")
        if os.environ.get("IRONLARK_LOCKSTEP"):
            self._lockstep_in_c(s, sitl_address, port, os.environ["IRONLARK_LOCKSTEP"])
            return
        size = self.controls_struct_size
        while True:
            s.sendto(self._get_fdm_struct(), (sitl_address, port + 1))
            command = None
            while command is None:
                if not select.select([s], [], [], 1.0)[0]:
                    # A lost sensor packet would stall both sides; resend it.
                    s.sendto(self._get_fdm_struct(), (sitl_address, port + 1))
                    continue
                # Keep only the newest control; older ones are SITL's resends.
                while select.select([s], [], [], 0)[0]:
                    data = s.recv(512)
                    if len(data) >= size:
                        command = struct.unpack(self.controls_struct_format, data[:size])
            self._handle_controls(command)
            if self.robot.step(self._timestep) == -1:
                break
            self._after_step()
        s.close()
        self._webots_connected = False
        print(f"Lost connection to Webots (I{self._instance})")


    def _lockstep_in_c(self, s, sitl_address, port, library):
        """The loop above, in C (lockstep.c): the same packets, controls and steps, with sensing after each step."""
        after_step_type = ctypes.CFUNCTYPE(None)
        after_step = after_step_type(self._sensors.after_step) if self._sensors else after_step_type()
        motors = (ctypes.c_int * len(self._motors))(*[motor._tag for motor in self._motors])
        ctypes.CDLL(library).ironlark_lockstep(
            s.fileno(), sitl_address.encode(), port, self._timestep, motors, len(self._motors),
            self.gyro._tag, self.accel._tag, self.imu._tag, self.gps._tag, after_step)
        s.close()
        self._webots_connected = False
        print(f"Lost connection to Webots (I{self._instance})")


vehicle = IrisBridge(motor_names=["m1_motor", "m2_motor", "m3_motor", "m4_motor"])
while vehicle.webots_connected():
    time.sleep(0.1)

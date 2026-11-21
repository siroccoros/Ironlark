"""Connect the drone (the Iris flight model) to the pinned upstream ArduPilot dynamics bridge."""

import math
import select
import socket
import struct
import sys
import time
from pathlib import Path

root = Path(__file__).resolve().parents[2]
upstream = root / ".cache/Webots_Python/controllers/ardupilot_vehicle_controller"
if not upstream.is_dir():
    raise SystemExit("Missing ArduPilot assets. Run ./ironlark setup first.")
sys.path.insert(0, str(upstream))

from webots_vehicle import WebotsArduVehicle

import sensor_stream


class IrisBridge(WebotsArduVehicle):
    _sensors = None

    def _after_step(self):
        """Sensing (IRONLARK_SENSING=1): sampled on the step just taken, in Webots time."""
        if self._sensors is None:
            self._sensors = sensor_stream.SensorStream(self.robot) if sensor_stream.enabled() else False
        if self._sensors:
            self._sensors.after_step()

    _caps = None

    def _handle_controls(self, command):
        """Upstream's propeller linearisation (omega = sqrt(throttle) * max velocity), once per physics step.

        SITL encodes disabled PWM outputs as -1. The upstream generic bridge
        treats that value as reverse thrust. Iris has unidirectional motors:
        disabled outputs must stop them, including before arming and at landing.
        """
        if self._caps is None:
            self._caps = [min(m.getMaxVelocity(), self.motor_velocity_cap) for m in self._motors]
        for motor, cap, value in zip(self._motors, self._caps, command):
            motor.setVelocity(math.sqrt(value) * cap if value > 0.0 else 0.0)

    def _handle_sitl(self, sitl_address="127.0.0.1", port=9002):
        """Strict lockstep: one sensor packet per physics step, one control per step.

        Upstream sends sensor packets in a busy loop and steps once per received
        control packet. When a rendered frame outlasts SITL's 100 ms resend, the
        queued resends each advanced physics an extra step, so dynamics ran ahead
        of the autopilot and slow rendering destabilised flight.

        Sensing runs between stepBegin and stepEnd, while Webots computes the next
        step: device values change only at stepEnd, so it still samples the step
        just taken, and its copies of camera and LiDAR data leave the loop that
        every physics step waits on.
        """
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("0.0.0.0", port))
        print(f"Listening for ardupilot SITL (I{self._instance}) at 127.0.0.1:{port}")
        while not select.select([s], [], [], 0)[0]:
            if self.robot.step(self._timestep) == -1:
                s.close()
                self._webots_connected = False
                return
            self._after_step()
        print(f"Connected to ardupilot SITL (I{self._instance})")
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
            if self.robot.stepBegin(self._timestep) == -1:
                break
            self._after_step()
            if self.robot.stepEnd() == -1:
                break
        s.close()
        self._webots_connected = False
        print(f"Lost connection to Webots (I{self._instance})")


vehicle = IrisBridge(motor_names=["m1_motor", "m2_motor", "m3_motor", "m4_motor"])
while vehicle.webots_connected():
    time.sleep(0.1)

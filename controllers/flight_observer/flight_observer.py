"""Record evaluation truth and render the player's map; never control flight.

With IRONLARK_FAST_START=1, the simulation runs as fast as it can until the flight check starts arming:
the drone waits on its pad while EKF3 learns its gyro biases (about 40 s of simulated time), and the
view refreshes 10 times a second meanwhile. Then real time and the world's frame rate again. Simulated
time, physics and the autopilot (in lockstep) are unaffected; only the wait's wall-clock length is.
"""

import json
import os
from pathlib import Path

from controller import Supervisor


robot = Supervisor()
drone = robot.getFromDef("IRONLARK_DRONE")
destination = os.environ.get("IRONLARK_RUN_DIR")
if destination is None:
    raise SystemExit("Start this world through ./ironlark so its evaluation output has a destination.")

scene_path = os.environ.get("IRONLARK_SCENE")
world_view = None
if scene_path:
    from minimap import WorldView
    world_view = WorldView(robot, scene_path, destination)

# Controllers start once the world has loaded; the launcher waits for this.
(Path(destination) / "world-ready").touch()

fast_start = os.environ.get("IRONLARK_FAST_START") == "1"
flight_state = Path(destination) / "flight-state.json"
if fast_start:
    children = robot.getRoot().getField("children")
    world_info = next(children.getMFNode(i) for i in range(children.getCount())
                      if children.getMFNode(i).getTypeName() == "WorldInfo")
    fps = world_info.getField("FPS")
    world_fps = fps.getSFFloat()
    fps.setSFFloat(10.0)  # a float: Webots' Python setter ignores an int, printing "wrong type or length"
    robot.simulationSetMode(Supervisor.SIMULATION_MODE_FAST)

with (Path(destination) / "ground_truth.jsonl").open("w", buffering=1) as stream:
    while robot.step(100) != -1:
        if fast_start and flight_state.exists():
            state = json.loads(flight_state.read_text()).get("state")
            if state not in ("CONNECTING", "WAITING_FOR_POSITION"):
                fps.setSFFloat(world_fps)
                robot.simulationSetMode(Supervisor.SIMULATION_MODE_REAL_TIME)
                fast_start = False
        position = drone.getPosition()
        stream.write(json.dumps({"time": robot.getTime(), "position": position,
                                 "velocity": drone.getVelocity()}) + "\n")
        if world_view:
            world_view.update(position, drone.getOrientation())

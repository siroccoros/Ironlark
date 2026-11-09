# Ironlark

A playable Linux world where real drone autonomy software perceives, plans and
flies. Missions will span departure, outdoor travel, GPS-denied exploration,
inspection and return. Everything is simulated. Named for the holding
pattern: sit back and watch autonomy fly.

## This branch: `nav-foundation`

A lean branch for GPS-denied LiDAR navigation. It keeps what that work needs:
the drone and its sensor rig, the empty world on stock Webots R2025a, ArduPilot
SITL on EKF3, the sensor stream to ROS 2 with rosbag2 recording, and the checks
that judge a flight and its recording. The Poly Haven pine forest and its build
pipeline, the patched Webots renderer and the research atlas live on `main`;
nothing here needs them, and `git checkout main -- <path>` brings any of them back.

Today: a reproducible Webots–ArduPilot flight (take off, hover, land on EKF3
with simulated GPS) in an empty test area with a calibration wall for the
sensors, or in a maze generated from a seed. What comes next is in [ROADMAP.md](ROADMAP.md).

## Quick start

Needs Linux x86-64, OpenGL, Docker, [uv](https://docs.astral.sh/uv/), `git`,
`xvfb-run`, and [Webots R2025a](https://github.com/cyberbotics/webots/releases/tag/R2025a)
extracted to `./webots/`. With a C compiler (`cc` or `gcc`) the flight bridge's
per-step loop runs in C; without one, in Python.

```sh
./ironlark setup                  # pinned SITL/ROS images, Python env and bridge assets
./ironlark doctor                 # list missing runtime requirements
./ironlark check                  # validate takeoff/hover/land, headless
./ironlark run --record           # watch the flight and record its sensors (rosbag2)
./ironlark check-recording RUN    # judge the recording: completeness, IMU, LiDAR/camera on the wall
./ironlark check-replay RUN       # replay it and compare, message by message
```

| Command | Does |
| --- | --- |
| `setup` | Builds ArduPilot Copter 4.7.1 (`dbe7921`) in an Ubuntu 22.04 image, which keeps only the SITL binary, its parameters and pymavlink (0.3 GB; Docker's build cache holds the rest); extracts its Iris model and Webots bridge to `.cache/`; creates `.venv/` (Python 3.12.11, NumPy, Pillow). System Python is untouched. |
| `doctor` | Lists missing runtime requirements: Docker and its daemon, Webots R2025a, the Python environment, the pinned ArduPilot bridge assets and SITL image, `xvfb-run` for `check`, a display for `run`, and the ROS image. |
| `build-drone` | Models the drone in Blender 4.2.9 (downloaded once and checksummed by [`tools/blender.py`](tools/blender.py)) and writes `protos/IronlarkDrone.proto`; also run by `run` and `check` when an input or output differs from its receipt, `protos/drone/receipt.json`. |
| `run` | Flies in the empty world in a fullscreen window, then leaves it open until Ctrl+C. Until the autopilot can arm (EKF3 learning its gyro biases, ~40 s of simulated time) the simulation runs as fast as it can, the view at 10 fps; takeoff comes ~16 s after launch. `--world empty` is the default; `--world maze --seed N` flies a maze generated from the seed ([`tools/generate_maze.py`](tools/generate_maze.py); a random seed, recorded in `manifest.json`, without `--seed`), hovering at 1.5 m below its 3 m walls, with the maze's `truth.json` in the run folder for the evaluator only; `--editor` for Webots' UI; `--record` streams the sensors to ROS 2 and records a rosbag2 of the flight in the run folder (in real time throughout). |
| `replay RUN` | Plays a recorded run's bag in the ROS 2 container; its `/clock` topic carries Webots time. |
| `check-recording RUN` | Checks a recorded flight: the bag is complete (every topic, a message every period, over the whole flight), and the IMU at rest reads gravity, no rotation and the rig's noise. In the empty world, then the calibration wall: LiDAR distance, orientation and scatter against the rig's range noise, camera ↔ LiDAR agreement, and the ground in front of the wall not red. In a maze, every LiDAR ray of the hovering scans against the run's `truth.json` ([`tools/maze_lidar.py`](tools/maze_lidar.py)): returns on the walls and ground it places on their rays, none missing where it has a surface in range, the layers at their configured angles, and the rig's range noise. |
| `check-replay RUN` | Replays a recorded flight while recording it again and compares: every topic present, the same messages byte for byte, in order. |
| `check` | The same flight on a virtual display. Exits nonzero on failure. |

Webots is `WEBOTS_HOME` when set, else `./webots/`; this branch never selects a
patched build. To ignore a `WEBOTS_HOME` override in sh or fish, use
`env -u WEBOTS_HOME ./ironlark run`.
First setup needs internet and several GB of disk. OS packages in the image float,
so builds are source-pinned, not bit-identical.

Configure in files, not panels; restart to apply:
[`worlds/flight_foundation.wbt`](worlds/flight_foundation.wbt) (empty-world scene, vehicle, camera, timestep),
[`config/flight.parm`](config/flight.parm) (autopilot overrides),
[`config/sensors.json`](config/sensors.json) (sensor poses, rates, ranges, noise),
[`tests/flight_smoke.py`](tests/flight_smoke.py) (the flight check),
[`ironlark`](ironlark) (startup, versions, cleanup),
[`tools/`](tools) (the drone build, the recording checks, the maze generator `generate_maze.py` and its LiDAR check `maze_lidar.py`).

Tests: `.venv/bin/python -m unittest tests.test_sensing tests.test_checks tests.test_build_diagnostics tests.test_onboarding_workflows tests.test_maze tests.test_maze_lidar`.

## What `check` proves

Native Webots runs the physics; containerized ArduPilot SITL flies. The upstream
bridge carries dynamics and sensors to SITL and motor outputs back; a small Iris
adapter treats disabled outputs as stopped, not reversed, and runs the per-step
lockstep in C (`controllers/flight_bridge/lockstep.c`, compiled by the launcher into each
run's folder; the Python loop when no compiler is found). MAVLink uses TCP 5760,
the bridge UDP 9002–9003. Occupied ports and concurrent runs are refused.

Pass requires EKF3, GPS fix, a valid position estimate, accepted commands,
sustained hover and disarm after landing. A separate supervisor records ground
truth, read only after landing: peak height 2.5–4 m, final ≤ 0.2 m, drift
≤ 1.5 m, autopilot and Webots clocks within 1 s.

Parameters load as: Copter SITL defaults, the Iris example, then
`flight.parm`, which enables EKF3 and tunes gains for 4.7 at 500 Hz. Pre-arm
checks stay on. Every run starts with fresh autopilot storage.

[`docker/webots-clock.patch`](docker/webots-clock.patch) fixes the pinned
bridge: after a packet delay it could reset its timestamp against a stale
wall clock, repeatedly adding elapsed time to the autopilot. The patch keeps the
timestamp across delays and rebases only when Webots time goes backwards.

**Localization boundary.** The Iris example uses simulator truth
(`AHRS_EKF_TYPE=10`); Ironlark sets EKF3 (`3`), and the check rejects truth mode.
Position still comes from simulated GPS, which ArduPilot synthesizes from the
physical state Webots reports. GPS denial must be cut at the measurement layer,
never by removing that state. Feeding evaluator pose in as odometry is not
localization.

Each run writes `runs/<timestamp>/`: `manifest.json` (versions, Webots patches,
world, estimator), `result.json` (verdict, transitions, measurements),
`telemetry.jsonl`, evaluator-only `ground_truth.jsonl`, and Webots, SITL and
ArduPilot logs. Ctrl+C or failure tears everything down; that is cleanup, not a
recovery policy.

## Sensors and ROS 2

The drone ([`tools/build_drone.py`](tools/build_drone.py)) keeps the Iris flight model (motors,
propeller constants, mass, inertia, collision box, and the noiseless IMU and GPS ArduPilot reads)
and carries the rig of `config/sensors.json`: a Mid-360-class LiDAR (360° × 59°, −7° to +52°,
0.1–40 m, 10 Hz, 32 × 900 returns), an IMU (250 Hz, seeded white noise) and a front camera
(640 × 480, 90°, 20 Hz). With `--record`, the flight
controller samples them after each 2 ms physics step, on Webots time, and streams them over local
TCP ([`sensing/protocol.py`](sensing/protocol.py)); a full queue drops and counts, never stalling
physics, and pausing stops the clock. After the flight the stream ends: the bridge acknowledges
every message, and a run whose stream dropped, failed or left a message unsent fails, naming the loss
(`sensing.json` in the run folder). A ROS 2 Humble container ([`ros/`](ros)) publishes `/clock`,
`/imu/data`, `/lidar/points` (x, y, z, ring, time), `/camera/image_raw` (bgra8),
`/camera/camera_info` and `/tf_static`, ground truth only on `/ground_truth/pose` and
`/ground_truth/velocity`, and records all of it with rosbag2 (reliable, deep queues: `ros/record_qos.yaml`; each message compressed with zstd, losslessly: a minute of flight is 0.16 GB). A recorded check adds sensing's cost to `result.json` (real-time factor, messages sent); every flight adds what it cost the machine: peak RAM and CPU time of Webots with its controllers, SITL and ROS, and GPU memory, Webots' own and the card's, each named when it could not be measured. Frames are REP-103: Webots' world is
ENU and bodies FLU; [`sensing/frames.py`](sensing/frames.py) is the one place for ENU/FLU ↔ NED/FRD.

## References and licenses

[ArduPilot Webots integration](https://ardupilot.org/dev/docs/sitl-with-webots-python.html) ·
[pinned example](https://github.com/ArduPilot/ardupilot/tree/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/SITL/examples/Webots_Python) ·
[ArduPilot ROS 2](https://ardupilot.org/dev/docs/ros2-install.html) ·
[GPS/non-GPS transitions](https://ardupilot.org/copter/docs/common-non-gps-to-gps.html)

ArduPilot, Webots and Blender keep their own terms; setup saves ArduPilot's
license beside its cache.

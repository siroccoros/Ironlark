# Ironlark

Simulated quadrotor autonomy on Linux using stock Webots R2025a, ArduPilot Copter 4.7.1 SITL (EKF3), and ROS 2 Humble.

## Architecture

- **Simulator & autopilot**: stock Webots R2025a running ODE physics at 500 Hz in strict lockstep with containerized ArduPilot Copter 4.7.1 (`config/flight.parm`: `AHRS_EKF_TYPE=3`, real EKF3, default EKF failsafe enabled).
- **Sensor rig & ROS 2 bridge**: `config/sensors.json` defines a 32×900 10 Hz LiDAR (360° × 59°), 250 Hz noisy IMU, and 640×480 20 Hz front camera streamed on Webots simulation time (`sensing/protocol.py` → `ros/ironlark_bridge.py`). Ground truth is isolated to `/ground_truth/*` and `ground_truth.jsonl`.
- **Autonomy stack (`autonomy/`)** — subscribes only to `/imu/data`, `/lidar/points`, `/camera/image_raw`, and `/tf_static`:
  - `odometry.py`: IMU-aided 6-DoF LiDAR odometry (raw noisy gyro propagation with static bias calibration, stationary-only accel tilt alignment, and rolling-submap hybrid point-to-plane/point-to-point Gauss-Newton ICP; never reads Webots' noiseless `InertialUnit` orientation). This is a scan matcher with a gyro prior, not a tightly-coupled preintegrated LIO filter.
  - `mavlink_interface.py`: streams `VISION_POSITION_ESTIMATE` to EKF3 (`EK3_SRC2_* = 6`), tracks the slow GPS↔odometry frame offset outdoors and freezes it when GPS is lost, switches EKF3 live via `MAV_CMD_SET_EKF_SOURCE_SET`, and sends GUIDED setpoints (position, velocity, acceleration, yaw, yaw rate; `type_mask` 0). Poses carry the scan stamp; ArduPilot's jitter correction plus `VISO_DELAY_MS` place them in time.
  - `mapping.py` & `planner.py`: 3D log-odds voxel occupancy grid (`84 × 84 × 7 m`, `0.5 m` resolution) built from scratch via vectorized raycasting, C-space obstacle dilation, 3D frontier scoring, and 18-connected 3D A* with line-of-sight shortcutting.
  - `trajectory.py`: C² cubic B-spline over the A* path, corners rounded and shrunk until clear of the inflated map, timed by a jerk-limited speed profile (curvature, clearance and unmapped-space speed limits) that ends at rest; yaw follows the tangent. Replans start from the committed trajectory's state, so motion stays continuous.
  - `mission.py` & `gcs.py`/`gcs.html`: interruptible executive and browser Supervisory GCS (`http://127.0.0.1:8080`) for live map visualization, camera feed, `GPS ↔ ExternalNav` status, click-to-Go-To, drag-to-Explore, Hold, Return Home, and Land.
- **Simulated GPS denial**: `controllers/flight_observer` writes a 1-bit zone flag when physically inside the interior; `tools/sim_gps_zone.py` toggles `SIM_GPS1_ENABLE=0` and `SIM_GPS1_NUMSATS=0` over MAVLink `serial1` (TCP 5762). Autonomy only observes `GPS_RAW_INT` on `serial0` (TCP 5760).
- **Post-flight evaluation**: `tests/flight_smoke.py` (smoke takeoff/hover/land) and `tests/mission_eval.py` (outdoor-to-interior mission verdict from independent Webots ground truth, plus `tests/motion_quality.py`: tilt, rates, jerk, EKF error, stop overshoot, obstacle clearance and contacts from truth).

## Quick start

Requirements: Linux x86-64, OpenGL, Docker, [uv](https://docs.astral.sh/uv/), `xvfb-run`, and stock [Webots R2025a](https://github.com/cyberbotics/webots/releases/tag/R2025a) extracted to `./webots/` (or `WEBOTS_HOME`).

```sh
./ironlark setup                     # build SITL & ROS 2 images, extract Iris assets, create .venv
./ironlark doctor                    # verify runtime prerequisites
./ironlark check                     # headless takeoff/hover/land check (empty world)
./ironlark check --world mission     # headless outdoor-to-interior GPS-denied mission + judge (unified 4-biome world)
./ironlark run                       # graphical unified 4-biome world (Forest/Bunker E, Urban/Deck N, Yard/Warehouse S, Canyon/Mine W) + GCS
./ironlark run --standby             # boot on pad and explore any biome via Supervisory GCS (http://127.0.0.1:8080)
./ironlark check --world warehouse   # optional standalone world (also: canyon, compound)
./ironlark check --record            # record lossless zstd rosbag2
./ironlark check-recording runs/...  # verify bag completeness, IMU noise, LiDAR & camera calibration
./ironlark check-replay runs/...     # verify bit-exact rosbag2 replay
```

## Layout

| Path | Responsibility |
| --- | --- |
| `ironlark` | CLI launcher (`setup`, `doctor`, `run`, `check`, `replay`, `check-recording`, `check-replay`) |
| `autonomy/` | `odometry.py`, `mapping.py`, `planner.py`, `mavlink_interface.py`, `mission.py`, `gcs.py`, `gcs.html` |
| `worlds/` | `generate.py` (`mission`, `warehouse`, `canyon`, `compound`, `forest`), `assets.py` (hybrid Webots PROTOs & faceted meshes), `textures.py` |
| `controllers/` | `flight_bridge/` (Webots↔SITL lockstep + sensor stream), `flight_observer/` (ground truth, zone flag, chase cam) |
| `sensing/` | `protocol.py` (TCP framing), `frames.py` (ENU/FLU ↔ NED/FRD transforms) |
| `ros/` | `ironlark_bridge.py` (TCP → ROS 2 topics), `record_qos.yaml`, `Dockerfile` |
| `config/` | `sensors.json`, `flight.parm`, `external_nav.parm` |
| `docker/` | Pinned ArduPilot Copter 4.7.1 SITL image + `webots-clock.patch` |
| `tools/` | `sim_gps_zone.py`, `bag_reader.py`, `check_recording.py`, `check_replay.py` |
| `tests/` | `flight_smoke.py`, `mission_eval.py`, `test_autonomy.py`, `test_checks.py`, `test_sensing.py` |

## Unit tests

```sh
.venv/bin/python -m unittest tests/test_sensing.py tests/test_checks.py -v
docker run --rm -w /project --mount type=bind,src="$(pwd)",dst=/project,readonly \
  ironlark-ros:humble python3 /project/tests/test_autonomy.py -v
```

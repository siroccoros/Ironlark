# Roadmap

This is the `nav-foundation` branch: GPS-denied LiDAR navigation, built on the
flight foundation. The world-fidelity work (the Poly Haven pine forest, its
build pipeline and the patched Webots renderer) continues on `main`; its regions
come to navigation when a capability needs them.

## Done: the simulator as a test bench

The drone (`tools/build_drone.py`, `protos/IronlarkDrone.proto`: the Iris flight model, a new body,
the rig of `config/sensors.json`) passes the flight check and carries a Mid-360-class LiDAR, an IMU
and a front camera. `./ironlark check --record` (or `run --record`):

- records a complete rosbag2 (every message the controller sent, on Webots time, stamps strictly
  increasing; `check-recording` checks each topic's period over the whole flight) and adds sensing's
  cost to `result.json`; Webots time slows rather than drop anything;
- `check-recording`: the calibration wall reads at 5.897 m (face at 5.9 m), its normal within 0.6°,
  and 99.9% of its LiDAR returns land on its red in the camera frame of the same instant;
- `check-replay`: replayed, every topic keeps its messages, byte for byte, in order.

Ground truth stays on `/ground_truth/*` and in `ground_truth.jsonl`, for the evaluator only.

Found on the way: Webots misplaces a 360° Lidar's returns when `tiltAngle` is set (the multi-camera
merge ignores it: rays span ±29.5° but are labelled −7°…+52°, shifted ~22° in azimuth), so the rig
renders 56 layers over ±52.3° and the stream keeps the top 32 (+52.3° to −6.7°); rclpy checks
`bytes` assigned to `uint8[]` fields byte by byte (the bridge reached 320 messages/s until it passed
`array('B')`: now ~840/s); `ros2 bag record -a` and replay with `--clock` both mislead (late
subscriptions; a wall-time clock beside the bag's Webots `/clock`). Arming waits on EKF3 learning
the gyro biases (~41 s of simulated time), not on the GPS: faster GPS detection brings takeoff no
earlier.

## Now: GPS-denied navigation

Goal: with GPS cut, the drone escapes a maze it has never seen. It takes off inside, localizes by
its own LiDAR and IMU, maps the walls as it flies, plans towards the exit, turns back from dead
ends, replans around an obstacle that appears mid-flight, and lands. The empty-world flight check
stays the regression gate.

1. **A generated maze world.** `tools/generate_maze.py` turns a seed into a maze of Webots
   primitives on stock Webots (walls about 3 m high, corridors about 2.5 m wide; the numbers are
   parameters) and writes `truth.json` beside it: the true walls, the start, the exit and the
   shortest path, for the evaluator only. Visual and collision shapes agree; its LiDAR returns are
   checked against `truth.json`, not just its looks. A fixed seed always gives the same maze.
2. **LiDAR(-inertial) odometry.** `check-odometry` replays only sensor topics from recorded maze
   flights, then scores ATE/RPE against evaluator truth. The seed-7 data route covers three
   waypoints including a 5.4 m straight corridor at up to 0.6 m/s. The run recorded in
   `runs/20261111-231941-58621` passed: 0.076 m overall ATE, 0.022 m one-second RPE,
   0.104 m moving ATE, 0.134 m corridor ATE, with all 67 corridor scans healthy. A synthetic
   corridor without cross-wall features is flagged as degenerate. Webots' nominal per-return
   times describe one geometric snapshot at the latest return; the estimator uses that time
   and measured gyro rotation instead of inventing per-point motion.
3. **External navigation into EKF3.** `--external-nav` sends pose, body-frame velocity, uncertainty
   and health on MAVLink serial1 port 5761. The in-flight GPS → ExternalNav → GPS check passed a
   five-second hover and 0.5 m forward/return move on seed 7, with 0.89 m minimum wall
   clearance. The live node runs on
   Webots time, and frame conversions live in `sensing/frames.py`. GPS stays enabled for this
   transition check; cutting it is step 4.
4. **GPS cut at the measurement layer.** `SIM_GPS1_ENABLE 0`: SITL's receiver stops reporting; the
   physical state Webots reports is untouched (see the localization boundary in the README).
   Takeoff, hover and a fixed route through the maze. A `--gps-denied` check in the style of
   `check` fails if GPS is enabled or if ground truth reaches navigation code.
5. **Mapping and planning in an unknown maze.** A local occupancy map from the LiDAR, scored against
   `truth.json`. Unknown space stays unknown in the map. The planner may route through it as
   unverified and replans as it is observed: dead ends discovered, turned back from, another route
   taken.
6. **Obstacles and failsafes.** An obstacle spawned in a corridor mid-flight forces a replan.
   Odometry loss, unhealthy estimates and lost links end in a defined hover or landing, tested by
   injecting each in flight.
7. **Batch evaluation.** Headless flights over random seeds and starts, each judged on its run
   folder (`result.json`, `ground_truth.jsonl`, `telemetry.jsonl`) and `truth.json`: escaped or
   not, time, path length against the shortest path, ATE, map accuracy, minimum clearance, replan
   time, and what the machine spent; a summary across the batch.

**Done when** a batch of 10 random unknown mazes with `SIM_GPS1_ENABLE 0` is flown, every run
judged as above, the escape rate and the other metrics reported, and every injected failure ends in
a hover or landing.

Rules that carry over:

- World geometry and `truth.json` are never an autonomy map; unknown space stays unknown.
- GPS loss is a simulated rule, not a side effect of a roof; degradation short of a cut needs an
  explicit model.
- Feeding evaluator pose in as odometry is not localization.
- Measure rendering, sensing and physics together before promising scale or frame rate.

## Later

- A primitive tank/crane world (tanks, pipes, a crane, open ground) generated the same way, for
  overhangs and the site layout of real customers.
- The forest regions from `main`, when a capability needs them.
- Real recorded LiDAR (a public drone dataset or the team's own hardware) to compare the simulated
  returns and the odometry's drift with reality.

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
   five-second hover and 0.5 m forward/return move on seed 7, with 0.79 m minimum wall
   clearance. The live node runs on
   Webots time, and frame conversions live in `sensing/frames.py`. GPS stays enabled for this
   transition check; cutting it is step 4.
4. **GPS cut at the measurement layer.** `SIM_GPS1_ENABLE 0`: SITL's receiver stops reporting; the
   physical state Webots reports is untouched (see the localization boundary in the README).
   `check --world maze --seed 7 --gps-denied` passed takeoff, hover, the three-waypoint route and
   landing in `runs/20261112-082733-10428`, with no GPS fix, 1.085 m minimum wall clearance,
   and the EKF3 position sourced from LiDAR odometry. The fixed route is a flight gate, not maze
   exploration. The navigation process reads sensor topics only; the evaluator opens truth after flight.
5. **Mapping and planning in an unknown maze.** A local occupancy map from the LiDAR, scored against
   `truth.json`. Unknown space stays unknown in the map. The frontier explorer plans from sensor
   observations, turns back from blocked routes, and commits to the open exit when it finds one.
   The seed-7 GPS-denied exploration in `runs/20261112-203537-125331` passed escape, landing,
   and clearance: exit at 224.2 s, minimum wall clearance 0.847 m. Its replayed sensor-only map
   scored 95.1% accuracy on observed cells (54.5% of maze cells observed); odometry passed with
   0.141 m ATE RMSE and 0.018 m one-second RPE. Other seeds remain for the batch gate in step 7.
6. **Obstacles and failsafes.** An obstacle spawned in a corridor mid-flight forces a replan.
   Odometry loss, unhealthy estimates and lost links end in a defined hover or landing, tested by
   injecting each in flight. The GPS-denied check now monitors odometry age and health while armed
   and commands LAND on a mission failure. Stopping the odometry process after takeoff in
   `runs/20261112-084733-20171` caused the monitor to detect a stale stream, command LAND,
   and record `FAILSAFE_LANDED`; independent Webots truth ended at 0.055 m height.
   The explorer tolerates brief corridor degeneracy, while sustained loss still lands. A
   detour-capable obstacle spawned during `runs/20261112-210226-148307`; the planner rejected
   its blocked route after 1.7 s, escaped and landed, with 0.387 m minimum clearance. The long
   detour took 778.6 s and 3.78 times the evaluator's shortest path. The GCS-link-loss injection
   in `runs/20261112-211921-155989` triggered LAND and disarm, with 1.211 m minimum clearance.
   Marking the estimate unhealthy in flight (`--inject-unhealthy-odometry`,
   `runs/20261114-125440-15252`) was detected after 2.1 s and landed with 0.49 m drift and
   0.972 m minimum clearance. Beyond an exit gate only the outer wall is left, so the odometry
   turns degenerate along it; the explorer may commit to an exit on such an estimate, and the
   monitor grants at most 8 s of IMU coasting in an opening of at least 150° (inside the batch
   mazes no reading exceeded 120°). Seed 135227 had failsafe-landed at its gate without these
   (`runs/20261115-163215-13755`, `runs/20261118-205724-49600`) and escaped with them
   (`runs/20261118-211552-60336`).
7. **Batch evaluation.** Headless flights over random seeds and starts, each judged on its run
   folder (`result.json`, `ground_truth.jsonl`, `telemetry.jsonl`) and `truth.json`: escaped or
   not, time, path length against the shortest path, ATE, map accuracy, minimum clearance, replan
   time, and what the machine spent; a summary across the batch. `tools/evaluate_batch.py` now
   scores existing run folders, keeping metrics null when their source artifact is absent.
   `./ironlark batch --count 10 --batch-seed 20261114` (`runs/batch-20261118-212622/`) flew
   ten unknown mazes with `SIM_GPS1_ENABLE 0`: all ten escaped and landed, with no failsafe;
   mean escape 520 s (378-711 s), mean path 1.38 times the evaluator's shortest path
   (0.97-2.09), mean minimum wall clearance 0.726 m (lowest 0.607 m), mean observed-map
   accuracy 91.6% over 82.7% of maze cells, longest scan gap 0.34 s, peak RAM under 0.9 GiB.
   The shortest path runs through cell centres, so a flight that cuts corners can beat it
   (ratio below 1). Mean ATE was 0.266 m; two long flights drifted past the odometry check's
   0.3 m limit (seed 507877 0.578 m, 529927 0.473 m) yet escaped with 0.70 m clearance, and the
   same seeds scored 0.25 m and 0.14 m in an earlier batch: drift varies run to run and is not
   bounded. Not yet measured: random start cells (every maze starts at its corner cell) and
   the replan time across a batch (no obstacles are spawned in it).

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

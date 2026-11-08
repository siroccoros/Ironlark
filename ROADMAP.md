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

Goal: with GPS cut, the drone takes off, flies out through a cluttered site to a goal and back,
and lands at home, localized by its own LiDAR and IMU. The empty-world flight check stays the
regression gate.

1. **LiDAR(-inertial) odometry.** Evaluate it first on recorded flights (`run --record` bags),
   scoring pose and velocity against `/ground_truth/*`. Per-return LiDAR timing if the odometry
   needs it: model the acquisition, never fake it.
2. **External navigation into EKF3.** Feed the odometry's pose, velocity, uncertainty and health to
   ArduPilot's external-navigation input, with EKF3's position and velocity sources set to it. Hover
   and make short moves, measuring error against the evaluator.
3. **GPS cut at the measurement layer.** `SIM_GPS1_ENABLE 0`: SITL's receiver stops reporting; the
   physical state Webots reports is untouched (see the localization boundary in the README). Then
   GPS ↔ external-navigation transitions tested in motion, keeping frame and return route; GPS
   degradation needs an explicit model.
4. **A primitive tank/crane world.** Tanks, pipes, a crane and open ground from Webots primitives,
   on stock Webots, generated from a file with a fixed seed: enough structure for the LiDAR to hold
   on to, and gaps and overhangs to plan around. Its visual and collision shapes agree; its sensor
   returns are checked, not just its looks.
5. **Planning and replanning.** A local map from the LiDAR, a collision-aware path to the goal and
   back, replanned as the map grows. Unknown space stays unknown.
6. **Failsafes.** Odometry tracking loss, unhealthy estimates and lost links end in a defined hold
   or landing, tested by injecting each in flight.
7. **Batch evaluation.** Many headless flights over seeds and starts, each judged on its run folder
   (`result.json`, `ground_truth.jsonl`, `telemetry.jsonl`): success, position error, drift, time,
   and what the machine spent; a summary across the batch.

**Done when** a batch of out-and-back flights in the tank/crane world with `SIM_GPS1_ENABLE 0`
passes the evaluator's checks, every run's error and failures named.

Rules that carry over:

- World geometry is never an autonomy map; unknown areas stay unknown.
- GPS loss is a simulated rule, not a side effect of a roof.
- Feeding evaluator pose in as odometry is not localization.
- Measure rendering, sensing and physics together before promising scale or frame rate.

## Later

First full mission: leave home, reach a given entrance, map a bounded unknown
interior, exit, return, land. Its behaviours become reusable tasks under a
manager that handles priority, interruption and resumption while perception,
mapping, planning and control run concurrently. General-purpose behaviour
grows from these tested pieces.

Richer regions (the forest on `main`, an outdoor-to-cave region) join the
navigation work one at a time, when its capabilities need them.

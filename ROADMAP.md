# Roadmap

## Completed

- **Lean stock-Webots test bench**: stock Webots R2025a (no custom C++ patches or Blender build pipeline), pinned ArduPilot Copter 4.7.1 SITL with EKF3 and strict lockstep, deterministic ROS 2 Humble sensor streaming & lossless zstd `rosbag2` recording/replay, and calibration-wall cross-sensor verification.
- **Procedural collision-accurate worlds**: on-the-fly `IronlarkDrone.proto` generation from `config/sensors.json` plus seeded `forest` and `mission` (outdoor forest + enclosed GPS-denied multi-chamber cave) worlds with matching visual and ODE collision geometry.
- **LiDAR-inertial odometry (`autonomy/odometry.py`)**: 250 Hz IMU propagation + rolling-submap 6-DoF hybrid point-to-plane/point-to-point Gauss-Newton ICP with Huber weighting and Hessian observability damping.
- **Live GPS ↔ ExternalNav EKF3 transition (`autonomy/mavlink_interface.py`)**: measurement-layer GPS denial inside the interior (`SIM_GPS1_ENABLE=0`, `NUMSATS=0`), automatic EKF3 Source Set 1 ↔ 2 switching, and tracking-loss hover safety.
- **Online 3D mapping, planning & exploration (`autonomy/mapping.py`, `autonomy/planner.py`, `autonomy/mission.py`)**: log-odds 3D occupancy grid built from scratch, C-space obstacle inflation, 3D Yamauchi frontier selection, 3D A* with line-of-sight shortcutting, and end-to-end independent ground-truth mission grading (`tests/mission_eval.py`).

## Next extensions (optional thesis directions)

- **Camera-assisted perception**: semantic target inspection or visual loop closure using `/camera/image_raw` alongside the 3D voxel map.
- **Multi-level / sloped cave worlds**: vertical shafts and uneven `ElevationGrid` terrain exercising full 3D altitude planning.
- **Dynamic obstacles & replanning stress tests**: moving actors or narrow doorway squeezes with real-time velocity obstacles.
- **Hardware transfer**: replace `ros/ironlark_bridge.py` with a physical Livox Mid-360 + Pixhawk MAVLink bridge while keeping `autonomy/` unchanged.

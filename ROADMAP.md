# Roadmap

## World

One connected, visually rich world: outdoor travel, forest, rocky terrain,
caves, industrial sites, and a GTA-style minimap. Art style is open; pick what
coheres and build whatever import support it needs. Engine format support is
never the reason to reject an environment.

### Now

[Poly Haven Pine Forest](https://polyhaven.com/collections/pine_forest) (CC0),
built from its Blender scene by `./ironlark build-world`: authored tree variants
and saplings, source placements, cover, river, terrain, sky and baked canopy
light. Goal: match the source renders. Rule: no enshittification. Fix engine
and pipeline bottlenecks; don't cut asset quality.

Rejected: Kenney Nature Kit (look).

Matched to the source: Filmic "Medium High Contrast" view, both suns (strength,
color, direction), the camera-visible sky (the HDRI with its rotation, at 0.5 where the
source shows it at 0.2: over the open valley its far haze turned the land grey), the
lighting sky (Nishita × 0.7), and HD (1K) object textures downsampled from the
source's originals, the source's exposure (+1) and no bloom. Both skies are written
as linear radiance; Blender's own HDR save had stored them display-encoded (2.4×
too bright at mid-grey). Light under the canopy comes from baked visibility
layers traced through the real foliage (`tools/bake_forest_light.py`,
`Background.lightOcclusion*`), which light the fog too.

The region is a hilltop above a valley. The viewer is fenced inside the exported
source terrain (203 × 200 m, `Viewpoint.boundsMin/boundsMax`). Past it, its ground and forest are
mirrored across each edge of the rectangle the terrain fully covers (trees and
cover turned, so none faces its twin; `tools/forest_backdrop.py`) and fall away
(`tools/forest_edge.py`): level at the seam, over a brow of about 10 m, then at
27–42° with spurs and gullies; past the south-east corner, where the river rises,
a knoll stands up to 18 m above the region. The near band (120 m) keeps the
mirrored ground and trees; past it `tools/forest_valley.py` carries the
hillside down about 250 m to a valley floor, its forest as impostor trees (all to 170 m
out, 35% to 300 m) over a painted canopy. The valley: meadows, fields, woods, a
river, a lake and a village, forested hills from 2 km and mountains from 4 km; one
graded mesh of rings (109 k triangles, finer along the lake's shore and the river) with
painted land-use textures (BC1 DDS). The lake lies in a basin with a bank, a beach along
stretches of it and shallows over sand. The river meanders in a channel of its own, deepest
towards the outside of each bend, with gravel bars inside them and trees along its banks in
stretches; it comes down from the hills as a stream, runs through the lake and leaves the valley
into the haze. Both are water with depth colour and wind ripples (`PBRAppearance.attenuationColor`,
`scatterColor`, `waves`). The hillside's trees take their light from outer light layers at the ground
as it lies, opening to full light where they end (`Background.lightOcclusionOuter*`). Land past the fog
box takes the horizon's colour with distance (`Fog.hazeDistance` 6 km).
Trees past 150 m are impostors captured from the trees as drawn
(`tools/bake_impostors.py`, `Shape.impostor`). The fog is a layer with a soft top
(`Fog.boxFalloff`) whose far haze meets the sky at the horizon (`Fog.horizonRadiance`).

Open fidelity gaps against the source (Cycles lighting the same geometry with the
source's lights, `tools/render_lighting_reference.py` and `tools/compare_views.py`:
0.96–1.23× as bright at the pinned views):

- **Bounced light**: Webots has none; under the canopy Cycles' sunlight bounces
  off leaves and ground. Inside the forest the fog adds ×1.67 light to Cycles' ×1.37.
- **Fog**: calibrate against `./ironlark render-reference` (needs more than
  ~26 GB RAM).
- **River**: done: the source's clear water (`PBRAppearance.transmission`) with flowing white foam
  (`flowFoam`), mirroring the banks and trees through screen-space reflections.
  No refraction offset yet: the bed shows straight through.
- **Foliage shimmer**: temporal anti-aliasing covers the main view; robot cameras
  render single frames, so needles thinner than a pixel still flip in and out
  there. Supersample them if vision needs it.

Known bugs: with ambient occlusion off (`ambientOcclusionRadius 0` or the GTAO
preference) most trees do not draw; cause not yet found.

### Candidates for later regions

Not purchased, imported or tested in Webots.

- [Mountain Forests and Meadows](https://www.fab.com/listings/5c2e9ea7-ea36-413c-bd03-1dbb48f0c403) ([flythrough](https://youtu.be/dPvsIqHenrk))
- [Mountain Lake Valley](https://assetstore.unity.com/packages/3d/environments/landscapes/mountain-lake-valley-environment-built-in-version-140485)
- [Yos3D Cave Mine](https://yos3d.com/asset/environment-cave-mine/): assembled scene plus FBX sources
- [Scans Factory Industrial Abandoned Buildings](https://www.fab.com/listings/7bf45f1b-9ed7-4e83-b071-4aa46f073ea3) ([flythrough](https://www.youtube.com/watch?v=Li7rrcwHslw))

| Source | Offers | Work needed |
| --- | --- | --- |
| [LTU cave world](https://github.com/LTU-RAI/gazebo_cave_world) (MIT) | ~150 × 102 × 33 m cave: chambers, shafts, stalactites, squeezes | Convert SDF and meshes; keep DARPA notices; validate collision |
| [DARPA SubT](https://github.com/osrf/subt/wiki/World-Creation-Tutorial) | Robotics-grade caves and tunnels | Convert Gazebo placements and meshes; check terms; plugins need reimplementation |
| [SubTGraph](https://github.com/LTU-RAI/SubTGraph) | Configurable multi-level cave, mine and lava-tube layouts | Import OBJ or tiles; keep generator topology away from the drone's map |
| [Kenney Cave](https://kenney.nl/assets/modular-cave-kit) / [Factory](https://kenney.nl/assets/factory-kit) kits (CC0) | Modular interiors | Assemble, check clearances; restyle to match |
| [Forest3D](https://github.com/unitsSpaceLab/Forest3D) (AGPL-3.0) | Procedural terrain and vegetation placement | Adapt exporter; benchmark vegetation cost |
| [Webots OSM importer](https://github.com/cyberbotics/webots/blob/R2025a/docs/automobile/openstreetmap-importer.md) | Real street layouts | Add detail and interiors |

Assets from different sources won't fit together by themselves. Expect work on
scale, materials, lighting, terrain seams and collision.

### Pipeline rules

- Keep originals, their terms and checksums in `.cache/`; generate runtime
  assets and scenes reproducibly from code.
- Rendered surface and collision must agree wherever navigation depends on
  them. Vegetation gets deliberate sensor and collision behaviour.
- World geometry is never an autonomy map; unknown areas stay unknown.
- GPS loss is a simulated rule, not a side effect of a roof.
- Measure rendering, sensing and physics together before promising scale or
  frame rate.
- Presentation is the world itself: fullscreen, configured in files.

## Now: the simulator as a test bench

Goal: a simulator the drone's autonomy can be developed and judged in: the
source's looks, a drone carrying real sensors, and time-consistent data out
through ROS 2. The takeoff check stays the regression gate. New scenes come
later, each when a capability needs it.

### 1. Finish the forest's looks

Approved look: tag `looks-good-2026-09-28`. Judge a change by A/B renders of main and
the branch at the same views, and against Cycles (`tools/render_lighting_reference.py`;
render it at the preview's aspect, 1920×861 or half, or crop to the same vertical FOV).

Handoff state (2026-09-26), partly unverified:

- Done, verified: light layers baked without cover under 3 m (it shaded itself:
  dark rocks); cover gets the source's normal and roughness maps (the lookup had a
  wrong path); degenerate normal-map frames no longer turn foliage black.
- Verified in a recorded forest flight: the drone camera sees the fog, and range sensors see
  instanced foliage (a LiDAR scan at 2.5 m: 20.8 k of 28.8 k returns off trunks, crowns and cover).
- River rocks black under the sun: fixed by casting no cascade shadow from dense
  small clutter (over 2000 instances and under 0.25 m placed height: the river bed's
  `rock_moss_set_02_rock07`…`13` pebbles, ~78 k instances, and two dry-branch sets;
  `tools/build_forest_world.py`). Root cause still unknown, recorded for later: in
  the shadow maps such a carpet shades itself black. A lone pebble on the bed stays
  dark, the same pebble 0.5 m up is lit and casts a correct shadow; the water, LOD1
  meshes, mesh winding, backdrop terrain, cascade caching (tried as a patch,
  reverted) and a 4× normal offset are ruled out. The rocks of `set_01`, as small but
  ~950 per mesh, shade correctly. Repro: `/tmp`-style world with the terrain, lights
  and one `rock07` instance (instance 729 at −21.01, 32.14, −0.92), `--lights main`.
- Trunk bands: fixed. 9 of 11 big pine and fir variants tile their bark past the UV square
  above ~2 m; the trunk bake wrote only the square, so 13–21% of each trunk read unbaked
  texels (black, roughness 0: glossy dark bands, stretched bark). The bake now writes the
  tiled faces shifted into the square first, then the trunk's own islands over them.
- River close up: done. The water shows its sunlit bed and the banks.
- Terrain detail: the source's three tiled layers (read from its `main_terrain`:
  forest_leaves_04 at 1/150, forest_ground_04 at 1/22, rocky_trail at 1/20 of the
  terrain's bounds, blended by its `path` and `river` attributes, baked as
  `terrain_*_mask.png`) add detail within 15–40 m over the 2.5 cm bake. Built; verify
  up close against the checkpoint and Cycles.
- GTAO off (`OpenGL/GTAO` 0) strips the big trees' crowns in the main view (their
  twig-card shapes; impostors vanish too); sensors are unaffected (the drone camera has
  its own ambient occlusion and sees full crowns, the LiDAR returns off them). Old: in
  the 09-25 checkpoint too. Established with a logging build: the twig shapes are drawn
  (same instance counts, program, depth, blend, colour mask and framebuffer as with
  GTAO), yet leave no pixel with the alpha test off, nor with the depth test off: their
  vertices likely land off screen. Not the far plane, fog, BC7, texture binding cache or
  the scene copy (screen-space reflections). Parked: it needs a GPU capture of one twig draw.
- Shimmer: done (temporal anti-aliasing in the main view; 4x multisampling with
  alpha to coverage had cut it only by a quarter). Robot cameras keep single
  frames: supersample them if vision needs it.

### 2. Sensor rig and ROS 2

Done (2026-09-27). The drone (`tools/build_drone.py`, `protos/IronlarkDrone.proto`: the Iris
flight model, a new body, the rig of `config/sensors.json`) passes the flight check in both worlds
and carries a Mid-360-class LiDAR, an IMU and a front camera. `./ironlark check --record`:

- records a complete rosbag2 (every message the controller sent, on Webots time, stamps strictly
  increasing; `check-recording` checks each topic's period over the whole flight) and adds sensing's cost to `result.json`: real time in the empty world
  (with real-time pacing), 0.47× in the forest; Webots time slows, nothing is dropped;
- `check-recording`: the calibration wall reads at 5.898 m (face at 5.9 m), its normal within 0.8°,
  and 99.1–99.9% of its LiDAR returns land on its red in the camera frame of the same instant (the
  rest on its edges' pixels);
- `check-replay`: replayed, every topic keeps its messages, byte for byte, in order.

Found on the way: Webots misplaces a 360° Lidar's returns when `tiltAngle` is set (the multi-camera
merge ignores it: rays span ±29.5° but are labelled −7°…+52°, shifted ~22° in azimuth), so the rig
renders 56 layers over ±52.3° and the stream keeps the top 32 (+52.3° to −6.7°); rclpy checks
`bytes` assigned to `uint8[]` fields byte by byte (the bridge reached 320 messages/s until it passed
`array('B')`: now ~840/s); `ros2 bag record -a` and replay with `--clock` both mislead (late
subscriptions; a wall-time clock beside the bag's Webots `/clock`).

Sensing's cost in the forest is the camera's (alone: 0.57× real time; the LiDAR alone: 0.76×). Its
640 × 480 frame at takeoff takes 75 ms: 57 ms of opaque geometry, ~115 M triangles (from the ground
it sees ~700 full trees of 120–160 k triangles; trees are full out to 150 m), and 13 ms for the near
shadow cascade (4.9 M). Vertex-bound: 160 × 120 saves 14%; shadows, fog and GTAO off change ≤ 7%.
Switching level of detail at equal on-screen size (the 150 m impostor switch is set for 1920 px
over 1 rad, so ~30 m for this camera) cut the frame to 32 ms, but impostors are visibly flatter and
lighter than trees at 30–150 m, where no haze hides them: rejected. The source has no lighter trees
to use: in the .blend each tree scatter's lod1 branch instances the same full collection as lod0 (its
proxies show in the viewport only). Occlusion culling now draws only what is not hidden:
the frame at takeoff draws 62 M triangles instead of 130 M, 59 ms instead of 82, and the forest runs
with sensing at 0.61× real time instead of 0.49×.

Next: per-return LiDAR timing if the odometry needs it (model acquisition, never fake it).

### 3. Performance

Measured with the frame log (`IRONLARK_FRAME_LOG`, `tools/frame_log.py`), headless at 1920×1080 on the GTX 1060;
images and sensor data compared with the approved look (`looks-good-2026-09-28`) built in a worktree.
Frames are GPU-bound, by pixels more than vertices: at a quarter of the pixels the forest view takes
35 ms instead of 65, the drone view 11 instead of 33. Foliage overdraws heavily: the opaque passes run
the fragment shader 7–23 times per pixel (sub-pixel needle cards, each quad shaded whole), and PBR
lighting is 6–19 ms of a view. The forest view also draws ~80 M triangles, the small saplings
(120–157 k triangles of open needle cards each) half of them. Robot cameras (640×480) and the LiDAR
(106 M triangles a scan into 287×110 faces) are vertex-bound.

Tried and dropped:

- Culling the LiDAR's six faces by occlusion (as colour views are culled): a 40 m scan hides only 8% of
  its triangles, and waiting on six small faces cost more (38 → 40 ms of GPU a scan).
- Scattering fog: hoisting its per-step terms out of the march, and batching its reads four steps at a
  time, were slower; its cost is in reads of the baked light layers (4–11 ms of its 7–15).
- The PBR shader's light-layer search without variable indexing (as the fog's is): within noise.
- Culling back faces of the saplings: their needles are open cards, seen from both sides.
- Keeping every instance's transform on the GPU and sending only indices per draw: neutral in time,
  +32 MB of GPU memory.
- Drawing a view's instances front to back: slower.
- Faster GPS detection (`GPS_DRV_OPTIONS 4`): the GPS is ready sooner, but arming waits on EKF3
  learning the gyro biases (~41 s of simulated time), so takeoff comes no earlier.
- Immutable storage for mesh buffers (`glBufferStorage`): the driver reserves a RAM copy of every
  `glBufferData` buffer but never touches it, so resident memory is unchanged.
- A depth pre-pass for instanced foliage (alpha-tested depth first, then shading at equal depth): the
  shading pass drops ~11 ms in the forest view but the pre-pass costs ~32 (all the vertex work again,
  and its own alpha-tested overdraw); a visibility buffer would pay the same first pass.
- Discarding masked texels before normal and material maps are read: no faster, and neighbours'
  derivatives change (up to 17% of a view's pixels by more than 8 levels). Impostors do:
  their coverage is one atlas lookup, taken everywhere on the quad.
- Leaving the pen code out of the PBR fragment shader as well (the pen variant changes only the vertex shaders):
  faster, but the driver then shades distant impostors ~12% brighter in the high view, a difference
  from code that never runs. The fragment shader is left as is.

## Later: regions

Added one at a time as the drone's capabilities need them; candidates under
[Candidates for later regions](#candidates-for-later-regions).

### Outdoor-to-cave region

From [the world](#world): an outdoor home, rocks,
vegetation and an accessible entrance to a real cave section. The drone departs
outdoors; it never spawns inside.

- Asset manifest: source, revision or checksum, license, units, conversion.
- Convert placements, meshes, textures and collision into reusable Webots
  components, adding format support where needed.
- Generate the scene from config with a fixed seed; keep full geometry and
  generator topology out of navigation.
- Check openings and sensor returns, not just looks.
- Profile physics, sensing and rendering together; fix measured costs.

**Done when** the drone takes off outdoors and correctly senses entrance,
terrain, vegetation and cave walls. Autonomous cave travel is not yet claimed.

## Next: autonomy

### GPS-free flight

Sensor stream fixed (2026-10-29): readings fresh at their stamps (were 98 ms old for the LiDAR, 48 ms
for the camera, 2 ms for the IMU; the first scan all zeros), LiDAR hits only at the scan's instant
(were infinite misses and a fake 100 ms sweep), no orientation on `/imu/data` (was truth), ground
truth off the ROS graph (`truth.bin`). Still to fix: the recorder can lose the last ~0.6 s of a
high-rate topic (3 of 20 recorded runs). Then realism: the sensors' datasheet behaviour (Mid-360
sweep, IMU noise and bias, ArduPilot's IMU and barometer).

Evaluate LiDAR-inertial odometry on the recordings. Feed pose, velocity,
uncertainty and health to ArduPilot's external-navigation input. Hover and make
short moves with navigation GPS off; measure error against the evaluator.

Next: local mapping, collision-aware motion, tracking-loss handling, and
GPS ↔ external-navigation transitions tested in motion, keeping frame and
return route. GPS degradation needs an explicit model.

First full mission: leave home, reach a given entrance, map a bounded unknown
interior, exit, return, land. Its behaviours become reusable tasks under a
manager that handles priority, interruption and resumption while perception,
mapping, planning and control run concurrently. General-purpose behaviour
grows from these tested pieces.

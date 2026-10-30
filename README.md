# Ironlark

A playable Linux world where real drone autonomy software perceives, plans and
flies. Missions will span departure, outdoor travel, GPS-denied exploration,
inspection and return. Everything is simulated. Named for the holding
pattern: sit back and watch autonomy fly.

Today: a reproducible Webots–ArduPilot flight (take off, hover, half turn, land on EKF3
with simulated GPS) over a Poly Haven pine forest, on a hilltop above a
valley. What comes next is in
[ROADMAP.md](ROADMAP.md).

## Quick start

Needs Linux x86-64, OpenGL, Docker, [uv](https://docs.astral.sh/uv/), `git`,
`xvfb-run`, and [Webots R2025a](https://github.com/cyberbotics/webots/releases/tag/R2025a)
extracted to `./webots/`. `build-renderer` also needs gcc, g++, make, and the
development files of OpenGL, GLU, OpenAL, FreeType and zlib; `doctor`
compiles and links against each to list what's missing.

```sh
./ironlark setup           # pinned SITL/ROS images, Python env and bridge assets
./ironlark doctor          # list missing runtime and build requirements
./ironlark build-renderer  # compile the patched renderer
./ironlark check           # validate takeoff/hover/land in the empty world
./ironlark build-world     # resumable forest export and drone build
./ironlark run             # watch the forest flight, fullscreen
```

| Command | Does |
| --- | --- |
| `setup` | Builds ArduPilot Copter 4.7.1 (`dbe7921`) in an Ubuntu 22.04 image, which keeps only the SITL binary, its parameters and pymavlink (0.3 GB; Docker's build cache holds the rest); extracts its Iris model and Webots bridge to `.cache/`; creates `.venv/` (Python 3.12.11, NumPy, Pillow). System Python is untouched. |
| `doctor` | Lists missing runtime and renderer build requirements, and forest readiness. |
| `build-world` | Downloads Blender 4.2.9 and the [Poly Haven Pine Forest](https://polyhaven.com/collections/pine_forest) scene (checksummed), exports it stage by stage, and assembles `worlds/pine_forest/` in an empty folder beside it, writing last its receipt (`tools/receipts.py`): the pipeline's code as the build began, the NumPy, Pillow and etcpak versions, and every output's size and sha256. The finished folder then replaces the world, so an interrupted build leaves the last one, and a build whose code changed while it ran fails. Every stage keeps its result in `.cache/forest-stages/` by the bytes it reads (its code, inputs and toolchain; `tools/stage_cache.py`): each export stage, run in a folder holding only what it declares, the bakes, and the assembly's steps (valley, terrain tiles and detail, edge ground, impostor atlases, textures, meshes). An unchanged one is cloned back (a btrfs reflink: no extra disk), so a build redoes only what an edit changed. A file is hashed once, and known after by its inode, size and timestamps (`.cache/digests.jsonl`; deleted, the next build hashes everything again), and a world already matching today's code byte for byte is left as it is. Other export code empties `.cache/forest-build` first, and the store refills it; after a full build the store keeps only what each checkout's latest build used. Forest runs refuse a world whose receipt does not match today's code. `--skip-export` only reassembles, from bakes today's code finished. |
| `build-drone` | Models the drone in Blender and writes `protos/IronlarkDrone.proto` (also run by `build-world`, `run` and `check` when an input or output differs from its receipt, `protos/drone/receipt.json`). |
| `build-renderer` | Builds patched Webots into `.cache/webots-renderer/`. See [Webots patches](#webots-patches). |
| `render-reference` | Cycles renders of the source scene at the preview cameras, into `runs/reference/`: the ground truth for looks. Needs more than ~26 GB RAM (15 GB + zram was not enough); needs no built world. |
| `run` | Flies over the forest in a fullscreen window, then exits. Until the autopilot can arm (EKF3 learning its gyro biases, ~40 s of simulated time) the simulation runs as fast as it can, the view at 10 fps; takeoff comes ~16 s after launch. `--world empty` for the bare test area; `--editor` for Webots' UI; `--record` streams the sensors to ROS 2 and records a rosbag2 of the flight in the run folder (in real time throughout). |
| `replay RUN` | Plays a recorded run's bag in the ROS 2 container; its `/clock` topic carries Webots time. |
| `check-recording RUN` | Checks a recorded flight: the bag is complete (every topic, a message every period, over the whole flight), and the IMU at rest reads gravity, no rotation and the rig's noise. Every LiDAR frame's times and lines. In the empty world, then the calibration wall: LiDAR distance, orientation and scatter against the rig's noise, each point placed at its own instant; turning, the same scatter, and much more with every point at its frame's start; camera ↔ LiDAR agreement, and the ground in front of the wall not red. |
| `check-replay RUN` | Replays a recorded flight while recording it again and compares: every topic present, the same messages byte for byte, in order. |
| `check` | The same flight on a virtual display, empty world by default. Exits nonzero on failure. |

For empty-world flights, Webots is chosen in order: `WEBOTS_HOME`, a current
patched build, `./webots/`. The forest needs a current `build-renderer`: stock
Webots cannot read its patched fields, DDS textures or HDR sky. A change to the
patch series, the native sources or the Webots checkout makes a build stale,
except an edit to a shader or node file the build already serves from the
checkout, which is live. To ignore a `WEBOTS_HOME` override in sh or fish, use
`env -u WEBOTS_HOME ./ironlark run`.

`build-renderer` compiles uncommitted edits in the Webots checkout and serves
changed resources from its working tree. A failed compile leaves the previous
runtime in place and prints the compiler and linker errors; downloads are
checked through `.part` files. Build stages run with two jobs and stop at
4.5 GiB of memory (PSS: pages a stage's processes share count once); logs and metrics go to `.cache/build-logs/`.
First setup needs internet and several GB of disk. OS packages in the image float,
so builds are source-pinned, not bit-identical.

Configure in files, not panels; restart to apply:
[`worlds/flight_foundation.wbt`](worlds/flight_foundation.wbt) (empty-world scene, vehicle, camera, timestep),
[`config/flight.parm`](config/flight.parm) (autopilot overrides),
[`config/sensors.json`](config/sensors.json) (sensor poses, rates, ranges, noise),
[`tests/flight_smoke.py`](tests/flight_smoke.py) (the flight check),
[`ironlark`](ironlark) (startup, versions, cleanup),
[`tools/`](tools) (the forest pipeline).

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
`telemetry.jsonl`, evaluator-only `ground_truth.jsonl`, `world-map.png` (forest), and Webots, SITL and
ArduPilot logs. Ctrl+C or failure tears everything down; that is cleanup, not a
recovery policy.

## Sensors and ROS 2

The drone ([`tools/build_drone.py`](tools/build_drone.py)) keeps the Iris flight model (motors,
propeller constants, mass, inertia, collision box, and the noiseless IMU and GPS ArduPilot reads, which are also ground truth; SITL adds its own noise, the barometer an MS5611's: `config/flight.parm`)
and carries the rig of `config/sensors.json`: a Livox Mid-360 (below), the Mid-360's own IMU (an ICM-40609-D 4.4 cm under the LiDAR's origin, at 250 Hz: datasheet noise, and bias, scale and cross-axis errors drawn once from a seed within its tolerances) and a front camera
(640 × 480, 90°, 20 Hz, with the scattering fog). With `--record`, the flight
controller enables them before the first 2 ms physics step, so each refreshes on its period's
multiples, reads them there, on Webots time, and streams them over local
TCP ([`sensing/protocol.py`](sensing/protocol.py)); a full queue drops and counts, never stalling
physics, and pausing stops the clock. A reading older than its stamp fails the stream: the IMU's gyro against the Iris's; the LiDAR's render if it changed before use. After the flight the stream ends: the bridge acknowledges
every message, and a run whose stream dropped, failed or left a message unsent fails, naming the loss
(`sensing.json` in the run folder). A ROS 2 Humble container ([`ros/`](ros)) publishes `/clock`,
`/imu/data` (angular velocity and acceleration; an IMU measures no orientation), `/lidar/points`
(x, y, z, ring, time: returns only, each in the sensor frame at its own instant, time in seconds after the frame's stamp), `/camera/image_raw` (bgra8), `/camera/camera_info` and `/tf_static`, and records all of it with rosbag2 (reliable, deep queues: `ros/record_qos.yaml`; each message compressed with zstd, losslessly: a forest flight's minute is 1.1 GB, the empty world's 0.15 GB). The recorder writes each message as it arrives and is stopped only once the bag holds every message the bridge published (`published.json`): rosbag2 drops what it has not yet taken, and lagged up to 0.6 s at the end of a flight. Ground truth never enters ROS: the controller writes it to `truth.bin` in the run folder (position, attitude, velocity every step; `sensing/protocol.py`), for checks only. A recorded check adds sensing's cost to `result.json` (real-time factor, messages sent); every flight adds what it cost the machine: peak RAM and CPU time of Webots with its controllers, SITL and ROS, and GPU memory, Webots' own and the card's, each named when it could not be measured. Frames are REP-103: Webots' world is
ENU and bodies FLU; [`sensing/frames.py`](sensing/frames.py) is the one place for ENU/FLU ↔ NED/FRD.

The LiDAR sweeps as a Mid-360 ([`sensing/mid360.py`](sensing/mid360.py)): a slot every 4.947 µs
(200,000/s) on 4 lines, a head spinning 185 times a second, the beams nodding between −7° and +52°
every 101 ms; spin and nod share no period, so the pattern never repeats. Fitted by
[`tools/fit_mid360_pattern.py`](tools/fit_mid360_pattern.py) to a real unit's recording
(outdoor_kidnap_a of the Hard Point Cloud Localization Dataset, Zenodo
[10122133](https://zenodo.org/records/10122133), CC BY 4.0): residuals 1.8° elevation, 1.7° the
beams' fan. Webots renders a 360° range panorama (0.1° pixels) every 20 ms; each slot is cast into
the render nearest its instant from where ground truth puts the sensor then, and meets the plane
the render saw there (exact on flat surfaces; across depth edges, the rendered point: Webots' panorama
blends ranges only across a surface, patch 0024). A return needs
enough light back, every surface Lambertian at 30% reflectivity (forest at 905 nm) against the
datasheet's 40 m at 10%; then range noise 2 cm and angular noise 0.15° (datasheet). Frames of
100 ms go out once whole, 10 ms after they end. Not modelled: the beam's footprint (sharp edges, no
mixed returns), range accuracy beyond precision, returns past 40 m (the render's far plane).

## Webots patches

Ironlark patches Webots where Webots is the bottleneck, rather than degrading
assets to suit it. No fork: [`native/patches/`](native/patches) is a series of
`git format-patch` commits, one per feature, applied in order to
[R2025a](https://github.com/cyberbotics/webots/tree/c6793d8f7230a311c4bc2a3101d9f1a8bc0aa01b)
(`c6793d8`, Apache 2.0). Each patch's message says what it changes and why, and
what each patch of the earlier numbered series it replaces measured.
`./ironlark build-renderer` fetches sparse sources and matching headers, makes
the series branch `ironlark` of `.cache/webots-source/` (upstream plus one commit
per patch; the same series always gives the same commits), compiles `glad`,
`wren` and `webots` against the installed R2025a libraries, and installs to
`.cache/webots-renderer/`, beside the untouched stock `./webots/`. Moving the
branch rewrites only the files that differ, so only they recompile: a one-line
edit to the last patch saves in 1 s and builds in 7 s; to the fog patch, in 9 s,
the later patches' files coming back from `ccache` (when installed).

Measured with `tools/profile_forest.py` (GTX 1060 3 GB, i5-4670):

| Patch | Changes | Measured |
| --- | --- | --- |
| **0001-Filmic-tone-mapping-and-HDR-sky** | Blender's Filmic "Medium High Contrast" as a 65³ table (`tools/bake_filmic_lut.py`) replaces `1 − e^(−x)` plus gamma, within 1.3/255 of Blender at the 99th percentile. `Background` accepts `.hdr` faces, kept as half-float linear radiance. PBR diffuse light is linear in intensity, which equals Blender sun strength. | frame unchanged; +12 MB VRAM |
| **0002-Alpha-masked-foliage** | `PBRAppearance.alphaCutoff` (glTF `MASK`): texels cut, not blended, with mip-scaled alpha. Masked materials draw both faces, back faces shaded towards the viewer. Minified normal maps shade by the mipmapped normal, darkened by its length (how much its texels disagree), instead of a frame from screen derivatives; degenerate frames fall back to the surface normal. | frame 31 → 22 ms (shadows off) |
| **0003-Instanced-shapes-and-visibility-ranges** | `Shape.instancesUrl`: one shape at every transform in a binary file, culled per instance, one instanced call (mirroring transforms in a second, front face inverted). `Shape.visibilityRange`: a camera-distance band, per instance. Passes that cannot place instances skip them (picking caught phantoms at the origin). A camera's frustum follows it after `view()` (robot cameras culled with their first frustum). | load 42 → 20 s, frame 25 → 5.7 ms, RAM 3.7 → 2.3 GB, 16,814 → 185 geometries |
| **0004-Cascaded-shadow-maps** | Directional lights cast through three cascaded shadow maps (2048², 100 m), not stencil volumes: any mesh size, alpha masks included, instanced casters culled per cascade at the viewer's LOD; far cascades reused until the view leaves them. Alpha-masked casters keep each shadow texel with probability equal to their coverage. | frame 5.7 → 12.3 ms; main sun under canopy vs Cycles 0.30× → 0.79× |
| **0005-Water-transmission-and-flowing-foam** | `PBRAppearance.transmission` (thin surface, Fresnel for IOR 1.333, premultiplied) and `flowFoam`: the source river's foam recipe on the UVs, white and opaque, animated by simulation time. | river pixels only |
| **0006-Tree-impostors** | `Shape.impostor`: instances drawn as quads showing the three captured views (8 × 8 hemi-octahedral atlas) nearest the camera's direction, lit from captured mean normals, with per-texel depth (`depth_greater`: hidden fragments rejected before shading). | backdrop 59 → 20 ms per step; impostors ≈ full trees within 4%; impostor pass 7.1 → 4.2 ms |
| **0007-Baked-light-occlusion** | `Background.lightOcclusion*`: baked light visibility layers at heights above the ground (sky, suns without shadow maps, the shadowed sun past its cascades, the sky overhead), mirrored outside. PBR surfaces scale image-based lighting and the unshadowed suns by them; near the viewer those suns also take the shadowed sun's maps, looked up once per pixel. | sky light under canopy vs Cycles 3.5–3.9× → 1.5–1.7×; +2 ms |
| **0008-Screen-space-reflections-on-water** | The opaque scene (colour, depth) is copied before translucent objects draw; transmissive surfaces march their reflection across it, taking the colour of what they hit or pass behind, the sky only where rays leave the screen. | river colour vs Cycles within 10%; +0.3 ms |
| **0009-Scattering-fog** | `Fog { fogType "SCATTERING" }`: single-scattering medium in HDR (density, colour, Henyey–Greenstein anisotropy, box) for the main view and robot cameras, lit through the light occlusion layers, the shadow maps near the viewer (shafts) and the sky it scatters along each view direction (`Fog.ambientRadiance`). `boxFalloff`, `boxFalloffHorizontal`: density thins past the box's faces. `horizonRadiance`: far haze meets the sky at the horizon. `hazeDistance`: land past the box takes the horizon's colour with distance. | ~1–2 ms; its added light matches Cycles (×1.34 vs ×1.34); fog pass 6.9–8.9 ms at 1920×1080 |
| **0010-Viewpoint-bounds-and-navigation** | `Viewpoint.boundsMin/boundsMax` and `groundClearance` (a physics ray down): navigation and scripts slide along the limits. Left-drag orbits the surface the main view drew at the press (its depth, nearest within 4 px) when within 30 m (or 1.5× the view's height) and the bounds, else turns in place; both 0.25°/px, within 85° of level. Pan and zoom scale by that surface; the wheel by a physics ray (floor 2 m). | 200 px drags: valley from the high view 135 m → 0 (turns); overview 97 → 29–44 m; 1000 px tilt flipped over → stops at 85° |
| **0011-Multisampling-and-temporal-anti-aliasing** | Opt-in multisampling (`OpenGL/multisampling`, default 0) with alpha to coverage. Temporal anti-aliasing in the main view (`OpenGL/temporalAntiAliasing`, on): Halton jitter, history reprojected through depth, Catmull-Rom, neighbourhood clamp, depth rejection; robot cameras single-frame. | 4 samples: flicker −31%, +9.4 ms; TAA: flickering needles 3.9% → 0.36% of canopy pixels, +1.5% frame time |
| **0012-Terrain-detail** | `Background.terrainDetail*` and `PBRAppearance.terrainDetail`: near the viewer the ground takes the source's tiled layers (leaves, ground, rocky trail, by its `path`/`river` masks) as detail over its 2.5 cm bake. Roughness and mask stored as R8 and RG8. | GPU −56 MB (R8/RG8) |
| **0013-Range-sensors-and-device-overlays** | Range sensors draw instanced shapes and cut alpha-masked texels (impostors stay out). A camera or range finder overlay created after the world restored its perspective takes its saved visibility, size and position. | forest scan at 2.5 m: 20.8 k of 28.8 k returns, median 17 m |
| **0014-Frame-profiler** | `IRONLARK_FRAME_LOG=FILE`: each rendered view (main, robot cameras, range sensors) logs the CPU and GPU time of its passes, each main-view frame and physics step its clock times; `IRONLARK_FRAME_LOG_DRAWS=1` times every draw. [`tools/frame_log.py`](tools/frame_log.py) summarizes. | nothing unless set |
| **0015-Real-time-pacing** | Each step is due when the clock reaches the time it simulates; more than three frames late, the simulation slows instead. A frame is due one period after the last one started. Frames showed 36, 136, 38 ms of simulated time in turn. | forest flight 1920×1080: 6.8 → 7.8 fps, simulated time per frame p99 175 → 124 ms, 0.77× → 0.90× real time; empty world with sensing 0.82× → 1.0× |
| **0016-Instance-culling-cells-and-occlusion** | Instances sorted into 64 m cells and 16 m sub-cells, culled a cell at a time, spheres computed once, cells packed. Colour views cull instances hidden behind what they drew: last frame's visible set first, then a max-depth pyramid tests the rest; a view hiding under 10 M triangles draws in one pass until it draws 10 M more. Exact. `IRONLARK_OCCLUSION=0` turns it off. | camera frame 130 → 62 M triangles, 82 → 59 ms; forest view 88 → 70 ms; LiDAR scan CPU 22 → 14 ms; RAM −45 MB |
| **0017-Mesh-memory-and-load-time** | WREN merges identical vertices ([`native/vertex_index.hpp`](native/vertex_index.hpp)), builds each GPU mesh once, orders it for the vertex cache ([meshoptimizer](https://github.com/zeux/meshoptimizer), vendored in [`native/meshoptimizer/`](native/meshoptimizer)), uses 16-bit indices where they fit and builds stencil shadow volumes on first use. `WEBOTS_MESH_CACHE=DIR` keeps each mesh file's parse and vertex-cache order; file meshes reach WREN indexed; Webots' CPU copy keeps one set of coordinates and per-vertex float normals and texcoords. Images, LiDAR and physics unchanged (vertex-cache order: ≤0.02% of pixels, depth ties). | load 98 → 49 s (first three steps), 24 → 13 s (mesh cache), 8.0 → 7.0 s (indexed); RAM −1.5 GB, later −0.77 GB (CPU copies); GPU −45 MB; forest view 70.0 → 67.1 ms (vertex-cache order) |
| **0018-Compressed-textures** | `ImageTexture` reads BC7 and opaque BC1 DDS (mip levels included), no CPU copy; cached compressed textures never regenerate mipmaps. Decoded PNG textures are freed once uploaded. | GPU 2830 → 2376 MB, RAM 2199 → 1865 MB; RAM −335 MB (PNGs freed) |
| **0019-Cheaper-PBR-shaders** | Same images for less GPU work: PBR vertex shaders without per-vertex matrix inverses; pen texcoords read only in a `#define PEN` twin program; impostors test coverage first. | forest view −3.6 ms (vertex transforms), −2.0 ms (pen variant); overview −0.45 ms (impostors) |
| **0020-Faster-load-lazy-icons** | `webots-bin` links the system zlib ahead of assimp's copy; a world's PNG/JPEG files decode on a thread pool before its nodes finalize; toolbar and menu icons decode when first drawn. Same pixels. | forest load 12.8 → 11.0 s, 9.5 → 8.0 s; RAM 719 → 655 MB |
| **0021-Outer-light-occlusion** | `Background.lightOcclusionOuter*`: light layers over a larger rectangle at the ground as it lies, used past the first by surfaces and the fog; past both, open land in full light. | — |
| **0022-Water-depth-colour-and-wind-ripples** | `PBRAppearance.attenuationColor`/`attenuationDistance` (glTF volume attenuation, along the path to what lies behind, from the scene's depth copy), `scatterColor` (deep water's own colour) and `waves` (wind ripples 3 m to 0.1 m drifting downwind in gusts, finer ones as roughness). Drawn by a `WATER` variant of the material's program (the pen twin generalized), so other materials' shaders are unchanged. The valley's lake and river use it. | GPU +0.3 ms with the lake filling the view (15.4 → 15.7 ms), other valley views within noise; forest views bit-identical |
| **0023-Range-views-draw-nothing-past-their-reach** | Panoramas (spherical, cylindrical) discard fragments past their range, measured from the camera, and skip instances wholly past it: their flat far planes reached 1.7× as far. Exact: a forest flight's `/lidar/points` byte-identical with the cull off (`IRONLARK_RANGE_REACH=0`). | Mid-360 panorama 37.3 → 27.7 ms GPU, 98.2 → 77.6 M triangles; forest flight 0.24 → 0.3× real time |
| **0024-Range-images-blend-only-across-a-surface** | A panorama's range blends the four sub-camera texels around a sample only within 2% of the nearest one's range (a plane at 85° incidence); across a depth edge (trunks, needle cards, the drone's propellers) it takes the nearest, not a point in the air between. Colour unchanged. | forest returns off every surface (beyond 4× the rig's spread) 4.62% → 0.035% |

After a build, `.cache/webots-source/` is a git checkout on branch `ironlark`,
where `git log`, `git blame` and `git rebase -i` work on the series. A new
feature is a new commit there (message: what it changes, why, what it measured).
An edit to an existing patch stays uncommitted:
`.venv/bin/python tools/save_webots_patch.py fog` folds it into the patch whose
name holds `fog`. Without a name, the tool writes the branch to
`native/patches/`. Then `./ironlark build-renderer`, `./ironlark check`,
`./ironlark check --world forest`, and the patch's row here (the tool names
patches without one). The builder never
moves the branch past commits not saved as patches, and git refuses to overwrite
uncommitted edits. Keep each patch one feature, clean enough to become an
upstream pull request.

A newer Webots release: in the checkout, `git fetch --depth 1 origin tag R2025b`
and `git rebase --onto R2025b c6793d8f`, resolving conflicts patch by patch;
then `RELEASE` and `COMMIT` in `tools/renderer_support.py`, that release
installed as `./webots/`, and a save.

`tools/render_lighting_reference.py` renders the built world in Cycles under the
source's own suns, world, fog and Filmic view (4.6 GB; the full source needs more
than ~26 GB); `tools/compare_views.py runs/lighting-reference runs/preview` scores
`tools/render_forest.py` views against it. `render_forest.py --repeatable` renders a frame after every physics step, so two captures of one build are bit-identical (temporal anti-aliasing otherwise follows wall time); `compare_views.py MAIN BRANCH --tiles` lists the 60 px tiles where two such captures differ.
`tools/profile_forest.py --out runs/<name>` reports load time, real-time factor,
frame rate, GPU and CPU time per render pass, and peak RAM, CPU time and GPU memory (Webots' own and the card's; `tools/run_resources.py`) for the forest as a
viewer sees it (headless: under `xvfb-run -a -s "-screen 0 1920x1080x24"`, which
renders on the NVIDIA GPU; its swap waits for a readback, so frames come slower than
on a display). Any run can log frames: `IRONLARK_FRAME_LOG=/tmp/f.log ./ironlark run`,
then `tools/frame_log.py /tmp/f.log --from 41 --to 60` (simulated seconds).
`tests/test_vertex_index.cpp` proves hard normals and UV seams survive indexing
and that a large mesh's attributes reconstruct byte-for-byte:
`g++ -std=c++11 -O2 tests/test_vertex_index.cpp -o .cache/test_vertex_index && .cache/test_vertex_index`.

## Research atlas

159 drone autonomy projects and 126 sourced relationships, in
[`research/dataset.json`](research/dataset.json), which omits the fields that follow
from others (`identity`, `referenceOnly`, `assemblyGroup`, `classificationReason`, a
project's repository as first source, edge `id`); the viewer rebuilds them and its
download button exports the full records.
`python3 research/build_html.py` builds `research/drone-autonomy-atlas.html`,
which opens offline in a browser; source links need internet.

## References and licenses

[ArduPilot Webots integration](https://ardupilot.org/dev/docs/sitl-with-webots-python.html) ·
[pinned example](https://github.com/ArduPilot/ardupilot/tree/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/SITL/examples/Webots_Python) ·
[ArduPilot ROS 2](https://ardupilot.org/dev/docs/ros2-install.html) ·
[GPS/non-GPS transitions](https://ardupilot.org/copter/docs/common-non-gps-to-gps.html)

ArduPilot, Webots, Blender, Poly Haven and saved research material keep their
own terms; setup saves ArduPilot's license beside its cache. meshoptimizer (MIT)
is vendored with its license in [`native/meshoptimizer/`](native/meshoptimizer).

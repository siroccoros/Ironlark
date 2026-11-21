<div align="center">

<br/>

```
██╗     ██╗ ██████╗ ███╗   ██╗ █████╗ ██╗   ██╗
██║     ██║██╔═══██╗████╗  ██║██╔══██╗██║   ██║
██║     ██║██║   ██║██╔██╗ ██║███████║██║   ██║
██║     ██║██║   ██║██║╚██╗██║██╔══██║╚██╗ ██╔╝
███████╗██║╚██████╔╝██║ ╚████║██║  ██║ ╚████╔╝ 
╚══════╝╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚═╝  ╚═╝  ╚═══╝  
```

### End-to-End Autonomous Drone Navigation Stack

**GPS-denied flight in caves, mines, tunnels, and high-risk industrial interiors**

<br/>

![Platform](https://img.shields.io/badge/Platform-ROS_2_Humble-blue?style=flat-square&logo=ros)
![Language](https://img.shields.io/badge/Language-C++17_|_Python_3-informational?style=flat-square)
![GPU](https://img.shields.io/badge/Mapping-CUDA_|_nvblox-76b900?style=flat-square&logo=nvidia)
![Status](https://img.shields.io/badge/Status-Private_Production-red?style=flat-square)
![License](https://img.shields.io/badge/Source-Private_IP-lightgrey?style=flat-square)

<br/>

> *This repository is intentionally documentation-only for IP and safety reasons.*  
> *The full implementation lives in a private repository.*  
> *This README reflects the actual architecture, interfaces, and deployed capabilities.*

<br/>

</div>

---

## What Is LIONav?

LIONav is a **production-grade, end-to-end autonomous drone flight stack** built for GPS-denied environments where off-the-shelf autopilots fail. It is not a research prototype or a stitched-together demo — it is a complete autonomy system engineered around failure modes, constrained edge hardware, degraded communications, and real-world repeat operations.

It was designed, architected, and implemented entirely by a single engineer, covering every layer:

> Sensor fusion → 3D mapping → SLAM → planning → safety → flight execution → comms protocol → ground control → mission data lifecycle

The system can fly autonomously through an unexplored cave with no GPS, build a 3D map in real time, maintain a continuously-valid escape route home, and return safely under communications blackout — all without operator intervention.

---

## System Architecture

```mermaid
flowchart TD
    subgraph SENSORS["① Sensor Layer"]
        S1[LiDAR + IMU]
        S2[Flight Controller Telemetry]
        S3[RGB + Thermal Cameras]
    end

    subgraph PERCEPTION["② Perception & Mapping"]
        P1["FAST-LIO\nLiDAR-Inertial Odometry + Registered Cloud"]
        P2["lio_continuity_bridge\nOdometry / Frame Continuity across LIO Discontinuities"]
        P3["nvblox_fastlio_node\nGPU ESDF — Sliding 3D Window · CUDA TSDF Pipeline"]
    end

    subgraph SLAM["③ SLAM Backend"]
        G1["pose_graph_backend_node\nGTSAM iSAM2 Factor Graph · Scan Context + ICP/NDT Loop Closure"]
        G2["flyable_pose_graph_builder_node\nESDFValidated Traversability Graph over Global Pose Graph"]
        G3["flyable_pose_graph_planner_node\nPath Planning over Flyable Graph"]
    end

    subgraph AUTONOMY["④ Autonomy Manager"]
        A1["lionav_planner_manager\n6-Level Hierarchy with Auto-Degradation"]
        A2["lionav_rtl_planner\nSmart RTL — Always-On · 0.5s Replan Cycle"]
        A3["safety_monitor_node\nClearance / Data-Age / IMU Watchdog"]
        A4["collision_shield_node\nESDFFiltered cmd_vel Gate"]
        A5["lionav_rts_planner\nStream-Health Watchdog → Auto Failsafe"]
    end

    subgraph FC["⑤ Flight Controller"]
        F1["ardupilot_mavlink_bridge\nMAVLink Mode & Mission Control · ODOMETRY + Obstacle Forwarding"]
    end

    subgraph TRANSPORT["⑥ Transport Layer"]
        T1["lionav_native_bridge\nCustom LION UDP Binary Protocol\nFragmentation + Bi-directional Commands"]
        T2["lionav_web_bridge\nWebSocket Binary · WebGPU / WebGL2 · Multi-client"]
    end

    subgraph GCS["⑦ Ground Station"]
        V1["lionav_viewer_gcs\nQt6 / C++17 — Linux · Windows · Android\n50M Points · Telemetry · Camera · Mission UI"]
        V2["lionav_project_manager\nRecording · Replay · Project Lifecycle"]
    end

    S1 --> P1
    S1 --> P2
    S2 --> F1
    S3 --> T1

    P1 --> P3
    P2 --> P3
    P3 --> G1
    G1 --> G2
    G2 --> G3
    G3 --> A1

    A1 --> A2
    A1 --> A3
    A1 --> A4
    A1 --> A5
    A2 --> F1
    A5 --> F1

    F1 --> T1
    A1 --> T1
    A2 --> T1
    A1 --> T2

    T1 --> V1
    T2 --> V1
    V2 --> V1
```

---

## Three Core Pillars

### Pillar 1 — Autonomous Flight Stack (`flyable_autonomy.launch.py`)

The primary mission launch entrypoint. A single launch file that deterministically brings up the complete autonomy chain with runtime feature toggles per deployment profile.

**What it orchestrates:**

| Component | Role |
|---|---|
| `FAST-LIO` | Real-time LiDAR-inertial odometry; registered point cloud output |
| `lio_continuity_bridge` | Maintains odometry and frame continuity across LIO restarts or time gaps |
| `nvblox_fastlio_node` | GPU-accelerated local ESDF on a sliding 3D window (CUDA/TSDF pipeline) |
| `pose_graph_backend_node` | Global pose graph with GTSAM iSAM2; loop closure via Scan Context + ICP/NDT |
| `flyable_pose_graph_builder_node` | Builds ESDF-validated traversability graph over the global pose graph |
| `flyable_pose_graph_planner_node` | Path planning over the flyable graph |
| `lionav_planner_manager` | 6-level autonomy hierarchy with automatic degradation (see below) |
| `lionav_rtl_planner` | Always-on Smart RTL with continuous obstacle-aware replanning |
| `lionav_rts_planner` | Stream-health watchdog; auto-triggers backtrack/SmartRTL/failsafe on signal loss |
| `ardupilot_mavlink_bridge` | MAVLink mode and mission control, ODOMETRY integration, obstacle forwarding |
| `lionav_native_bridge` | Custom UDP binary transport to ground station |
| `lionav_web_bridge` | WebSocket binary bridge for browser-based operators |
| `lionav_project_manager` | Mission recording, replay, project lifecycle backend |

**Design intent:** Clear separation of perception, planning, control, transport, and ops tooling — composable, not monolithic.

---

### Pillar 2 — Multi-Session Relocalization (`relocate_and_fly.launch.py`)

Enables repeat inspections of the same site across separate sessions — a hard problem in GPS-denied SLAM. Rather than building a new map each flight, the drone loads a prior map bundle and localizes itself within it before resuming full autonomy.

**Relocalization pipeline:**

```mermaid
flowchart LR
    subgraph OFFLINE["Offline — Prior Session"]
        O1["rosbag\nPrior Flight Recording"]
        O2["generate_map.launch.py\nFAST-LIO + Pose Graph Optimization\n+ Map Bundle Builder"]
        O3[("Map Bundle\nKeyframes · Descriptors\nGlobal Map Artifacts")]
        O1 --> O2 --> O3
    end

    subgraph ONLINE["Online — New Session"]
        N1["map_bundle_server_node\nLoads + Serves Map Artifacts"]
        N2["global_relocalizer_node\nScan Context Retrieval — Coarse\nICP / NDT Refinement — Precise"]
        N3["/pose_graph/global_pose\n+ map→odom TF Correction"]
        N4["Full Autonomy Stack\nESDFGraph · Planner · RTL · MAVLink"]
        N5["ros2 bag record\nContinuous Ops Traceability"]

        N1 --> N2 --> N3 --> N4
        N4 --> N5
    end

    O3 -->|Load| N1
```

**Use cases:** Infrastructure inspection on repeat schedules, mining shaft re-entry, survey missions where previously mapped environments must be revisited without a ground truth reference.

---

### Pillar 3 — Cross-Platform Ground Station (`lionav_viewer_gcs`)

A purpose-built operator cockpit targeting Linux, Windows, and Android from a single C++/Qt codebase. Engineered for high-density real-time visualization under constrained hardware and poor network conditions.

| Capability | Detail |
|---|---|
| **Rendering** | Qt6 / C++17 with OpenGL; up to **50,000,000-point** accumulation buffer |
| **Live visualization** | Point cloud, odometry trajectory, planned path, Smart RTL path overlays |
| **Telemetry** | Battery, temperature, signal, mode, attitude, timing, flight state, GPS fix, status text |
| **Camera feeds** | RGB + thermal via RTSP / GStreamer / Qt Multimedia fallback; simultaneous dual feed |
| **Mission control** | Project browser, start/stop recording, replay with play/pause/stop/seek/speed |
| **Smart RTL trigger** | One-click trigger with safety confirmation — autonomy stack computes and executes obstacle-free path in real time |
| **Transport** | Custom `LION` binary protocol — not ROS, not MAVLink, not generic middleware |

---

## Autonomy Framework — 6-Level Hierarchy

`lionav_planner_manager` runs a structured hierarchy with **automatic degradation toward safer behavior** on any anomaly:

```mermaid
flowchart TD
    L5["� Level 5 — Full Exploration\nFrontier-based autonomous mapping"]
    L4["� Level 4 — Risk Exploration\nUnknown-space traversal with risk constraints"]
    L3["� Level 3 — In-Map Navigation\nGoal-directed planning in known free space"]
    L2["� Level 2 — Shortcut Backtrack\nESDFvalidated retreat shortcuts"]
    L1["� Level 1 — Pure Backtrack\nConservative path-history retreat"]
    L0["� Level 0 — Failsafe\nHover · Land · Backtrack · RTL emergency"]

    L5 -->|anomaly detected| L4
    L4 -->|anomaly detected| L3
    L3 -->|anomaly detected| L2
    L2 -->|anomaly detected| L1
    L1 -->|anomaly detected| L0
```

> Autonomy level can be set at launch time or commanded in flight via ROS services and actions.  
> Promotion to a higher level requires explicit clearance — degradation is always automatic.

---

## Smart Return-To-Launch — Always-On Escape Path

`lionav_rtl_planner` is **continuously running**, not activated on demand. It maintains a valid, obstacle-free path home at all times — so when it's needed, it's already computed.

**Real-time shortest return-to-launch path discovery:**

![Smart RTL path discovery](docs/smart_rtl.gif)

```mermaid
flowchart LR
    subgraph INPUTS["Inputs — updated continuously"]
        I1["Live ESDF\n0.5s replan cycle"]
        I2["Path History\nRecorded flight corridor"]
        I3["IMU Gravity Offset\nInclined terrain handling"]
    end

    subgraph PLANNER["RTL Planner Core"]
        P1["Weighted A*\nClearance-aware cost function"]
        P2["Dynamic Prefix\nNear-drone — replanned every 0.5s"]
        P3["Static Suffix\nCached known-safe corridor"]
    end

    subgraph OUTPUTS["Published Outputs"]
        O1["Full RTL Path\noperator visualization"]
        O2["Trigger Service\nplanner + FC bridge integration"]
    end

    I1 --> P1
    I2 --> P1
    I3 --> P1
    P1 --> P2
    P1 --> P3
    P2 --> O1
    P3 --> O1
    O1 --> O2
```

When the operator triggers Smart RTL from the GCS — or when the RTS watchdog detects sustained signal degradation — this path is **immediately executable**.

---

## Live Mapping

**Live mapping output: point cloud evolution, trajectory updates, and planner response:**

![Live mapping and point cloud evolution](docs/live_mapping.gif)

---

## Safety Architecture

Safety is woven into every layer — not added on top:

```mermaid
flowchart TD
    subgraph WATCHDOGS["Independent Watchdogs"]
        W1["safety_monitor_node\nClearance · Data-age · IMU anomaly checks"]
        W2["lionav_rts_planner\nStream-health monitor\nbacktrack → SmartRTL → failsafe cascade"]
    end

    subgraph GATES["Active Safety Gates"]
        G1["collision_shield_node\nESDFfiltered cmd_vel\nBlocks unsafe velocity commands"]
        G2["lio_continuity_bridge\nPrevents odometry frame jumps\nfrom destabilizing planning stack"]
        G3["Relocalization Gating\nVerification + reset thresholds\nbefore autonomy re-attaches"]
    end

    subgraph FALLBACK["Fallback Chain"]
        F1["Planner auto-degrades\nLevel 5 → 4 → 3 → 2 → 1 → 0"]
        F2["Smart RTL\nContinuously maintained\nnever recomputed under pressure"]
        F3["Failsafe\nHover · Land · Emergency RTL"]
    end

    W1 --> G1
    W2 --> F1
    G2 --> F1
    G3 --> F1
    F1 --> F2
    F2 --> F3
    G1 --> F1
```

> **The system is designed to land safely, not crash safely.**

---

## Communication Protocol — `LION` Binary Protocol

The ground station uses a **custom binary protocol** designed from scratch — not ROS over the air, not MAVLink GCS extensions, not a generic UDP wrapper.

**Packet layout:**

```mermaid
packet-beta
  0-31: "LION magic (4 bytes)"
  32-39: "Version"
  40-47: "Packet type"
  48-79: "Fragment header (4 bytes)"
  80-127: "Payload (variable)"
```

**Why custom — not protobuf, not ROS2 DDS:**

| Concern | Solution |
|---|---|
| 50M-point cloud streaming | Binary packing; zero-copy fragmentation and reassembly at receiver |
| Packet loss resilience | Fragment header enables partial reassembly without full retransmit |
| Parsing overhead | No schema serialization — raw binary, receiver knows the layout |
| Control latency | Bi-directional command plane over the same socket; no separate channel |
| Constrained links | Per-client rate limiting and throttling baked into protocol design |

**Downlink streams:** point cloud · odometry trajectory · planned path · Smart RTL path · telemetry · autonomy status · safety status

**Uplink commands:** planner level · mission commands · failsafe/backtrack/exploration requests · FC execute/stop hooks · project manager controls · replay controls

The `lionav_web_bridge` exposes the same data model over WebSocket binary for browser clients with a WebGPU renderer (WebGL2 fallback) and per-client rate limiting.

---

## Technology Stack

| Layer | Technology |
|---|---|
| **Middleware** | ROS 2 Humble, `ament_cmake`, `colcon` |
| **Languages** | C++17 — core, GCS, bridges · Python 3 — launch, tooling |
| **Localization** | FAST-LIO — LiDAR-inertial odometry |
| **GPU Mapping** | NVIDIA nvblox — CUDA TSDF/ESDF |
| **SLAM Backend** | GTSAM iSAM2 · Scan Context · ICP + NDT loop closure |
| **Point Cloud** | PCL · Eigen |
| **Planning** | Custom flyable graph planner · 6-level autonomy manager · weighted A* RTL |
| **Flight Integration** | MAVLink / ArduPilot |
| **Ground Station** | Qt6 (Widgets, Network, Multimedia, OpenGL) · C++17 · OpenGL |
| **Web Client** | WebSocket · WebGPU · WebGL2 |
| **Transport** | Custom `LION` UDP binary protocol · WebSocket binary |
| **Build** | `colcon` · `ament` · `CMake` |

---

## Engineering Ownership

This is a solo engineering project. Every component was designed, architected, and implemented by one engineer:

| Domain | Scope |
|---|---|
| **System architecture** | Package decomposition, component boundaries, data flow design |
| **Launch orchestration** | Deterministic startup chains, feature toggles, deployment profiles |
| **Sensor fusion** | FAST-LIO integration, frame continuity bridge, IMU handling |
| **GPU mapping** | nvblox ESDF pipeline integration and sliding-window configuration |
| **SLAM backend** | Pose graph construction, loop closure pipeline, relocalization |
| **Planning** | Flyable graph construction, 6-level autonomy hierarchy, Smart RTL |
| **Safety systems** | Monitor watchdog, collision shield, RTS watchdog, failsafe chain |
| **Multi-session workflows** | Map bundle builder, relocalization gating, map server |
| **Protocol design** | LION binary protocol, fragmentation, bi-directional command plane |
| **Ground station** | Qt6 GCS, OpenGL renderer, 50M-point accumulation, telemetry UI, camera feeds, mission controls |
| **Web client** | WebSocket bridge, WebGPU/WebGL2 renderer, multi-client management |
| **Mission data lifecycle** | Project manager, recording pipeline, replay with seek/speed, metadata indexing |

---

## What Makes This Production-Relevant

Most autonomy stacks are built for demos. LIONav is built for operations:

- **Failure-first design** — every component has a degraded-mode behavior; nothing assumes nominal conditions
- **Edge compute** — runs on constrained onboard hardware; GPU mapping is bounded by sliding-window ESDF, not unbounded voxel growth
- **Comms-degraded ops** — Smart RTL is precomputed; RTS watchdog triggers autonomous return before comms fully drop
- **Repeat operations** — persistent map bundles and multi-session relocalization mean the map is an asset, not discarded after each flight
- **Operator observability** — telemetry, safety status, autonomy level, and path visualization are live at the ground station throughout the mission
- **Field data continuity** — chunked recording, metadata indexing, and replay mean every flight is reconstructible for review or analysis

---

## Repository Note

```
This repository contains documentation only.
The implementation is private for IP and safety reasons.
All architecture, interfaces, and capabilities described here
reflect the actual deployed system.
```

---

<div align="center">

*Built for the environments where other systems refuse to fly.*

</div>

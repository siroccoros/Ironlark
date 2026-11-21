// The four programs and the channels between them. Click a channel or a box for what crosses it and where in the code.
import { svg, h } from "../lib/data.js";

const C = { gold: "#f2bd52", moss: "#a4d483", sky: "#7cc6ff", violet: "#b7a3ff", orange: "#ff7a2f" };

const boxes = [
  { id: "host", x: 14, y: 14, w: 1172, h: 600, c: "#6c7a72", frame: true, label: "host · Linux · every container on --network host" },
  { id: "launcher", x: 40, y: 56, w: 250, h: 96, c: C.gold, title: "ironlark", sub: "launcher · system Python",
    info: ["Launcher", "ironlark (756 lines, one file)", "Checks, builds and spawns everything, owns the run folder, writes result.json's resources, and tears it all down in a finally block.", "ironlark:406 flight()"] },
  { id: "runs", x: 40, y: 410, w: 250, h: 176, c: C.gold, dashed: true, title: "runs/<stamp>/", sub: "the run folder",
    files: ["manifest.json", "world-ready", "flight-state.json", "ground_truth.jsonl", "telemetry.jsonl", "sensing.json · bag/", "result.json"],
    info: ["Run folder", "runs/20261017-175412-220691/", "Evidence and signals. world-ready and sensing-end signal by existing; state files are renamed into place so no reader sees half of one.", "ironlark:429"] },
  { id: "webots", x: 352, y: 56, w: 380, h: 330, c: C.moss, title: "Webots R2025a", sub: "patched renderer · physics at 500 Hz", group: true,
    info: ["Webots", ".cache/webots-renderer/ (22 patches)", "Native on the host: physics (ODE) every 2 ms, rendering, the drone's devices, and two controller processes.", "worlds/flight_foundation.wbt:7"] },
  { id: "bridge", x: 376, y: 112, w: 332, h: 112, c: C.moss, title: "flight_bridge", sub: "Robot · the drone's controller", lines: ["lockstep.c: one packet each way per step", "sensor_stream.py: samples after each step"],
    info: ["flight_bridge", "controllers/flight_bridge/", "Python controller whose hot loop runs in C. Talks UDP to SITL and TCP to the ROS bridge.", "flight_bridge.py:23"] },
  { id: "observer", x: 376, y: 252, w: 332, h: 108, c: C.moss, title: "flight_observer", sub: "Supervisor · the evaluator", lines: ["reads the drone's true pose every 100 ms", "never commands anything"],
    info: ["flight_observer", "controllers/flight_observer/", "A bodiless supervisor robot. Writes ground truth; switches fast start off at ARMING; draws the forest's minimap.", "flight_observer.py:16"] },
  { id: "sitl", x: 812, y: 56, w: 348, h: 262, c: C.sky, title: "ironlark-sitl:copter-4.7.1", sub: "Docker · Ubuntu 22.04 · 0.3 GB", group: true,
    info: ["SITL container", "docker/Dockerfile", "ArduPilot Copter 4.7.1 at dbe7921, built for SITL with the clock patch applied; only the binary, parameters and pymavlink are kept.", "docker/Dockerfile:12"] },
  { id: "arducopter", x: 836, y: 108, w: 300, h: 78, c: C.sky, title: "arducopter", sub: "--model webots-python · EKF3",
    info: ["arducopter", "SITL binary", "The real Copter firmware compiled for Linux. Its webots-python backend exchanges packets with the bridge and synthesizes GPS, compass and barometer from them.", "ironlark:558"] },
  { id: "smoke", x: 836, y: 222, w: 300, h: 74, c: C.sky, title: "flight_smoke.py", sub: "docker exec · the ground station",
    info: ["flight_smoke.py", "tests/flight_smoke.py", "Connects over MAVLink, commands GUIDED, arm, takeoff, LAND, then judges the ground-truth trajectory.", "flight_smoke.py:153"] },
  { id: "ros", x: 812, y: 372, w: 348, h: 214, c: C.violet, title: "ironlark-ros:humble", sub: "Docker · ROS 2 Humble · with --record", group: true,
    info: ["ROS container", "ros/Dockerfile", "ros:humble-ros-base. Mounts the project read-only and the run folder for the bag and logs.", "ironlark:332"] },
  { id: "rosbridge", x: 836, y: 424, w: 300, h: 64, c: C.violet, title: "ironlark_bridge.py", sub: "TCP → 8 topics",
    info: ["ironlark_bridge.py", "ros/ironlark_bridge.py", "Reads the framed stream and publishes /clock, /imu/data, /lidar/points, /camera/*, /tf_static and /ground_truth/*.", "ironlark_bridge.py:152"] },
  { id: "bag", x: 836, y: 508, w: 300, h: 60, c: C.violet, title: "ros2 bag record", sub: "zstd per message · reliable QoS",
    info: ["rosbag2", "ros/record_qos.yaml", "Records the eight named topics with reliable, deep queues; each message compressed with zstd. A forest minute is about 1.2 GB.", "ironlark:528"] },
];

const channels = [
  { id: "fdm", c: C.moss, d: "M708 150 C 760 150, 780 136, 836 136", label: [770, 128], text: "UDP :9003", rate: 500,
    info: ["Sensor packet", "flight_bridge → arducopter", "16 doubles, 128 bytes: time, gyro, accelerometer, roll-pitch-yaw, GPS velocity and position.", "every 2 ms of simulated time", "lockstep.c:23"] },
  { id: "servo", c: C.sky, d: "M836 164 C 780 164, 760 182, 708 182", label: [770, 196], text: "UDP :9002", rate: 500, reverse: false,
    info: ["Motor packet", "arducopter → flight_bridge", "16 floats, 64 bytes: (pwm − 1000) / 1000 per output; disabled outputs arrive as −1.", "one per sensor packet, resent every 100 ms while waiting", "SIM_Webots_Python.cpp:71"] },
  { id: "mavlink", c: C.sky, d: "M986 222 L 986 186", label: [996, 208], text: "TCP :5760", rate: 10, both: true,
    info: ["MAVLink", "flight_smoke.py ↔ arducopter", "Heartbeats both ways, COMMAND_LONG and COMMAND_ACK, and all data streams at 10 Hz.", "10 Hz telemetry, commands as needed", "flight_smoke.py:133"] },
  { id: "stream", c: C.violet, d: "M708 206 C 770 206, 760 456, 836 456", label: [748, 330], text: "TCP :5790", rate: 1030,
    info: ["Sensor stream", "sensor_stream.py → ironlark_bridge.py", "Framed messages: 20-byte header (IRLK, type, Webots ns, length) then the payload. Acknowledged by count at the end.", "1,030 messages per simulated second", "protocol.py:11"] },
  { id: "dds", c: C.violet, d: "M986 488 L 986 508", label: [996, 502], text: "DDS", rate: 1030,
    info: ["ROS 2 topics", "ironlark_bridge → ros2 bag record", "Reliable QoS with deep queues; ROS_LOCALHOST_ONLY=1 keeps DDS on this machine.", "same rates as the stream", "ironlark_bridge.py:43"] },
  { id: "truth", c: C.moss, d: "M376 330 C 320 330, 340 480, 290 480", label: [300, 400], text: "ground_truth.jsonl", rate: 10,
    info: ["Ground truth", "flight_observer → run folder", "One JSON line per 100 ms of simulated time: time, position, velocity, read straight from the drone's node.", "10 Hz, read by judge() only after landing", "flight_observer.py:51"] },
  { id: "state", c: C.sky, d: "M836 270 C 780 270, 790 560, 290 560", label: [470, 552], text: "telemetry.jsonl · flight-state.json · result.json", rate: 10,
    info: ["Mission files", "flight_smoke.py → run folder", "Every MAVLink message as JSON; the current state (read by the observer for fast start); the verdict and measurements.", "10 Hz telemetry", "flight_smoke.py:65"] },
  { id: "spawn", c: C.gold, d: "M290 104 L 352 104", label: [300, 96], text: "", rate: 0, dashed: true,
    info: ["Spawning", "ironlark → everything", "Popen for Webots, docker run for SITL and ROS, docker exec for the mission, each in its own session so teardown can kill whole groups.", "once", "ironlark:535"] },
  { id: "signals", c: C.gold, d: "M165 152 L 165 410", label: [176, 280], text: "world-ready · sensing-end", rate: 2, both: true,
    info: ["Signals by file", "run folder ↔ ironlark", "The launcher waits for world-ready before the mission, touches sensing-end after landing, and reads sensing.json and result.json.", "a few times per run", "ironlark:607"] },
];

export default function system(root) {
  root.classList.add("sysmap");
  root.innerHTML = `<div class="w-title"><p>Processes and channels · click any line or box</p><p class="w-meta">packets drawn at relative rates</p></div>`;
  const s = svg("svg", { viewBox: "0 0 1200 630" }, root);
  const defs = svg("defs", {}, s);
  const blur = svg("filter", { id: "glow", x: "-50%", y: "-50%", width: "200%", height: "200%" }, defs);
  svg("feGaussianBlur", { stdDeviation: "3", result: "b" }, blur);
  const merge = svg("feMerge", {}, blur);
  svg("feMergeNode", { in: "b" }, merge);
  svg("feMergeNode", { in: "SourceGraphic" }, merge);

  const detail = h(`<div class="sys-detail"></div>`);
  root.append(detail);
  const show = (info, extra = "") => {
    detail.innerHTML = `<div><p class="k">${info[1]}</p><h5>${info[0]}</h5></div>
      <div><p class="k">carries</p><p>${info[2]}</p>${extra}</div>
      <div>${info.length > 4 ? `<p class="k">rate</p><p>${info[3]}</p>` : ""}<p class="k" style="margin-top:10px">in the code</p><p><code>${info[info.length - 1]}</code></p></div>`;
  };

  const nodes = {};
  for (const b of boxes) {
    const g = svg("g", { class: "node" }, s);
    nodes[b.id] = g;
    if (b.frame) {
      svg("rect", { x: b.x, y: b.y, width: b.w, height: b.h, rx: 22, fill: "none", stroke: "rgba(214,232,220,0.08)", "stroke-dasharray": "2 6" }, g);
      svg("text", { x: b.x + 22, y: b.y + 26, "font-size": 11, fill: "#6c7a72" }, g).textContent = b.label;
      continue;
    }
    svg("rect", { class: "box", x: b.x, y: b.y, width: b.w, height: b.h, rx: b.group ? 18 : 12,
      fill: b.group ? "rgba(255,255,255,0.012)" : `${b.c}12`, stroke: b.c, "stroke-opacity": b.group ? 0.35 : 0.75,
      "stroke-dasharray": b.dashed || b.group ? "4 5" : "none" }, g);
    const tx = b.x + 16, ty = b.y + (b.group ? 28 : 30);
    svg("text", { x: tx, y: ty, "font-size": b.group ? 12 : 15, fill: b.c, "font-weight": 500 }, g).textContent = b.title;
    svg("text", { x: tx, y: ty + 18, "font-size": 11, fill: "#8b978f" }, g).textContent = b.sub;
    (b.lines ?? []).forEach((l, i) => { svg("text", { x: tx, y: ty + 44 + i * 18, "font-size": 11, fill: "#a8b3ab" }, g).textContent = "· " + l; });
    (b.files ?? []).forEach((l, i) => { svg("text", { x: tx, y: ty + 40 + i * 17, "font-size": 11, fill: "#a8b3ab" }, g).textContent = l; });
    if (b.info) {
      g.style.cursor = "pointer";
      g.addEventListener("click", (e) => { e.stopPropagation(); select(null); g.classList.add("on"); show(b.info); });
    }
  }

  const live = [];
  const chans = {};
  for (const ch of channels) {
    const g = svg("g", { class: "chan" }, s);
    chans[ch.id] = g;
    const wire = svg("path", { class: "wire", d: ch.d, stroke: ch.c, "stroke-opacity": 0.7, "stroke-dasharray": ch.dashed ? "3 5" : "none" }, g);
    svg("path", { class: "hit", d: ch.d }, g);
    if (ch.text) {
      const t = svg("text", { x: ch.label[0], y: ch.label[1], "font-size": 10.5, fill: ch.c }, g);
      t.textContent = ch.text;
    }
    g.addEventListener("click", (e) => { e.stopPropagation(); select(ch.id); });
    if (ch.rate) {
      const n = Math.min(6, Math.max(1, Math.round(Math.log10(ch.rate + 1) * 2)));
      const len = wire.getTotalLength();
      for (let i = 0; i < n; i++) {
        const dot = svg("circle", { r: 2.6, fill: ch.c, filter: "url(#glow)" }, g);
        live.push({ dot, wire, len, phase: i / n, speed: 0.12 + Math.log10(ch.rate + 1) * 0.12, back: ch.both && i % 2 === 1 });
      }
    }
  }

  function select(id) {
    Object.values(nodes).forEach((n) => n.classList.remove("on"));
    Object.entries(chans).forEach(([k, g]) => g.classList.toggle("on", k === id));
    root.classList.toggle("has-on", !!id);
    if (id) show(channels.find((c) => c.id === id).info);
  }
  s.addEventListener("click", () => select(null));
  select("fdm");

  let visible = false;
  new IntersectionObserver(([e]) => { visible = e.isIntersecting; }).observe(root);
  const t0 = performance.now();
  const tick = () => {
    requestAnimationFrame(tick);
    if (!visible) return;
    const t = (performance.now() - t0) / 1000;
    for (const p of live) {
      let u = (p.phase + t * p.speed) % 1;
      if (p.back) u = 1 - u;
      const pt = p.wire.getPointAtLength(u * p.len);
      p.dot.setAttribute("cx", pt.x);
      p.dot.setAttribute("cy", pt.y);
    }
  };
  tick();
}

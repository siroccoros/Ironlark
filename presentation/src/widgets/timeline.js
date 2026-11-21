// The whole flight on the autopilot's clock: height (EKF and ground truth), EKF status flags, GPS, mode, armed, and the
// autopilot's status texts. Everything from runs/<forest>/telemetry.jsonl and ground_truth.jsonl.
import flight from "../generated/flight-data.js";
import { h, svg, lerpSeries, stepSeries } from "../lib/data.js";

export const FLAGS = [[1, "attitude"], [2, "horiz. velocity"], [4, "vert. velocity"], [8, "horiz. position (rel)"],
  [16, "horiz. position (abs)"], [32, "vert. position (abs)"], [64, "vert. position (AGL)"], [128, "constant-position mode"],
  [256, "pred. horiz. pos (rel)"], [512, "pred. horiz. pos (abs)"], [1024, "uninitialised"]];
export const MODES = { 0: "STABILIZE", 4: "GUIDED", 9: "LAND" };
const decode = (f) => FLAGS.filter(([b]) => f & b).map(([, n]) => n);

export default function timeline(root) {
  const f = flight.forest, t = f.telemetry;
  root.innerHTML = `<div class="w-title"><p>The run's telemetry · move across it</p><p class="w-meta">autopilot time (time_boot_ms) · 10 Hz</p></div>`;
  const chart = h(`<div class="chart"></div>`);
  root.append(chart);
  const W = 1180, X0 = 150, X1 = 1160, T = 62;
  const x = (s) => X0 + (s / T) * (X1 - X0);
  const lanes = { alt: [62, 170], ekf: [256, 30], gps: [296, 22], mode: [328, 22], armed: [360, 22], text: [392, 26] };
  const H = 440;
  const s = svg("svg", { viewBox: `0 0 ${W} ${H}` }, chart);
  const grid = svg("g", { class: "grid" }, s);
  for (let k = 0; k <= T; k += 5) {
    svg("line", { x1: x(k), x2: x(k), y1: 56, y2: 422 }, grid);
    svg("text", { x: x(k), y: 436, "text-anchor": "middle" }, grid).textContent = `${k} s`;
  }
  const [ay, ah] = lanes.alt;
  const y = (m) => ay + ah - ((m + 0.2) / 3.8) * ah;
  for (const m of [0, 1, 2, 3]) {
    svg("line", { x1: X0, x2: X1, y1: y(m), y2: y(m), stroke: "rgba(214,232,220,0.06)" }, s);
    svg("text", { x: X0 - 10, y: y(m) + 4, "text-anchor": "end" }, s).textContent = `${m} m`;
  }
  svg("rect", { x: X0, width: X1 - X0, y: y(3.5), height: y(2.5) - y(3.5), fill: "rgba(255,122,47,0.06)" }, s);
  svg("text", { x: 0, y: ay + 18, fill: "#e7ece5" }, s).textContent = "height";
  svg("text", { x: 0, y: ay + 34 }, s).textContent = "EKF  ·  truth";
  const line = (pts, color, width = 1.6, dash) => svg("polyline", { points: pts.map(([a, b]) => `${x(a).toFixed(1)},${y(b).toFixed(1)}`).join(" "),
    fill: "none", stroke: color, "stroke-width": width, "stroke-dasharray": dash ?? "none", "stroke-linejoin": "round" }, s);
  line(f.truth.map((r) => [r[0], r[3]]), "#a4d483", 2.2);
  line(t.alt.map((r) => [r[0], r[1]]), "#7cc6ff", 1.4, "4 3");

  const band = (lane, label, segs) => {
    const [ly, lh] = lanes[lane];
    svg("text", { x: 0, y: ly + lh / 2 + 4 }, s).textContent = label;
    for (const [a, b, color, text, op = 0.22] of segs) {
      svg("rect", { x: x(a), y: ly, width: Math.max(1, x(b) - x(a)), height: lh, rx: 4, fill: color, "fill-opacity": op, stroke: color, "stroke-opacity": 0.5 }, s);
      if (text && x(b) - x(a) > 40) svg("text", { x: x(a) + 8, y: ly + lh / 2 + 4, fill: color }, s).textContent = text;
    }
  };
  const segments = (rows, end, fn) => rows.map((r, i) => [r[0], rows[i + 1]?.[0] ?? end, ...fn(r)]);
  band("ekf", "EKF3 flags", segments(t.ekf, T, (r) => {
    const v = r[1];
    if (v & 1024) return ["#6c7a72", "1024 uninitialised"];
    if (v & 128) return ["#f2bd52", `${v} · constant-position mode`];
    if (v & 16) return ["#a4d483", `${v} · absolute position`];
    return ["#6c7a72", `${v}`];
  }));
  band("gps", "GPS fix", segments(t.gps, T, (r) => r[1] >= 3 ? ["#a4d483", `fix_type ${r[1]} · ${r[2]} satellites`] : ["#6c7a72", `fix_type ${r[1]}`]));
  band("mode", "mode", segments(t.mode, T, (r) => [r[1] === 9 ? "#ff6f7f" : r[1] === 4 ? "#7cc6ff" : "#6c7a72", `${MODES[r[1]] ?? r[1]}`]));
  band("armed", "armed", segments(t.armed, T, (r) => r[1] ? ["#ff7a2f", "armed", 0.35] : ["#6c7a72", "", 0.06]));
  const [ty] = lanes.text;
  svg("text", { x: 0, y: ty + 17 }, s).textContent = "status texts";
  const texts = t.text.filter((r) => r[0] > 0);
  for (const r of texts) svg("line", { x1: x(r[0]), x2: x(r[0]), y1: ty + 2, y2: ty + 24, stroke: "#e7ece5", "stroke-opacity": 0.55 }, s);

  // Key moments the mission waits on.
  const marks = [[9.54, "GPS fix", 2], [19.64, "EKF origin set", 2], [41.14, "EKF absolute · GUIDED · arm", 0], [42.04, "armed · takeoff", 1], [47.04, "2.7 m: HOVERING", 2], [52.14, "LAND", 0], [60.34, "disarm", 1]];
  marks.forEach(([at, label, row]) => {
    const ly = 14 + row * 14;
    svg("line", { x1: x(at), x2: x(at), y1: ly + 3, y2: 232, stroke: "#ff7a2f", "stroke-opacity": 0.35, "stroke-dasharray": "2 3" }, s);
    svg("text", { x: x(at) + (row === 0 && at > 40 && at < 45 ? -4 : 4), y: ly, fill: "#ff9a5c", "font-size": 10, "text-anchor": row === 0 && at > 40 && at < 45 ? "end" : "start" }, s).textContent = label;
  });

  const cursor = svg("g", { class: "cursor" }, s);
  const cl = svg("line", { y1: 56, y2: 422 }, cursor);
  const dotE = svg("circle", { r: 4, fill: "#7cc6ff" }, cursor);
  const dotT = svg("circle", { r: 4, fill: "#a4d483" }, cursor);
  const readout = h(`<div class="chart-read"></div>`);
  root.append(readout);
  const at = (sec) => {
    cl.setAttribute("x1", x(sec)); cl.setAttribute("x2", x(sec));
    const ea = lerpSeries(t.alt, sec, 1), tz = lerpSeries(f.truth, sec, 3);
    dotE.setAttribute("cx", x(sec)); dotE.setAttribute("cy", y(ea));
    dotT.setAttribute("cx", x(sec)); dotT.setAttribute("cy", y(tz));
    const flags = stepSeries(t.ekf, sec) ?? 0;
    const gps = t.gps.filter((r) => r[0] <= sec).at(-1) ?? [0, 0, 0];
    const mode = stepSeries(t.mode, sec);
    const armed = stepSeries(t.armed, sec);
    const last = texts.filter((r) => r[0] <= sec).at(-1);
    readout.innerHTML = `<span><b>${sec.toFixed(2)} s</b></span>
      <span><i style="--c:#7cc6ff"></i>EKF ${ea.toFixed(2)} m</span><span><i style="--c:#a4d483"></i>truth ${tz.toFixed(3)} m</span>
      <span>flags <b>${flags}</b> ${decode(flags).join(", ") || "none"}</span>
      <span>GPS fix ${gps[1]}</span><span>${MODES[mode] ?? mode}${armed ? " · armed" : ""}</span>
      ${last ? `<span class="dim">last text @${last[0].toFixed(1)} s: “${last[1]}”</span>` : ""}`;
  };
  s.addEventListener("pointermove", (e) => {
    const r = s.getBoundingClientRect();
    const px = ((e.clientX - r.left) / r.width) * W;
    at(Math.max(0, Math.min(T, ((px - X0) / (X1 - X0)) * T)));
  });
  at(41.2);
}

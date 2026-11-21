// judge() on this run's ground truth: the height band and the longest stay in it, the drift seen from above, and the
// five verdicts as result.json recorded them.
import flight from "../generated/flight-data.js";
import { h, svg } from "../lib/data.js";

export default function judge(root) {
  const f = flight.forest, truth = f.truth, m = f.result.measurements;
  // judge(), tests/flight_smoke.py:30-41, on the same samples.
  let hover = 0, entered = null, hoverSpan = null;
  for (const [at, , , z] of truth) {
    if (!(z >= 2.5 && z <= 3.5)) { entered = null; continue; }
    entered ??= at;
    if (at - entered > hover) { hover = at - entered; hoverSpan = [entered, at]; }
  }
  const [x0, y0] = [truth[0][1], truth[0][2]];
  root.innerHTML = `<div class="w-title"><p>judge() on this run · the ground-truth file, 619 samples</p><p class="w-meta">computed again here from ground_truth.jsonl with the same rules</p></div>`;
  const wrap = h(`<div style="display:grid;grid-template-columns:1.5fr 1fr;gap:28px"></div>`);
  root.append(wrap);

  const left = h(`<div class="chart"></div>`);
  wrap.append(left);
  const W = 700, H = 320, X0 = 46, X1 = 690, T0 = 40, T1 = 62;
  const x = (s) => X0 + ((s - T0) / (T1 - T0)) * (X1 - X0);
  const y = (z) => 16 + (1 - (z + 0.2) / 4.2) * (H - 50);
  const s = svg("svg", { viewBox: `0 0 ${W} ${H}` }, left);
  svg("rect", { x: X0, width: X1 - X0, y: y(3.5), height: y(2.5) - y(3.5), fill: "rgba(255,122,47,0.08)", stroke: "rgba(255,122,47,0.25)" }, s);
  svg("text", { x: X1 - 6, y: y(3.5) + 14, "text-anchor": "end", fill: "#ff9a5c" }, s).textContent = "hover band 2.5–3.5 m";
  svg("rect", { x: X0, width: X1 - X0, y: y(4), height: y(2.5) - y(4), fill: "none", stroke: "rgba(124,198,255,0.35)", "stroke-dasharray": "3 4" }, s);
  svg("text", { x: X0 + 6, y: y(4) + 14, fill: "#7cc6ff" }, s).textContent = "peak must be 2.5–4 m";
  svg("rect", { x: X0, width: X1 - X0, y: y(0.2), height: y(-0.2) - y(0.2), fill: "rgba(164,212,131,0.08)" }, s);
  for (const z of [0, 1, 2, 3, 4]) svg("text", { x: X0 - 8, y: y(z) + 4, "text-anchor": "end" }, s).textContent = `${z}`;
  for (let k = 40; k <= 62; k += 4) svg("text", { x: x(k), y: H - 12, "text-anchor": "middle" }, s).textContent = `${k} s`;
  if (hoverSpan) {
    svg("rect", { x: x(hoverSpan[0]), width: x(hoverSpan[1]) - x(hoverSpan[0]), y: y(3.5), height: y(2.5) - y(3.5), fill: "rgba(255,122,47,0.22)" }, s);
    svg("text", { x: (x(hoverSpan[0]) + x(hoverSpan[1])) / 2, y: y(2.5) + 16, "text-anchor": "middle", fill: "#ff7a2f" }, s).textContent = `longest stay ${hover.toFixed(1)} s`;
  }
  const pts = truth.filter((r) => r[0] >= T0).map((r) => `${x(r[0]).toFixed(1)},${y(r[3]).toFixed(1)}`).join(" ");
  svg("polyline", { points: pts, fill: "none", stroke: "#a4d483", "stroke-width": 2.2 }, s);
  const peak = truth.reduce((a, r) => (r[3] > a[3] ? r : a));
  svg("circle", { cx: x(peak[0]), cy: y(peak[3]), r: 4.5, fill: "#7cc6ff" }, s);
  svg("text", { x: x(peak[0]) + 8, y: y(peak[3]) - 8, fill: "#7cc6ff" }, s).textContent = `peak ${peak[3].toFixed(3)} m`;
  const last = truth.at(-1);
  svg("circle", { cx: x(last[0]), cy: y(last[3]), r: 4.5, fill: "#a4d483" }, s);
  svg("text", { x: x(last[0]) - 6, y: y(last[3]) - 10, "text-anchor": "end", fill: "#a4d483" }, s).textContent = `final ${last[3].toFixed(3)} m`;

  const right = h(`<div class="chart"></div>`);
  wrap.append(right);
  const S = 320, R = 0.1, c = S / 2, k = (S / 2 - 18) / R;
  const top = svg("svg", { viewBox: `0 0 ${S} ${S}` }, right);
  for (const r of [0.025, 0.05, 0.075, 0.1]) svg("circle", { cx: c, cy: c, r: r * k, fill: "none", stroke: "rgba(214,232,220,0.07)" }, top);
  svg("text", { x: c + 0.1 * k * 0.71 + 4, y: c - 0.1 * k * 0.71 - 4 }, top).textContent = "10 cm";
  svg("circle", { cx: c, cy: c, r: m.max_horizontal_drift_m * k, fill: "rgba(255,122,47,0.06)", stroke: "#ff7a2f", "stroke-dasharray": "3 3" }, top);
  const tp = truth.map((r) => `${(c + (r[1] - x0) * k).toFixed(1)},${(c - (r[2] - y0) * k).toFixed(1)}`).join(" ");
  svg("polyline", { points: tp, fill: "none", stroke: "#a4d483", "stroke-width": 1.3, "stroke-opacity": 0.9 }, top);
  svg("circle", { cx: c, cy: c, r: 3, fill: "#f2bd52" }, top);
  svg("text", { x: 8, y: 16, fill: "#e7ece5" }, top).textContent = "seen from above, ENU";
  svg("text", { x: 8, y: S - 8 }, top).textContent = `the 1.5 m limit is ${Math.round(1.5 / m.max_horizontal_drift_m)}× the dashed circle`;

  const rows = [
    ["peak height", `${m.peak_height_m.toFixed(3)} m`, "2.5 – 4 m"],
    ["longest hover in band", `${m.hover_s.toFixed(1)} s`, "≥ 5 s"],
    ["final height", `${m.final_height_m.toFixed(3)} m`, "−0.2 – 0.2 m"],
    ["max horizontal drift", `${(m.max_horizontal_drift_m * 100).toFixed(1)} cm`, "≤ 1.5 m"],
    ["autopilot vs Webots clock", `${m.clock_difference_s.toFixed(3)} s`, "≤ 1 s"],
  ];
  const v = h(`<div class="metrics" style="grid-template-columns:repeat(5,1fr);margin-top:22px">${rows.map(([k2, val, lim]) =>
    `<div class="metric pass"><div class="k">${k2}</div><div class="v">${val}</div><div class="s">limit ${lim}</div></div>`).join("")}</div>`);
  root.append(v);
  const mq = matchMedia("(max-width: 900px)");
  const lay = () => { wrap.style.gridTemplateColumns = mq.matches ? "1fr" : "1.5fr 1fr"; v.style.gridTemplateColumns = mq.matches ? "1fr 1fr" : "repeat(5,1fr)"; };
  mq.addEventListener("change", lay);
  lay();
}

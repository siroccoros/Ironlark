// 100 ms of the sensor schedule: which messages each 2 ms step emits (sensor_stream._sample), and what a second costs.
import flight from "../generated/flight-data.js";
import { h, svg } from "../lib/data.js";

const ROWS = [
  ["CLOCK", "/clock", 2, "#f2bd52", 0],
  ["IMU", "/imu/data", 4, "#b7a3ff", 80],
  ["TRUTH", "/ground_truth/*", 4, "#a4d483", 80],
  ["IMAGE", "/camera/image_raw", 50, "#ff7a2f", 4 + 640 * 480 * 4],
  ["LIDAR", "/lidar/points", 100, "#7cc6ff", 4 + 28800 * 20],
];

export default function schedule(root) {
  const sense = flight.forest.result.sensing;
  root.innerHTML = `<div class="w-title"><p>What each 2 ms step sends · the first 100 ms of every 100 ms</p><p class="w-meta">sensor_stream.py:147–174</p></div>`;
  const chart = h(`<div class="chart"></div>`);
  root.append(chart);
  const W = 1180, X0 = 170, STEP = (W - X0 - 10) / 50, RH = 30;
  const s = svg("svg", { viewBox: `0 0 ${W} ${ROWS.length * RH + 40}` }, chart);
  const cells = [];
  ROWS.forEach(([name, topic, period, color], r) => {
    const y = r * RH + 6;
    svg("text", { x: 0, y: y + 15, fill: color }, s).textContent = name;
    svg("text", { x: 58, y: y + 15 }, s).textContent = topic;
    for (let k = 0; k < 50; k++) {
      const due = (k * 2) % period === 0;
      const c = svg("rect", { x: X0 + k * STEP + 1, y, width: STEP - 2, height: RH - 8, rx: 3,
        fill: due ? color : "rgba(214,232,220,0.04)", "fill-opacity": due ? 0.55 : 1 }, s);
      if (due) cells.push({ c, k });
    }
  });
  for (let k = 0; k <= 50; k += 5) svg("text", { x: X0 + k * STEP, y: ROWS.length * RH + 26, "text-anchor": "middle" }, s).textContent = `${k * 2} ms`;
  const head = svg("rect", { x: X0, y: 0, width: STEP, height: ROWS.length * RH + 4, fill: "rgba(255,255,255,0.08)", rx: 3 }, s);

  const perSecond = ROWS.map(([name, , period, color, payload]) => {
    const n = 1000 / period;
    return { name, color, n, bytes: n * (20 + payload) };
  });
  const total = perSecond.reduce((a, r) => a + r.n, 0), bytes = perSecond.reduce((a, r) => a + r.bytes, 0);
  const fmtB = (b) => (b > 1e6 ? `${(b / 1e6).toFixed(1)} MB` : `${(b / 1e3).toFixed(0)} kB`);
  root.append(h(`<div class="metrics" style="grid-template-columns:repeat(6,1fr);margin-top:20px">
    ${perSecond.map((r) => `<div class="metric"><div class="k" style="color:${r.color}">${r.name}</div><div class="v">${r.n}/s</div><div class="s">${fmtB(r.bytes)}/s on the wire</div></div>`).join("")}
    <div class="metric"><div class="k">all</div><div class="v">${total}/s</div><div class="s">${fmtB(bytes)}/s · this run: ${sense.sent.toLocaleString("en")} sent, ${sense.dropped} dropped</div></div></div>`));

  let visible = false;
  new IntersectionObserver(([e]) => { visible = e.isIntersecting; }).observe(root);
  const t0 = performance.now();
  const tick = () => {
    requestAnimationFrame(tick);
    if (!visible) return;
    const k = Math.floor(((performance.now() - t0) / 60) % 50);
    head.setAttribute("x", X0 + k * STEP);
    for (const c of cells) c.c.setAttribute("fill-opacity", c.k === k ? 1 : 0.42);
  };
  tick();
}

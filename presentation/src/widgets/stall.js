// What a slow frame does to each loop: SITL resends its last motor packet every 100 ms while Webots renders; afterwards
// upstream steps physics once per queued packet, Ironlark drains the queue and steps once. A model of the two rules.
import { h, svg } from "../lib/data.js";

export default function stall(root) {
  root.innerHTML = `<div class="w-title"><p>One slow frame, two loops</p><p class="w-meta">a model of the two rules, not a replay</p></div>`;
  const chart = h(`<div class="chart"></div>`);
  root.append(chart);
  const W = 1100, H = 150, X0 = 120, X1 = 1080, T = 1000;
  const s = svg("svg", { viewBox: `0 0 ${W} ${H}` }, chart);
  const x = (ms) => X0 + (ms / T) * (X1 - X0);
  svg("text", { x: 0, y: 46 }, s).textContent = "Webots";
  svg("text", { x: 0, y: 106 }, s).textContent = "ArduPilot SITL";
  const axis = svg("g", { class: "axis" }, s);
  for (let ms = 0; ms <= T; ms += 100) {
    svg("line", { x1: x(ms), x2: x(ms), y1: 20, y2: 126 }, axis);
    svg("text", { x: x(ms), y: 144, "text-anchor": "middle" }, axis).textContent = `${ms}`;
  }
  const render = svg("rect", { x: x(0), y: 30, height: 24, rx: 5, fill: "rgba(164,212,131,0.22)", stroke: "#a4d483" }, s);
  const renderLabel = svg("text", { x: x(0) + 10, y: 46, fill: "#a4d483" }, s);
  const sends = svg("g", {}, s);
  const head = svg("line", { x1: x(0), x2: x(0), y1: 18, y2: 128, stroke: "#ff7a2f", "stroke-width": 1.5 }, s);

  const controls = h(`<div class="controls"><span>frame takes</span><input type="range" min="0" max="950" step="10" value="340"><b class="mono" style="min-width:64px">340 ms</b><button class="btn">play it</button></div>`);
  root.append(controls);
  const slider = controls.querySelector("input");
  const grid = h(`<div class="stall-grid" style="margin-top:22px">
    <div class="stall-lane up"><h6>Upstream</h6><p class="rule">"read one control packet, step once", for every packet in the socket</p>
      <div class="queue"></div><div class="steps"></div><div class="stall-out"></div></div>
    <div class="stall-lane il"><h6>Ironlark</h6><p class="rule">"drain the socket, keep the newest, step once" (lockstep.c:71–79)</p>
      <div class="queue"></div><div class="steps"></div><div class="stall-out"></div></div></div>`);
  root.append(grid);
  const up = grid.querySelector(".up"), il = grid.querySelector(".il");

  let S = 340, playing = null;
  function draw(progress = 1) {
    const now = progress * (S + 120);
    const k = Math.floor(S / 100);
    render.setAttribute("width", Math.max(2, x(S) - x(0)));
    renderLabel.textContent = S >= 120 ? `rendering a frame · ${S} ms` : "";
    sends.innerHTML = "";
    const shown = [];
    for (let j = 0; j <= k; j++) {
      const at = j * 100;
      if (at > now) break;
      const stale = j > 0;
      svg("circle", { cx: x(at), cy: 98, r: 6, fill: stale ? "#ff6f7f" : "#7cc6ff" }, sends);
      svg("text", { x: x(at), y: 84, "text-anchor": "middle", fill: stale ? "#ff6f7f" : "#7cc6ff" }, sends).textContent = stale ? "resend" : "control";
      shown.push(stale);
    }
    head.setAttribute("x1", x(Math.min(now, T)));
    head.setAttribute("x2", x(Math.min(now, T)));
    const done = now >= S;
    // The control already applied before the frame is not in the queue: what is queued is the resends.
    const queued = shown.slice(1);
    for (const [lane, drain] of [[up, false], [il, true]]) {
      const q = lane.querySelector(".queue"), st = lane.querySelector(".steps"), out = lane.querySelector(".stall-out");
      q.innerHTML = queued.length ? queued.map((_, i) => `<span class="pk stale ${drain && done && i < queued.length - 1 ? "gone" : ""}">r${i + 1}</span>`).join("") : `<span class="dim mono" style="font-size:11px">socket empty</span>`;
      st.innerHTML = "";
      if (!done) { out.innerHTML = `<span class="dim">Webots is rendering; nothing steps.</span>`; continue; }
      const n = drain ? 1 : Math.max(1, queued.length);
      for (let i = 0; i < n; i++) st.append(h(`<i class="${queued.length && (!drain || i === 0) ? "stale" : ""}"></i>`));
      out.innerHTML = drain
        ? `<b>1</b> physics step, 2 ms · ${queued.length ? `${queued.length - 1} older resend${queued.length - 1 === 1 ? "" : "s"} discarded` : "nothing to discard"}`
        : `<b>${n}</b> physics step${n === 1 ? "" : "s"}, ${n * 2} ms, all on the same motor command`;
    }
  }
  slider.addEventListener("input", () => {
    S = Number(slider.value);
    controls.querySelector("b").textContent = `${S} ms`;
    cancelAnimationFrame(playing);
    draw(1);
  });
  controls.querySelector("button").addEventListener("click", () => {
    cancelAnimationFrame(playing);
    const t0 = performance.now(), dur = 900 + S * 2.4;
    const step = () => {
      const p = Math.min(1, (performance.now() - t0) / dur);
      draw(p);
      if (p < 1) playing = requestAnimationFrame(step);
    };
    step();
  });
  draw(1);
}

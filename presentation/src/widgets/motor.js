// PWM → (pwm − 1000)/1000 → clamp → √ → ω → thrust, and the run's own servo outputs against the PWM that the drone's
// mass predicts for hover (4 · 0.0012 · 100² · u = 1.5 · 9.81).
import flight from "../generated/flight-data.js";
import { h, svg } from "../lib/data.js";

const K = 0.0012, WMAX = 100, MASS = 1.5, G = 9.81;
const HOVER_U = (MASS * G) / (4 * K * WMAX * WMAX);

export default function motor(root) {
  root.innerHTML = `<div class="w-title"><p>One motor command, end to end</p><p class="w-meta">k = 0.0012 · ω<sub>max</sub> = 100 rad/s · mass 1.5 kg (protos/IronlarkDrone.proto)</p></div>`;
  const wrap = h(`<div class="split" style="margin:0;gap:34px"></div>`);
  root.append(wrap);
  const left = h(`<div></div>`);
  wrap.append(left);
  const stages = h(`<div class="motor-stages" style="display:grid;gap:8px"></div>`);
  left.append(stages);
  const rows = [
    ["PWM from ArduPilot", "pwm", "µs"],
    ["SITL sends (pwm − 1000) / 1000", "u", "SIM_Webots_Python.cpp:71"],
    ["IrisBridge clamps negatives to 0", "c", "flight_bridge.py:37"],
    ["linearize: √u", "r", "lockstep.c:85"],
    ["rotor speed ω = √u · 100", "w", "rad/s · lockstep.c:86"],
    ["thrust per motor k·ω²", "t", "N"],
    ["four motors vs weight 14.72 N", "net", ""],
  ];
  const cells = {};
  for (const [label, key, unit] of rows) {
    const row = h(`<div style="display:grid;grid-template-columns:1fr auto;align-items:baseline;gap:12px;padding:9px 14px;border:1px solid var(--line);border-radius:10px;background:rgba(0,0,0,.18)">
      <span style="font-size:13.5px;color:var(--text-2)">${label}<br><span class="mono dim" style="font-size:10.5px">${unit}</span></span>
      <span class="mono" style="font-size:20px;color:var(--text)" data-k="${key}"></span></div>`);
    cells[key] = row.querySelector("[data-k]");
    stages.append(row);
  }
  const controls = h(`<div class="controls"><span>PWM</span><input type="range" min="1000" max="1600" value="1306"><span class="seg"><button data-p="1000">1000</button><button data-p="1306" class="on">hover</button><button data-p="1450">1450</button><button data-p="0">disabled (0)</button></span></div>`);
  left.append(controls);
  const slider = controls.querySelector("input");

  const set = (pwm) => {
    const u = (pwm - 1000) / 1000, c = Math.max(0, u), r = Math.sqrt(c), w = r * WMAX, t = K * w * w, total = 4 * t;
    cells.pwm.textContent = pwm === 0 ? "0 (no output)" : `${pwm}`;
    cells.u.textContent = u.toFixed(3);
    cells.u.style.color = u < 0 ? "var(--rose)" : "";
    cells.c.textContent = c.toFixed(3);
    cells.r.textContent = r.toFixed(3);
    cells.w.textContent = w.toFixed(1);
    cells.t.textContent = t.toFixed(2);
    const net = total - MASS * G;
    cells.net.innerHTML = `${total.toFixed(2)} N · <span style="color:${Math.abs(net) < 0.3 ? "var(--moss)" : net > 0 ? "var(--orange)" : "var(--sky)"}">${Math.abs(net) < 0.3 ? "holds" : net > 0 ? "climbs" : "sinks"}</span>`;
    if (pwm === 0) cells.c.innerHTML = `0.000 <span class="dim" style="font-size:11px">(upstream: −1 → √1 reversed, full reverse thrust)</span>`;
  };
  slider.addEventListener("input", () => { set(Number(slider.value)); controls.querySelectorAll(".seg button").forEach((b) => b.classList.remove("on")); });
  controls.querySelectorAll(".seg button").forEach((b) => b.addEventListener("click", () => {
    controls.querySelectorAll(".seg button").forEach((o) => o.classList.toggle("on", o === b));
    const p = Number(b.dataset.p);
    if (p) slider.value = p;
    set(p);
  }));
  set(1306);

  // The run's servo outputs.
  const right = h(`<div class="chart"></div>`);
  wrap.append(right);
  const servoAll = flight.forest.telemetry.servo;
  const W = 560, H = 380, X0 = 54, X1 = 548, T0 = 40, T1 = 62, P0 = 980;
  const P1 = Math.ceil((Math.max(...servoAll.filter((r) => r[0] >= T0).flatMap((r) => r.slice(1, 5))) + 30) / 100) * 100;
  const x = (s) => X0 + ((s - T0) / (T1 - T0)) * (X1 - X0);
  const y = (p) => 20 + (1 - (p - P0) / (P1 - P0)) * (H - 60);
  const s = svg("svg", { viewBox: `0 0 ${W} ${H}` }, right);
  for (let p = 1000; p <= P1; p += 100) {
    svg("line", { x1: X0, x2: X1, y1: y(p), y2: y(p), stroke: "rgba(214,232,220,0.06)" }, s);
    svg("text", { x: X0 - 8, y: y(p) + 4, "text-anchor": "end" }, s).textContent = p;
  }
  for (let k = 40; k <= 62; k += 4) svg("text", { x: x(k), y: H - 18, "text-anchor": "middle" }, s).textContent = `${k} s`;
  const hp = 1000 + HOVER_U * 1000;
  svg("line", { x1: X0, x2: X1, y1: y(hp), y2: y(hp), stroke: "#a4d483", "stroke-dasharray": "5 4" }, s);
  svg("text", { x: X1, y: y(hp) - 8, "text-anchor": "end", fill: "#a4d483" }, s).textContent = `weight ÷ (4·k·ω²max) → ${hp.toFixed(1)} µs`;
  const colors = ["#ff7a2f", "#ffb07a", "#7cc6ff", "#b7a3ff"];
  const servo = flight.forest.telemetry.servo.filter((r) => r[0] >= T0 && r[0] <= T1);
  for (let m = 0; m < 4; m++) {
    svg("polyline", { points: servo.map((r) => `${x(r[0]).toFixed(1)},${y(r[m + 1]).toFixed(1)}`).join(" "), fill: "none",
      stroke: colors[m], "stroke-width": 1.4, "stroke-opacity": 0.85 }, s);
  }
  svg("text", { x: X0, y: 12, fill: "#e7ece5" }, s).textContent = "SERVO1–4 in this run (µs)";
  const legend = h(`<div class="chart-read">${colors.map((c, i) => `<span><i style="--c:${c}"></i>motor ${i + 1}</span>`).join("")}<span class="dim">1000 µs is u = 0: no thrust</span></div>`);
  right.append(legend);
}

// The drone's front camera through the flight, scrubbed by scroll, with the run's telemetry and ground truth at each frame.
import frames from "../generated/camera.js";
import flight from "../generated/flight-data.js";
import { h, lerpSeries, stepSeries } from "../lib/data.js";
import { MODES } from "./timeline.js";

const T0 = 40.0, T1 = 61.4;
const EVENTS = [[41.14, "ARMING"], [42.04, "TAKING_OFF"], [47.04, "HOVERING"], [52.14, "LANDING"], [61.04, "EVALUATING"]];

export default function ride(root, { lenis }) {
  const f = flight.forest, t = f.telemetry;
  const list = frames.filter(([s]) => s >= T0 - 0.2 && s <= T1 + 0.2);
  const images = list.map(([s, name]) => ({ s, img: null, src: `./img/cam/${name}` }));
  // Load nearest-first from the start, a few at a time.
  let next = 0;
  const pump = () => {
    for (let k = 0; k < 6 && next < images.length; k++, next++) {
      const it = images[next];
      const img = new Image();
      img.decoding = "async";
      img.onload = () => { it.img = img; if (Math.abs(it.s - cur) < 0.2) draw(); };
      img.src = it.src;
    }
    if (next < images.length) setTimeout(pump, 40);
  };
  pump();

  const canvas = root.querySelector(".ride-frame");
  canvas.width = 1280; canvas.height = 960;
  const g = canvas.getContext("2d");
  g.imageSmoothingQuality = "high";
  const hud = root.querySelector(".ride-hud");
  const box = h(`<div class="frame-box">
    <div class="hud-tl"><div class="hud-k">mission state</div><div class="hud-state" data-h="state"></div></div>
    <div class="hud-tr"><div class="hud-k">autopilot time</div><div class="hud-big" data-h="time"></div><div data-h="mode"></div></div>
    <div class="hud-bl"><div class="hud-k">true height</div><div class="hud-big" data-h="truth"></div><div data-h="ekf"></div></div>
    <div class="hud-br"><div class="hud-k">motors</div><div class="hud-big" data-h="pwm"></div><div data-h="u"></div></div>
    <div class="hud-alt"><i data-h="bar"></i><span style="top:8%">3 m</span><span style="top:92%">0 m</span></div>
    <div class="hud-scrub"><i data-h="scrub"></i>${EVENTS.filter(([, n]) => n !== "ARMING").map(([s, n]) => `<b style="left:${((s - T0) / (T1 - T0)) * 100}%">${n.replace("_", " ").toLowerCase()}</b>`).join("")}</div>
  </div>`);
  hud.append(box);
  const q = (k) => box.querySelector(`[data-h="${k}"]`);

  let cur = T0;
  function draw() {
    let best = null, bd = Infinity;
    for (const it of images) {
      if (!it.img) continue;
      const d = Math.abs(it.s - cur);
      if (d < bd) { bd = d; best = it; }
    }
    if (best) g.drawImage(best.img, 0, 0, canvas.width, canvas.height);
    const state = [...EVENTS].reverse().find(([s]) => cur >= s)?.[1] ?? "WAITING_FOR_POSITION";
    q("state").textContent = state.replaceAll("_", " ").toLowerCase();
    q("time").textContent = `${cur.toFixed(1)} s`;
    const mode = stepSeries(t.mode, cur), armed = stepSeries(t.armed, cur);
    q("mode").textContent = `${MODES[mode] ?? mode}${armed ? " · armed" : " · disarmed"}`;
    const tz = lerpSeries(f.truth, cur, 3), ez = lerpSeries(t.alt, cur, 1), vz = lerpSeries(f.truth, cur, 6);
    q("truth").textContent = `${tz.toFixed(2)} m`;
    q("ekf").textContent = `EKF ${ez.toFixed(2)} m · climb ${vz >= 0 ? "+" : "−"}${Math.abs(vz).toFixed(2)} m/s`;
    const s = t.servo.find((r) => r[0] >= cur) ?? t.servo.at(-1);
    const pwm = (s[1] + s[2] + s[3] + s[4]) / 4;
    q("pwm").textContent = `${pwm.toFixed(0)} µs`;
    q("u").textContent = `u = ${((pwm - 1000) / 1000).toFixed(3)} · ω ≈ ${(Math.sqrt(Math.max(0, (pwm - 1000) / 1000)) * 100).toFixed(0)} rad/s`;
    q("bar").style.top = `${92 - (Math.max(0, tz) / 3) * 84}%`;
    q("scrub").style.width = `${((cur - T0) / (T1 - T0)) * 100}%`;
  }

  const update = () => {
    const r = root.getBoundingClientRect();
    const p = Math.min(1, Math.max(0, -r.top / (r.height - innerHeight)));
    const next = T0 + p * (T1 - T0);
    if (Math.abs(next - cur) > 0.01) { cur = next; draw(); }
  };
  lenis.on("scroll", update);
  update();
  draw();
}

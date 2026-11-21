// The two UDP packets of the lockstep, cell by cell, with this flight's values where the run logged them.
import flight from "../generated/flight-data.js";
import { h, lerpSeries } from "../lib/data.js";

const AT = 48.0;

export default function packets(root) {
  const f = flight.forest;
  const truth = f.truth;
  const val = (col) => lerpSeries(truth, AT, col);
  const servo = f.telemetry.servo.find((r) => r[0] >= AT);
  const groups = [
    { name: "time", c: "#f2bd52", src: "wb_robot_get_time()", cells: [["t", AT.toFixed(3)]] },
    { name: "gyro", c: "#b7a3ff", src: "wb_gyro_get_values(gyro)", cells: [["gx"], ["gy", null, 1], ["gz", null, 1]] },
    { name: "accel", c: "#7cc6ff", src: "wb_accelerometer_get_values(accel)", cells: [["ax"], ["ay", null, 1], ["az", null, 1]] },
    { name: "attitude", c: "#a4d483", src: "wb_inertial_unit_get_roll_pitch_yaw(imu)", cells: [["roll"], ["pitch", null, 1], ["yaw", null, 1]] },
    { name: "velocity", c: "#ff9a5c", src: "wb_gps_get_speed_vector(gps)", cells: [["vx", val(4)], ["vy", -val(5), 1], ["vz", -val(6), 1]] },
    { name: "position", c: "#ff6f7f", src: "wb_gps_get_values(gps)", cells: [["x", val(1)], ["y", -val(2), 1], ["z", -val(3), 1]] },
  ];
  root.innerHTML = `<div class="w-title"><p>Packet anatomy</p><p class="w-meta">values: this flight at ${AT.toFixed(1)} s · position and velocity from ground truth, motors from SERVO_OUTPUT_RAW</p></div>`;

  const fdm = h(`<div class="packet"><div class="packet-label"><span><b>fdm_packet</b> · Webots → SITL · UDP 9003</span><span>16 × double = 128 bytes</span></div><div class="cells" style="grid-template-columns:repeat(16,minmax(0,1fr))"></div></div>`);
  const cells = fdm.querySelector(".cells");
  let index = 0;
  for (const g of groups) {
    g.cells.forEach(([name, value, neg], k) => {
      const i = index++;
      const v = value == null ? "—" : typeof value === "number" ? fmt(value) : value;
      const cell = h(`<div class="cell" style="--c:${g.c}"><div class="nm">${name}</div><div class="vl">${v}</div>${neg ? '<span class="neg">−</span>' : ""}</div>`);
      const expr = g.name === "time" ? "wb_robot_get_time()" : `${neg ? "−" : ""}${g.src}[${k}]`;
      cell.addEventListener("mouseenter", () => info(`<code>packet[${i}] = ${expr}</code> · bytes ${i * 8}–${i * 8 + 7}${neg ? " · negated: FLU/ENU y and z become FRD/NED" : ""}${value == null ? " · the Iris's own device; the run does not log it" : ""}`));
      cells.append(cell);
    });
  }
  root.append(fdm);

  const pwm = servo.slice(1, 5);
  const sv = h(`<div class="packet"><div class="packet-label"><span><b>servo_packet</b> · SITL → Webots · UDP 9002</span><span>16 × float = 64 bytes</span></div><div class="cells" style="grid-template-columns:repeat(16,minmax(0,1fr))"></div></div>`);
  const scells = sv.querySelector(".cells");
  for (let i = 0; i < 16; i++) {
    const raw = i < 4 ? pwm[i] : 0;
    const u = (raw - 1000) / 1000;
    const c = i < 4 ? "#7cc6ff" : "#6c7a72";
    const cell = h(`<div class="cell" style="--c:${c}"><div class="nm">${i < 4 ? `m${i + 1}` : `out${i + 1}`}</div><div class="vl">${u.toFixed(3)}</div></div>`);
    cell.addEventListener("mouseenter", () => info(i < 4
      ? `<code>motor_speed[${i}] = (${raw} − 1000) / 1000 = ${u.toFixed(3)}</code> · SERVO${i + 1} at ${AT.toFixed(1)} s · the bridge turns it into ω = √${u.toFixed(3)} × 100 = ${(Math.sqrt(u) * 100).toFixed(1)} rad/s`
      : `<code>motor_speed[${i}] = (0 − 1000) / 1000 = −1.000</code> · an output with no PWM. Upstream's bridge would read this as full reverse; IrisBridge clamps it to 0.`));
    scells.append(cell);
  }
  root.append(sv);
  const out = h(`<div class="packet-info">Hover a cell.</div>`);
  root.append(out);
  const info = (html) => { out.innerHTML = html; };
}

function fmt(v) {
  const a = Math.abs(v);
  return (v < 0 ? "−" : "") + (a < 1e-3 ? a.toExponential(1) : a.toFixed(a < 10 ? 3 : 2));
}

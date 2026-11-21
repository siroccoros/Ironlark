// The stream's framing (sensing/protocol.py): a 20-byte little-endian header, then the payload of each message type.
import { h } from "../lib/data.js";

const HEADER = [["magic", 4, "b\"IRLK\"", "#f2bd52", "4s: the bytes I R L K. A reader that finds anything else raises \"stream out of sync\"."],
  ["type", 1, "1–5", "#ff7a2f", "B: CLOCK 1, IMU 2, LIDAR 3, IMAGE 4, TRUTH 5."],
  ["pad", 3, "xxx", "#3a4540", "xxx: three padding bytes, so the time field starts on an 8-byte boundary."],
  ["time_ns", 8, "q", "#7cc6ff", "q: Webots time of the step in nanoseconds, signed 64-bit."],
  ["length", 4, "I", "#b7a3ff", "I: payload length in bytes, unsigned 32-bit."]];
const TYPES = {
  CLOCK: { n: 1, parts: [], note: "No payload: the header's time is the message." },
  IMU: { n: 2, parts: [["orientation x y z w", 32], ["angular velocity", 24], ["linear acceleration", 24]], note: "VECTOR10: ten little-endian doubles, 80 bytes." },
  TRUTH: { n: 5, parts: [["position", 24], ["orientation x y z w", 32], ["velocity", 24]], note: "Ten doubles, world ENU. Published only on /ground_truth/*." },
  LIDAR: { n: 3, parts: [["count", 4], ["28,800 × (x y z float32, layer int32, time float32)", 576000]], note: "Webots' LidarPoint struct, 20 bytes each, copied from C memory unchanged." },
  IMAGE: { n: 4, parts: [["w h", 4], ["640 × 480 × BGRA", 1228800]], note: "Rows from the top; published as bgra8 with step 4 × width." },
};

export default function protocol(root) {
  root.innerHTML = `<div class="w-title"><p>The wire · struct "&lt;4sBxxxqI" + payload</p><p class="w-meta">sensing/protocol.py:11–24</p></div>`;
  const seg = h(`<div class="seg" style="margin-bottom:18px">${Object.keys(TYPES).map((k) => `<button data-t="${k}">${k}</button>`).join("")}</div>`);
  root.append(seg);
  const bar = h(`<div style="display:flex;gap:4px;align-items:stretch;min-height:74px"></div>`);
  root.append(bar);
  const info = h(`<div class="packet-info"></div>`);
  root.append(info);
  const pick = (name) => {
    seg.querySelectorAll("button").forEach((b) => b.classList.toggle("on", b.dataset.t === name));
    const t = TYPES[name];
    bar.innerHTML = "";
    let offset = 0;
    for (const [nm, size, val, c, desc] of HEADER) {
      const v = nm === "type" ? `${t.n}` : nm === "length" ? `${t.parts.reduce((a, p) => a + p[1], 0)}` : val;
      const cell = h(`<div class="cell" style="--c:${c};flex:${size * 3} 1 0;min-width:0"><div class="nm">${nm}</div><div class="vl">${v}</div><div class="nm" style="color:var(--text-3);margin-top:4px">${offset}–${offset + size - 1}</div></div>`);
      cell.addEventListener("mouseenter", () => { info.innerHTML = desc; });
      bar.append(cell);
      offset += size;
    }
    for (const [nm, size] of t.parts) {
      const cell = h(`<div class="cell" style="--c:#e7ece5;flex:${Math.min(60, 8 + Math.log2(size) * 3.5)} 1 0;min-width:0"><div class="nm" style="white-space:normal">${nm}</div><div class="vl">${size.toLocaleString("en")} B</div></div>`);
      cell.addEventListener("mouseenter", () => { info.innerHTML = t.note; });
      bar.append(cell);
    }
    const total = 20 + t.parts.reduce((a, p) => a + p[1], 0);
    info.innerHTML = `${t.note} <span class="dim">· ${total.toLocaleString("en")} bytes per message</span>`;
  };
  seg.querySelectorAll("button").forEach((b) => b.addEventListener("click", () => pick(b.dataset.t)));
  pick("IMU");
}

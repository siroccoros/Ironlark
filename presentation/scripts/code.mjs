// Highlight the source excerpts the presentation reads, straight from the repository, into src/generated/code.js.
// Each excerpt is found by anchors (the first line containing `from`, then the first line containing `to` at or after
// it), so line numbers are the files' own and an edited file is quoted as it stands at build time.
import { readFileSync, writeFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";
import { createHighlighter } from "shiki";

const here = dirname(fileURLToPath(import.meta.url));
const repo = resolve(here, "../..");
const upstream = resolve(here, "upstream");

const excerpts = [
  { id: "world", file: "worlds/flight_foundation.wbt", from: "#VRML_SIM", to: "randomSeed" },
  { id: "world-robots", file: "worlds/flight_foundation.wbt", from: "DEF IRONLARK_DRONE", to: 'controller "flight_observer"', extra: 1 },
  { id: "launcher", file: "ironlark", from: "def flight(headless", to: "stop_process(webots)" },
  { id: "exclusive", file: "ironlark", from: "@contextlib.contextmanager", to: "yield" },
  { id: "build-lockstep", file: "ironlark", from: "def build_lockstep", to: "return library.read_bytes()" },
  { id: "flight-parm", file: "config/flight.parm", from: "# Use the actual", to: "LOG_DISARMED" },
  { id: "bridge", file: "controllers/flight_bridge/flight_bridge.py", from: "class IrisBridge", to: "time.sleep(0.1)" },
  { id: "lockstep", file: "controllers/flight_bridge/lockstep.c", from: "typedef void", to: "after_step();", extra: 2 },
  { id: "clock-patch", file: "docker/webots-clock.patch", from: "--- a/", to: "return;", extra: 1, lang: "diff" },
  { id: "up-sitl", file: "upstream:webots_vehicle.py", from: "# main loop handling communications", to: "if step_success == -1", extra: 1 },
  { id: "up-controls", file: "upstream:webots_vehicle.py", from: "# linearize propeller thrust", to: "m.setVelocity(" },
  { id: "ap-servos", file: "upstream:SIM_Webots_Python.cpp", from: "void WebotsPython::send_servos", to: "socket_sitl.sendto", extra: 1 },
  { id: "ap-fdm", file: "upstream:SIM_Webots_Python.cpp", from: "void WebotsPython::recv_fdm", to: "last_timestamp = pkt.timestamp;", extra: 2 },
  { id: "frames", file: "sensing/frames.py", from: "def enu_to_ned", to: "return quaternion_multiply(quaternion_multiply(ENU_TO_NED" },
  { id: "smoke-wait", file: "tests/flight_smoke.py", from: "    def poll(self):", to: "raise RuntimeError(f\"Timeout: {description}\")" },
  { id: "smoke-command", file: "tests/flight_smoke.py", from: "    def command(self, command", to: "lambda m: m.get_type() == \"HEARTBEAT\" and m.custom_mode == value)" },
  { id: "smoke-connect", file: "tests/flight_smoke.py", from: "    def connect(self):", to: "refusing to validate" },
  { id: "smoke-execute", file: "tests/flight_smoke.py", from: "    def execute(self):", to: "return measurements" },
  { id: "judge", file: "tests/flight_smoke.py", from: "def judge(truth", to: "\"clock_difference_s\": clock_difference}" },
  { id: "observer", file: "controllers/flight_observer/flight_observer.py", from: "robot = Supervisor()", to: "world_view.update(" },
  { id: "sensors-json", file: "config/sensors.json", from: "\"lidar\": {", to: "\"render_layers\"" },
  { id: "stream-init", file: "controllers/flight_bridge/sensor_stream.py", from: "        def period(rate_hz):", to: "self.camera.enable(self.camera_period)" },
  { id: "stream-put", file: "controllers/flight_bridge/sensor_stream.py", from: "    def _put(self, message):", to: "self.dropped += 1" },
  { id: "stream-step", file: "controllers/flight_bridge/sensor_stream.py", from: "    def after_step(self):", to: "self._put(protocol.pack(protocol.IMAGE" },
  { id: "protocol", file: "sensing/protocol.py", from: "MAGIC = ", to: "return ACK.unpack" },
  { id: "ros-qos", file: "ros/ironlark_bridge.py", from: "        def reliable(depth):", to: "self.static_published = False" },
  { id: "ros-message", file: "ros/ironlark_bridge.py", from: "        elif kind == protocol.LIDAR:", to: "self.camera_info.publish(info)" },
  { id: "ros-serve", file: "ros/ironlark_bridge.py", from: "def serve(node, address):", to: "connection.close()" },
  { id: "completeness", file: "tools/check_recording.py", from: "def completeness(", to: "return problems" },
  { id: "fit-wall", file: "tools/check_recording.py", from: "def fit_wall(", to: "return distance, angle, residual" },
  { id: "replay-digests", file: "tools/check_replay.py", from: "def digests(bag):", to: "return out" },
  { id: "losses", file: "ironlark", from: "def sensing_losses(report):", to: "return losses" },
];

const languages = { py: "python", c: "c", wbt: "python", parm: "ini", json: "json", patch: "diff", cpp: "cpp" };

const highlighter = await createHighlighter({ themes: ["vesper"], langs: ["python", "c", "cpp", "diff", "ini", "json"] });

const escape = (s) => s.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
const out = {};
for (const e of excerpts) {
  const isUpstream = e.file.startsWith("upstream:");
  const path = isUpstream ? resolve(upstream, e.file.slice(9)) : resolve(repo, e.file);
  const lines = readFileSync(path, "utf8").split("\n");
  const start = lines.findIndex((l) => l.includes(e.from));
  if (start < 0) throw new Error(`${e.id}: no line in ${e.file} contains ${JSON.stringify(e.from)}`);
  const stop = lines.findIndex((l, i) => i >= start && l.includes(e.to));
  if (stop < 0) throw new Error(`${e.id}: no line in ${e.file} after ${start + 1} contains ${JSON.stringify(e.to)}`);
  const end = Math.min(stop + (e.extra ?? 0), lines.length - 1);
  const text = lines.slice(start, end + 1).join("\n");
  const ext = e.file === "ironlark" ? "py" : e.file.split(".").pop();
  const lang = e.lang ?? languages[ext] ?? "python";
  const tokens = highlighter.codeToTokensBase(text, { lang, theme: "vesper" });
  const html = tokens.map((line) =>
    line.map((t) => {
      const style = `color:${t.color}${t.fontStyle & 1 ? ";font-style:italic" : ""}`;
      return `<span style="${style}">${escape(t.content)}</span>`;
    }).join(""));
  out[e.id] = {
    file: isUpstream
      ? (e.file.endsWith(".cpp") ? "ardupilot/libraries/SITL/SIM_Webots_Python.cpp" : "ardupilot/…/Webots_Python/…/webots_vehicle.py")
      : e.file,
    upstream: isUpstream,
    start: start + 1,
    lines: html,
  };
}
writeFileSync(resolve(here, "../src/generated/code.js"), `export default ${JSON.stringify(out)};\n`);

// The renderer patches, as the README's table describes and measures them.
const patches = [];
for (const line of readFileSync(resolve(repo, "README.md"), "utf8").split("\n")) {
  const m = line.match(/^\| \*\*(\d{4})-([^*]+)\*\* \| (.*) \| (.*) \|\s*$/);
  if (m) patches.push({ n: m[1], name: m[2].replaceAll("-", " "), changes: m[3], measured: m[4] });
}
writeFileSync(resolve(here, "../src/generated/patches.js"), `export default ${JSON.stringify(patches)};\n`);
console.log(Object.entries(out).map(([id, e]) => `${id} ${e.file}:${e.start}-${e.start + e.lines.length - 1}`).join("\n"));

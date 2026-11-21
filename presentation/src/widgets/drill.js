// Questions a committee is likely to ask, each answered from the code and this run.
import { h } from "../lib/data.js";

const QA = [
  ["How do you keep the physics and the autopilot in step?",
    "Lockstep over UDP. Every 2 ms physics step the bridge sends one 128-byte sensor packet and waits for a motor packet; it drains the socket, keeps the newest, applies it and steps exactly once. ArduPilot advances its own clock by the difference between packet timestamps, so its time is Webots time.",
    "controllers/flight_bridge/lockstep.c:62–91 · SIM_Webots_Python.cpp:95–124"],
  ["What happens when rendering is slow?",
    "Simulated time slows down; nothing else changes. Webots does not step until the frame is done, the bridge discards ArduPilot's resends, the mission's timeouts count simulated seconds, and the sensor stream stamps Webots time. This forest run went at 0.82× real time and passed.",
    "flight_bridge.py:39–46 · flight_smoke.py:91–96 · result.json sensing.realtime_factor"],
  ["Is the autopilot using the simulator's ground truth to fly?",
    "No. flight.parm sets AHRS_EKF_TYPE 3 (EKF3 on simulated IMU, barometer, compass and GPS) instead of the example's 10, and the check reads the parameter back and refuses to run otherwise.",
    "config/flight.parm:2 · tests/flight_smoke.py:146–151"],
  ["How do you know the drone actually flew, rather than the autopilot just reporting it?",
    "A separate Webots supervisor records the drone node's true position every 100 ms. judge() reads that file only after disarm and checks the peak, the longest stay in the hover band, the final height, the drift and the agreement of the two clocks.",
    "controllers/flight_observer/flight_observer.py:42–52 · tests/flight_smoke.py:13–51"],
  ["Why the square root on the motor command?",
    "Webots' propeller thrust is k·|ω|·ω, quadratic in rotor speed; ArduPilot with MOT_THST_EXPO 0 expects thrust linear in its output. ω = √u·ω_max gives thrust = k·ω_max²·u. Check: hovering needs 4 · 0.0012 · 100² · u = 1.5 kg · g, so u = 0.307, PWM ≈ 1307 µs; this run hovered at 1305 µs.",
    "lockstep.c:84–86 · upstream webots_vehicle.py:255–259"],
  ["Why clamp negative motor values?",
    "ArduPilot's SITL backend sends (pwm − 1000)/1000, so an output with no PWM arrives as −1. Upstream's generic bridge reads that as full reverse thrust. The Iris's motors are unidirectional, so disabled means stopped, including before arming and after landing.",
    "flight_bridge.py:33–37 · SIM_Webots_Python.cpp:67–74"],
  ["Why does it sit 40 seconds on the pad?",
    "The mission waits for an EKF report with an absolute horizontal position and without constant-position mode. In this run GPS had a fix at 9.5 s but EKF3 reported using GPS at 41.0 s. The roadmap's measurement: faster GPS detection did not help, because arming waits on EKF3 learning the gyro biases.",
    "tests/flight_smoke.py:156–159 · ROADMAP.md, Performance, 'Faster GPS detection'"],
  ["Which way is north?",
    "For ArduPilot, Webots +x: the bridge sends (x, −y, −z) and SITL reads it directly as NED. sensing/frames.py defines ENU→NED as (y, x, −z), north = +y. Nothing uses frames.py's conversion yet; external navigation must use the bridge's convention or the EKF sees a 90° disagreement.",
    "lockstep.c:31–35 · SIM_Webots_Python.cpp:118–121 · sensing/frames.py:12–14"],
  ["How do you know the recording is complete?",
    "Three layers. The controller counts what it queued, sent and dropped, and the ROS bridge acknowledges how many it read: any difference fails the run. check-recording then requires every topic, every period exactly, from the clock's first stamp to its last. check-replay replays the bag and compares sha256 digests message by message.",
    "ironlark:289–300 · tools/check_recording.py:48–71 · tools/check_replay.py:25–37"],
  ["How do you know the LiDAR and camera are right?",
    "The empty world's red calibration wall. Hovering scans are placed in the world by ground truth and the static transforms; a least-squares plane lands 3.4 mm from the wall's true face, 0.10° off, with scatter 1.1× the configured noise, and 99.1% of wall returns project onto red pixels in the camera frame of the same instant.",
    "tools/check_recording.py:114–126, 282–301 · runs/20261009-160306-33522/sensor_check.json"],
  ["Can the autonomy cheat by reading ground truth?",
    "It is on separate topics, /ground_truth/pose and /ground_truth/velocity, never on the autonomy topics, and the project's rule is that evaluator pose fed in as odometry is not localization. Enforcing it in phase two means autonomy nodes never subscribe to /ground_truth/*.",
    "ros/ironlark_bridge.py:1–7, 138–149 · README.md, Localization boundary"],
  ["Why patch Webots instead of simplifying the forest?",
    "The project's rule is to fix engine bottlenecks rather than cut asset quality. Each patch is one feature with measurements, e.g. instancing took the forest from 16,814 to 185 geometries and the frame from 25 to 5.7 ms; occlusion culling cut the drone camera's frame from 82 to 59 ms.",
    "native/patches/0003, 0016 · README.md, Webots patches"],
  ["Why is the inner loop in C?",
    "It runs 500 times per simulated second. lockstep.c computes exactly what the Python loop does through the same Webots C API; Python still does sensing through a callback. The launcher compiles it per run and records which loop flew and the library's hash.",
    "ironlark:46–64, 454–457 · flight_bridge.py:84–97"],
  ["Is a run reproducible?",
    "Inputs are pinned: ArduPilot at dbe7921 with a hash-recorded patch, Webots c6793d8 plus a hash-recorded series, autopilot storage wiped every run, Webots randomSeed 7, IMU noise from a fixed seed, the world refused unless its receipt matches the code. OS packages in the image float, so builds are source-pinned rather than bit-identical.",
    "ironlark:433–465, 563 · worlds/flight_foundation.wbt:9 · README.md"],
  ["What does --record cost?",
    "In the empty world nothing measurable (1.0× real time). In the forest this run ran at 0.82× real time, mostly the camera: its 640×480 frame draws tens of millions of triangles. Simulated time slows, nothing is dropped: 63,236 messages, all acknowledged.",
    "result.json sensing · ROADMAP.md, Sensor rig and ROS 2"],
  ["What is still missing for phase two?",
    "The autonomy itself: LiDAR-inertial odometry evaluated on recordings, external navigation into EKF3, flight with GPS cut at the measurement layer, then mapping, collision-aware motion and the first mission. Open items from this reading: the north convention, LiDAR per-point time, the first IMU sample, the recording tail.",
    "ROADMAP.md, Next: autonomy"],
];

// In plain words: the idea behind each answer, and the follow-up a committee member is likely to ask next.
const PLAIN = [
  ["Two programs fly the drone together: Webots computes the physics, ArduPilot the motor commands. They take turns every 2 ms of simulated time. Webots sends what the drone's sensors read, waits for ArduPilot's motor command, applies it, and only then moves the world forward 2 ms. Neither can get ahead, so ArduPilot never decides on a world that has already moved on.", "What if a packet is lost?", "UDP can drop a packet, so both sides resend: ArduPilot repeats its motor packet every 100 ms, the bridge repeats its sensor packet after 1 s of silence. The bridge keeps only the newest motor packet, so a repeat can never add an extra physics step."],
  ["Nothing measured in real seconds matters to the flight. If drawing a frame takes long, Webots simply does not take its next step until it is done, and every part of the system (ArduPilot's clock, the mission's time limits, the sensor timestamps) counts simulated time. The simulation runs slower than real life, but what happens inside it is the same.", "Then why did ArduPilot need a patch?", "Its Webots backend had one real-time check: after 5 s without a packet it reset its time reference, so the next packet added the whole simulation time to its clock a second time. The patch removes that reset."],
  ["ArduPilot's example settings let the autopilot read its position straight from the simulator, which would make the test meaningless. Ironlark switches to ArduPilot's real estimator, EKF3, which must work its position out from simulated sensors, as it would on a real drone. The mission script reads this setting back and refuses to fly otherwise.", "Where do the simulated GPS, compass and barometer come from?", "ArduPilot's SITL generates them from the true state the bridge sends every step: position, velocity, attitude. That is how its simulator backends are designed."],
  ["A separate, invisible robot in Webots, with access to the true physics, writes down the drone's real position ten times a second. After landing, judge() grades the flight on that record: how high, how long it hovered, whether it came down, how far it wandered. The autopilot never sees that record and cannot influence it.", "Why not just trust the autopilot's telemetry?", "Because it is the autopilot's own belief, produced by the software under test. If its estimate were wrong, its telemetry would be wrong in the same way."],
  ["ArduPilot assumes that doubling its motor command doubles the thrust. A propeller's thrust really grows with the square of its speed. Setting each motor's speed to the square root of the command cancels the square, so thrust is proportional to the command again, as ArduPilot expects.", "How do you know it is right?", "By hand: k = 0.0012 and a top speed of 100 rad/s give four motors 48·u newtons. A 1.5 kg drone weighs 14.7 N, so hover needs u ≈ 0.307, about 1307 µs. The recorded hover was 1305 µs."],
  ["ArduPilot encodes a motor output that is switched off as −1. The generic bridge reads that as \"spin backwards at full speed\". The Iris's motors only spin one way, so Ironlark's bridge turns every negative value into zero: off means stopped.", "When does ArduPilot send −1?", "For any output with no pulse at all: unused channels, and the motors before arming and after landing."],
  ["The mission waits until the estimator says it knows where it is. GPS is ready after 9.5 s, but EKF3 first learns the gyros' small constant errors (their biases) while the drone sits still, and only trusts its position at about 41 s. Making GPS ready sooner was measured, and did not help.", "Could you skip the wait?", "Only by weakening the check, and arming before the estimator has settled is what pre-arm checks exist to prevent. Unrecorded runs fast-forward the wait instead: the same 41 simulated seconds, in less real time."],
  ["For ArduPilot, north is Webots' +x direction, because the bridge sends its numbers in that order and ArduPilot uses them as they are. A helper file in the project uses the textbook convention, where north is +y. Nothing uses that helper in flight yet, but phase two must use ArduPilot's convention when it feeds in its own position, or the two would disagree by 90°.", "Why is the bridge's choice acceptable?", "It is a proper rotation, and ArduPilot builds its simulated GPS and compass from the same numbers, so everything ArduPilot sees agrees. Only a second source using a different convention would conflict."],
  ["Three independent checks. While flying, the sender counts every message and the receiver reports how many it got; any difference fails the run. Afterwards, every topic must have a message at every expected instant, from the first to the last. Finally, playing the recording back and recording it again must give byte-identical messages.", "Has it caught anything?", "Yes. Today two of three empty-world recordings failed the coverage check: the last 0.5–0.8 s was missing on some topics, although the stream itself delivered every message. The cause is still open."],
  ["The empty world has a red wall at a known place. Scans taken while hovering are placed in the world using the drone's true position. A flat plane fitted to the wall's points lands 3.4 mm from the true wall and almost exactly parallel, scattered by about the configured sensor noise. And 99% of those points, projected into the camera image from the same moment, land on red pixels: LiDAR, camera, timing and mounting positions all agree.", "Why check the scatter at all?", "Too little scatter would mean the configured 2 cm range noise is not being applied. The check wants 0.5–1.5× that noise."],
  ["The true position travels on its own ROS topics, separate from the sensor topics. The project's rule is that autonomy software never reads them; they exist only for scoring after the flight.", "How would you enforce that?", "Today it is a written rule, not a mechanism. One way: develop and score the autonomy on recordings with the ground-truth topics filtered out, and keep the scoring as a separate step."],
  ["Webots could not draw the forest well: see-through needles, hundreds of thousands of plants, fog. The project's rule is to fix the engine rather than make the forest worse, because the drone's camera and LiDAR see through that same engine. Each patch adds one feature and records what it changed and what it measured.", "Isn't maintaining patches a burden?", "They are a git series against one pinned Webots commit, applied and compiled by one command, ./ironlark build-renderer. Moving to a newer Webots means rebasing that series."],
  ["The loop runs 500 times per simulated second, and each pass costs far more in Python than in C. The C version does exactly what the Python one does, through Webots' C library; Python still samples the sensors, through a callback. Each run compiles it and records which loop flew.", "What if there is no C compiler?", "The identical Python loop flies, and the manifest records \"language\": \"Python\"."],
  ["Everything that matters is pinned: the ArduPilot and Webots versions and their patches, ArduPilot's settings wiped every run, Webots' random seed, the sensor-noise seed, and a world that must match today's code. Not pinned: the operating-system packages inside the container images, so a rebuild uses the same source but not necessarily identical binaries.", "Would two runs give identical numbers?", "That is something to measure, not claim: compare two runs' ground truth. Each run's manifest records exactly what ran, so any difference can be traced."],
  ["In the empty world, nothing measurable. In the forest, the simulation ran at 0.82× real time, mostly because drawing the camera's view of the forest is expensive. Simulated time simply slows; no message is dropped.", "Why keep recorded runs in real time?", "So the measured slowdown means what it says. Fast-forwarding the wait would hide the cost of sensing."],
  ["The autonomy itself: estimating the drone's motion from LiDAR and IMU, feeding that to ArduPilot's estimator in place of GPS, flying with GPS cut, then mapping, avoiding obstacles and a first real mission. Plus this reading's open questions: the north convention, the LiDAR point timing, the first IMU message and the recording tail.", "Why start the odometry on recordings rather than live?", "A recording replays exactly, so an odometry can be developed and scored against ground truth repeatably, without flying each time."],
];

export default function drill(root) {
  const bar = h(`<div class="drill-bar"><span data-c>0 of ${QA.length} revealed</span><button class="btn" data-all>reveal all</button></div>`);
  root.before(bar);
  const count = () => { bar.querySelector("[data-c]").textContent = `${root.querySelectorAll(".q.open").length} of ${QA.length} revealed`; };
  QA.forEach(([q, a, ref], i) => {
    const [plain, fq, fa] = PLAIN[i];
    const card = h(`<article class="q"><div class="qn">${String(i + 1).padStart(2, "0")}</div><p class="qq">${q}</p>
      <div class="qa"><div>
        <div class="q-plain"><p>${plain}</p><div class="say">Say it like this</div></div>
        <p class="say-it">${a}</p><div class="ref">${ref}</div>
        <div class="q-plain"><div class="follow"><b>Likely follow-up: ${fq}</b><p>${fa}</p></div></div>
      </div></div><div class="reveal">click to reveal</div></article>`);
    card.addEventListener("click", () => { card.classList.toggle("open"); count(); });
    root.append(card);
  });
  bar.querySelector("[data-all]").addEventListener("click", () => { root.querySelectorAll(".q").forEach((q) => q.classList.add("open")); count(); });
}

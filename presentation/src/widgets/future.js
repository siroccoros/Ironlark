// ROADMAP.md's "Next: autonomy", as a sequence from what exists to what does not yet.
import { h } from "../lib/data.js";

const STEPS = [
  ["exists", "Sensors on ROS 2", "IMU, LiDAR and camera on Webots time, recorded complete and replayable; ground truth on its own topics."],
  ["next", "LiDAR-inertial odometry", "Run on the recorded bags first, scored against /ground_truth/pose: no flight needed to measure error."],
  ["next", "External navigation", "Feed pose, velocity, uncertainty and health to ArduPilot's external-navigation input, in the frame EKF3 already uses."],
  ["next", "GPS off", "Hover and make short moves with navigation GPS cut at the measurement layer; measure error against the evaluator."],
  ["next", "Map, move, mission", "Local mapping, collision-aware motion, tracking-loss handling, GPS ↔ external-navigation transitions in motion; then leave home, map a bounded unknown interior, return, land."],
];

export default function future(root) {
  root.innerHTML = `<div class="w-title"><p>From the bench to autonomy · ROADMAP.md</p><p class="w-meta">solid: exists today · dashed: phase two</p></div>`;
  const flow = h(`<div class="flow"></div>`);
  STEPS.forEach(([kind, name, text], i) => flow.append(h(`<div class="st ${kind === "exists" ? "exists" : "todo"}">
    <div class="sk">${kind === "exists" ? "exists" : `step ${i}`}</div><div class="sn">${name}</div><p>${text}</p></div>`)));
  root.append(flow);
}

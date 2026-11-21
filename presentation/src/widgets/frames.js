// Webots' ENU axes, the drone's FLU body, and the two "north"s: the one the bridge gives ArduPilot (Webots +x) and the
// one sensing/frames.py defines (Webots +y).
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { h } from "../lib/data.js";

// ENU vector to three.js (y up).
const v3 = (x, y, z) => new THREE.Vector3(x, z, -y);

export default function frames(root) {
  root.innerHTML = `<div class="w-title"><p>Frames · drag to orbit</p><p class="w-meta">Webots world ENU · body FLU</p></div>`;
  const stage = h(`<div class="stage"><canvas></canvas><div class="hud">grid: 1 m</div></div>`);
  root.append(stage);
  const canvas = stage.querySelector("canvas");
  const controls = h(`<div class="controls"><span>drone yaw in Webots (counter-clockwise from +x)</span><input type="range" min="-180" max="180" value="30"><b class="mono">30°</b></div>`);
  root.append(controls);
  const read = h(`<div class="readouts">
    <div><div class="k">Webots ψ</div><div class="v" data-r="w">30°</div><div class="s">ENU, from +x</div></div>
    <div><div class="k" style="color:#7cc6ff">ArduPilot heading</div><div class="v" data-r="a">330°</div><div class="s">−ψ, from the bridge's north (+x)</div></div>
    <div><div class="k" style="color:#b7a3ff">frames.py heading</div><div class="v" data-r="f">60°</div><div class="s">90° − ψ, from north = +y</div></div></div>`);
  root.append(read);

  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(40, 4 / 3, 0.1, 100);
  camera.position.copy(v3(-5.5, -6.5, 5.2));
  const orbit = new OrbitControls(camera, canvas);
  orbit.target.copy(v3(0, 0, 0.7));
  orbit.enableDamping = true;
  orbit.enableZoom = false;

  const grid = new THREE.GridHelper(10, 10, 0x2a3530, 0x18201c);
  scene.add(grid);
  const tags = [];
  const tag = (text, pos, color) => {
    const el = h(`<span class="tag3" style="color:${color}">${text}</span>`);
    stage.append(el);
    tags.push({ el, pos });
  };
  const arrow = (dir, len, color, origin = v3(0, 0, 0)) => {
    const a = new THREE.ArrowHelper(dir.clone().normalize(), origin, len, color, 0.28, 0.14);
    a.line.material.linewidth = 2;
    return a;
  };
  // Webots world axes.
  scene.add(arrow(v3(1, 0, 0), 4.2, 0xa4d483), arrow(v3(0, 1, 0), 4.2, 0xa4d483), arrow(v3(0, 0, 1), 3.2, 0xa4d483));
  tag("Webots +x (east)", v3(5.0, 0.0, 0.5), "#a4d483");
  tag("Webots +y (north)", v3(0, 4.9, 0.5), "#a4d483");
  tag("+z up", v3(0, 0, 3.5), "#a4d483");

  // The bridge's NED as ArduPilot reads it: N = +x, E = −y, D = −z. And frames.py's: N = +y, E = +x.
  const ring = (radius, color, dashed) => {
    const pts = [];
    for (let i = 0; i <= 128; i++) { const a = (i / 128) * Math.PI * 2; pts.push(v3(Math.cos(a) * radius, Math.sin(a) * radius, 0.01)); }
    const g = new THREE.BufferGeometry().setFromPoints(pts);
    const m = dashed ? new THREE.LineDashedMaterial({ color, dashSize: 0.15, gapSize: 0.12 }) : new THREE.LineBasicMaterial({ color });
    const l = new THREE.Line(g, m);
    if (dashed) l.computeLineDistances();
    return l;
  };
  scene.add(ring(3.0, 0x7cc6ff, false), ring(3.6, 0xb7a3ff, true));
  scene.add(arrow(v3(1, 0, 0), 3.0, 0x7cc6ff, v3(0, 0, 0.02)), arrow(v3(0, -1, 0), 2.2, 0x4f86b0, v3(0, 0, 0.02)));
  scene.add(arrow(v3(0, 1, 0), 3.6, 0xb7a3ff, v3(0, 0, 0.03)), arrow(v3(1, 0, 0), 2.2, 0x7b6db3, v3(0, 0, 0.03)));
  tag("N · ArduPilot", v3(3.0, -0.6, 0), "#7cc6ff");
  tag("E · ArduPilot", v3(0.35, -2.55, 0), "#7cc6ff");
  tag("N · frames.py", v3(-0.9, 3.7, 0), "#b7a3ff");
  tag("E · frames.py", v3(2.1, 0.55, 0), "#b7a3ff");

  // The drone, FLU: nose along body +x.
  const drone = new THREE.Group();
  drone.position.copy(v3(0, 0, 1.3));
  drone.scale.setScalar(1.7);
  const dark = new THREE.MeshStandardMaterial({ color: 0x59646a, roughness: 0.45, metalness: 0.2 });
  const accent = new THREE.MeshStandardMaterial({ color: 0xff7a2f, roughness: 0.4, emissive: 0x501a00 });
  drone.add(new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.12, 0.32), dark));
  for (const [sx, sy] of [[1, 1], [1, -1], [-1, 1], [-1, -1]]) {
    const arm = new THREE.Mesh(new THREE.BoxGeometry(0.62, 0.04, 0.05), dark);
    arm.position.set(sx * 0.2, 0, -sy * 0.2);
    arm.rotation.y = Math.atan2(sy, sx);
    drone.add(arm);
    const disc = new THREE.Mesh(new THREE.CylinderGeometry(0.22, 0.22, 0.01, 32),
      new THREE.MeshBasicMaterial({ color: 0x9fb3a8, transparent: true, opacity: 0.18 }));
    disc.position.set(sx * 0.42, 0.06, -sy * 0.42);
    drone.add(disc);
  }
  const nose = new THREE.Mesh(new THREE.ConeGeometry(0.08, 0.22, 16), accent);
  nose.rotation.z = -Math.PI / 2;
  nose.position.set(0.36, 0, 0);
  drone.add(nose);
  const body = (dir, color, len) => new THREE.ArrowHelper(dir, new THREE.Vector3(), len, color, 0.16, 0.08);
  drone.add(body(new THREE.Vector3(1, 0, 0), 0xff7a2f, 1.3), body(new THREE.Vector3(0, 0, -1), 0xffb07a, 0.9), body(new THREE.Vector3(0, 1, 0), 0xffb07a, 0.8));
  scene.add(drone);
  const noseTag = { el: h(`<span class="tag3" style="color:#ff7a2f">body +x (forward)</span>`), pos: null };
  stage.append(noseTag.el);
  tags.push(noseTag);

  scene.add(new THREE.HemisphereLight(0xdfe8e0, 0x101412, 2.2));
  const sun = new THREE.DirectionalLight(0xffffff, 1.5);
  sun.position.set(3, 6, 2);
  scene.add(sun);

  const slider = controls.querySelector("input");
  const setYaw = (deg) => {
    drone.rotation.y = (deg * Math.PI) / 180;
    controls.querySelector("b").textContent = `${deg}°`;
    const wrap = (a) => ((a % 360) + 360) % 360;
    read.querySelector('[data-r="w"]').textContent = `${deg}°`;
    read.querySelector('[data-r="a"]').textContent = `${wrap(-deg).toFixed(0)}°`;
    read.querySelector('[data-r="f"]').textContent = `${wrap(90 - deg).toFixed(0)}°`;
  };
  slider.addEventListener("input", () => setYaw(Number(slider.value)));
  setYaw(30);

  const resize = () => {
    const w = stage.clientWidth, hh = stage.clientHeight;
    renderer.setSize(w, hh, false);
    camera.aspect = w / hh;
    camera.updateProjectionMatrix();
  };
  resize();
  addEventListener("resize", resize);
  let visible = false;
  new IntersectionObserver(([e]) => { visible = e.isIntersecting; }).observe(stage);
  const p = new THREE.Vector3();
  const loop = () => {
    requestAnimationFrame(loop);
    if (!visible) return;
    orbit.update();
    renderer.render(scene, camera);
    noseTag.pos = drone.localToWorld(new THREE.Vector3(1.0, 0.12, 0));
    for (const t of tags) {
      p.copy(t.pos).project(camera);
      t.el.style.left = `${(p.x * 0.5 + 0.5) * stage.clientWidth}px`;
      t.el.style.top = `${(-p.y * 0.5 + 0.5) * stage.clientHeight}px`;
      t.el.style.opacity = p.z < 1 ? 1 : 0;
    }
  };
  loop();
}

// The cover: every LiDAR return of the forest flight, placed by ground truth, with the drone's path.
import * as THREE from "three";
import clouds from "../generated/clouds.js";
import flight from "../generated/flight-data.js";
import { int16, enuToThree } from "../lib/data.js";

const vertex = /* glsl */ `
  uniform float uTime, uReveal, uSize, uPixel;
  attribute float aSeed;
  varying vec3 vColor;
  varying float vAlpha;
  void main() {
    vec4 mv = modelViewMatrix * vec4(position, 1.0);
    float d = length(position.xz);
    float h = position.y;
    // Height ramp: forest floor moss, trunks, crowns towards pale gold.
    vec3 low = vec3(0.20, 0.33, 0.24);
    vec3 mid = vec3(0.52, 0.74, 0.46);
    vec3 top = vec3(0.98, 0.84, 0.58);
    float t = clamp((h + 2.0) / 22.0, 0.0, 1.0);
    vec3 c = mix(low, mid, smoothstep(0.0, 0.35, t));
    c = mix(c, top, smoothstep(0.35, 1.0, t));
    // A scan ring sweeping out from the drone.
    float ring = mod(uTime * 9.0, 64.0);
    float pulse = exp(-pow((d - ring) / 1.4, 2.0));
    c += vec3(1.0, 0.48, 0.18) * pulse * 0.9;
    float reveal = smoothstep(uReveal, uReveal - 6.0, d);
    float fade = 1.0 - smoothstep(30.0, 46.0, d);
    vAlpha = reveal * fade * (0.55 + 0.45 * aSeed) * (0.9 + 1.4 * pulse);
    vColor = c;
    gl_Position = projectionMatrix * mv;
    gl_PointSize = max(1.0, uSize * uPixel * (0.5 + 0.8 * aSeed) * (40.0 / -mv.z));
  }`;
const fragment = /* glsl */ `
  varying vec3 vColor;
  varying float vAlpha;
  void main() {
    vec2 p = gl_PointCoord - 0.5;
    float r = dot(p, p);
    if (r > 0.25) discard;
    float a = smoothstep(0.25, 0.0, r) * vAlpha;
    gl_FragColor = vec4(vColor * a, a);
  }`;

export default function hero(canvas, { lenis }) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: false, alpha: false, powerPreference: "high-performance" });
  const pixel = Math.min(devicePixelRatio, 1.75);
  renderer.setPixelRatio(pixel);
  renderer.setClearColor(0x060807, 1);
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(36, 1, 0.1, 400);

  const f = clouds.forest;
  const pos = enuToThree(int16(f.data), f.scale);
  const seed = new Float32Array(f.count);
  for (let i = 0; i < f.count; i++) seed[i] = Math.random();
  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.BufferAttribute(pos, 3));
  geo.setAttribute("aSeed", new THREE.BufferAttribute(seed, 1));
  const uniforms = { uTime: { value: 0 }, uReveal: { value: 0 }, uSize: { value: 1.05 }, uPixel: { value: pixel } };
  const mat = new THREE.ShaderMaterial({ vertexShader: vertex, fragmentShader: fragment, uniforms, transparent: true,
    depthWrite: false, blending: THREE.CustomBlending, blendSrc: THREE.OneFactor, blendDst: THREE.OneMinusSrcAlphaFactor });
  scene.add(new THREE.Points(geo, mat));

  // The flight path from ground truth, and the drone riding it.
  const truth = flight.forest.truth.filter((r) => r[0] >= 44 && r[0] <= 59.2);
  const path = new Float32Array(truth.length * 3);
  truth.forEach((r, i) => { path[i * 3] = r[1]; path[i * 3 + 1] = r[3]; path[i * 3 + 2] = -r[2]; });
  const line = new THREE.Line(new THREE.BufferGeometry().setAttribute("position", new THREE.BufferAttribute(path, 3)),
    new THREE.LineBasicMaterial({ color: 0xff7a2f, transparent: true, opacity: 0.9 }));
  scene.add(line);
  const glow = new THREE.Sprite(new THREE.SpriteMaterial({ map: glowTexture(), color: 0xff8a40, transparent: true,
    blending: THREE.AdditiveBlending, depthWrite: false }));
  glow.scale.setScalar(3.2);
  scene.add(glow);
  const core = new THREE.Mesh(new THREE.SphereGeometry(0.22, 16, 12), new THREE.MeshBasicMaterial({ color: 0xffd2b0 }));
  scene.add(core);
  // The pad.
  const pad = new THREE.Mesh(new THREE.RingGeometry(0.75, 0.8, 64), new THREE.MeshBasicMaterial({ color: 0xf2bd52, transparent: true, opacity: 0.7, side: THREE.DoubleSide }));
  pad.rotation.x = -Math.PI / 2;
  pad.position.y = 0.02;
  scene.add(pad);

  let w = 0, hgt = 0;
  const resize = () => {
    w = canvas.clientWidth; hgt = canvas.clientHeight;
    renderer.setSize(w, hgt, false);
    camera.aspect = w / hgt;
    // Push the forest to the right of the title.
    if (w > 900) camera.setViewOffset(w, hgt, -w * 0.27, -hgt * 0.04, w, hgt); else camera.clearViewOffset();
    camera.updateProjectionMatrix();
  };
  resize();
  addEventListener("resize", resize);

  const mouse = { x: 0, y: 0, tx: 0, ty: 0 };
  addEventListener("pointermove", (e) => { mouse.tx = e.clientX / innerWidth - 0.5; mouse.ty = e.clientY / innerHeight - 0.5; });

  let visible = true, scroll = 0;
  new IntersectionObserver(([e]) => { visible = e.isIntersecting; }).observe(canvas);
  lenis.on("scroll", ({ scroll: s }) => { scroll = s; });

  const clock = new THREE.Timer();
  const start = performance.now();
  const loop = () => {
    requestAnimationFrame(loop);
    if (!visible) return;
    clock.update();
    const t = clock.getElapsed();
    uniforms.uTime.value = t;
    uniforms.uReveal.value = Math.min(80, ((performance.now() - start) / 1000) * 22);
    const k = Math.min(1, scroll / innerHeight);
    mouse.x += (mouse.tx - mouse.x) * 0.04;
    mouse.y += (mouse.ty - mouse.y) * 0.04;
    const angle = t * 0.035 + 0.8 + mouse.x * 0.5;
    const radius = 64 - k * 14;
    const elev = 0.3 + mouse.y * 0.12 + k * 0.3;
    camera.position.set(Math.cos(angle) * radius * Math.cos(elev), 4 + Math.sin(elev) * radius, Math.sin(angle) * radius * Math.cos(elev));
    camera.lookAt(0, 7 + k * 2, 0);
    // The drone rises, holds and lands on a 14 s loop, along the recorded heights.
    const ft = 44 + ((t * 1.1) % 15.2);
    const r = truth.find((row) => row[0] >= ft) ?? truth[truth.length - 1];
    glow.position.set(r[1], r[3] + 0.1, -r[2]);
    core.position.copy(glow.position);
    glow.material.opacity = 0.75 + 0.25 * Math.sin(t * 4);
    renderer.render(scene, camera);
  };
  loop();
}

function glowTexture() {
  const c = document.createElement("canvas");
  c.width = c.height = 128;
  const g = c.getContext("2d");
  const grad = g.createRadialGradient(64, 64, 0, 64, 64, 64);
  grad.addColorStop(0, "rgba(255,200,160,1)");
  grad.addColorStop(0.25, "rgba(255,122,47,0.55)");
  grad.addColorStop(1, "rgba(255,122,47,0)");
  g.fillStyle = grad;
  g.fillRect(0, 0, 128, 128);
  return new THREE.CanvasTexture(c);
}

// One LiDAR scan from the forest flight at hover, in the sensor's own frame, coloured by ring.
import clouds from "../generated/clouds.js";
import flight from "../generated/flight-data.js";
import { int16, bytes, enuToThree } from "../lib/data.js";
import { stage, pointMaterial } from "../lib/stage.js";

export default function scan(root) {
  const hs = flight.forest.hoverScan;
  const { scene, THREE } = stage(root, {
    title: "One scan · drag to orbit",
    meta: `t = ${hs.time.toFixed(1)} s · lidar_link`,
    hud: `${hs.returns.toLocaleString("en")} of ${hs.of.toLocaleString("en")} rays returned · rings 0 (+52.3°) → 31 (−6.7°)`,
    camera: [-26, 20, 28], target: [0, 2, 0],
  });
  const c = clouds.scan;
  const pos = enuToThree(int16(c.data), c.scale);
  const ring = bytes(c.ring);
  const col = new Float32Array(c.count * 3);
  const top = new THREE.Color("#f2bd52"), mid = new THREE.Color("#ff7a2f"), low = new THREE.Color("#7cc6ff"), tmp = new THREE.Color();
  for (let i = 0; i < c.count; i++) {
    const t = ring[i] / 31;
    tmp.copy(t < 0.5 ? top : mid).lerp(t < 0.5 ? mid : low, t < 0.5 ? t * 2 : (t - 0.5) * 2);
    col.set([tmp.r, tmp.g, tmp.b], i * 3);
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.BufferAttribute(pos, 3));
  geo.setAttribute("color", new THREE.BufferAttribute(col, 3));
  scene.add(new THREE.Points(geo, pointMaterial(1.3)));
  // The sensor and its vertical field: +52.3° to −6.7°.
  scene.add(new THREE.Mesh(new THREE.SphereGeometry(0.18, 16, 12), new THREE.MeshBasicMaterial({ color: 0xffffff })));
  const fan = [];
  for (const deg of [52.3, -6.7]) {
    const a = (deg * Math.PI) / 180;
    for (let k = 0; k < 8; k++) {
      const az = (k / 8) * Math.PI * 2;
      fan.push(new THREE.Vector3(0, 0, 0), new THREE.Vector3(Math.cos(az) * Math.cos(a) * 6, Math.sin(a) * 6, -Math.sin(az) * Math.cos(a) * 6));
    }
  }
  scene.add(new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(fan), new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.18 })));
}

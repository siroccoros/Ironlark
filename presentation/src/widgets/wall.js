// The empty world's calibration wall as the hovering scans saw it (world frame, placed by ground truth), and the check's
// numbers, from the last empty-world recording whose check passed.
import clouds from "../generated/clouds.js";
import flight from "../generated/flight-data.js";
import { int16, h } from "../lib/data.js";
import { stage, pointMaterial } from "../lib/stage.js";

export default function wall(root) {
  const { scene, THREE } = stage(root, {
    title: "Calibration wall · 20 hovering scans",
    meta: "world ENU · 3 cm grid",
    hud: "red: returns inside the wall's box · plane: the face at x = 5.9 m",
    camera: [-4, 7, 9], target: [3, 1.2, 0], autoRotate: 0.4,
  });
  const c = clouds.wall;
  const q = int16(c.data);
  const pos = new Float32Array(c.count * 3), col = new Float32Array(c.count * 3);
  for (let i = 0; i < c.count; i++) {
    const x = q[i * 3] * c.scale, y = q[i * 3 + 1] * c.scale, z = q[i * 3 + 2] * c.scale;
    pos.set([x, z, -y], i * 3);
    const onWall = x > 5 && x < 7 && Math.abs(y) < 2.2 && z > 0.05 && z < 3;
    col.set(onWall ? [1, 0.36, 0.3] : [0.55, 0.78, 0.6], i * 3);
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.BufferAttribute(pos, 3));
  geo.setAttribute("color", new THREE.BufferAttribute(col, 3));
  scene.add(new THREE.Points(geo, pointMaterial(0.8)));
  const plane = new THREE.Mesh(new THREE.PlaneGeometry(4, 3), new THREE.MeshBasicMaterial({ color: 0xff7a2f, transparent: true, opacity: 0.08, side: THREE.DoubleSide }));
  plane.rotation.y = Math.PI / 2;
  plane.position.set(5.9, 1.5, 0);
  scene.add(plane);
  const edges = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.PlaneGeometry(4, 3)), new THREE.LineBasicMaterial({ color: 0xff7a2f, transparent: true, opacity: 0.6 }));
  edges.rotation.y = Math.PI / 2;
  edges.position.copy(plane.position);
  scene.add(edges);
  const drone = new THREE.Mesh(new THREE.SphereGeometry(0.12, 16, 12), new THREE.MeshBasicMaterial({ color: 0xffffff }));
  drone.position.set(0, 3, 0);
  scene.add(drone);
  const pad = new THREE.Mesh(new THREE.RingGeometry(0.75, 0.8, 64), new THREE.MeshBasicMaterial({ color: 0xf2bd52, side: THREE.DoubleSide }));
  pad.rotation.x = -Math.PI / 2;
  pad.position.y = 0.01;
  scene.add(pad);
}

export function metrics(node) {
  const sc = flight.empty.sensorCheck;
  const rows = [
    ["wall distance", `${sc.wall_distance_m.toFixed(4)} m`, `face at 5.9 m · off by ${(sc.distance_error_m * 1000).toFixed(1)} mm (limit 50)`],
    ["normal error", `${sc.normal_error_deg.toFixed(3)}°`, "limit 2°"],
    ["plane residual", `${(sc.plane_residual_m * 100).toFixed(2)} cm`, `${(sc.plane_residual_m / 0.02).toFixed(2)}× the 2 cm range noise (0.5–1.5×)`],
    ["camera agreement", `${(sc.camera_agreement * 100).toFixed(2)}%`, `${sc.camera_points.toLocaleString("en")} wall returns that landed on red · limit 95%`],
  ];
  node.append(...rows.map(([k, v, s]) => h(`<div class="metric pass"><div class="k">${k}</div><div class="v">${v}</div><div class="s">${s}</div></div>`)));
  node.classList.add("metrics");
}

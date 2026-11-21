// A small three.js stage for point clouds: orbit, auto-rotate, render only while visible.
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { h } from "./data.js";

export function stage(root, { title, meta, hud, camera: eye, target, autoRotate = 0.6 }) {
  root.innerHTML = `<div class="w-title"><p>${title}</p><p class="w-meta">${meta}</p></div>`;
  const box = h(`<div class="stage"><canvas></canvas><div class="hud">${hud}</div></div>`);
  root.append(box);
  const canvas = box.querySelector("canvas");
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(45, 4 / 3, 0.05, 300);
  camera.position.set(...eye);
  const orbit = new OrbitControls(camera, canvas);
  orbit.target.set(...target);
  orbit.enableDamping = true;
  orbit.autoRotate = autoRotate > 0;
  orbit.autoRotateSpeed = autoRotate;
  orbit.enableZoom = false; // the wheel scrolls the page
  const resize = () => {
    renderer.setSize(box.clientWidth, box.clientHeight, false);
    camera.aspect = box.clientWidth / box.clientHeight;
    camera.updateProjectionMatrix();
  };
  resize();
  addEventListener("resize", resize);
  let visible = false;
  new IntersectionObserver(([e]) => { visible = e.isIntersecting; }).observe(box);
  const hooks = [];
  const loop = () => {
    requestAnimationFrame(loop);
    if (!visible) return;
    orbit.update();
    hooks.forEach((f) => f());
    renderer.render(scene, camera);
  };
  loop();
  return { scene, camera, box, hooks, THREE };
}

export const pointMaterial = (size = 2.2) => new THREE.ShaderMaterial({
  uniforms: { uSize: { value: size * Math.min(devicePixelRatio, 2) } },
  vertexShader: `attribute vec3 color; varying vec3 vC; uniform float uSize;
    void main(){ vec4 mv = modelViewMatrix * vec4(position,1.0); gl_Position = projectionMatrix * mv; gl_PointSize = max(1.6, uSize * (22.0 / -mv.z)); vC = color; }`,
  fragmentShader: `varying vec3 vC; void main(){ vec2 p = gl_PointCoord - .5; float r = dot(p,p); if (r > .25) discard; gl_FragColor = vec4(vC, smoothstep(.25,.05,r)); }`,
  transparent: true, depthWrite: false,
});

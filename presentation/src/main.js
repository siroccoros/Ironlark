import "@fontsource/instrument-serif/400.css";
import "@fontsource/instrument-serif/400-italic.css";
import "@fontsource-variable/geist";
import "@fontsource-variable/geist-mono";
import "./style.css";
import Lenis from "lenis";
import { mountReadings, mountCodecards } from "./lib/reading.js";
import * as lexicon from "./lib/lexicon.js";

const widgets = {
  system: () => import("./widgets/system.js"),
  packets: () => import("./widgets/packets.js"),
  stall: () => import("./widgets/stall.js"),
  frames: () => import("./widgets/frames.js"),
  timeline: () => import("./widgets/timeline.js"),
  motor: () => import("./widgets/motor.js"),
  ride: () => import("./widgets/ride.js"),
  judge: () => import("./widgets/judge.js"),
  schedule: () => import("./widgets/schedule.js"),
  protocol: () => import("./widgets/protocol.js"),
  scan: () => import("./widgets/scan.js"),
  wall: () => import("./widgets/wall.js"),
  tail: () => import("./widgets/tail.js"),
  patches: () => import("./widgets/patches.js"),
  future: () => import("./widgets/future.js"),
  drill: () => import("./widgets/drill.js"),
};

const lenis = new Lenis({ lerp: 0.11, wheelMultiplier: 0.95 });
window.__lenis = lenis;
function raf(time) {
  lenis.raf(time);
  requestAnimationFrame(raf);
}
requestAnimationFrame(raf);

const chapters = [...document.querySelectorAll(".chapter")];

// Chapter numerals behind each heading.
for (const c of chapters) {
  const head = c.querySelector(".chapter-head");
  if (head && c.dataset.index) head.dataset.num = c.dataset.index;
}

// Rail and index.
const rail = document.getElementById("rail");
const overlay = document.getElementById("index-overlay");
const toggle = document.getElementById("index-toggle");
const list = document.createElement("ol");
chapters.forEach((c, i) => {
  const a = document.createElement("a");
  a.href = `#${c.id}`;
  a.innerHTML = `<span class="tick"></span><span class="lbl">${c.dataset.index ?? "00"} · ${c.dataset.chapter}<span class="t">${c.dataset.t ?? ""}</span></span>`;
  a.addEventListener("click", (e) => { e.preventDefault(); go(i); });
  rail.append(a);
  const li = document.createElement("li");
  li.innerHTML = `<a href="#${c.id}"><span class="n">${c.dataset.index ?? "00"}</span><span class="c">${c.dataset.chapter}</span><span class="t">${c.dataset.t ?? ""}</span></a>`;
  li.querySelector("a").addEventListener("click", (e) => { e.preventDefault(); closeIndex(); go(i); });
  list.append(li);
});
const hint = document.createElement("li");
hint.className = "hint";
hint.innerHTML = "J / K: next and previous chapter · G: this index · P: plain words · /: terms · ?: keys · Esc: close";
list.append(hint);
overlay.append(list);

function openIndex() { overlay.hidden = false; lenis.stop(); }
function closeIndex() { overlay.hidden = true; lenis.start(); }
toggle.addEventListener("click", () => (overlay.hidden ? openIndex() : closeIndex()));
overlay.addEventListener("click", (e) => { if (e.target === overlay) closeIndex(); });

let current = 0;
let heading = null; // the chapter a J/K scroll is on its way to
function go(i) {
  heading = Math.max(0, Math.min(chapters.length - 1, i));
  lenis.scrollTo(chapters[heading], { duration: 1.4, easing: (t) => 1 - Math.pow(1 - t, 4), onComplete: () => { heading = null; } });
}
addEventListener("keydown", (e) => {
  if (e.target.closest("input, textarea") || e.ctrlKey || e.metaKey || e.altKey) return;
  if (e.key === "Escape") { if (!lexicon.close() && !(lexicon.keysOpen() && (lexicon.toggleKeys(false), true))) closeIndex(); }
  else if (e.key === "p" || e.key === "P") lexicon.setPlain(!document.documentElement.classList.contains("plainmode"), lenis);
  else if (e.key === "/") { e.preventDefault(); closeIndex(); lexicon.open("/"); }
  else if (e.key === "?") lexicon.toggleKeys();
  else if (e.key === "g" || e.key === "G") (overlay.hidden ? openIndex() : closeIndex());
  else if (e.key === "j" || e.key === "J") go((heading ?? current) + 1);
  else if (e.key === "k" || e.key === "K") go((heading ?? current) - 1);
});

const bar = document.querySelector("#progress div");
lenis.on("scroll", ({ scroll, limit }) => {
  bar.style.transform = `scaleX(${limit ? scroll / limit : 0})`;
  const mid = scroll + innerHeight * 0.35;
  let idx = 0;
  chapters.forEach((c, i) => { if (c.offsetTop <= mid) idx = i; });
  if (idx !== current || !rail.dataset.init) {
    rail.dataset.init = "1";
    current = idx;
    [...rail.children].forEach((a, i) => { a.classList.toggle("active", i === idx); a.classList.toggle("done", i < idx); });
    toggle.querySelector(".ix-num").textContent = chapters[idx].dataset.index ?? "00";
    toggle.querySelector(".ix-name").textContent = chapters[idx].dataset.chapter;
  }
});

// Code.
mountCodecards();
mountReadings(lenis);

// Reveal headings and blocks as they arrive.
const revealables = document.querySelectorAll(".chapter-head > *, .prose > p, .widget, .codecard, .cards .card, .gallery figure, .legend, .subhead, .finding");
revealables.forEach((n) => n.classList.add("rv"));
const ro = new IntersectionObserver((entries) => {
  for (const e of entries) if (e.isIntersecting) { e.target.classList.add("in"); ro.unobserve(e.target); }
}, { rootMargin: "0px 0px -8% 0px" });
revealables.forEach((n) => ro.observe(n));

// Every widget mounts at load, so chapters keep their positions while J/K scrolls; each renders only while visible.
const mounted = [...document.querySelectorAll("[data-widget]")].map((n) => {
  const name = n.dataset.widget;
  return widgets[name]?.().then((m) => m.default(n, { lenis })).catch((err) => console.error(name, err));
});
// The second layer marks terms once every widget's text (the drill's answers among them) is on the page.
Promise.all(mounted).then(() => lexicon.mount({ lenis, chapters }));
document.querySelectorAll("[data-metrics]").forEach((n) => import("./widgets/wall.js").then((m) => m.metrics(n)));

// The cover's point cloud.
import("./widgets/hero.js").then((m) => m.default(document.getElementById("hero-canvas"), { lenis }))
  .finally(() => setTimeout(() => document.getElementById("loader").classList.add("gone"), 250));

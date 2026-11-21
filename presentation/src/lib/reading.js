// Code panels: excerpts from src/generated/code.js (the repository's own lines, highlighted at build time) shown
// with their real line numbers. A .reading pairs scrolling notes with a sticky panel: the note nearest the reading line
// is active and its lines (data-lines="a-b,c") are lit, the panel scrolling them into view.
import code from "../generated/code.js";
import raw from "../content/literal.txt?raw";
import { parse, read } from "./literal-core.js";

// What each line literally does, shown in a strip at the panel's foot while a line is hovered.
const literal = parse(raw);
const readings = {};
function readingsOf(id, panelRoot) {
  readings[id] ??= read(literal[id], [...panelRoot.querySelectorAll(".ln .lc")].map((n) => n.textContent), code[id].file).lines;
  return readings[id];
}
const IDLE = "Hover a line to see what it literally does.";
addEventListener("pointerover", (e) => {
  const ln = e.target.closest?.(".ln");
  const panelRoot = e.target.closest?.(".code-panel");
  if (!panelRoot) return;
  const strip = panelRoot.querySelector(".code-strip");
  if (!strip) return;
  const r = ln && readingsOf(panelRoot.dataset.id, panelRoot)[[...ln.parentNode.children].indexOf(ln)];
  strip.classList.toggle("live", !!r);
  strip.classList.toggle("idle", !r);
  strip.querySelector(".n").textContent = r ? `L${ln.dataset.n}` : "";
  strip.querySelector(".r").textContent = r || IDLE;
});
addEventListener("pointerout", (e) => {
  const panelRoot = e.target.closest?.(".code-panel");
  if (!panelRoot || panelRoot.contains(e.relatedTarget)) return;
  const strip = panelRoot.querySelector(".code-strip");
  if (!strip) return;
  strip.classList.remove("live"); strip.classList.add("idle");
  strip.querySelector(".n").textContent = ""; strip.querySelector(".r").textContent = IDLE;
});

export function panel(id) {
  const e = code[id];
  if (!e) throw new Error(`no excerpt ${id}`);
  const root = document.createElement("div");
  root.className = "code-panel";
  const end = e.start + e.lines.length - 1;
  root.innerHTML = `<div class="code-head"><span class="lights"><i></i><i></i><i></i></span>
    <span class="path">${e.file}</span>${e.upstream ? '<span class="tag">upstream</span>' : ""}
    <span class="range">L${e.start}–${end}</span></div>
    <div class="code-body"><pre>${e.lines.map((l, i) => `<span class="ln" data-n="${e.start + i}"><span class="lc">${l || " "}</span></span>`).join("")}</pre></div>
    <div class="code-strip idle" aria-live="polite"><span class="n"></span><span class="r">${IDLE}</span></div>`;
  root.dataset.id = id;
  return root;
}

export function parseLines(spec) {
  const out = new Set();
  for (const part of spec.split(",")) {
    const [a, b] = part.split("-").map(Number);
    for (let n = a; n <= (b || a); n++) out.add(n);
  }
  return out;
}

export function focus(root, lines, smooth = true) {
  const body = root.querySelector(".code-body");
  let first = null, last = null;
  root.querySelectorAll(".ln").forEach((ln) => {
    const on = lines.has(Number(ln.dataset.n));
    ln.classList.toggle("hl", on);
    if (on) { first ??= ln; last = ln; }
  });
  root.classList.toggle("focused", lines.size > 0);
  if (first) {
    const span = last.offsetTop + last.offsetHeight - first.offsetTop;
    const top = span > body.clientHeight * 0.8 ? first.offsetTop - 24 : first.offsetTop - (body.clientHeight - span) / 2;
    body.scrollTo({ top: Math.max(0, top), behavior: smooth ? "smooth" : "auto" });
  }
}

export function mountCodecards() {
  document.querySelectorAll(".codecard").forEach((card) => {
    const p = panel(card.dataset.code);
    card.append(...p.childNodes);
    card.classList.add("code-panel");
    card.dataset.id = card.dataset.code;
    if (card.dataset.lines) focus(card, parseLines(card.dataset.lines), false);
  });
}

export function mountReadings(lenis) {
  const readings = [...document.querySelectorAll(".reading")];
  const state = readings.map((r) => {
    const notes = [...r.querySelectorAll(".note")];
    const wrap = document.createElement("div");
    wrap.className = "notes";
    wrap.append(...notes);
    const slot = document.createElement("div");
    slot.className = "code-slot";
    r.append(wrap, slot);
    for (const n of notes) {
      n.dataset.code ??= r.dataset.code;
      const chip = document.createElement("span");
      chip.className = "lref";
      const id = n.dataset.code;
      const file = code[id].file.split("/").pop();
      chip.textContent = `${file} : ${n.dataset.lines.replace("-", "–")}`;
      n.prepend(chip);
      n.addEventListener("mouseenter", () => activate(s, n));
    }
    const s = { r, notes, slot, panel: null, active: null };
    show(s, r.dataset.code);
    return s;
  });

  function show(s, id) {
    if (s.panel?.dataset.id === id) return;
    const next = panel(id);
    if (s.panel) {
      const old = s.panel;
      old.animate([{ opacity: 1 }, { opacity: 0 }], { duration: 180, fill: "forwards" }).finished.then(() => old.remove());
      next.animate([{ opacity: 0, transform: "translateY(10px)" }, { opacity: 1, transform: "none" }], { duration: 380, easing: "cubic-bezier(.2,.7,.2,1)" });
    }
    s.slot.append(next);
    s.panel = next;
  }

  function activate(s, note) {
    if (s.active === note) return;
    s.active?.classList.remove("active");
    s.active = note;
    note.classList.add("active");
    const switching = s.panel?.dataset.id !== note.dataset.code;
    show(s, note.dataset.code);
    focus(s.panel, parseLines(note.dataset.lines), !switching);
  }

  const update = () => {
    const line = innerHeight * 0.48;
    for (const s of state) {
      const box = s.r.getBoundingClientRect();
      if (box.bottom < 0 || box.top > innerHeight) continue;
      let best = null, bestD = Infinity;
      for (const n of s.notes) {
        const b = n.getBoundingClientRect();
        const d = Math.abs((b.top + b.bottom) / 2 - line);
        if (d < bestD) { bestD = d; best = n; }
      }
      if (best) activate(s, best);
    }
  };
  lenis.on("scroll", update);
  addEventListener("resize", update);
  setTimeout(update, 100);
}

// The second layer: plain words (P), the terms underlined in the text and the drawer that explains them (/ lists them
// all), a key card (?), and a one-time note that says these exist. Nothing here moves the page until it is asked for.
import TERMS from "../content/terms.js";

const byId = Object.fromEntries(TERMS.map((t) => [t.id, t]));
const store = {
  get(k) { try { return localStorage.getItem(k); } catch { return null; } },
  set(k, v) { try { localStorage.setItem(k, v); } catch { /* private window */ } },
};

// ── marking ───────────────────────────────────────────────
const BLOCKS = [".note > p", ".note .pl p", ".prose p", ".prose.narrow", ".lede", ".card li", ".cover-caption", ".qa p", ".q-plain p", ".lx-text"]
  .join(", ");
const spellings = TERMS.flatMap((t) => t.m.map((s) => [s, t.id])).sort((a, b) => b[0].length - a[0].length);
const exact = new Map(spellings.map(([s, id]) => [s.replace(/&nbsp;/g, "\u00a0"), id]));
const escape = (s) => s.replace(/[.*+?^${}()|[\]\\]/g, "\\$&").replace(/&nbsp;/g, "[\\u00a0 ]");
const pattern = new RegExp(`(?<![\\w/+\\-])(${spellings.map(([s]) => escape(s)).join("|")})(?![\\w])`, "g");
const idOf = (text) => exact.get(text) ?? exact.get(text.replace(/ /g, "\u00a0")) ??
  spellings.find(([s]) => new RegExp(`^${escape(s)}$`).test(text))?.[1];

function wrap(node, id, text) {
  const s = document.createElement("span");
  s.className = "term";
  s.dataset.t = id;
  s.tabIndex = 0;
  s.setAttribute("role", "button");
  if (text != null) s.textContent = text; else s.append(node.cloneNode(true));
  return s;
}

// Marks the first mention of each term in a block (in a chapter, for common terms). Inside <code>, only a code element
// that is exactly a term's spelling is marked, so paths and expressions stay intact.
export function mark(root, { skip } = {}) {
  // Spans written by hand in the lexicon's own text, first, so the walk below leaves them alone.
  root.querySelectorAll("span[data-t]:not(.term, .chip, .chip-row)").forEach((s) => {
    if (!byId[s.dataset.t]) return;
    s.className = "term"; s.tabIndex = 0; s.setAttribute("role", "button");
  });
  const chapterSeen = new Map();
  root.querySelectorAll(BLOCKS).forEach((block) => {
    if (block.closest(".code-panel, .widget, .term")) return;
    const chapter = block.closest(".chapter, #lexicon")?.id ?? "";
    const seenC = chapterSeen.get(chapter) ?? new Set(), seen = new Set();
    chapterSeen.set(chapter, seenC);
    const plain = !!block.closest(".pl");
    const fresh = (id) => {
      const t = byId[id];
      if (id === skip || seen.has(id) || (t.common && seenC.has(`${plain}:${id}`))) return false;
      seen.add(id); seenC.add(`${plain}:${id}`);
      return true;
    };
    const walker = document.createTreeWalker(block, NodeFilter.SHOW_ELEMENT | NodeFilter.SHOW_TEXT, {
      acceptNode: (n) => n.nodeType === 1
        ? (n.matches(".term, a, button, kbd, sub, sup") ? NodeFilter.FILTER_REJECT : NodeFilter.FILTER_SKIP)
        : NodeFilter.FILTER_ACCEPT,
    });
    const texts = [];
    while (walker.nextNode()) texts.push(walker.currentNode);
    const codes = new Set();
    for (const node of texts) {
      const parent = node.parentElement;
      if (parent.closest("code")) {
        const code = parent.closest("code");
        if (codes.has(code)) continue;
        codes.add(code);
        const id = idOf(code.textContent);
        if (id && fresh(id)) code.replaceWith(wrap(code, id));
        continue;
      }
      const value = node.nodeValue;
      pattern.lastIndex = 0;
      let m, last = 0;
      const frag = document.createDocumentFragment();
      while ((m = pattern.exec(value))) {
        const id = idOf(m[1]);
        if (!id || !fresh(id)) continue;
        frag.append(value.slice(last, m.index), wrap(null, id, m[1]));
        last = m.index + m[1].length;
      }
      if (last) { frag.append(value.slice(last)); node.replaceWith(frag); }
    }
  });
}

// ── the drawer ────────────────────────────────────────────
let drawer, body, search, history = [], order = [];

function chaptersOf(id) {
  const out = new Map();
  document.querySelectorAll(`main .term[data-t="${id}"]`).forEach((s) => {
    const c = s.closest(".chapter");
    if (c && !out.has(c.id)) out.set(c.id, c);
  });
  return [...out.values()];
}

function card(id) {
  const t = byId[id];
  const n = order.indexOf(id);
  const part = (k, label) => t[k] ? `<section><h5>${label}</h5><p class="lx-text">${t[k]}</p></section>` : "";
  const where = chaptersOf(id);
  body.innerHTML = `
    <p class="lx-k">term${n >= 0 ? ` · ${n + 1} of ${order.length}` : ""}</p>
    <h3 class="lx-name">${t.name}</h3>
    <p class="lx-short lx-text">${t.short}</p>
    ${part("problem", "The problem")}${part("idea", "The idea")}${part("here", "In Ironlark")}
    ${t.see?.length ? `<div class="lx-see"><h5>See also</h5>${t.see.filter((s) => byId[s]).map((s) =>
      `<span class="chip" data-t="${s}" role="button" tabindex="0">${byId[s].name}</span>`).join("")}</div>` : ""}
    ${where.length ? `<div class="lx-where"><h5>Appears in</h5>${where.map((c) =>
      `<a href="#${c.id}" data-go="${c.id}"><span>${c.dataset.index ?? "00"}</span>${c.dataset.chapter}</a>`).join("")}</div>` : ""}`;
  mark(body, { skip: id });
  body.scrollTop = 0;
}

function list(query = "") {
  const q = query.trim().toLowerCase();
  // Names first, then spellings, then descriptions; page order within each.
  const rank = (t) => !q ? 0 : t.name.toLowerCase().startsWith(q) ? 1 : t.name.toLowerCase().includes(q) ? 2
    : t.m.some((s) => s.toLowerCase().includes(q)) ? 3 : t.short.toLowerCase().includes(q) ? 4 : 0;
  const ids = order.filter((id) => !q || rank(byId[id])).sort((a, b) => rank(byId[a]) - rank(byId[b]));
  body.innerHTML = `<p class="lx-k">${ids.length} terms${q ? "" : ", in the order the page introduces them"}</p>
    <ol class="lx-list">${ids.map((id) => `<li><span class="chip-row" data-t="${id}" role="button" tabindex="0">
      <b>${byId[id].name}</b><span>${byId[id].short}</span></span></li>`).join("")}</ol>`;
  if (!ids.length) body.innerHTML += `<p class="lx-empty">Nothing matches "${query.replace(/</g, "&lt;")}".</p>`;
}

function render() {
  const top = history.at(-1);
  drawer.classList.toggle("listing", top === "/");
  drawer.querySelector(".lx-back").hidden = history.length < 2;
  if (top === "/") list(search.value); else card(top);
  document.querySelectorAll(".term.on").forEach((s) => s.classList.remove("on"));
}

export function open(what, from) {
  if (history.at(-1) !== what) history.push(what);
  if (drawer.hidden) {
    drawer.hidden = false;
    requestAnimationFrame(() => drawer.classList.add("shown"));
  }
  render();
  if (from) from.classList.add("on");
  if (what === "/") setTimeout(() => search.focus({ preventScroll: true }), 60);
  used();
}

export function close() {
  if (drawer.hidden) return false;
  drawer.classList.remove("shown");
  setTimeout(() => { drawer.hidden = true; }, 320);
  history = [];
  document.querySelectorAll(".term.on").forEach((s) => s.classList.remove("on"));
  return true;
}

// ── plain words ───────────────────────────────────────────
function twinOf(el) {
  if (el.matches(".pl")) return el.previousElementSibling;
  return el.nextElementSibling?.matches(".pl") ? el.nextElementSibling : el;
}

export function setPlain(on, lenis) {
  const root = document.documentElement;
  if (root.classList.contains("plainmode") === on) return;
  // Keep the block being read where it is on screen.
  const line = innerHeight * 0.42;
  let anchor = null, best = Infinity;
  document.querySelectorAll("main .note, main p, main ul.plain").forEach((el) => {
    const b = el.getBoundingClientRect();
    if (!b.height) return;
    const d = Math.abs(b.top - line);
    if (d < best && b.top < innerHeight && b.bottom > 0) { best = d; anchor = el; }
  });
  const before = anchor?.getBoundingClientRect().top;
  root.classList.toggle("plainmode", on);
  store.set("irl.plain", on ? "1" : "0");
  const target = anchor && (anchor.matches(".note") ? anchor : twinOf(anchor));
  if (target && before != null) {
    const delta = target.getBoundingClientRect().top - before;
    if (Math.abs(delta) > 1) lenis.scrollTo(lenis.scroll + delta, { immediate: true, force: true });
  }
  document.querySelectorAll("main .note, main .pl, main .lede, main .prose p").forEach((el) => {
    const b = el.getBoundingClientRect();
    if (b.height && b.bottom > 0 && b.top < innerHeight) {
      el.animate([{ opacity: 0.15, filter: "blur(2px)" }, { opacity: 1, filter: "none" }], { duration: 420, easing: "cubic-bezier(.2,.7,.2,1)" });
    }
  });
  document.querySelectorAll("[data-plain-toggle]").forEach((b) => b.setAttribute("aria-pressed", on));
  dispatchEvent(new CustomEvent("plainmode", { detail: on }));
  used();
}

// ── the guide ─────────────────────────────────────────────
let whisper, keys;
function used() {
  store.set("irl.layer", "1");
  if (whisper && !whisper.hidden) hideWhisper();
}
function hideWhisper() {
  whisper.classList.remove("shown");
  setTimeout(() => { whisper.hidden = true; }, 400);
}

export function mount({ lenis, chapters }) {
  mark(document.querySelector("main"));
  order = [...new Set([...document.querySelectorAll("main .term")].map((s) => s.dataset.t))];
  order.push(...TERMS.map((t) => t.id).filter((id) => !order.includes(id)));

  drawer = document.getElementById("lexicon");
  drawer.innerHTML = `<header><button class="lx-back" aria-label="Back" hidden>←</button>
      <input class="lx-search" type="search" placeholder="search terms" aria-label="Search terms" spellcheck="false">
      <button class="lx-all" aria-label="All terms" title="All terms (/)">/</button>
      <button class="lx-close" aria-label="Close" title="Close (Esc)">×</button></header>
    <div class="lx-body" data-lenis-prevent></div>`;
  body = drawer.querySelector(".lx-body");
  search = drawer.querySelector(".lx-search");
  search.addEventListener("input", () => { if (history.at(-1) !== "/") history.push("/"); render(); });
  search.addEventListener("keydown", (e) => {
    if (e.key === "Enter") body.querySelector("[data-t]")?.click();
    if (e.key === "Escape") { e.stopPropagation(); close(); }
  });
  drawer.querySelector(".lx-back").addEventListener("click", () => { history.pop(); render(); });
  drawer.querySelector(".lx-all").addEventListener("click", () => { search.value = ""; open("/"); });
  drawer.querySelector(".lx-close").addEventListener("click", close);
  body.addEventListener("click", (e) => {
    const go = e.target.closest("[data-go]");
    if (go) { e.preventDefault(); close(); const i = chapters.findIndex((c) => c.id === go.dataset.go); lenis.scrollTo(chapters[i], { duration: 1.2 }); }
  });

  // Any term, anywhere: open its card. Captured, so a term inside a drill card does not also flip the card.
  const activate = (e) => {
    const t = e.target.closest(".term, .chip, .chip-row");
    if (!t || !byId[t.dataset.t]) return;
    e.preventDefault(); e.stopPropagation();
    open(t.dataset.t, t.closest("main") ? t : null);
  };
  document.addEventListener("click", activate, true);
  document.addEventListener("keydown", (e) => { if (e.key === "Enter" && e.target.closest?.(".term, .chip, .chip-row")) activate(e); }, true);
  document.addEventListener("pointerdown", (e) => {
    if (!drawer.hidden && !drawer.contains(e.target) && !e.target.closest(".term, #dock")) close();
  });

  // Dock buttons.
  document.querySelectorAll("[data-plain-toggle]").forEach((b) => b.addEventListener("click", () =>
    setPlain(!document.documentElement.classList.contains("plainmode"), lenis)));
  document.querySelectorAll("[data-terms]").forEach((b) => b.addEventListener("click", () => (drawer.hidden ? open("/") : close())));
  document.querySelectorAll("[data-keys]").forEach((b) => b.addEventListener("click", () => toggleKeys()));

  keys = document.getElementById("keys");
  whisper = document.getElementById("whisper");
  whisper.querySelector("button").addEventListener("click", () => { store.set("irl.layer", "1"); hideWhisper(); });

  if (store.get("irl.plain") === "1") setPlain(true, lenis);

  // The one-time note: when the reader first reaches chapter 01, unless they have already used the layer.
  if (!store.get("irl.layer")) {
    const io = new IntersectionObserver((es) => {
      if (!es.some((e) => e.isIntersecting)) return;
      io.disconnect();
      whisper.hidden = false;
      requestAnimationFrame(() => whisper.classList.add("shown"));
      setTimeout(() => { if (!whisper.hidden) { store.set("irl.layer", "1"); hideWhisper(); } }, 16000);
    }, { rootMargin: "0px 0px -60% 0px" });
    io.observe(chapters[1]);
  }
}

export function toggleKeys(force) {
  const on = force ?? keys.hidden;
  if (on) { keys.hidden = false; requestAnimationFrame(() => keys.classList.add("shown")); used(); }
  else { keys.classList.remove("shown"); setTimeout(() => { keys.hidden = true; }, 250); }
}
export const keysOpen = () => keys && !keys.hidden;

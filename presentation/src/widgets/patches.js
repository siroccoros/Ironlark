// The 22 Webots patches, with what each changes and what it measured (README.md's table, read at build time).
import patches from "../generated/patches.js";
import { h } from "../lib/data.js";

const md = (s) => s.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/`([^`]+)`/g, "<code>$1</code>")
  .replace(/\[([^\]]+)\]\([^)]+\)/g, "$1").replace(/\*\*([^*]+)\*\*/g, "<strong>$1</strong>");

export default function patchGrid(root) {
  root.innerHTML = `<div class="w-title"><p>native/patches/ · applied in order to Webots R2025a c6793d8 · click one</p><p class="w-meta">measured on a GTX 1060 3 GB, i5-4670</p></div>`;
  const grid = h(`<div class="patches"></div>`);
  root.append(grid);
  for (const p of patches) {
    const card = h(`<div class="patch"><div class="pn">${p.n}</div><div class="pt">${p.name}</div><div class="pm">${md(p.measured)}</div><div class="pd">${md(p.changes)}</div></div>`);
    card.addEventListener("click", () => card.classList.toggle("open"));
    grid.append(card);
  }
}

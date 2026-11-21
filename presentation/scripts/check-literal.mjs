// Coverage of src/content/literal.txt against the excerpts: entries that match no line (stale), and code lines with
// no reading. Run after scripts/code.mjs.
import { readFileSync } from "node:fs";
import code from "../src/generated/code.js";
import { parse, read } from "../src/lib/literal-core.js";

const table = parse(readFileSync(new URL("../src/content/literal.txt", import.meta.url), "utf8"));
const text = (html) => html.replace(/<[^>]+>/g, "").replace(/&lt;/g, "<").replace(/&gt;/g, ">").replace(/&quot;/g, '"')
  .replace(/&#39;/g, "'").replace(/&amp;/g, "&");
let stale = 0, bare = 0, total = 0;
for (const [id, e] of Object.entries(code)) {
  const texts = e.lines.map(text);
  const { lines, unused } = read(table[id], texts, e.file);
  for (const u of unused) { console.log(`stale  ${id}: ${u}`); stale++; }
  lines.forEach((r, i) => {
    if (!texts[i].trim()) return;
    total++;
    if (!r) { console.log(`bare   ${id}:${e.start + i}  ${texts[i].trim().slice(0, 70)}`); bare++; }
  });
}
console.log(`${total} lines, ${bare} without a reading, ${stale} stale entries`);
process.exit(stale ? 1 : 0);

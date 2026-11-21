// What each code line literally does, from src/content/literal.txt. Entries match lines in order by the start of their
// trimmed text, looking a few entries ahead so one stale entry cannot shift the rest; comments, docstrings and lone
// brackets get a generic reading.
export function parse(raw) {
  const table = {};
  let cur = null;
  for (const line of raw.split("\n")) {
    if (line.startsWith("## ")) { cur = table[line.slice(3).trim()] = []; continue; }
    if (!cur || !line.trim() || line.startsWith("# ")) continue;
    const at = line.indexOf(" ⇒ ");
    if (at > 0) cur.push([line.slice(0, at).trim(), line.slice(at + 3).trim()]);
  }
  return table;
}

const COMMENT = "A comment: written for people reading the code; the program ignores it.";
const DOC = "A docstring: the function's own description, for people. It does not change what runs.";
const CLOSE = "Closes the bracket opened above.";
const OPEN = "Opens a group; it closes further down.";

// texts: the excerpt's lines as plain text. Returns one reading (or null) per line, and the entries never used.
export function read(entries = [], texts, file = "") {
  const python = /\.py$|^ironlark$/.test(file.split("/").pop());
  const out = [];
  const used = new Set();
  let j = 0, inDoc = false, inBlock = false;
  for (const raw of texts) {
    const t = raw.trim();
    if (!t) { out.push(null); continue; }
    if (python && (inDoc || t.startsWith('"""'))) {
      const quotes = t.split('"""').length - 1;
      if (inDoc || quotes) { if (quotes % 2) inDoc = !inDoc; out.push(DOC); continue; }
    }
    if (inBlock || (t.startsWith("/*") && !t.includes("*/"))) {
      inBlock = !t.includes("*/");
      out.push(COMMENT);
      continue;
    }
    let hit = -1;
    for (let k = j; k < Math.min(entries.length, j + 6); k++) {
      if (t.startsWith(entries[k][0])) { hit = k; break; }
    }
    if (hit >= 0) { out.push(entries[hit][1]); used.add(hit); j = hit + 1; continue; }
    if (/^(#|\/\/|\/\*|\*)/.test(t) && !t.startsWith("#VRML")) out.push(COMMENT);
    else if (/^[\])}]+[,;:)]*$/.test(t)) out.push(CLOSE);
    else if (/^[[({]$/.test(t)) out.push(OPEN);
    else out.push(null);
  }
  return { lines: out, unused: entries.filter((_, k) => !used.has(k)).map((e) => e[0]) };
}

export function bytes(b64) {
  const s = atob(b64);
  const out = new Uint8Array(s.length);
  for (let i = 0; i < s.length; i++) out[i] = s.charCodeAt(i);
  return out;
}

export const int16 = (b64) => new Int16Array(bytes(b64).buffer);

// ENU (x east, y north, z up) to three.js (y up): (x, z, -y).
export function enuToThree(q, scale, out = new Float32Array(q.length)) {
  for (let i = 0; i < q.length; i += 3) {
    out[i] = q[i] * scale;
    out[i + 1] = q[i + 2] * scale;
    out[i + 2] = -q[i + 1] * scale;
  }
  return out;
}

export function lerpSeries(rows, t, col) {
  // rows sorted by rows[i][0]; linear value of column col at time t.
  let lo = 0, hi = rows.length - 1;
  if (t <= rows[0][0]) return rows[0][col];
  if (t >= rows[hi][0]) return rows[hi][col];
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (rows[mid][0] <= t) lo = mid; else hi = mid;
  }
  const a = rows[lo], b = rows[hi];
  const f = (t - a[0]) / (b[0] - a[0] || 1);
  return a[col] + (b[col] - a[col]) * f;
}

export function stepSeries(rows, t, col = 1) {
  let v = rows[0]?.[col];
  for (const r of rows) { if (r[0] <= t) v = r[col]; else break; }
  return v;
}

export function svg(tag, attrs = {}, parent) {
  const n = document.createElementNS("http://www.w3.org/2000/svg", tag);
  for (const [k, v] of Object.entries(attrs)) n.setAttribute(k, v);
  parent?.append(n);
  return n;
}

export function h(html) {
  const t = document.createElement("template");
  t.innerHTML = html.trim();
  return t.content.firstElementChild;
}

export const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

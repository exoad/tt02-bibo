// The 2D view: the last revolution around the car, the hub's radar in
// miniature. The sensor's zero is the car's front and it points UP; bearings
// run clockwise, which is the screen's own sense. Nothing here animates: a
// frame is drawn on new data or a resize, and the reason for having no data
// is drawn as large as the data would have been.

import * as T from './theme.js';

const MIN_RANGE_MM = 3000;

let canvas = null, ctx = null, W = 0, H = 0;
let rangeMm = MIN_RANGE_MM, smallerFor = 0;
let lastScan = null, lastGone = 'no answer';

export function init(c) {
  canvas = c;
  ctx = c.getContext('2d');
  resize();
}

export function resize() {
  const s = T.fitCanvas(canvas, ctx);
  W = s.w; H = s.h;
  render();
}

// `scan` is the parsed revolution or null; `gone` says why when it is null.
export function draw(scan, gone) {
  lastScan = scan; lastGone = gone;
  updateRange();
  render();
}

export function range() { return rangeMm; }

function ringStep(mm) { return mm > 10000 ? 5000 : mm > 5000 ? 2000 : 1000; }

function updateRange() {
  const far = lastScan ? lastScan.far : 0;
  const step = ringStep(far);
  const want = Math.max(MIN_RANGE_MM, Math.ceil(far / step) * step);
  // Grow at once - a wall that just appeared must be on the page - but shrink
  // only after it has been smaller for about two seconds (twenty revolutions),
  // or the picture breathes with every dropped return at the far wall.
  if (want >= rangeMm) { rangeMm = want; smallerFor = 0; }
  else if (++smallerFor > 20) { rangeMm = want; smallerFor = 0; }
}

function arrow(x0, y0, angRad, len, head) {
  // From (x0, y0) along a bearing (0 = up, clockwise), with a barbed head.
  const tx = x0 + len * Math.sin(angRad), ty = y0 - len * Math.cos(angRad);
  const b1 = angRad + 2.618, b2 = angRad - 2.618;   // 150 degrees back from the tip
  ctx.beginPath();
  ctx.moveTo(x0, y0); ctx.lineTo(tx, ty);
  ctx.moveTo(tx + head * Math.sin(b1), ty - head * Math.cos(b1)); ctx.lineTo(tx, ty);
  ctx.lineTo(tx + head * Math.sin(b2), ty - head * Math.cos(b2));
  ctx.stroke();
}

function render() {
  if (!ctx || W < 40 || H < 40) return;
  const cx = W / 2, cy = H / 2;
  const R = Math.min(W, H) / 2 - 18;      // the rim; room outside for 0 and 180
  const s = R / rangeMm;                  // px per mm

  ctx.fillStyle = T.BG; ctx.fillRect(0, 0, W, H);
  ctx.font = '12px ' + T.MONO; ctx.lineWidth = 1;

  // Range rings, labelled inside so the outer label stays on the page, and the
  // four bearings at the rim.
  const step = ringStep(rangeMm);
  ctx.strokeStyle = T.GRID; ctx.fillStyle = T.INK_DIM; ctx.textAlign = 'right';
  for (let r = step; r <= rangeMm; r += step) {
    ctx.beginPath(); ctx.arc(cx, cy, r * s, 0, 2 * Math.PI); ctx.stroke();
    ctx.fillText((r / 1000) + ' m', cx + r * s * 0.707 - 4, cy - r * s * 0.707 + 14);
  }
  ctx.beginPath();
  ctx.moveTo(cx, cy - R); ctx.lineTo(cx, cy + R);
  ctx.moveTo(cx - R, cy); ctx.lineTo(cx + R, cy);
  ctx.stroke();
  ctx.textAlign = 'center';
  ctx.fillText('0', cx, cy - R - 5);
  ctx.fillText('180', cx, cy + R + 13);
  ctx.textAlign = 'left'; ctx.fillText('270', cx - R + 4, cy - 4);
  ctx.textAlign = 'right'; ctx.fillText('90', cx + R - 4, cy - 4);

  const scan = lastScan;
  const d = scan ? scan.drive : null;
  const hw = T.HALF_WIDTH_MM * s;
  if (d) {
    // The corridor: what the pilot looks down, two lines ahead at +-halfWidth.
    // Only while a pilot is deciding - without one the picture is the lidar's,
    // and a corridor would claim a reasoning nobody is doing.
    ctx.strokeStyle = T.INK_DIM; ctx.beginPath();
    ctx.moveTo(cx - hw, cy); ctx.lineTo(cx - hw, cy - R);
    ctx.moveTo(cx + hw, cy); ctx.lineTo(cx + hw, cy - R);
    ctx.stroke();
  }

  if (scan) {
    ctx.fillStyle = T.INK;
    const n = scan.n, a = scan.a, dd = scan.d;
    for (let i = 0; i < n; i++) {
      const r = dd[i] * s;
      ctx.fillRect(cx + r * Math.sin(a[i]) - 1, cy - r * Math.cos(a[i]) - 1, 2, 2);
    }
    if (scan.near >= 0) {
      // The nearest return, ringed and labelled, as the hub marks it: the one
      // number a person behind the car wants without reading anything.
      const r = dd[scan.near] * s;
      const nx = cx + r * Math.sin(a[scan.near]), ny = cy - r * Math.cos(a[scan.near]);
      ctx.strokeStyle = T.BAD; ctx.lineWidth = 1.5; ctx.beginPath();
      ctx.arc(nx, ny, 7, 0, 2 * Math.PI); ctx.stroke();
      ctx.fillStyle = T.BAD; ctx.textAlign = nx > W - 70 ? 'right' : 'left';
      ctx.fillText(dd[scan.near] + ' mm', nx > W - 70 ? nx - 10 : nx + 10, ny + 4);
    }
    if (d) {
      const col = T.decisionColor(d);
      // The clearance: a bar across the corridor at the distance the pilot acted on.
      if (d.clearMm > 0 && d.clearMm <= rangeMm) {
        ctx.strokeStyle = col; ctx.lineWidth = 2; ctx.beginPath();
        ctx.moveTo(cx - hw - 4, cy - d.clearMm * s);
        ctx.lineTo(cx + hw + 4, cy - d.clearMm * s);
        ctx.stroke();
      }
      // The heading: where the wheels point, at full lock = STEER_DEG.
      ctx.strokeStyle = col; ctx.lineWidth = 2;
      arrow(cx, cy, d.steer * T.STEER_DEG * Math.PI / 180, R * 0.45, 9);
    }
  }

  // The car, nose up, at the scan's origin. To scale, but never smaller than
  // a thumbnail can be pointed at.
  const cw = Math.max(8, T.CAR_MM[0] * s), ch = Math.max(16, T.CAR_MM[1] * s);
  ctx.fillStyle = T.INK; ctx.fillRect(cx - cw / 2, cy - ch / 2, cw, ch);
  ctx.fillStyle = T.BG; ctx.fillRect(cx - cw / 2, cy - ch / 2 + ch * 0.25, cw, 1);   // the windscreen

  // The front, always: the hub's heading arrow, in the hub's aqua, from the
  // nose straight up. Scan or no scan, the page must say which way is forward.
  ctx.strokeStyle = T.ACCENT_DIM; ctx.lineWidth = 2;
  arrow(cx, cy - ch / 2, 0, Math.max(ch * 0.9, R * 0.3), 7);

  if (!scan) reason(cx, cy + R * 0.42);
}

function reason(cx, top) {
  // Nothing to draw is drawn as nothing, plus the reason, big: below the car,
  // where no ring label lives, each line on a patch of ground so no ring cuts
  // it. Wrapped at words and shrunk only until it fits in three lines.
  const text = lastGone.toUpperCase();
  let size = Math.min(26, Math.max(14, Math.floor(W / 16)));
  let lines = wrap(text, size);
  while (lines.length > 3 && size > 14) { size -= 3; lines = wrap(text, size); }
  ctx.font = 'bold ' + size + 'px ' + T.MONO; ctx.textAlign = 'center';
  for (let i = 0; i < lines.length; i++) {
    const y = top + i * size * 1.3;
    const tw = ctx.measureText(lines[i]).width;
    ctx.fillStyle = T.BG; ctx.fillRect(cx - tw / 2 - 8, y - size, tw + 16, size * 1.35);
    ctx.fillStyle = T.INK; ctx.fillText(lines[i], cx, y);
  }
}

function wrap(text, size) {
  const maxChars = Math.max(8, Math.floor((W - 24) / (size * 0.62)));
  const words = text.split(' '), out = [];
  let line = '';
  for (let i = 0; i < words.length; i++) {
    const next = line ? line + ' ' + words[i] : words[i];
    if (line && next.length > maxChars) { out.push(line); line = words[i]; }
    else line = next;
  }
  if (line) out.push(line);
  return out;
}

// The 2D view for the field: the hub's radar (hub/src/radar.cxx, Points
// mode) drawn for a phone held at arm's length in daylight. The sensor's
// zero is the car's front and it points UP; bearings run clockwise, which is
// the screen's own sense. Range rings a metre apart and a compass ring at
// the fit radius, all in the outline tokens - they are furniture; the
// returns are the data, on-surface and three pixels; the nearest return
// ringed in error with its distance in label-large; the heading arrow in
// primary; the car to scale, never smaller than a fingertip. With a D line,
// the pilot overlay from app_ui.cxx drawPilotOverlay: the corridor and the
// clearance bar in primary, the steer arrow in the decision's colour. The
// corner text - feed and host, points and rate, the scale bar, the fit - is
// label-medium in on-surface-variant. The mode and the clearance are the
// DOM's, over the top-left.
//
// The canvas is transparent: the stage behind it is the ground. The circle
// is centred in whatever box the stage has and its radius is half the
// shorter side less an inset, so the view fills a tall or a wide box alike.
// Nothing here animates: a frame is drawn on new data or a resize.

import * as T from './theme.js';

const MIN_RANGE_MM = 3000;              // never tighter than three metres: a wall at 2 m must not fill the screen
const RING_LABEL_BEARING = 25;          // deg; every ring label sits on this bearing, a column
const INSET = 20;                       // px outside the compass ring

let canvas = null, ctx = null, W = 0, H = 0;
let rangeMm = MIN_RANGE_MM, smallerFor = 0;
let lastScan = null, lastHud = null;
let showSector = false;     // the Drive destination's forward sector

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

// `scan` is the parsed revolution or null. When it is null only the car and
// the heading are drawn: the reason is the page's mode word, not the map's.
export function draw(scan, hud) {
  lastScan = scan; lastHud = hud;
  updateRange();
  render();
}

export function range() { return rangeMm; }

// Drawn only while the Drive destination is showing: the sector the "ahead"
// number is measured over, so the number and the picture agree.
export function setSector(on) {
  if (on === showSector) return;
  showSector = on;
  render();
}

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

function formatRing(mm) { return (mm / 1000).toFixed(0) + ' m'; }

// An arrow with a soft glow under it: a wide dim pass under a narrow bright
// one, an open head of two strokes swept back 26 deg. From (x0, y0) along a
// bearing; `headK` is the head's length as a fraction of the shaft's.
function arrow(x0, y0, ang, len, th, col, headK) {
  const tx = x0 + len * Math.sin(ang), ty = y0 - len * Math.cos(ang);
  const head = len * headK;
  const strokes = function () {
    ctx.beginPath();
    ctx.moveTo(x0, y0); ctx.lineTo(tx, ty);
    for (let s = -1; s <= 1; s += 2) {
      const back = ang + Math.PI + s * 26 * Math.PI / 180;
      ctx.moveTo(tx, ty); ctx.lineTo(tx + head * Math.sin(back), ty - head * Math.cos(back));
    }
    ctx.stroke();
  };
  ctx.strokeStyle = col; ctx.lineCap = 'round'; ctx.lineJoin = 'round';
  ctx.globalAlpha = 0.18; ctx.lineWidth = th * 3; strokes();
  ctx.globalAlpha = 1; ctx.lineWidth = th; strokes();
  ctx.lineCap = 'butt'; ctx.lineJoin = 'miter';
}

function render() {
  if (!ctx || W < 40 || H < 40) return;
  const C = T.color;
  const cx = W / 2, cy = H / 2;
  const R = Math.min(W, H) / 2 - INSET;   // the compass ring
  const ppm = R / rangeMm;                // px per mm

  ctx.clearRect(0, 0, W, H);
  ctx.lineWidth = 1; ctx.textBaseline = 'alphabetic'; ctx.textAlign = 'left';

  // Radials every 45 deg out to the compass ring, the two axes a touch stronger.
  for (let b = 0; b < 360; b += 45) {
    const a = b * Math.PI / 180;
    ctx.strokeStyle = b % 90 === 0 ? C.outline : C.outlineVariant;
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx + R * Math.sin(a), cy - R * Math.cos(a)); ctx.stroke();
  }

  // Range rings, major every fifth.
  const step = ringStep(rangeMm);
  for (let i = 1, r = step; r <= rangeMm + 1; i++, r += step) {
    ctx.strokeStyle = i % 5 === 0 ? C.outline : C.outlineVariant;
    ctx.beginPath(); ctx.arc(cx, cy, r * ppm, 0, 2 * Math.PI); ctx.stroke();
  }

  // The compass ring at the fit radius, ticked every 15 deg, longer every 45.
  ctx.strokeStyle = C.outline;
  ctx.beginPath(); ctx.arc(cx, cy, R, 0, 2 * Math.PI); ctx.stroke();
  for (let b = 0; b < 360; b += 15) {
    const a = b * Math.PI / 180, sx = Math.sin(a), sy = -Math.cos(a);
    const major = b % 45 === 0, len = major ? 7 : 4;
    ctx.strokeStyle = major ? C.outline : C.outlineVariant;
    ctx.beginPath();
    ctx.moveTo(cx + sx * (R - len), cy + sy * (R - len)); ctx.lineTo(cx + sx * (R + len), cy + sy * (R + len));
    ctx.stroke();
  }

  // The forward sector, under everything: a faint primary wedge of the same
  // half-angle drive.js measures its "ahead" number over. Canvas angles run
  // from +x and clockwise on screen, so bearing b is b - 90 deg here.
  if (showSector) {
    const lim = T.AHEAD_DEG * Math.PI / 180;
    ctx.fillStyle = C.primary;
    ctx.globalAlpha = 0.10;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.arc(cx, cy, R, -Math.PI / 2 - lim, -Math.PI / 2 + lim);
    ctx.closePath();
    ctx.fill();
    ctx.globalAlpha = 0.45; ctx.strokeStyle = C.primary; ctx.lineWidth = 1.5;
    ctx.beginPath();
    for (let s = -1; s <= 1; s += 2) {
      ctx.moveTo(cx, cy);
      ctx.lineTo(cx + R * Math.sin(s * lim), cy - R * Math.cos(s * lim));
    }
    ctx.stroke();
    ctx.globalAlpha = 1; ctx.lineWidth = 1;
  }

  const scan = lastScan;
  const d = scan ? scan.drive : null;

  // The pilot overlay, under the points as the hub draws it: the corridor to
  // its horizon at primary 12 %, its edges and the clearance bar in primary.
  // Not for blind - a corridor drawn from no measurement is a picture of a
  // scan the pilot did not have.
  const overlay = d && d.mode !== 'blind';
  if (overlay) {
    const hw = T.HALF_WIDTH_MM * ppm;
    const yTop = cy - Math.max(T.HORIZON_MM, d.clearMm) * ppm;
    const yClear = cy - d.clearMm * ppm;
    if (hw >= 2) {
      ctx.fillStyle = C.primary; ctx.globalAlpha = 0.12;
      ctx.fillRect(cx - hw, yTop, hw * 2, cy - yTop);
      ctx.strokeStyle = C.primary; ctx.globalAlpha = 0.75; ctx.lineWidth = 2; ctx.beginPath();
      ctx.moveTo(cx - hw, cy); ctx.lineTo(cx - hw, yTop);
      ctx.moveTo(cx + hw, cy); ctx.lineTo(cx + hw, yTop);
      ctx.stroke();
      ctx.globalAlpha = 1; ctx.lineWidth = 4; ctx.lineCap = 'round'; ctx.beginPath();
      ctx.moveTo(cx - hw, yClear); ctx.lineTo(cx + hw, yClear); ctx.stroke();
      ctx.lineCap = 'butt'; ctx.lineWidth = 1;
    }
  }

  // The points: on-surface, three pixels, one fill each.
  if (scan) {
    ctx.fillStyle = C.onSurface;
    const n = scan.n, a = scan.a, dd = scan.d;
    for (let i = 0; i < n; i++) {
      const r = dd[i] * ppm;
      ctx.fillRect(cx + r * Math.sin(a[i]) - 1.5, cy - r * Math.cos(a[i]) - 1.5, 3, 3);
    }
  }

  // The heading: which way is forward, always, from the sensor straight up.
  // Two metres of the world, kept between two screen sizes.
  arrow(cx, cy, 0, Math.min(R * 0.5, Math.max(2000 * ppm, 56)), 3.5, C.primary, 0.18);

  // The steer arrow: half a metre of the world, swung by the steering fraction
  // at full lock, over the heading so the decision reads on top of the fact.
  if (overlay) {
    arrow(cx, cy, d.steer * T.STEER_DEG * Math.PI / 180, Math.min(110, Math.max(40, 600 * ppm)), 4, T.pilotColor(d), 0.32);
  }

  // The car, nose up, to the TT-02's plan at the scan's scale - never smaller
  // than a fingertip. A rounded box in primary-container with an on-surface
  // edge, a windscreen line a quarter back from the nose.
  const cw = Math.max(16, T.CAR_MM[0] * ppm), ch = Math.max(32, T.CAR_MM[1] * ppm);
  T.rrect(ctx, cx - cw / 2, cy - ch / 2, cw, ch, Math.min(5, cw / 4));
  ctx.fillStyle = C.primaryContainer; ctx.fill();
  ctx.strokeStyle = C.onSurface; ctx.lineWidth = 1.5; ctx.stroke();
  ctx.beginPath(); ctx.moveTo(cx - cw / 2, cy - ch * 0.25); ctx.lineTo(cx + cw / 2, cy - ch * 0.25); ctx.stroke();
  // The sensor itself: a primary hub with an on-surface core.
  ctx.fillStyle = C.primary; ctx.beginPath(); ctx.arc(cx, cy, 4, 0, 2 * Math.PI); ctx.fill();
  ctx.fillStyle = C.onSurface; ctx.beginPath(); ctx.arc(cx, cy, 1.6, 0, 2 * Math.PI); ctx.fill();

  // The nearest return, ringed and labelled, over everything: the one number
  // a person behind the car wants without reading anything.
  if (scan && scan.near >= 0) {
    const r = scan.d[scan.near] * ppm;
    const nx = cx + r * Math.sin(scan.a[scan.near]), ny = cy - r * Math.cos(scan.a[scan.near]);
    ctx.strokeStyle = C.error; ctx.lineWidth = 2.5; ctx.beginPath();
    ctx.arc(nx, ny, 11, 0, 2 * Math.PI); ctx.stroke();
    // The label on the side away from the car, so it never crosses the
    // heading or the steer arrow; the other side when that would leave the
    // screen.
    ctx.font = T.LABEL_LARGE;
    const label = T.metres(scan.d[scan.near]), lw = ctx.measureText(label).width + 17;
    let left = nx < cx;
    if (left && nx - lw < 4) left = false;
    if (!left && nx + lw > W - 4) left = true;
    ctx.textAlign = left ? 'right' : 'left';
    T.haloText(ctx, left ? nx - 17 : nx + 17, ny + 5, label, C.error);
    ctx.textAlign = 'left';
  }

  // ---- labels, on-surface-variant, over the picture -------------------------
  ctx.font = T.LABEL_MEDIUM;

  // Ring distances, a column at one bearing, only when the rings are far
  // enough apart for the column not to collide with itself.
  if (step * ppm > 30) {
    const a = RING_LABEL_BEARING * Math.PI / 180;
    for (let r = step; r <= rangeMm + 1; r += step) {
      const x = cx + Math.sin(a) * r * ppm, y = cy - Math.cos(a) * r * ppm;
      T.haloText(ctx, x + 4, y + 4, formatRing(r), C.onSurfaceVariant);
    }
  }

  // Bearing numbers, inside the compass ring: 0 in primary, the rest variant.
  if (R > 60) {
    const rl = R - 20;
    ctx.textAlign = 'center';
    for (let b = 0; b < 360; b += 45) {
      const a = b * Math.PI / 180;
      const x = cx + Math.sin(a) * rl, y = cy - Math.cos(a) * rl;
      T.haloText(ctx, x, y + 4, String(b), b === 0 ? C.primary : C.onSurfaceVariant);
    }
    ctx.textAlign = 'left';
  }

  // The corners: what the page knows about the feed and the board top right,
  // the scale bar bottom left, the fit bottom right.
  const hud = lastHud || {};
  const barMm = step, barPx = barMm * ppm;
  T.corners(ctx, W, H, { tr: hud.tr, br: ['fit ' + formatRing(2 * rangeMm) + ' across'] });
  if (barPx > 24 && barPx < W * 0.6) {
    const bx = 12, by = H - 12;
    ctx.strokeStyle = C.onSurfaceVariant; ctx.lineWidth = 2; ctx.beginPath();
    ctx.moveTo(bx, by); ctx.lineTo(bx + barPx, by);
    ctx.moveTo(bx, by - 5); ctx.lineTo(bx, by + 0.5);
    ctx.moveTo(bx + barPx, by - 5); ctx.lineTo(bx + barPx, by + 0.5);
    ctx.stroke(); ctx.lineWidth = 1;
    ctx.font = T.LABEL_MEDIUM;
    T.haloText(ctx, bx, by - 9, formatRing(barMm), C.onSurfaceVariant);
  }
  ctx.lineWidth = 1;
}

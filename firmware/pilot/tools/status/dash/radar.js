// The 2D view: the hub's radar (hub/src/radar.cxx, Points mode) on a canvas.
// The sensor's zero is the car's front and it points UP; bearings run
// clockwise, which is the screen's own sense. Black inside the frame, the
// axes through the sensor, range rings a metre apart with their distances in
// a column at bearing 25 deg, a compass ring at the fit radius ticked every
// 5 deg with the bearing numbers inside it, white points, the nearest return
// ringed in red with its millimetres, the emissive cyan heading arrow, the
// car to scale, a map scale bottom-left and the HUD from theme.js. With a D
// line, the pilot overlay from app_ui.cxx drawPilotOverlay: the corridor,
// the clearance bar and the steer arrow, in the accent or the warning.
//
// Nothing here animates: a frame is drawn on new data or a resize.

import * as T from './theme.js';

const MIN_RANGE_MM = 3000;
const RING_LABEL_BEARING = 25;          // deg; every ring label sits on this bearing, a column

let canvas = null, ctx = null, W = 0, H = 0;
let rangeMm = MIN_RANGE_MM, smallerFor = 0;
let lastScan = null, lastGone = 'no answer', lastHud = null;

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

// `scan` is the parsed revolution or null; `gone` says why when it is null;
// `hud` is app.js's summary for the overlay text.
export function draw(scan, gone, hud) {
  lastScan = scan; lastGone = gone; lastHud = hud;
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

// 1, 2 or 5 times a power of ten, rounded DOWN: the scale bar's drawn length
// must never exceed its budget.
function niceStepDown(mm) {
  const p = Math.pow(10, Math.floor(Math.log10(mm)));
  const m = mm / p;
  return (m >= 5 ? 5 : m >= 2 ? 2 : 1) * p;
}

function formatRing(mm) { return (mm / 1000).toFixed(1) + ' m'; }

// An emissive arrow, the hub's: a wide dim pass under a narrow bright one, an
// open head of two strokes swept back 26 deg. From (x0, y0) along a bearing;
// `headK` is the head's length as a fraction of the shaft's.
function arrow(x0, y0, ang, len, th, color, headK) {
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
  ctx.strokeStyle = color; ctx.lineCap = 'round';
  ctx.globalAlpha = 0.13; ctx.lineWidth = th * 3.2; strokes();
  ctx.globalAlpha = 1; ctx.lineWidth = th; strokes();
  ctx.lineCap = 'butt';
}

function render() {
  if (!ctx || W < 40 || H < 40) return;
  const cx = W / 2, cy = H / 2;
  const R = Math.min(W, H) / 2 - 18;      // the compass ring; room outside it for nothing but air
  const ppm = R / rangeMm;                // px per mm

  ctx.fillStyle = T.ansi.BLACK; ctx.fillRect(0, 0, W, H);
  ctx.lineWidth = 1; ctx.textBaseline = 'alphabetic';

  // The axes, across the whole widget: the one furniture drawn under the rings.
  ctx.strokeStyle = T.map.AXIS; ctx.beginPath();
  ctx.moveTo(cx, 0); ctx.lineTo(cx, H); ctx.moveTo(0, cy); ctx.lineTo(W, cy); ctx.stroke();

  // Range rings, major every fifth, at the grid's two alphas.
  const step = ringStep(rangeMm);
  for (let i = 1, r = step; r <= rangeMm + 1; i++, r += step) {
    const major = i % 5 === 0;
    ctx.strokeStyle = major ? T.map.GRID_MAJOR : T.map.GRID;
    ctx.globalAlpha = major ? 0.65 : 0.6;
    ctx.beginPath(); ctx.arc(cx, cy, r * ppm, 0, 2 * Math.PI); ctx.stroke();
  }
  ctx.globalAlpha = 1;

  // The compass ring at the fit radius, ticked every 5 deg, longer every 45.
  ctx.strokeStyle = T.map.GRID_MAJOR; ctx.globalAlpha = 0.65;
  ctx.beginPath(); ctx.arc(cx, cy, R, 0, 2 * Math.PI); ctx.stroke();
  ctx.globalAlpha = 1;
  for (let b = 0; b < 360; b += 5) {
    const a = b * Math.PI / 180, sx = Math.sin(a), sy = -Math.cos(a);
    const major = b % 45 === 0, len = major ? 6 : 3;
    ctx.strokeStyle = major ? T.map.GRID_MAJOR : T.map.TICK;
    ctx.beginPath();
    ctx.moveTo(cx + sx * (R - len), cy + sy * (R - len)); ctx.lineTo(cx + sx * (R + len), cy + sy * (R + len));
    ctx.stroke();
  }

  const scan = lastScan;
  const d = scan ? scan.drive : null;

  // The pilot overlay, under the points as the hub draws it: the corridor to
  // its horizon, the clearance bar where the Nth-nearest return was. Not for
  // blind - a corridor drawn from no measurement is a picture of a scan the
  // pilot did not have.
  const overlay = d && d.mode !== 'blind';
  if (overlay) {
    const col = T.pilotColor(d);
    const hw = T.HALF_WIDTH_MM * ppm;
    const yTop = cy - Math.max(T.HORIZON_MM, d.clearMm) * ppm;
    const yClear = cy - d.clearMm * ppm;
    if (hw >= 1.5) {
      ctx.strokeStyle = col; ctx.globalAlpha = 0.69; ctx.lineWidth = 1.4; ctx.beginPath();
      ctx.moveTo(cx - hw, cy); ctx.lineTo(cx - hw, yTop);
      ctx.moveTo(cx + hw, cy); ctx.lineTo(cx + hw, yTop);
      ctx.stroke();
      ctx.globalAlpha = 1; ctx.lineWidth = 2.2; ctx.beginPath();
      ctx.moveTo(cx - hw, yClear); ctx.lineTo(cx + hw, yClear); ctx.stroke();
    }
  }

  // The points: white, two pixels, one fill each.
  if (scan) {
    ctx.fillStyle = T.map.POINT;
    const n = scan.n, a = scan.a, dd = scan.d;
    for (let i = 0; i < n; i++) {
      const r = dd[i] * ppm;
      ctx.fillRect(cx + r * Math.sin(a[i]) - 1, cy - r * Math.cos(a[i]) - 1, 2, 2);
    }
  }

  // The heading: which way is forward, always, from the sensor straight up.
  // Two metres of the world, kept between two screen sizes.
  arrow(cx, cy, 0, Math.min(R * 0.45, Math.max(2000 * ppm, 36)), 2, T.map.HEADING, 0.16);

  // The steer arrow: half a metre of the world, swung by the steering fraction
  // at full lock, over the heading so the decision reads on top of the fact.
  if (overlay) {
    arrow(cx, cy, d.steer * T.STEER_DEG * Math.PI / 180, Math.min(80, Math.max(24, 500 * ppm)), 2, T.pilotColor(d), 0.30);
  }

  // The car, nose up, to the TT-02's plan at the scan's scale - never smaller
  // than a thumbnail can be pointed at.
  const cw = Math.max(8, T.CAR_MM[0] * ppm), ch = Math.max(16, T.CAR_MM[1] * ppm);
  ctx.strokeStyle = T.map.LABEL; ctx.lineWidth = 1;
  ctx.strokeRect(cx - cw / 2 + 0.5, cy - ch / 2 + 0.5, cw - 1, ch - 1);
  ctx.beginPath(); ctx.moveTo(cx - cw / 2, cy - ch * 0.25); ctx.lineTo(cx + cw / 2, cy - ch * 0.25); ctx.stroke();
  // The sensor itself: the cyan hub with a white core.
  ctx.fillStyle = T.map.HEADING; ctx.beginPath(); ctx.arc(cx, cy, 3, 0, 2 * Math.PI); ctx.fill();
  ctx.fillStyle = T.map.LABEL; ctx.beginPath(); ctx.arc(cx, cy, 1.2, 0, 2 * Math.PI); ctx.fill();

  // The nearest return, ringed and labelled, over everything: the one number
  // a person behind the car wants without reading anything.
  if (scan && scan.near >= 0) {
    const r = scan.d[scan.near] * ppm;
    const nx = cx + r * Math.sin(scan.a[scan.near]), ny = cy - r * Math.cos(scan.a[scan.near]);
    ctx.strokeStyle = T.map.NEAREST; ctx.lineWidth = 1.5; ctx.beginPath();
    ctx.arc(nx, ny, 7, 0, 2 * Math.PI); ctx.stroke();
    ctx.font = T.fontMono(T.size.SMALL); ctx.textAlign = nx > W - 80 ? 'right' : 'left';
    ctx.fillStyle = T.map.NEAREST;
    ctx.fillText(scan.d[scan.near] + ' mm', nx > W - 80 ? nx - 11 : nx + 11, ny + 4);
    ctx.textAlign = 'left';
  }

  // ---- labels, on plates, over the picture ----------------------------------
  ctx.font = T.fontUI(T.size.SMALL);

  // Ring distances, a column at one bearing, only when the rings are far
  // enough apart for the column not to collide with itself.
  if (step * ppm > T.size.SMALL * 1.9) {
    const a = RING_LABEL_BEARING * Math.PI / 180;
    for (let r = step; r <= rangeMm + 1; r += step) {
      const x = cx + Math.sin(a) * r * ppm, y = cy - Math.cos(a) * r * ppm;
      if (y < T.HUD_INSET + T.size.SMALL * 4 && x < W * 0.5) continue;   // under the HUD's lines
      T.plateText(ctx, x + 2, y + 4, formatRing(r), T.map.RING_TEXT);
    }
  }

  // Bearing numbers, inside the compass ring: 0 in the heading colour,
  // cardinals in white, the rest grey.
  if (R > 34) {
    const rl = R - 21;
    ctx.textAlign = 'center';
    for (let b = 0; b < 360; b += 45) {
      const a = b * Math.PI / 180;
      const x = cx + Math.sin(a) * rl, y = cy - Math.cos(a) * rl;
      const col = b === 0 ? T.map.HEADING : b % 90 === 0 ? T.map.CARDINAL : T.map.BEARING;
      const text = String(b), w = ctx.measureText(text).width;
      ctx.fillStyle = T.map.PLATE; ctx.fillRect(x - w / 2 - 2, y - T.size.SMALL * 0.45, w + 4, T.size.SMALL * 1.15);
      ctx.fillStyle = col; ctx.fillText(text, x, y + T.size.SMALL * 0.4);
    }
    ctx.textAlign = 'left';
  }

  // The map scale, bottom-left: half-filled like a map's, the midpoint tick
  // the free half-value, and its length on a plate above it.
  {
    const budget = Math.min(W * 0.24, 200);
    if (budget >= 30) {
      const lenMm = niceStepDown(budget / ppm), lenPx = lenMm * ppm;
      if (lenPx > 8) {
        const x0 = T.HUD_INSET + 4, y = H - T.HUD_INSET - 12, x1 = x0 + lenPx, cap = 5;
        ctx.fillStyle = T.map.SCALE; ctx.strokeStyle = T.map.SCALE; ctx.lineWidth = 1.8;
        ctx.fillRect(x0, y - cap / 2, lenPx / 2, cap);
        ctx.strokeRect(x0, y - cap / 2, lenPx, cap);
        ctx.beginPath();
        ctx.moveTo(x0, y - cap * 1.6); ctx.lineTo(x0, y + cap * 1.6);
        ctx.moveTo(x1, y - cap * 1.6); ctx.lineTo(x1, y + cap * 1.6);
        ctx.stroke();
        T.plateText(ctx, x0, y - cap * 1.8 - 3, formatRing(lenMm), T.map.SCALE);
      }
    }
  }

  if (!scan) T.noScan(ctx, cx, cy, Math.max(ch / 2 + T.size.BODY * 1.6, R * 0.3), lastGone);

  if (lastHud) {
    T.drawHud(ctx, W, H, lastHud, 'Points', scan ? scan.n + ' returns' : '',
              'fit   ' + (rangeMm * 2 / 1000).toFixed(1) + ' m across');
  }
}

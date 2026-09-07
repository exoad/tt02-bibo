// The 3D view: the hub's Cloud scene (hub/src/scene3d.cxx) on a canvas. The
// same revolution as returns standing on the ground around a car box at the
// origin, seen from an orbiting camera. World axes are the hub's scene: x
// right, y forward, z up; a lidar bearing b at range d lands on the scan
// plane at (d sin b, d cos b, SCAN_Z) and is drawn as a tick from the ground
// up to it, which is how the hub draws a return that has a height but no
// depth. The ground is range rings and radials, the way the flat map's is,
// with the heading along +y in cyan so which way is forward survives an
// orbit. The default camera is the hub's - behind and above, looking
// forward.
//
// A 2D-canvas perspective projection, not WebGL, on purpose. The scene is
// ~500 ticks, a few rings and a box: a few thousand line calls a frame,
// which a phone's 2D canvas does at 10 Hz without noticing. And the 2D path
// is the one that cannot be missing: a headless browser, a phone in a saver
// mode or an older WebView gives no GL context, or a software one slower
// than this, and a 3D view that is blank in the field would be worse than
// none. One path, so there is no fallback to keep honest.
//
// The hot path - projecting the points - touches only module-level numbers
// and the scan's typed arrays: no objects, closures or arrays per frame.

import * as T from './theme.js';

const NEAR_MM = 60;                     // camera-space depth under which a point is behind the eye
const RING_STEP = 1000, RING_N = 12;    // one metre; twelve out, as far as the hub's ground goes
const FOV_Y = 0.85;                     // radians, the hub's
const DEFAULT_YAW = 0, DEFAULT_PITCH = 0.42, DEFAULT_DIST = 4200;   // the hub's Camera defaults
const TARGET_Z = 120;
const ARROW_MM = 900;                   // the steer arrow's length on the ground

let canvas = null, ctx = null, W = 0, H = 0;
let yaw = DEFAULT_YAW, pitch = DEFAULT_PITCH, dist = DEFAULT_DIST;

// The camera as numbers: the eye and its right/up/forward basis, world frame.
let ex = 0, ey = 0, ez = 0;
let rx = 1, ry = 0, rz = 0, ux = 0, uy = 0, uz = 1, fx = 0, fy = 1, fz = 0;
let focal = 1, hw = 0, hh = 0;
// The last projection's output. A function returning a pair would allocate.
let px = 0, py = 0;

let lastScan = null, lastGone = 'no answer', lastRange = 3000, lastHud = null;
let pending = false;    // a frame is queued; a drag at 60 Hz and data at 10 Hz share it

export function init(c) {
  canvas = c;
  ctx = c.getContext('2d');
  bindInput();
  resize();
}

export function resize() {
  const s = T.fitCanvas(canvas, ctx);
  W = s.w; H = s.h; hw = W / 2; hh = H / 2;
  render();
}

// `scan` is the parsed revolution or null; `rangeMm` is the radar's current
// range, so the corridor is drawn as far as the other view draws it.
export function draw(scan, gone, rangeMm, hud) {
  lastScan = scan; lastGone = gone; lastRange = rangeMm; lastHud = hud;
  schedule();
}

function schedule() {
  if (pending) return;
  pending = true;
  requestAnimationFrame(render);
}

function updateCamera() {
  const cp = Math.cos(pitch), sp = Math.sin(pitch);
  ex = dist * cp * Math.sin(yaw);
  ey = -dist * cp * Math.cos(yaw);
  ez = TARGET_Z + dist * sp;
  fx = -ex; fy = -ey; fz = TARGET_Z - ez;               // toward the target
  const fl = Math.hypot(fx, fy, fz); fx /= fl; fy /= fl; fz /= fl;
  rx = fy; ry = -fx; rz = 0;                             // forward x up(0,0,1)
  const rl = Math.hypot(rx, ry) || 1; rx /= rl; ry /= rl;
  ux = ry * fz - rz * fy; uy = rz * fx - rx * fz; uz = rx * fy - ry * fx;   // right x forward
  focal = hh / Math.tan(FOV_Y / 2);
}

// World point to screen (px, py). False when it is behind the near plane.
function project(x, y, z) {
  const dx = x - ex, dy = y - ey, dz = z - ez;
  const cz = dx * fx + dy * fy + dz * fz;
  if (cz < NEAR_MM) return false;
  const k = focal / cz;
  px = hw + (dx * rx + dy * ry + dz * rz) * k;
  py = hh - (dx * ux + dy * uy + dz * uz) * k;
  return true;
}

// A world segment added to the current path, clipped to the near plane in
// camera space first - a ring running under the eye would otherwise flip
// across the screen.
function line3(x0, y0, z0, x1, y1, z1) {
  let ax = x0 - ex, ay = y0 - ey, az = z0 - ez;
  let bx = x1 - ex, by = y1 - ey, bz = z1 - ez;
  let ca = ax * fx + ay * fy + az * fz, cb = bx * fx + by * fy + bz * fz;
  if (ca < NEAR_MM && cb < NEAR_MM) return;
  if (ca < NEAR_MM) {
    const t = (NEAR_MM - ca) / (cb - ca);
    ax += (bx - ax) * t; ay += (by - ay) * t; az += (bz - az) * t; ca = NEAR_MM;
  } else if (cb < NEAR_MM) {
    const t = (NEAR_MM - cb) / (ca - cb);
    bx += (ax - bx) * t; by += (ay - by) * t; bz += (az - bz) * t; cb = NEAR_MM;
  }
  const ka = focal / ca, kb = focal / cb;
  ctx.moveTo(hw + (ax * rx + ay * ry + az * rz) * ka, hh - (ax * ux + ay * uy + az * uz) * ka);
  ctx.lineTo(hw + (bx * rx + by * ry + bz * rz) * kb, hh - (bx * ux + by * uy + bz * uz) * kb);
}

// A ring on the ground, as 72 chords.
function ring3(r) {
  let lx = 0, ly = r;
  for (let k = 1; k <= 72; k++) {
    const a = k * Math.PI / 36, x = r * Math.sin(a), y = r * Math.cos(a);
    line3(lx, ly, 0, x, y, 0);
    lx = x; ly = y;
  }
}

function arrow3(x0, y0, z0, bearing, len, head) {
  // On a horizontal plane at z0, along a bearing (0 = +y, clockwise).
  const sb = Math.sin(bearing), cb = Math.cos(bearing);
  const tx = x0 + len * sb, ty = y0 + len * cb;
  const b1 = bearing + Math.PI + 26 * Math.PI / 180, b2 = bearing + Math.PI - 26 * Math.PI / 180;
  ctx.beginPath();
  line3(x0, y0, z0, tx, ty, z0);
  line3(tx, ty, z0, tx + head * Math.sin(b1), ty + head * Math.cos(b1), z0);
  line3(tx, ty, z0, tx + head * Math.sin(b2), ty + head * Math.cos(b2), z0);
  ctx.stroke();
}

function render() {
  pending = false;
  if (!ctx || W < 40 || H < 40) return;
  updateCamera();
  ctx.fillStyle = T.ansi.BLACK; ctx.fillRect(0, 0, W, H);
  ctx.textBaseline = 'alphabetic';
  const far = RING_STEP * RING_N;

  // The ground: rings a metre apart, major every fifth, radials every 45 deg
  // with the two axes stronger, all at the grid's alphas.
  ctx.lineWidth = 1;
  ctx.strokeStyle = T.map.GRID; ctx.globalAlpha = 0.69; ctx.beginPath();
  for (let i = 1; i <= RING_N; i++) if (i % 5) ring3(i * RING_STEP);
  for (let b = 45; b < 360; b += 90) {
    const s = Math.sin(b * Math.PI / 180), c = Math.cos(b * Math.PI / 180);
    line3(0, 0, 0, far * s, far * c, 0);
  }
  ctx.stroke();
  ctx.strokeStyle = T.map.GRID_MAJOR; ctx.globalAlpha = 0.5; ctx.beginPath();
  for (let i = 5; i <= RING_N; i += 5) ring3(i * RING_STEP);
  line3(-far, 0, 0, far, 0, 0); line3(0, -far, 0, 0, 0, 0);
  ctx.stroke();
  // The heading along +y, in its own colour, so which way is forward survives an orbit.
  ctx.strokeStyle = T.map.HEADING; ctx.globalAlpha = 0.6; ctx.lineWidth = 1.6;
  ctx.beginPath(); line3(0, 0, 0, 0, far, 0); ctx.stroke();
  ctx.globalAlpha = 1; ctx.lineWidth = 1;

  const scan = lastScan;
  const d = scan ? scan.drive : null;
  const overlay = d && d.mode !== 'blind';
  if (overlay) {
    // The corridor on the ground at +-halfWidth to the pilot's horizon, and the
    // clearance bar across it, in the decision's colour like the flat map's.
    const col = T.pilotColor(d);
    const w = T.HALF_WIDTH_MM, top = Math.max(T.HORIZON_MM, d.clearMm);
    ctx.strokeStyle = col; ctx.globalAlpha = 0.69; ctx.lineWidth = 1.4; ctx.beginPath();
    line3(-w, 0, 0, -w, top, 0); line3(w, 0, 0, w, top, 0);
    ctx.stroke();
    ctx.globalAlpha = 1; ctx.lineWidth = 2.2; ctx.beginPath();
    line3(-w, d.clearMm, 0, w, d.clearMm, 0);
    ctx.stroke();
    ctx.lineWidth = 1;
  }

  if (scan) {
    // Each return a white tick from the ground to the scan plane, with the
    // return itself a dot at the top.
    ctx.strokeStyle = T.map.POINT; ctx.fillStyle = T.map.POINT; ctx.globalAlpha = 0.8;
    ctx.beginPath();
    const n = scan.n, a = scan.a, dd = scan.d;
    for (let i = 0; i < n; i++) {
      const r = dd[i], b = a[i], x = r * Math.sin(b), y = r * Math.cos(b);
      line3(x, y, 0, x, y, T.SCAN_Z_MM);
    }
    ctx.stroke();
    ctx.globalAlpha = 1;
    for (let i = 0; i < n; i++) {
      const r = dd[i], b = a[i];
      if (project(r * Math.sin(b), r * Math.cos(b), T.SCAN_Z_MM)) ctx.fillRect(px - 1, py - 1, 2, 2);
    }
  }

  car();

  if (scan && scan.near >= 0) {
    const r = scan.d[scan.near], b = scan.a[scan.near];
    if (project(r * Math.sin(b), r * Math.cos(b), T.SCAN_Z_MM)) {
      ctx.strokeStyle = T.map.NEAREST; ctx.lineWidth = 1.5; ctx.beginPath();
      ctx.arc(px, py, 6, 0, 2 * Math.PI); ctx.stroke();
      ctx.fillStyle = T.map.NEAREST; ctx.font = T.fontMono(T.size.SMALL);
      ctx.textAlign = px > W - 80 ? 'right' : 'left';
      ctx.fillText(r + ' mm', px > W - 80 ? px - 9 : px + 9, py + 4);
      ctx.textAlign = 'left'; ctx.lineWidth = 1;
    }
  }

  if (overlay) {
    // The steer arrow, above the car's roof so the box does not hide it.
    ctx.strokeStyle = T.pilotColor(d); ctx.lineWidth = 2;
    arrow3(0, 0, T.CAR_HEIGHT_MM + 2, d.steer * T.STEER_DEG * Math.PI / 180, ARROW_MM, ARROW_MM * 0.3);
    ctx.lineWidth = 1;
  }

  const cx = W / 2, cy = H / 2;
  if (!scan) T.noScan(ctx, cx, cy, H * 0.16, lastGone);

  if (lastHud) {
    // The camera, in words, as the hub says it; and how much ground the view
    // spans at the car, as the flat map says its fit.
    const atHome = yaw === DEFAULT_YAW && pitch === DEFAULT_PITCH && dist === DEFAULT_DIST;
    const across = 2 * dist * Math.tan(FOV_Y / 2) * (W / H) / 1000;
    const diag = (scan ? scan.n + ' returns' : 'no returns') + '  |  car lock, orbit ' +
                 Math.round(yaw * 180 / Math.PI) + ' deg, ' + (dist / 1000).toFixed(1) + ' m out';
    T.drawHud(ctx, W, H, lastHud, 'Cloud', diag, (atHome ? 'fit   ' : 'manual   ') + across.toFixed(1) + ' m across');
  }
}

function car() {
  // A box to the TT-02's plan, nose at +y, the roof filled in the hub's car
  // blue so it reads as a solid, white edges, a windscreen line a quarter
  // back from the nose.
  const w = T.CAR_MM[0] / 2, l = T.CAR_MM[1] / 2, h = T.CAR_HEIGHT_MM;
  if (project(-w, l, h)) {
    const x0 = px, y0 = py;
    if (project(w, l, h)) {
      const x1 = px, y1 = py;
      if (project(w, -l, h)) {
        const x2 = px, y2 = py;
        if (project(-w, -l, h)) {
          ctx.fillStyle = T.map.CAR; ctx.beginPath();
          ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.lineTo(x2, y2); ctx.lineTo(px, py);
          ctx.closePath(); ctx.fill();
        }
      }
    }
  }
  ctx.strokeStyle = T.map.LABEL; ctx.lineWidth = 1; ctx.beginPath();
  line3(-w, l, 0, w, l, 0); line3(w, l, 0, w, -l, 0); line3(w, -l, 0, -w, -l, 0); line3(-w, -l, 0, -w, l, 0);
  line3(-w, l, h, w, l, h); line3(w, l, h, w, -l, h); line3(w, -l, h, -w, -l, h); line3(-w, -l, h, -w, l, h);
  line3(-w, l, 0, -w, l, h); line3(w, l, 0, w, l, h); line3(w, -l, 0, w, -l, h); line3(-w, -l, 0, -w, -l, h);
  ctx.stroke();
  ctx.strokeStyle = T.ansi.BLACK; ctx.beginPath(); line3(-w, l * 0.5, h, w, l * 0.5, h); ctx.stroke();
}

// ---- input: drag orbits, wheel or pinch zooms, a double tap resets -------

const pointers = new Map();     // id -> {x, y, x0, y0}
let pinchD = 0, lastTapAt = 0;

function bindInput() {
  canvas.addEventListener('pointerdown', function (e) {
    canvas.setPointerCapture(e.pointerId);
    pointers.set(e.pointerId, { x: e.clientX, y: e.clientY, x0: e.clientX, y0: e.clientY });
    if (pointers.size === 2) pinchD = pinchDist();
    e.preventDefault();
  });
  canvas.addEventListener('pointermove', function (e) {
    const p = pointers.get(e.pointerId);
    if (!p) return;
    if (pointers.size === 1) {
      orbit((e.clientX - p.x) * 0.008, (e.clientY - p.y) * 0.008);
    }
    p.x = e.clientX; p.y = e.clientY;
    if (pointers.size === 2) {
      const nd = pinchDist();
      if (pinchD > 0 && nd > 0) zoom(pinchD / nd);
      pinchD = nd;
    }
  });
  const up = function (e) {
    const p = pointers.get(e.pointerId);
    if (!p) return;
    pointers.delete(e.pointerId);
    pinchD = 0;
    if (pointers.size === 0 && e.type === 'pointerup' && Math.hypot(p.x - p.x0, p.y - p.y0) < 8) {
      const now = Date.now();
      if (now - lastTapAt < 350) reset();
      lastTapAt = now;
    }
  };
  canvas.addEventListener('pointerup', up);
  canvas.addEventListener('pointercancel', up);
  canvas.addEventListener('wheel', function (e) {
    zoom(Math.exp(e.deltaY * 0.0012));
    e.preventDefault();
  }, { passive: false });
}

function pinchDist() {
  let ax = 0, ay = 0, bx = 0, by = 0, i = 0;
  pointers.forEach(function (p) { if (i++ === 0) { ax = p.x; ay = p.y; } else { bx = p.x; by = p.y; } });
  return Math.hypot(bx - ax, by - ay);
}

function orbit(dYaw, dPitch) {
  // The hub's limits: never under the ground, never straight down.
  yaw += dYaw;
  if (yaw > Math.PI) yaw -= 2 * Math.PI;
  if (yaw < -Math.PI) yaw += 2 * Math.PI;
  pitch = Math.min(1.52, Math.max(0.02, pitch + dPitch));
  schedule();
}

function zoom(factor) {
  dist = Math.min(26000, Math.max(400, dist * factor));
  schedule();
}

export function reset() {
  yaw = DEFAULT_YAW; pitch = DEFAULT_PITCH; dist = DEFAULT_DIST;
  schedule();
}

// The 3D view: the same revolution as points on the ground around a car box
// at the origin, seen from an orbiting camera. World axes are the hub's
// scene: x right, y forward, z up; a lidar bearing b at range d lands on the
// ground at (d sin b, d cos b, 0). The default camera is the hub's - behind
// and above, looking forward.
//
// A 2D-canvas perspective projection, not WebGL, on purpose. The scene is
// ~500 points, a grid and a box: a few thousand rect and line calls a frame,
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
const GRID_STEP = 1000, GRID_N = 8;     // one meter; eight out, as far as a room goes
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

let lastScan = null, lastGone = 'no answer', lastRange = 3000;
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
export function draw(scan, gone, rangeMm) {
  lastScan = scan; lastGone = gone; lastRange = rangeMm;
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
// camera space first - a grid line running under the eye would otherwise
// flip across the screen.
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

function arrow3(x0, y0, z0, bearing, len, head) {
  // On a horizontal plane at z0, along a bearing (0 = +y, clockwise).
  const sb = Math.sin(bearing), cb = Math.cos(bearing);
  const tx = x0 + len * sb, ty = y0 + len * cb;
  const b1 = bearing + 2.618, b2 = bearing - 2.618;
  ctx.beginPath();
  line3(x0, y0, z0, tx, ty, z0);
  line3(tx + head * Math.sin(b1), ty + head * Math.cos(b1), z0, tx, ty, z0);
  line3(tx, ty, z0, tx + head * Math.sin(b2), ty + head * Math.cos(b2), z0);
  ctx.stroke();
}

function render() {
  pending = false;
  if (!ctx || W < 40 || H < 40) return;
  updateCamera();
  ctx.fillStyle = T.BG; ctx.fillRect(0, 0, W, H);
  const far = GRID_STEP * GRID_N;

  // The ground: minor lines, major every five, the two axes through the car.
  ctx.lineWidth = 1; ctx.strokeStyle = T.GRID; ctx.beginPath();
  for (let i = -GRID_N; i <= GRID_N; i++) {
    if (i % 5 === 0) continue;
    const t = i * GRID_STEP;
    line3(t, -far, 0, t, far, 0); line3(-far, t, 0, far, t, 0);
  }
  ctx.stroke();
  ctx.strokeStyle = T.GRID_MAJOR; ctx.beginPath();
  for (let i = -GRID_N; i <= GRID_N; i += 5) {
    const t = i * GRID_STEP;
    line3(t, -far, 0, t, far, 0); line3(-far, t, 0, far, t, 0);
  }
  ctx.stroke();
  // The heading along +y in the hub's aqua, so which way is forward survives
  // an orbit.
  ctx.strokeStyle = T.ACCENT_DIM; ctx.beginPath(); line3(0, 0, 0, 0, far, 0); ctx.stroke();

  const scan = lastScan;
  const d = scan ? scan.drive : null;
  if (d) {
    // The corridor: two lines on the ground at +-halfWidth, as far as the radar draws it.
    const w = T.HALF_WIDTH_MM;
    ctx.strokeStyle = T.INK_DIM; ctx.beginPath();
    line3(-w, 0, 0, -w, lastRange, 0); line3(w, 0, 0, w, lastRange, 0);
    ctx.stroke();
  }

  if (scan) {
    ctx.fillStyle = T.INK;
    const n = scan.n, a = scan.a, dd = scan.d;
    for (let i = 0; i < n; i++) {
      const r = dd[i], b = a[i];
      if (project(r * Math.sin(b), r * Math.cos(b), 0)) ctx.fillRect(px - 1, py - 1, 2, 2);
    }
    if (scan.near >= 0) {
      const r = dd[scan.near], b = a[scan.near];
      if (project(r * Math.sin(b), r * Math.cos(b), 0)) {
        ctx.strokeStyle = T.BAD; ctx.lineWidth = 1.5; ctx.beginPath();
        ctx.arc(px, py, 6, 0, 2 * Math.PI); ctx.stroke();
        ctx.fillStyle = T.BAD; ctx.font = '12px ' + T.MONO;
        ctx.textAlign = px > W - 70 ? 'right' : 'left';
        ctx.fillText(r + ' mm', px > W - 70 ? px - 9 : px + 9, py + 4);
      }
    }
    if (d) {
      const col = T.decisionColor(d);
      if (d.clearMm > 0 && d.clearMm <= lastRange) {
        const w = T.HALF_WIDTH_MM + 40;
        ctx.strokeStyle = col; ctx.lineWidth = 2; ctx.beginPath();
        line3(-w, d.clearMm, 0, w, d.clearMm, 0);
        ctx.stroke();
      }
      // The steer arrow, above the car's roof so the box does not hide it.
      ctx.strokeStyle = col; ctx.lineWidth = 2;
      arrow3(0, 0, T.CAR_HEIGHT_MM + 2, d.steer * T.STEER_DEG * Math.PI / 180, ARROW_MM, 120);
    }
  }

  car();

  // The front, always: from the nose, on the ground, in the hub's aqua.
  ctx.strokeStyle = T.ACCENT_DIM; ctx.lineWidth = 2;
  arrow3(0, T.CAR_MM[1] / 2, 0, 0, 500, 90);

  // The camera, in words, as the hub says it.
  ctx.fillStyle = T.INK_DIM; ctx.font = '11px ' + T.MONO; ctx.textAlign = 'left';
  ctx.fillText((scan ? scan.n + ' returns' : 'no scan - ' + lastGone) +
               '  |  orbit ' + Math.round(yaw * 180 / Math.PI) + ' deg, ' +
               (dist / 1000).toFixed(1) + ' m out', 8, H - 8);
}

function car() {
  // A box to the TT-02's plan, nose at +y, with the roof filled so it reads
  // as a solid and a windscreen line a quarter back from the nose.
  const w = T.CAR_MM[0] / 2, l = T.CAR_MM[1] / 2, h = T.CAR_HEIGHT_MM;
  if (project(-w, l, h)) {
    const x0 = px, y0 = py;
    if (project(w, l, h)) {
      const x1 = px, y1 = py;
      if (project(w, -l, h)) {
        const x2 = px, y2 = py;
        if (project(-w, -l, h)) {
          ctx.fillStyle = T.PANEL; ctx.beginPath();
          ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.lineTo(x2, y2); ctx.lineTo(px, py);
          ctx.closePath(); ctx.fill();
        }
      }
    }
  }
  ctx.strokeStyle = T.INK; ctx.lineWidth = 1; ctx.beginPath();
  line3(-w, l, 0, w, l, 0); line3(w, l, 0, w, -l, 0); line3(w, -l, 0, -w, -l, 0); line3(-w, -l, 0, -w, l, 0);
  line3(-w, l, h, w, l, h); line3(w, l, h, w, -l, h); line3(w, -l, h, -w, -l, h); line3(-w, -l, h, -w, l, h);
  line3(-w, l, 0, -w, l, h); line3(w, l, 0, w, l, h); line3(w, -l, 0, w, -l, h); line3(-w, -l, 0, -w, -l, h);
  ctx.stroke();
  ctx.strokeStyle = T.BG; ctx.beginPath(); line3(-w, l * 0.5, h, w, l * 0.5, h); ctx.stroke();
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

function reset() {
  yaw = DEFAULT_YAW; pitch = DEFAULT_PITCH; dist = DEFAULT_DIST;
  schedule();
}

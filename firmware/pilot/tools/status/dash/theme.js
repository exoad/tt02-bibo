// The canvases' side of the design tokens, and the car's constants the views
// draw to scale. A CSS custom property does not reach a canvas context, so
// refresh() reads the M3 tokens from the computed style - once per resize,
// which fitCanvas does - into `color`. Nothing here is a literal colour: the
// map's palette IS the page's, and dash.css is the one place a value lives.
//
// What each token means on the map: points on-surface, rings outline-variant
// (major rings outline), bearings and corner text on-surface-variant, the
// heading primary, the nearest return error, the corridor primary at 12 %,
// the clearance bar primary, the steer arrow the decision's colour: success
// drives, warning backs off or is blind, error halts.

export const color = {
  surface: '', scanBg: '', onSurface: '', onSurfaceVariant: '', outline: '', outlineVariant: '',
  primary: '', primaryContainer: '', secondary: '', error: '', success: '', warning: '',
};

const SYS = {
  surface: 'surface', onSurface: 'on-surface', onSurfaceVariant: 'on-surface-variant',
  outline: 'outline', outlineVariant: 'outline-variant', primary: 'primary',
  primaryContainer: 'primary-container', secondary: 'secondary', error: 'error',
};

// `el` is the element whose computed tokens apply - the stage, whose --scan-bg
// changes with the window class. Falls back to the document root.
export function refresh(el) {
  const root = getComputedStyle(document.documentElement);
  const here = el ? getComputedStyle(el) : root;
  for (const k in SYS) color[k] = root.getPropertyValue('--md-sys-color-' + SYS[k]).trim();
  color.success = root.getPropertyValue('--md-custom-color-success').trim();
  color.warning = root.getPropertyValue('--md-custom-color-warning').trim();
  color.scanBg = here.getPropertyValue('--scan-bg').trim() || color.surface;
}

// The M3 type scale, as canvas fonts. Roboto is on every Android; nothing fetched.
export const FAMILY = 'Roboto, "Segoe UI", system-ui, sans-serif';
export function fontUI(px, weight) { return (weight || 400) + ' ' + px + 'px ' + FAMILY; }
export const LABEL_LARGE = fontUI(14, 500);
export const LABEL_MEDIUM = fontUI(12, 500);

export const HALF_WIDTH_MM = 160;       // reactive::Config::halfWidthMm - the corridor the pilot reasons in
export const HORIZON_MM = 2500;         // reactive::Config::clearMm - beyond this nothing is in the way
export const STEER_DEG = 30;            // PILOT_STEER_LOCK_DEG: full lock, as the heading arrow draws it
export const CAR_MM = [186, 442];       // vehicle::CAR_WID_MM, CAR_LEN_MM - the TT-02 across the tires
export const CAR_HEIGHT_MM = 140;       // vehicle::CAR_HEIGHT_MM
export const SCAN_Z_MM = 167;           // vehicle::C1_SCAN_Z_MM: the plane the returns are in, the 3D tick's top
export const STALE_S = 3;               // the server's rule for /scan, applied here to /json as well

// The forward sector the Drive view watches, degrees either side of straight
// ahead. The number a person driving by hand actually wants is not the global
// nearest - that is as often a wall beside the car - but the nearest thing IN
// FRONT, and it is the same number reactive avoidance will act on later. Read
// by drive.js for the "ahead" readout and by radar.js to draw the wedge, so
// the number and the picture cannot disagree.
export const AHEAD_DEG = 30;

// A decision's tone, keyed on the mode's name: cruise and slow drive
// (success), stop halts (error), reverse and blind back off or see nothing
// (warning). The same word colours the mode, the stat values and the steer
// arrow, so one glance says the same thing everywhere. 'variant' is
// on-surface-variant: the lidar alone, nobody deciding.
export function modeTone(mode) {
  if (mode === 'cruise' || mode === 'slow') return 'success';
  if (mode === 'stop') return 'error';
  if (mode === 'reverse' || mode === 'blind') return 'warning';
  return 'surface';
}
export function toneColor(tone) {
  if (tone === 'success') return color.success;
  if (tone === 'warning') return color.warning;
  if (tone === 'error') return color.error;
  if (tone === 'variant') return color.onSurfaceVariant;
  return color.onSurface;
}
export function pilotColor(drive) { return toneColor(modeTone(drive.mode)); }

// Sizes a canvas to its CSS box at device pixels, capped at 2x: a phone's 3x
// panel would triple the fill for dots that are still three pixels. Rereads
// the tokens while it is at it. Returns the CSS size to draw in. Setting
// width resets the context, hence the transform.
export function fitCanvas(canvas, ctx) {
  refresh(canvas.parentElement);
  const w = canvas.clientWidth, h = canvas.clientHeight;
  const dpr = Math.min(2, window.devicePixelRatio || 1);
  const pw = Math.round(w * dpr), ph = Math.round(h * dpr);
  if (canvas.width !== pw || canvas.height !== ph) { canvas.width = pw; canvas.height = ph; }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { w: w, h: h };
}

// Text with a halo of the ground under it, so a ring or a point never cuts a
// glyph and nothing needs a plate behind it.
export function haloText(ctx, x, y, text, col) {
  ctx.lineJoin = 'round'; ctx.lineWidth = 4; ctx.strokeStyle = color.scanBg;
  ctx.strokeText(text, x, y);
  ctx.fillStyle = col;
  ctx.fillText(text, x, y);
  ctx.lineWidth = 1; ctx.lineJoin = 'miter';
}

// A rounded rectangle path; roundRect is not on every phone's canvas yet.
export function rrect(ctx, x, y, w, h, r) {
  r = Math.min(r, w / 2, h / 2);
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.lineTo(x + w - r, y); ctx.arcTo(x + w, y, x + w, y + r, r);
  ctx.lineTo(x + w, y + h - r); ctx.arcTo(x + w, y + h, x + w - r, y + h, r);
  ctx.lineTo(x + r, y + h); ctx.arcTo(x, y + h, x, y + h - r, r);
  ctx.lineTo(x, y + r); ctx.arcTo(x, y, x + r, y, r);
  ctx.closePath();
}

// The corner text every view carries, label-medium in on-surface-variant, no
// plates: `hud` is {tl: [], tr: [], bl: [], br: []}, lines from the corner
// inward. The top-left corner is the DOM's (the mode word), so views leave
// it empty.
export function corners(ctx, W, H, hud) {
  if (!hud) return;
  ctx.font = LABEL_MEDIUM; ctx.textBaseline = 'alphabetic';
  const lh = 16, pad = 12;
  const put = function (lines, right, bottom) {
    if (!lines || !lines.length) return;
    ctx.textAlign = right ? 'right' : 'left';
    for (let i = 0; i < lines.length; i++) {
      const y = bottom ? H - pad - 4 - (lines.length - 1 - i) * lh : pad + 12 + i * lh;
      haloText(ctx, right ? W - pad : pad, y, lines[i], color.onSurfaceVariant);
    }
  };
  put(hud.tl, false, false); put(hud.tr, true, false); put(hud.bl, false, true); put(hud.br, true, true);
  ctx.textAlign = 'left';
}

// Metres with one decimal ("1.6 m"), two under a metre ("0.33 m") so the
// nearest thing is never rounded to nothing.
export function metres(mm) { return (mm / 1000).toFixed(mm < 1000 ? 2 : 1) + ' m'; }

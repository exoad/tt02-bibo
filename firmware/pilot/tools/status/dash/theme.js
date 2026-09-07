// The page's palette, once, for the two canvases - a CSS variable does not
// reach a canvas context - and the car's constants the views draw to scale.
// dash.css carries the same values for the DOM.
//
// A field screen's set: a near-black neutral ground, white for what is
// measured, one accent for what is the car's own (the heading, the active
// control), and three state colours that mean one thing everywhere - green
// drives, amber backs off or is blind, red halts or is missing. Nothing on
// the map is any other colour.

export const ui = {
  bg: '#0b0f14',
  text: '#ffffff',
  muted: 'rgba(255,255,255,0.6)',
  accent: '#4f8cff',
  good: '#34d399', warn: '#fbbf24', bad: '#f87171',
};

// What each one means on the map.
export const map = {
  GRID: 'rgba(255,255,255,0.10)', GRID_MAJOR: 'rgba(255,255,255,0.18)',
  TICK: 'rgba(255,255,255,0.14)', LABEL: 'rgba(255,255,255,0.45)',
  HEADING: ui.accent, NEAREST: ui.bad, POINT: '#ffffff',
  CAR: '#ffffff', CAR_FILL: 'rgba(79,140,255,0.28)',
};

export const sem = { GOOD: ui.good, WARN: ui.warn, BAD: ui.bad, MUTED: ui.muted };

// The phone's own sans, Inter first when it has it; mono only for the log.
export const UI = 'Inter, system-ui, -apple-system, "Segoe UI", Roboto, "Helvetica Neue", sans-serif';
export const MONO = 'ui-monospace, "Cascadia Mono", Consolas, Menlo, monospace';

export const HALF_WIDTH_MM = 160;       // reactive::Config::halfWidthMm - the corridor the pilot reasons in
export const HORIZON_MM = 2500;         // reactive::Config::clearMm - beyond this nothing is in the way
export const STEER_DEG = 30;            // PILOT_STEER_LOCK_DEG: full lock, as the heading arrow draws it
export const CAR_MM = [186, 442];       // vehicle::CAR_WID_MM, CAR_LEN_MM - the TT-02 across the tires
export const CAR_HEIGHT_MM = 140;       // vehicle::CAR_HEIGHT_MM
export const SCAN_Z_MM = 167;           // vehicle::C1_SCAN_Z_MM: the plane the returns are in, the 3D tick's top
export const STALE_S = 3;               // the server's rule for /scan, applied here to /json as well

// A decision's tone, keyed on the mode's name: cruise and slow drive (good),
// stop halts (bad), reverse and blind back off or see nothing (warn). The
// same word colours the big word, the corridor, the clearance bar and the
// steer arrow, so one glance says the same thing everywhere.
export function modeTone(mode) {
  if (mode === 'cruise' || mode === 'slow') return 'good';
  if (mode === 'stop') return 'bad';
  if (mode === 'reverse' || mode === 'blind') return 'warn';
  return 'ink';
}
const TONE_COLOR = { good: ui.good, warn: ui.warn, bad: ui.bad, muted: ui.muted, ink: ui.text };
export function toneColor(tone) { return TONE_COLOR[tone] || ui.text; }
export function pilotColor(drive) { return toneColor(modeTone(drive.mode)); }

// Sizes a canvas to its CSS box at device pixels, capped at 2x: a phone's 3x
// panel would triple the fill for dots that are still three pixels. Returns
// the CSS size to draw in. Setting width resets the context, hence the
// transform.
export function fitCanvas(canvas, ctx) {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  const dpr = Math.min(2, window.devicePixelRatio || 1);
  const pw = Math.round(w * dpr), ph = Math.round(h * dpr);
  if (canvas.width !== pw || canvas.height !== ph) { canvas.width = pw; canvas.height = ph; }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { w: w, h: h };
}

export function fontUI(px, weight) { return (weight || 400) + ' ' + px + 'px ' + UI; }

// Text with a soft shadow under it, so a ring or a point never cuts a
// number, and nothing needs a box behind it. Clears the shadow after.
export function shadowText(ctx, x, y, text, color) {
  ctx.shadowColor = 'rgba(0,0,0,0.9)'; ctx.shadowBlur = 6;
  ctx.fillStyle = color;
  ctx.fillText(text, x, y);
  ctx.shadowBlur = 0; ctx.shadowColor = 'transparent';
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

// Metres with one decimal ("1.6 m"), two under a metre ("0.33 m") so the
// nearest thing is never rounded to nothing.
export function metres(mm) { return (mm / 1000).toFixed(mm < 1000 ? 2 : 1) + ' m'; }

// The palette and the car's constants, once, for the two canvases - a CSS
// variable does not reach a canvas context. Gruvbox, the hub's: one ground,
// one ink, one accent (aqua - "the UI is pointing at this", the hub's
// accent::CYAN and CYAN_HI exactly), and the hub's status set for the few
// marks that carry a state. Nothing else is a color.

export const BG = '#1d2021';
export const PANEL = '#3c3836';
export const INK = '#ebdbb2';
export const INK_DIM = '#a89984';
export const GRID = '#504945';
export const GRID_MAJOR = '#665c54';
export const ACCENT = '#8ec07c';        // the decision, when the car is going somewhere
export const ACCENT_DIM = '#689d6a';    // the heading: which way is forward, always
export const WARN = '#fe8019';          // the two modes where the car does what you did not ask
export const BAD = '#fb4934';           // the nearest return, as the hub marks it
export const MONO = 'ui-monospace, Consolas, "DejaVu Sans Mono", Menlo, monospace';

export const HALF_WIDTH_MM = 160;       // reactive::Config::halfWidthMm - the corridor the pilot reasons in
export const STEER_DEG = 30;            // full lock drawn at 30 degrees; a fraction has no angle of its own
export const CAR_MM = [190, 430];       // the TT-02's width and length, drawn to the scan's scale
export const CAR_HEIGHT_MM = 130;
export const STALE_S = 3;               // the server's rule for /scan, applied here to /json as well
export const WARN_MODES = { stop: true, reverse: true, blind: true };

// The color of a decision. Ink when the wheels only point somewhere, accent
// when the car is going there, and the warning tint when the mode is one the
// car chose over you.
export function decisionColor(drive) {
  if (WARN_MODES[drive.mode]) return WARN;
  return drive.throttle !== 0 ? ACCENT : INK;
}

// Sizes a canvas to its CSS box at device pixels, capped at 2x: a phone's 3x
// panel would triple the fill for dots that are still two pixels. Returns the
// CSS size to draw in. Setting width resets the context, hence the transform.
export function fitCanvas(canvas, ctx) {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  const dpr = Math.min(2, window.devicePixelRatio || 1);
  const pw = Math.round(w * dpr), ph = Math.round(h * dpr);
  if (canvas.width !== pw || canvas.height !== ph) { canvas.width = pw; canvas.height = ph; }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { w: w, h: h };
}

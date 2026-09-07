// The hub's palette, once, for the two canvases - a CSS variable does not
// reach a canvas context - and the car's constants the views draw to scale.
// Copied from hub/src/theme.hxx, which has TWO sets and so does this file:
//   CHROME  gruvbox dark - the casing, keys, wells and hairlines around the
//           picture. dash.css carries the same values for the DOM.
//   MAP     the sixteen xterm ANSI colours on pure black (ui::ansi), and
//           the rule that comes with them: a mark on the map is one of the
//           sixteen, unmodified, or it is not drawn. GRID and AXIS are the
//           one allowed departure - a range grid at full brightness competes
//           with the returns it measures.

export const chrome = {
  bg0h: '#1d2021', bg0: '#282828', bg0s: '#32302f', bg1: '#3c3836',
  bg2: '#504945', bg3: '#665c54', bg4: '#7c6f64',
  fg1: '#ebdbb2', fg2: '#d5c4a1', fg3: '#bdae93', fg4: '#a89984',
  aqua: '#689d6a', aquaHi: '#8ec07c',
};

export const ansi = {
  BLACK: '#000000', RED: '#cd0000', GREEN: '#00cd00', YELLOW: '#cdcd00',
  BLUE: '#0000ee', MAGENTA: '#cd00cd', CYAN: '#00cdcd', WHITE: '#e5e5e5',
  GRAY: '#7f7f7f', BRRED: '#ff0000', BRGREEN: '#00ff00', BRYELLOW: '#ffff00',
  BRBLUE: '#5c5cff', BRMAGENTA: '#ff00ff', BRCYAN: '#00ffff', BRWHITE: '#ffffff',
};

// What each one means on the map (ui::plot / radar.cxx's constants).
export const map = {
  GRID: '#3a3a3a', GRID_MAJOR: ansi.GRAY, AXIS: '#262626', TICK: '#4a4a4a',
  LABEL: ansi.WHITE, RING_TEXT: ansi.GRAY, BEARING: ansi.GRAY, CARDINAL: ansi.WHITE,
  HEADING: ansi.BRCYAN, ACCENT: ansi.BRCYAN, NEAREST: ansi.BRRED, SCALE: ansi.WHITE,
  POINT: ansi.BRWHITE, EMPTY: ansi.GRAY, CAR: ansi.BRBLUE,
  PLATE: 'rgba(0,0,0,0.88)',            // PLATE_BG: black at 0xE0, under every label
  OK: ansi.BRGREEN, WARN: ansi.BRYELLOW, BAD: ansi.BRRED, IDLE: ansi.GRAY,
};

// Semantic colours for text and lamps - forwarded to the map's set, so one
// green means one thing everywhere (ui::sem).
export const sem = { GOOD: ansi.BRGREEN, WARN: ansi.BRYELLOW, BAD: ansi.BRRED, MUTED: ansi.GRAY };

// The hub loads Segoe UI (regular / semibold / bold) for the interface and
// Cascadia Mono, then Consolas, then Lucida Console for anything in columns.
// A phone has none of those, so the closest it does have, in that order.
export const UI = '"Segoe UI", "Segoe UI Variable Text", Cantarell, Roboto, system-ui, sans-serif';
export const MONO = '"Cascadia Mono", Consolas, Menlo, "DejaVu Sans Mono", ui-monospace, monospace';

// The type scale, logical px (ui::size). One ~1.2 ratio anchored on BODY.
export const size = { SMALL: 12, BODY: 15, TITLE: 18, STAT: 22, BIG: 28, CODE: 14 };

// Spacing the HUD shares with the chrome: HUD_INSET equals IndentSpacing,
// and the line gap is ItemSpacing.y.
export const HUD_INSET = 16;
export const LINE_GAP = 6;
export const FRAME_PAD_Y = 4;

export const HALF_WIDTH_MM = 160;       // reactive::Config::halfWidthMm - the corridor the pilot reasons in
export const HORIZON_MM = 2500;         // reactive::Config::clearMm - beyond this nothing is in the way
export const STEER_DEG = 30;            // PILOT_STEER_LOCK_DEG: full lock, as the heading arrow draws it
export const CAR_MM = [186, 442];       // vehicle::CAR_WID_MM, CAR_LEN_MM - the TT-02 across the tires
export const CAR_HEIGHT_MM = 140;       // vehicle::CAR_HEIGHT_MM
export const SCAN_Z_MM = 167;           // vehicle::C1_SCAN_Z_MM: the plane the returns are in, the 3D tick's top
export const STALE_S = 3;               // the server's rule for /scan, applied here to /json as well

// A decision that halts or backs the car is the WARN semantic; one that
// drives it is the map's accent. Keyed on what was SENT, not on the mode's
// name, exactly as the hub's pilotColor(): stop and blind both send STOP,
// reverse sends a negative throttle.
export function pilotColor(drive) {
  return (drive.stop || drive.throttle < 0) ? map.WARN : map.ACCENT;
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

// ---- the HUD every view shares --------------------------------------------
// The hub's drawMapHud, on a canvas: the state lamp and word top-left with the
// host beside it, a second line naming the view and its reading, a third for
// the pilot's decision, throughput top-right, the zoom state bottom-right.
// `hud` is app.js's summary: { state, color, lit, host, ptsPerS, hz, pilot }.

export function fontUI(px) { return px + 'px ' + UI; }
export function fontMono(px) { return px + 'px ' + MONO; }

// An indicator lamp, as ui::led draws it: a dark socket always; lit throws a
// halo and has a hot spot up-left; unlit is a dim disc with a faint rim.
export function led(ctx, x, y, r, color, lit) {
  ctx.beginPath(); ctx.arc(x, y, r * 1.55, 0, 2 * Math.PI);
  ctx.fillStyle = 'rgba(0,0,0,0.43)'; ctx.fill();
  if (lit) {
    ctx.globalAlpha = 0.10; ctx.fillStyle = color;
    ctx.beginPath(); ctx.arc(x, y, r * 2.6, 0, 2 * Math.PI); ctx.fill();
    ctx.globalAlpha = 0.20;
    ctx.beginPath(); ctx.arc(x, y, r * 1.7, 0, 2 * Math.PI); ctx.fill();
    ctx.globalAlpha = 1;
    ctx.beginPath(); ctx.arc(x, y, r, 0, 2 * Math.PI); ctx.fill();
    ctx.fillStyle = 'rgba(255,255,255,0.59)';
    ctx.beginPath(); ctx.arc(x - r * 0.28, y - r * 0.28, r * 0.36, 0, 2 * Math.PI); ctx.fill();
  } else {
    ctx.globalAlpha = 0.235; ctx.fillStyle = color;
    ctx.beginPath(); ctx.arc(x, y, r, 0, 2 * Math.PI); ctx.fill();
    ctx.globalAlpha = 1;
    ctx.strokeStyle = 'rgba(255,255,255,0.11)'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.arc(x, y, r, 0, 2 * Math.PI); ctx.stroke();
  }
}

// Text on a plate: the label's box in PLATE black behind it, so a ring never
// cuts a number. Returns the text width.
export function plateText(ctx, x, y, text, color) {
  const w = ctx.measureText(text).width;
  const px = parseFloat(ctx.font);
  ctx.fillStyle = map.PLATE;
  ctx.fillRect(x - 2, y - px * 0.85, w + 4, px * 1.15);
  ctx.fillStyle = color;
  ctx.fillText(text, x, y);
  return w;
}

// `view` is the second line's word ("Points", "Cloud") and `diag` what
// follows it. Draws only the text; the caller has already drawn the picture.
export function drawHud(ctx, W, H, hud, view, diag, fit) {
  const S = size.SMALL, pad = HUD_INSET, step = S + LINE_GAP;
  ctx.textBaseline = 'alphabetic';

  // ---- top left: state + connection
  let x = pad, y = pad + S * 0.85;
  led(ctx, x + S * 0.28, pad + S * 0.55, S * 0.24, hud.color, hud.lit);
  x += S * 0.85;
  ctx.font = fontUI(S); ctx.textAlign = 'left';
  ctx.fillStyle = hud.color; ctx.fillText(hud.state, x, y);
  x += ctx.measureText(hud.state).width + 12;
  if (hud.host) { ctx.fillStyle = map.LABEL; ctx.fillText(hud.host, x, y); }

  // ---- second line: the view and its reading
  x = pad; y += step;
  ctx.fillStyle = map.ACCENT; ctx.fillText(view, x, y);
  x += ctx.measureText(view).width + 12;
  if (diag) { ctx.fillStyle = map.LABEL; ctx.fillText(diag, x, y); }

  // ---- third line: the pilot's decision, only while someone is deciding
  if (hud.pilot) {
    const d = hud.pilot;
    x = pad; y += step;
    const who = 'pilot  ' + d.mode;
    ctx.fillStyle = pilotColor(d); ctx.fillText(who, x, y);
    if (d.mode !== 'blind') {
      x += ctx.measureText(who).width + 12;
      ctx.font = fontMono(S); ctx.fillStyle = map.LABEL;
      ctx.fillText('clear ' + d.clearMm + ' mm   steer ' + (d.steer < 0 ? '' : '+') + d.steer.toFixed(2) +
                   '  thr ' + d.throttle.toFixed(2), x, y);
    }
  }

  // ---- top right: throughput
  ctx.font = fontMono(S); ctx.textAlign = 'right'; ctx.fillStyle = map.LABEL;
  ctx.fillText(hud.ptsPerS.toFixed(0) + ' pts/s   ' + hud.hz.toFixed(1) + ' Hz', W - pad, pad + S * 0.85);

  // ---- bottom right: zoom state
  ctx.fillText(fit, W - pad, H - pad - FRAME_PAD_Y - S * 0.15);
  ctx.textAlign = 'left';
}

// The words a view writes when there is no revolution: the hub's "No scan
// data", under the car so the marker does not cross it, and under that the
// reason, since the phone cannot hover. `drop` is how far below the centre.
export function noScan(ctx, cx, cy, drop, reason) {
  ctx.textAlign = 'center';
  ctx.font = fontUI(size.BODY); ctx.fillStyle = map.EMPTY;
  ctx.fillText('No scan data', cx, cy + drop);
  if (reason) { ctx.font = fontUI(size.SMALL); ctx.fillText(reason, cx, cy + drop + size.SMALL + LINE_GAP); }
  ctx.textAlign = 'left';
}

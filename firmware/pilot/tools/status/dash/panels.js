// The words around the pictures: the sidebar's panels and the status strip,
// and the one summary the two views share for their HUD. Every value is the
// board's own word, said once, and an absence is worded as an absence -
// "not running", "no answer" - never a zero and never the last thing we
// knew. The buttons follow /json's word on the pilot process, not the last
// tap: a tap that failed leaves them as they were, and says why beside them.
// Colour means a state or it is not used: good / warn / bad / muted are
// ui::sem, on the lamp and on the state word beside it, nowhere else.

import { STALE_S, sem } from './theme.js';

function el(id) { return document.getElementById(id); }

// A row: its lamp, its state word, its value column.
function row(id) {
  const n = el(id);
  return { lamp: n.querySelector('.lamp'), st: n.querySelector('.st'), val: n.querySelector('.val') };
}
// A strip field: its dot, its state, its detail.
function field(id) {
  const n = el(id);
  return { dot: n.querySelector('.dot'), st: n.querySelector('.st'), ex: n.querySelector('.ex') };
}

const rows = { pico: row('r-pico'), board: row('r-board'), lidar: row('r-lidar'), pilot: row('r-pilot'), feed: row('r-feed'), c1: row('r-c1') };
const strip = { lidar: field('s-lidar'), pico: field('s-pico'), board: field('s-board'), pilot: field('s-pilot') };
const modeEl = el('p-mode'), srcEl = el('p-src');
const pv = { clear: el('p-clear'), hits: el('p-hits'), steer: el('p-steer'), throttle: el('p-throttle'), sent: el('p-sent') };
const lookBtn = el('look'), stopBtn = el('stop'), replyEl = el('reply');

export function bind(onLook, onStop) {
  lookBtn.addEventListener('click', onLook);
  stopBtn.addEventListener('click', onStop);
  // A header folds its panel, as the hub's CollapsingHeader does. The float
  // key is inert - a page has one window - and says so on hover.
  document.querySelectorAll('.panel .hdr').forEach(function (h) {
    h.addEventListener('click', function (e) {
      if (e.target.closest('.float')) return;
      h.parentNode.classList.toggle('closed');
    });
  });
}

function text(node, s) { if (node.textContent !== s) node.textContent = s; }
function cls(node, c) { if (node.className !== c) node.className = c; }

function setRow(r, tone, lit, state, value) {
  cls(r.lamp, 'lamp ' + tone + (lit ? ' lit' : ''));
  cls(r.st, 'st ' + tone);
  text(r.st, state);
  text(r.val, value || '');
}
function setField(f, tone, state, extra) {
  cls(f.dot, 'dot ' + tone);
  cls(f.st, 'st ' + tone);
  text(f.st, state);
  text(f.ex, extra || '');
}

function signed(v) { return (v < 0 ? '' : '+') + v.toFixed(2); }
function nz(v, unit) { return v === undefined || v === null ? 'unknown' : v + (unit || ''); }

// The text page's lines, keyed by their first word: "pico          /dev/ttyACM0
// absent - Pico unplugged; pilot not running" becomes lines.pico = the rest.
function byLabel(beat) {
  const out = {};
  if (!beat || !beat.lines) return out;
  beat.lines.forEach(function (l) {
    const m = /^(\S+)\s+(.*)$/.exec(l);
    if (m) out[m[1]] = m[2];
  });
  return out;
}

// The lidar's state as the hub words it, from /scan's answer: a word, a tone,
// and whether the lamp is lit. Shared by the HUD, the strip and two rows.
export function lidarState(st) {
  if (st.scan) return { word: 'Scanning', tone: 'good', lit: true };
  const why = st.scanGone;
  if (why === 'no answer') return { word: 'No answer', tone: 'bad', lit: true };
  if (why === 'feed connecting' || why === 'lidar spinning up' || why === 'feed idle') return { word: 'Connecting', tone: 'warn', lit: true };
  if (why === 'scan stale') return { word: 'Stale', tone: 'warn', lit: true };
  if (why === 'motor off') return { word: 'Motor off', tone: 'warn', lit: false };
  if (why.indexOf('no scan feed') === 0 || why === 'pilot not running') return { word: 'No feed', tone: 'muted', lit: false };
  if (why.indexOf('feed') === 0) return { word: 'Feed error', tone: 'bad', lit: true };
  return { word: 'No scan', tone: 'muted', lit: false };
}

const TONE = { good: sem.GOOD, warn: sem.WARN, bad: sem.BAD, muted: sem.MUTED };

// The HUD's summary for the canvases: the state and its colour, where the
// page is talking to, the throughput, and the decision if one is being made.
export function hud(st) {
  const ls = lidarState(st), scan = st.scan;
  return {
    state: ls.word, color: TONE[ls.tone], lit: ls.lit,
    host: location.host,
    ptsPerS: scan ? scan.n * scan.hz : 0, hz: scan ? scan.hz : 0,
    pilot: scan && scan.drive ? scan.drive : null,
  };
}

// The pilot as a process. /json's pilotProc is only THIS page's child; a
// pilot started at a shell shows up as a fresh heartbeat and nothing else,
// and "not running" beside a decision being drawn would be a false absence.
function pilotWord(beat, live) {
  if (!beat) return { word: 'no answer', tone: 'bad', lit: true, extra: '' };
  const p = beat.pilotProc;
  if (!p) return { word: 'unknown', tone: 'muted', lit: false, extra: '' };
  if (p.running) return { word: p.mode + ' ' + p.sinceS.toFixed(0) + ' s', tone: 'good', lit: true, extra: 'pid ' + p.pid };
  if (live) return { word: 'running elsewhere', tone: 'warn', lit: true, extra: '' };
  if (p.exitCode !== null && p.exitCode !== undefined) return { word: 'exited ' + p.exitCode, tone: p.exitCode ? 'bad' : 'muted', lit: !!p.exitCode, extra: '' };
  return { word: 'not running', tone: 'muted', lit: false, extra: '' };
}

export function render(st) {
  const beat = st.beat, scan = st.scan;
  const pilot = beat && beat.pilot;
  const live = !!(pilot && beat.pilotAgeS <= STALE_S);
  const d = scan && scan.drive;
  const L = byLabel(beat);
  const ls = lidarState(st);
  const pw = pilotWord(beat, live);

  // ---- System ------------------------------------------------------------
  // Pico link: the pilot's own phrase while it is running; otherwise whether
  // the device is even there, from the text page's line.
  if (live) {
    const pico = String(pilot.pico || 'unknown').replace(/^pico\s+/, '');
    const gone = /absent|unplugged|lost|no /i.test(pico);
    setRow(rows.pico, gone ? 'muted' : /dry/i.test(pico) ? 'warn' : 'good', !gone, pico, '');
  } else if (beat) {
    const m = /^(\S+)\s+(absent|present)/.exec(L.pico || '');
    if (m) setRow(rows.pico, m[2] === 'present' ? 'muted' : 'muted', false, m[2] + (m[2] === 'present' ? ', no pilot' : ''), m[1]);
    else setRow(rows.pico, 'muted', false, 'unknown', '');
  } else {
    setRow(rows.pico, 'bad', true, 'no answer', '');
  }

  // Board: answering, and how long it has been up, from the text page's first
  // line - "<host>  <addresses>  <wifi>  up <time>  <clock>", two spaces
  // between - with the Wi-Fi profile beside it; the CPU temperature as the
  // value. The address goes on the strip.
  const first = beat && beat.lines && beat.lines[0] ? beat.lines[0].split('  ') : [];
  const up = first.length >= 4 ? first[3] : '';
  const wifi = first.length >= 3 ? first[2] : '';
  const addr = beat ? (beat.addresses.join(' ') || 'no address') : '';
  if (beat) setRow(rows.board, 'good', true, (up || 'answering') + (wifi ? '  ' + wifi : ''), beat.cpuC !== null && beat.cpuC !== undefined ? beat.cpuC.toFixed(1) + ' C' : 'no temp');
  else setRow(rows.board, 'bad', true, 'no answer', '');

  // Lidar: the device, and who has it - the pilot's rev/s, the feed's Hz, or
  // just present.
  if (live) {
    setRow(rows.lidar, pilot.lidarLost ? 'warn' : 'good', true, nz(pilot.revPerS, ' rev/s') + (pilot.lidarLost ? '  LOST' : '') + '  pilot', '');
  } else if (beat) {
    const m = /^(\S+)\s+(absent|present)/.exec(L.lidar || '');
    if (scan) setRow(rows.lidar, 'good', true, scan.hz.toFixed(1) + ' Hz  feed', m ? m[1] : '');
    else if (m) setRow(rows.lidar, 'muted', false, m[2], m[1]);
    else setRow(rows.lidar, 'muted', false, 'unknown', '');
  } else {
    setRow(rows.lidar, 'bad', true, 'no answer', '');
  }

  setRow(rows.pilot, pw.tone, pw.lit, pw.word, pw.extra);

  // Feed: /scan's own word.
  if (scan) setRow(rows.feed, 'good', true, 'live', scan.hz.toFixed(1) + ' Hz');
  else setRow(rows.feed, ls.tone, ls.lit, st.scanGone, '');

  // ---- Quick actions -----------------------------------------------------
  const p = beat && beat.pilotProc;
  const running = !!(p && p.running);
  lookBtn.disabled = !beat || running;
  stopBtn.disabled = !beat || !running;
  let state;
  if (!beat) state = 'pilot: no answer from the car';
  else if (running) state = 'pilot: ' + p.mode + ' ' + p.sinceS.toFixed(0) + ' s (pid ' + p.pid + ')';
  else if (live) state = 'pilot: running elsewhere - not started from this page, so STOP here cannot stop it';
  else if (p && p.exitCode !== null && p.exitCode !== undefined) state = 'pilot: exited ' + p.exitCode + (p.lastLine ? ' - ' + p.lastLine : '');
  else state = 'pilot: not running';
  text(replyEl, Date.now() < st.msgUntil ? st.msg : state);

  // ---- Sensors -----------------------------------------------------------
  if (scan) setRow(rows.c1, 'good', true, scan.hz.toFixed(1) + ' Hz' + (d ? '  pilot' : ''), '');
  else setRow(rows.c1, ls.tone, ls.lit, ls.word, '');

  // ---- Pilot -------------------------------------------------------------
  // The mode as the value, and under it where the word came from, every time.
  let mode, tone, source;
  if (d) {
    mode = d.mode; tone = (d.stop || d.throttle < 0) ? 'warn' : 'good';
    source = 'deciding on this scan, ' + scan.hz.toFixed(1) + ' Hz' + (scan.bad ? ' - F line unreadable, not drawn' : '');
    text(pv.clear, d.clearMm + ' mm'); text(pv.hits, String(d.hits));
    text(pv.steer, signed(d.steer)); text(pv.throttle, signed(d.throttle));
    text(pv.sent, d.stop ? 'STOP' : d.throttle > 0 ? 'drive' : d.throttle < 0 ? 'reverse' : 'neutral');
  } else if (live) {
    // No decision on the wire, but the heartbeat is fresh: its second-old
    // numbers, said to be so.
    mode = pilot.mode; tone = 'muted';
    source = 'the heartbeat\'s, ' + beat.pilotAgeS.toFixed(0) + ' s old - no D line on the feed';
    text(pv.clear, nz(pilot.clearanceMm, ' mm')); text(pv.hits, nz(pilot.hits));
    text(pv.steer, 'not in the heartbeat'); text(pv.throttle, 'not in the heartbeat'); text(pv.sent, '-');
  } else {
    mode = 'no pilot'; tone = 'muted';
    source = !beat ? 'no answer from the car' : scan ? 'the lidar through scanfeed; nobody deciding' : 'nothing deciding, no scan';
    text(pv.clear, '-'); text(pv.hits, '-'); text(pv.steer, '-'); text(pv.throttle, '-'); text(pv.sent, '-');
  }
  text(modeEl, mode); cls(modeEl, 'stat-v ' + tone);
  text(srcEl, source);

  // ---- the strip ---------------------------------------------------------
  setField(strip.lidar, ls.tone, ls.word, scan ? location.hostname + '  ' + scan.hz.toFixed(1) + ' Hz' + (d ? '  pilot' : '') : '');
  setField(strip.pico, rows.pico.st.className.replace('st ', ''), rows.pico.st.textContent, '');
  if (beat) setField(strip.board, 'good', beat.host, addr);
  else setField(strip.board, 'bad', 'no answer', '');
  setField(strip.pilot, pw.tone, pw.word, '');
}

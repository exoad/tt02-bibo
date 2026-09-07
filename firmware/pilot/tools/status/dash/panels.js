// The words around the pictures. On the dash: the mode word (the pilot's
// mode, LIDAR when only the lidar is on, or the reason there is no scan),
// the clearance under it, the six stat cards (steer, throttle, hits, nearest,
// lidar rate, sent), the four chips (feed, pilot, Pico, board), the keys and
// the corner text the canvases draw. On the support pane: the System,
// Sensors and Pilot cards. Every value is the board's own word, said once,
// and an absence is worded as an absence - "not running", "no answer" -
// never a zero and never the last thing we knew. The keys follow /json's
// word on the pilot process, not the last tap: a tap that failed leaves
// them as they were, and says why. Colour means a state or it is not used:
// the tone classes (success / warning / error / variant) on a chip, a list
// item's icon and supporting text, a stat's value, the mode word.

import { STALE_S, modeTone, metres } from './theme.js';

function el(id) { return document.getElementById(id); }

// A list item: its leading icon, its supporting text, its trailing value.
function row(id) {
  const n = el(id);
  return { lead: n.querySelector('.li-lead'), sup: n.querySelector('.li-sup'), trail: n.querySelector('.li-trail') };
}
// A stat card: its value and its unit.
function stat(id) {
  const n = el(id);
  return { v: n.querySelector('.stat-v'), u: n.querySelector('.stat-u') };
}
// A chip: itself (for the tone) and its label.
function chip(id) {
  const n = el(id);
  return { node: n, l: n.querySelector('.chip-l') };
}

const rows = { pico: row('r-pico'), board: row('r-board'), lidar: row('r-lidar'), pilot: row('r-pilot'), feed: row('r-feed'), c1: row('r-c1') };
const stats = { steer: stat('s-steer'), throttle: stat('s-throttle'), hits: stat('s-hits'), nearest: stat('s-nearest'), hz: stat('s-hz'), sent: stat('s-sent') };
const chips = { feed: chip('c-feed'), pilot: chip('c-pilot'), pico: chip('c-pico'), board: chip('c-board') };
const modeEl = el('p-mode'), srcEl = el('p-src');
const pv = { clear: el('p-clear'), hits: el('p-hits'), steer: el('p-steer'), throttle: el('p-throttle'), sent: el('p-sent'),
             revs: el('p-revs'), revolutions: el('p-revolutions'), timeouts: el('p-timeouts') };
const lookBtn = el('look'), stopBtn = el('stop');
const stateEl = el('pstate');
const wordEl = el('word'), clearV = el('clear-v'), clearC = el('clear-c');

export function bind(onLook, onStop) {
  lookBtn.addEventListener('click', onLook);
  stopBtn.addEventListener('click', onStop);
}

function text(node, s) { if (node.textContent !== s) node.textContent = s; }
function cls(node, c) { if (node.className !== c) node.className = c; }

// tone: success | warning | error | variant. `lit` puts the icon in the tone;
// unlit leaves it on-surface-variant, the way an idle lamp is dark.
function setRow(r, tone, lit, state, value) {
  cls(r.lead, 'li-lead' + (lit ? ' tone-' + tone : ''));
  cls(r.sup, 'li-sup body-medium tone-' + tone);
  text(r.sup, state);
  text(r.trail, value || '');
}
function setStat(s, value, unit, tone) {
  text(s.v, value); text(s.u, unit || '');
  cls(s.v, 'stat-v num' + (tone ? ' tone-' + tone : ''));
}
function setChip(c, tone, label) {
  cls(c.node, 'chip label-large tone-' + tone);
  text(c.l, label);
}

function signed(v) { return (v < 0 ? '−' : '+') + Math.abs(v).toFixed(2); }
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
// and whether the lamp is lit. Shared by the rows, the chips and the log.
export function lidarState(st) {
  if (st.scan) return { word: 'Scanning', tone: 'success', lit: true };
  const why = st.scanGone;
  if (why === 'no answer') return { word: 'No answer', tone: 'error', lit: true };
  if (why === 'feed connecting' || why === 'lidar spinning up' || why === 'feed idle') return { word: 'Connecting', tone: 'warning', lit: true };
  if (why === 'scan stale') return { word: 'Stale', tone: 'warning', lit: true };
  if (why === 'motor off') return { word: 'Motor off', tone: 'warning', lit: false };
  if (why.indexOf('no scan feed') === 0 || why === 'pilot not running') return { word: 'No feed', tone: 'variant', lit: false };
  if (why.indexOf('feed') === 0) return { word: 'Feed error', tone: 'error', lit: true };
  return { word: 'No scan', tone: 'variant', lit: false };
}

// The mode word when there is no scan: the reason, in caps, always error - a
// screen with no picture of now is a screen to act on.
export function goneWord(why) {
  if (why === 'no answer') return 'NO ANSWER';
  if (why === 'scan stale') return 'STALE';
  if (why === 'feed connecting' || why === 'lidar spinning up' || why === 'feed idle') return 'SPINNING UP';
  if (why === 'motor off') return 'MOTOR OFF';
  if (why.indexOf('no scan feed') === 0 || why === 'pilot not running') return 'NO FEED';
  if (why.indexOf('feed') === 0) return 'FEED ERROR';
  return 'NO SCAN';
}

// The pilot as a process. /json's pilotProc is only THIS page's child; a
// pilot started at a shell shows up as a fresh heartbeat and nothing else,
// and "not running" beside a decision being drawn would be a false absence.
function pilotWord(beat, live) {
  if (!beat) return { word: 'no answer', tone: 'error', lit: true, extra: '' };
  const p = beat.pilotProc;
  if (!p) return { word: 'unknown', tone: 'variant', lit: false, extra: '' };
  if (p.running) return { word: p.mode + ' ' + p.sinceS.toFixed(0) + ' s', tone: 'success', lit: true, extra: 'pid ' + p.pid };
  if (live) return { word: 'running elsewhere', tone: 'warning', lit: true, extra: '' };
  if (p.exitCode !== null && p.exitCode !== undefined) return { word: 'exited ' + p.exitCode, tone: p.exitCode ? 'error' : 'variant', lit: !!p.exitCode, extra: '' };
  return { word: 'not running', tone: 'variant', lit: false, extra: '' };
}

// What the D line's stop flag and throttle mean was sent to the Pico: STOP,
// or the steer and throttle fractions. The pulse in microseconds is the
// drivetrain's to derive, not the pilot's, so it is not invented here.
function sentWord(d) { return d.stop ? 'STOP' : d.throttle > 0 ? 'drive' : d.throttle < 0 ? 'reverse' : 'neutral'; }

// The corner text for the canvases: feed word and host, points a revolution
// and the rate, top right. Built here so the words match the chips'.
export function hud(st) {
  const ls = lidarState(st);
  const tr = [];
  tr.push((st.scan ? 'feed live' : 'feed: ' + st.scanGone) + ' · ' + (st.beat ? st.beat.host : 'no answer from the car'));
  if (st.scan) {
    tr.push(st.scan.n + ' pts/rev · ' + Math.round(st.scan.n * st.scan.hz) + ' pts/s');
    tr.push(st.scan.hz.toFixed(1) + ' Hz' + (st.scan.bad ? ' · F line unreadable' : ''));
  } else {
    tr.push(ls.word.toLowerCase());
  }
  return { tr: tr };
}

export function render(st) {
  const beat = st.beat, scan = st.scan;
  const pilot = beat && beat.pilot;
  const live = !!(pilot && beat.pilotAgeS <= STALE_S);
  const d = scan && scan.drive;
  const L = byLabel(beat);
  const ls = lidarState(st);
  const pw = pilotWord(beat, live);
  const tone = d ? modeTone(d.mode) : null;

  // ---- the dash --------------------------------------------------------------
  // The mode word: the decision's mode; LIDAR when a scan comes with nobody
  // deciding; the reason when there is none.
  if (d) { text(wordEl, d.mode.toUpperCase()); cls(wordEl, 'num tone-' + tone); }
  else if (scan) { text(wordEl, 'LIDAR'); cls(wordEl, 'num tone-variant'); }
  else { text(wordEl, goneWord(st.scanGone)); cls(wordEl, 'num tone-error'); }

  // The number: the pilot's clearance while it is deciding; the nearest
  // return when only the lidar is on, and the caption says which.
  if (d && d.mode !== 'blind') { text(clearV, metres(d.clearMm)); text(clearC, 'clear'); }
  else if (scan && scan.near >= 0) { text(clearV, metres(scan.d[scan.near])); text(clearC, 'nearest'); }
  else { text(clearV, '--'); text(clearC, ''); }

  // The stat cards: the decision's numbers while a pilot is deciding on
  // something it saw, dashes otherwise; the lidar's own numbers whenever
  // there is a scan.
  const deciding = d && d.mode !== 'blind';
  if (deciding) {
    setStat(stats.steer, signed(d.steer), d.steer < 0 ? 'left' : d.steer > 0 ? 'right' : '', tone);
    setStat(stats.throttle, signed(d.throttle), d.throttle < 0 ? 'rev' : d.throttle > 0 ? 'fwd' : '', tone);
    setStat(stats.hits, String(d.hits), '', tone);
    setStat(stats.sent, sentWord(d), '', d.stop ? 'error' : 'success');
  } else if (d) {
    // blind: the pilot saw nothing to decide on; the mode says so.
    setStat(stats.steer, '-', '', null); setStat(stats.throttle, '-', '', null);
    setStat(stats.hits, '0', '', tone); setStat(stats.sent, sentWord(d), '', d.stop ? 'error' : 'success');
  } else {
    setStat(stats.steer, '-', '', null); setStat(stats.throttle, '-', '', null);
    setStat(stats.hits, '-', '', null); setStat(stats.sent, '-', '', null);
  }
  if (scan && scan.near >= 0) setStat(stats.nearest, String(scan.d[scan.near]), 'mm', null);
  else setStat(stats.nearest, '-', '', null);
  if (scan) setStat(stats.hz, scan.hz.toFixed(1), 'Hz', null);
  else setStat(stats.hz, '-', '', null);

  // The keys follow the process word.
  const p = beat && beat.pilotProc;
  const running = !!(p && p.running);
  lookBtn.disabled = !beat || running;
  stopBtn.disabled = !beat || !running;

  // The chips: feed, pilot process, Pico, board - each in its state's tone.
  if (scan) setChip(chips.feed, 'success', 'LIVE ' + scan.hz.toFixed(1) + ' Hz');
  else setChip(chips.feed, ls.tone, goneWord(st.scanGone));
  setChip(chips.pilot, pw.tone, 'pilot ' + pw.word);

  const first = beat && beat.lines && beat.lines[0] ? beat.lines[0].split('  ') : [];
  const wifi = first.length >= 3 ? first[2] : '';
  const up = first.length >= 4 ? first[3] : '';
  const cpu = beat && beat.cpuC !== null && beat.cpuC !== undefined ? beat.cpuC.toFixed(0) + ' °C' : 'no temp';
  if (!beat) setChip(chips.board, 'error', 'board no answer');
  else setChip(chips.board, 'variant', beat.host + ' · ' + cpu + (wifi && wifi !== 'no wifi' ? ' · ' + wifi : ''));

  // ---- System --------------------------------------------------------------
  // Pico link: the pilot's own phrase while it is running; otherwise whether
  // the device is even there, from the text page's line.
  if (live) {
    const pico = String(pilot.pico || 'unknown').replace(/^pico\s+/, '');
    const gone = /absent|unplugged|lost|no /i.test(pico);
    const ptone = gone ? 'variant' : /dry/i.test(pico) ? 'warning' : 'success';
    setRow(rows.pico, ptone, !gone, pico, '');
    setChip(chips.pico, ptone, 'pico ' + pico);
  } else if (beat) {
    const m = /^(\S+)\s+(absent|present)/.exec(L.pico || '');
    if (m) { setRow(rows.pico, 'variant', false, m[2] + (m[2] === 'present' ? ', no pilot' : ''), m[1]); setChip(chips.pico, 'variant', 'pico ' + m[2]); }
    else { setRow(rows.pico, 'variant', false, 'unknown', ''); setChip(chips.pico, 'variant', 'pico unknown'); }
  } else {
    setRow(rows.pico, 'error', true, 'no answer', '');
    setChip(chips.pico, 'error', 'pico no answer');
  }

  // Board: answering, and how long it has been up, from the text page's first
  // line - "<host>  <addresses>  <wifi>  up <time>  <clock>", two spaces
  // between - with the Wi-Fi profile beside it; the CPU temperature as the
  // value.
  if (beat) setRow(rows.board, 'success', true, (up || 'answering') + (wifi ? ' · ' + wifi : ''), beat.cpuC !== null && beat.cpuC !== undefined ? beat.cpuC.toFixed(1) + ' °C' : 'no temp');
  else setRow(rows.board, 'error', true, 'no answer', '');

  // Lidar: the device, and who has it - the pilot's rev/s, the feed's Hz, or
  // just present.
  if (live) {
    setRow(rows.lidar, pilot.lidarLost ? 'warning' : 'success', true, nz(pilot.revPerS, ' rev/s') + (pilot.lidarLost ? ' · LOST' : '') + ' · pilot', '');
  } else if (beat) {
    const m = /^(\S+)\s+(absent|present)/.exec(L.lidar || '');
    if (scan) setRow(rows.lidar, 'success', true, scan.hz.toFixed(1) + ' Hz · feed', m ? m[1] : '');
    else if (m) setRow(rows.lidar, 'variant', false, m[2], m[1]);
    else setRow(rows.lidar, 'variant', false, 'unknown', '');
  } else {
    setRow(rows.lidar, 'error', true, 'no answer', '');
  }

  setRow(rows.pilot, pw.tone, pw.lit, pw.word, pw.extra);

  // Feed: /scan's own word.
  if (scan) setRow(rows.feed, 'success', true, 'live', scan.hz.toFixed(1) + ' Hz');
  else setRow(rows.feed, ls.tone, ls.lit, st.scanGone, '');

  // The process, in a sentence, under the rows.
  let state;
  if (!beat) state = 'pilot: no answer from the car';
  else if (running) state = 'pilot: ' + p.mode + ' ' + p.sinceS.toFixed(0) + ' s (pid ' + p.pid + ')';
  else if (live) state = 'pilot: running elsewhere - not started from this page, so STOP here cannot stop it';
  else if (p && p.exitCode !== null && p.exitCode !== undefined) state = 'pilot: exited ' + p.exitCode + (p.lastLine ? ' - ' + p.lastLine : '');
  else state = 'pilot: not running';
  text(stateEl, state);

  // ---- Sensors -------------------------------------------------------------
  if (scan) setRow(rows.c1, 'success', true, scan.hz.toFixed(1) + ' Hz' + (d ? ' · pilot' : ''), '');
  else setRow(rows.c1, ls.tone, ls.lit, ls.word, '');

  // ---- Pilot ---------------------------------------------------------------
  // The mode as the headline, and under it where the word came from, every
  // time; then the decision's numbers, and the heartbeat's counters when a
  // heartbeat is fresh.
  let mode, mtone, source;
  if (d) {
    mode = d.mode; mtone = modeTone(d.mode);
    source = 'deciding on this scan, ' + scan.hz.toFixed(1) + ' Hz' + (scan.bad ? ' - F line unreadable, not drawn' : '');
    text(pv.clear, d.clearMm + ' mm'); text(pv.hits, String(d.hits));
    text(pv.steer, signed(d.steer)); text(pv.throttle, signed(d.throttle));
    text(pv.sent, sentWord(d) + (d.stop ? '' : ' · steer ' + signed(d.steer) + ' thr ' + signed(d.throttle)));
  } else if (live) {
    // No decision on the wire, but the heartbeat is fresh: its second-old
    // numbers, said to be so.
    mode = pilot.mode; mtone = 'variant';
    source = 'the heartbeat\'s, ' + beat.pilotAgeS.toFixed(0) + ' s old - no D line on the feed';
    text(pv.clear, nz(pilot.clearanceMm, ' mm')); text(pv.hits, nz(pilot.hits));
    text(pv.steer, 'not in the heartbeat'); text(pv.throttle, 'not in the heartbeat'); text(pv.sent, '-');
  } else {
    mode = 'no pilot'; mtone = 'variant';
    source = !beat ? 'no answer from the car' : scan ? 'the lidar through scanfeed; nobody deciding' : 'nothing deciding, no scan';
    text(pv.clear, '-'); text(pv.hits, '-'); text(pv.steer, '-'); text(pv.throttle, '-'); text(pv.sent, '-');
  }
  if (live) {
    text(pv.revs, nz(pilot.revPerS, ' rev/s')); text(pv.revolutions, nz(pilot.revolutions)); text(pv.timeouts, nz(pilot.timeouts));
  } else {
    text(pv.revs, scan ? scan.hz.toFixed(1) + ' Hz · feed' : '-'); text(pv.revolutions, '-'); text(pv.timeouts, '-');
  }
  text(modeEl, mode); cls(modeEl, 'li-head headline-small tone-' + mtone);
  text(srcEl, source);
}

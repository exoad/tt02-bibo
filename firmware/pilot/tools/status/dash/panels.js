// The words around the pictures: the top bar, the sidebar of states, the
// bottom bar of controls. Every value is the board's own word, said once,
// and an absence is worded as an absence - "not measured", "pilot not
// running", "no answer from the car" - never a zero and never the last thing
// we knew. The buttons follow /json's word on the pilot process, not the last
// tap: a tap that failed leaves them as they were, and says why beside them.

import { STALE_S, WARN_MODES } from './theme.js';

function el(id) { return document.getElementById(id); }

const top = { board: el('t-board'), net: el('t-net'), feed: el('t-feed'), rate: el('t-rate'), pilot: el('t-pilot') };
const modeEl = el('mode'), sourceEl = el('source');
const rows = {};
['clear', 'hits', 'steer', 'throttle', 'stop', 'pico', 'lidar', 'cpu', 'revolutions', 'timeouts']
  .forEach(function (k) { rows[k] = el('r-' + k); });
const lookBtn = el('look'), stopBtn = el('stop'), msgEl = el('msg');

export function bind(onLook, onStop) {
  lookBtn.addEventListener('click', onLook);
  stopBtn.addEventListener('click', onStop);
}

function set(node, text, cls) {
  if (node.textContent !== text) node.textContent = text;
  if (cls !== undefined && node.className !== cls) node.className = cls;
}

function signed(v) { return (v < 0 ? '' : '+') + v.toFixed(3); }
function nz(v, unit) { return v === undefined || v === null ? 'unknown' : v + (unit || ''); }

// The feed's state as one word, from /scan's answer. The full reason is on
// the radar in large type; this is the glance.
function feedWord(st) {
  if (st.scan) return ['LIVE', 'v ok'];
  const why = st.scanGone;
  if (why === 'no answer') return ['NO ANSWER', 'v bad'];
  if (why === 'feed connecting' || why === 'lidar spinning up' || why === 'feed idle') return ['SPINNING UP', 'v dim'];
  if (why === 'scan stale') return ['STALE', 'v warn'];
  if (why === 'motor off') return ['MOTOR OFF', 'v warn'];
  if (why.indexOf('no scan feed') === 0 || why === 'pilot not running') return ['NO FEED', 'v warn'];
  if (why.indexOf('feed') === 0) return ['FEED ERROR', 'v warn'];
  return ['NO SCAN', 'v warn'];
}

// The pilot as a process. /json's pilotProc is only THIS page's child; a
// pilot started at a shell shows up as a fresh heartbeat and nothing else,
// and "not running" beside a decision being drawn would be a false absence.
function pilotWord(beat, live) {
  if (!beat) return 'no answer';
  const p = beat.pilotProc;
  if (!p) return 'unknown';
  if (p.running) return p.mode + ' ' + p.sinceS.toFixed(0) + ' s';
  if (live) return 'running elsewhere';
  if (p.exitCode !== null && p.exitCode !== undefined) return 'exited ' + p.exitCode;
  return 'not running';
}

export function render(st) {
  const beat = st.beat, scan = st.scan;
  const pilot = beat && beat.pilot;
  const live = !!(pilot && beat.pilotAgeS <= STALE_S);
  const d = scan && scan.drive;

  // ---- top bar
  set(top.board, beat ? beat.host + ' ' + (beat.addresses.join(' ') || 'no address') : 'no answer');
  // The Wi-Fi profile and the uptime are only in the text page's first line:
  // "<host>  <addresses>  <wifi>  up <time>  <clock>", two spaces between.
  const first = beat && beat.lines && beat.lines[0] ? beat.lines[0].split('  ') : [];
  set(top.net, first.length >= 4 ? first[2] + ', ' + first[3] : (beat ? 'unknown' : '-'));
  const fw = feedWord(st);
  set(top.feed, fw[0], fw[1]);
  set(top.rate, scan ? scan.hz.toFixed(1) + ' Hz' : live ? nz(pilot.revPerS, ' rev/s') : '-');
  set(top.pilot, pilotWord(beat, live));

  // ---- the states. The source line says where the big word came from, every time.
  let mode, cls = '', source;
  if (d) {
    mode = d.mode;
    source = 'scan live, ' + scan.hz.toFixed(1) + ' Hz, the pilot deciding' + (scan.bad ? ' - F line unreadable, not drawn' : '');
    set(rows.clear, d.clearMm + ' mm');
    set(rows.hits, String(d.hits));
    set(rows.steer, signed(d.steer));
    set(rows.throttle, signed(d.throttle));
    set(rows.stop, d.stop ? 'STOP' : 'no', d.stop ? 'v warn' : 'v');
  } else if (scan) {
    // The lidar through the feed, no pilot deciding: a picture, not a decision.
    mode = 'LIDAR'; cls = 'dim';
    source = 'scan live, ' + scan.hz.toFixed(1) + ' Hz - no pilot, lidar via scanfeed' + (scan.bad ? ' - F line unreadable, not drawn' : '');
    set(rows.clear, '-'); set(rows.hits, '-'); set(rows.steer, '-'); set(rows.throttle, '-'); set(rows.stop, '-', 'v');
  } else if (live) {
    // No scan, but the heartbeat is fresh: its second-old numbers, said to be so.
    mode = pilot.mode;
    source = 'scan none - ' + st.scanGone + '; mode is the heartbeat\'s, ' + beat.pilotAgeS.toFixed(0) + ' s old';
    set(rows.clear, nz(pilot.clearanceMm, ' mm'));
    set(rows.hits, nz(pilot.hits));
    set(rows.steer, 'not in the heartbeat'); set(rows.throttle, 'not in the heartbeat');
    set(rows.stop, '-', 'v');
  } else {
    mode = 'NO SCAN'; cls = 'dim';
    source = 'scan none - ' + st.scanGone;
    set(rows.clear, '-'); set(rows.hits, '-'); set(rows.steer, '-'); set(rows.throttle, '-'); set(rows.stop, '-', 'v');
  }
  if (!cls) cls = (mode === 'cruise' || mode === 'slow') ? 'ok' : WARN_MODES[mode] && mode !== 'stop' ? 'warn' : '';
  set(modeEl, mode.toUpperCase(), cls);
  set(sourceEl, source);

  if (live) {
    const pico = String(pilot.pico || 'unknown');
    set(rows.pico, pico.indexOf('pico ') === 0 ? pico.slice(5) : pico);
    set(rows.lidar, nz(pilot.revPerS, ' rev/s') + (pilot.lidarLost ? '   LOST' : ''), pilot.lidarLost ? 'v warn' : 'v');
    set(rows.revolutions, nz(pilot.revolutions));
    set(rows.timeouts, nz(pilot.timeouts));
  } else if (pilot) {
    set(rows.pico, 'pilot STALE - last heard ' + beat.pilotAgeS.toFixed(0) + ' s ago');
    set(rows.lidar, 'pilot STALE', 'v');
    set(rows.revolutions, 'pilot STALE'); set(rows.timeouts, 'pilot STALE');
  } else if (beat) {
    set(rows.pico, 'pilot not running');
    set(rows.lidar, scan ? 'via scanfeed' : 'pilot not running', 'v');
    set(rows.revolutions, '-'); set(rows.timeouts, '-');
  } else {
    set(rows.pico, 'no answer from the car');
    set(rows.lidar, 'no answer from the car', 'v');
    set(rows.revolutions, '-'); set(rows.timeouts, '-');
  }
  set(rows.cpu, beat && beat.cpuC !== null && beat.cpuC !== undefined ? beat.cpuC.toFixed(1) + ' C' : 'unknown');

  // ---- the controls
  const p = beat && beat.pilotProc;
  const running = !!(p && p.running);
  lookBtn.disabled = !beat || running;
  stopBtn.disabled = !beat || !running;
  lookBtn.className = running || !beat ? '' : 'go';
  stopBtn.className = running ? 'halt' : '';
  let state;
  if (!beat) state = 'pilot: no answer from the car';
  else if (running) state = 'pilot: ' + p.mode + ' ' + p.sinceS.toFixed(0) + ' s (pid ' + p.pid + ')';
  else if (live) state = 'pilot: running elsewhere - not started from this page, so STOP here cannot stop it';
  else if (p && p.exitCode !== null && p.exitCode !== undefined) state = 'pilot: exited ' + p.exitCode + (p.lastLine ? ' - ' + p.lastLine : '');
  else state = 'pilot: not running';
  set(msgEl, Date.now() < st.msgUntil ? st.msg : state);
}

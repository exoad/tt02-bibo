// The page's spine: the state, the two polls, the two views, the
// destinations, and who redraws on what. The server serves data - /scan at
// 10 Hz, /json at 1 Hz, two one-line controls - and everything drawn is
// drawn here, on the phone. One request of each kind in flight at a time,
// and a render only on new data: a phone on a hotspot must never stack
// requests behind a slow one, and must never repaint a picture that has not
// changed.

import { makeScan, parseScan } from './scan.js';
import * as radar from './radar.js';
import * as cloud from './cloud.js';
import * as panels from './panels.js';
import * as log from './log.js';
import * as icons from './icons.js';

const st = {
  scan: null,             // the parsed revolution, or null
  scanGone: 'no answer',  // why, when it is null
  beat: null,             // the last /json body, or null when the car did not answer
};
const scanBuf = makeScan();
let scanText = '';
let scanBusy = false, beatBusy = false;

// Three seconds, and two misses in a row before the word changes: a hotspot
// stalls for a second or two all the time outside, and a big NO ANSWER that
// flashes on every stall is a word nobody trusts by the second outing.
function signal() { return AbortSignal.timeout ? AbortSignal.timeout(3000) : undefined; }
let scanMisses = 0;

function drawScan() {
  const hud = panels.hud(st);
  radar.draw(st.scan, hud);
  cloud.draw(st.scan, hud);
  panels.render(st);
  note();
}

// ---- the log: transitions, not samples ------------------------------------
// The feed's word, the board answering, the pilot process and its last line
// are logged when they change, so the log reads as what happened and when.
const seen = { feed: '', board: '', proc: '', last: '' };
function note() {
  const ls = panels.lidarState(st);
  const feed = st.scan ? 'feed: ' + ls.word.toLowerCase() : 'feed: ' + st.scanGone;
  if (feed !== seen.feed) { seen.feed = feed; log.add(feed); }
  const board = st.beat ? 'board: ' + st.beat.host + ' answering' : 'board: no answer';
  if (board !== seen.board) { seen.board = board; log.add(board); }
  const p = st.beat && st.beat.pilotProc;
  const proc = !p ? '' : p.running ? 'pilot: running ' + p.mode + ' (pid ' + p.pid + ')'
             : p.exitCode !== null && p.exitCode !== undefined ? 'pilot: exited ' + p.exitCode : 'pilot: not running';
  if (proc && proc !== seen.proc) { seen.proc = proc; log.add(proc); }
  const last = p && p.lastLine ? 'pilot said: ' + p.lastLine : '';
  if (last && last !== seen.last) { seen.last = last; log.add(last); }
}

function pollScan() {
  if (scanBusy) return;   // the last one has not come back; do not stack another behind it
  scanBusy = true;
  fetch('/scan', { cache: 'no-store', signal: signal() }).then(function (r) {
    scanMisses = 0;
    return r.text().then(function (t) {
      if (r.status === 200) {
        if (t === scanText) return;   // same revolution; nothing new to draw
        scanText = t; st.scan = parseScan(t, scanBuf); st.scanGone = '';
      } else {
        scanText = ''; st.scan = null; st.scanGone = t.trim() || ('http ' + r.status);
      }
      drawScan();
    });
  }).catch(function () {
    // The car did not answer (hotspot dropped, board rebooted). The last
    // picture is not a picture of now - but one missed poll is a stall, not
    // an absence; the second in a row is.
    if (++scanMisses < 2) return;
    if (st.scan || st.scanGone !== 'no answer') {
      scanText = ''; st.scan = null; st.scanGone = 'no answer';
      drawScan();
    }
  }).then(function () { scanBusy = false; });
}

function pollBeat() {
  if (beatBusy) return;
  beatBusy = true;
  fetch('/json', { cache: 'no-store', signal: signal() })
    .then(function (r) { return r.json(); })
    .then(function (j) { st.beat = j; drawScan(); })
    .catch(function () { if (st.beat) { st.beat = null; drawScan(); } })
    .then(function () { beatBusy = false; });
}

function post(path, verb) {
  fetch(path, { method: 'POST', cache: 'no-store', signal: signal() })
    .then(function (r) { return r.text(); })
    .then(function (t) { say(verb + ': ' + t.trim()); pollBeat(); })
    .catch(function () { say(verb + ': no answer from the car'); });
}

// ---- the snackbar: a control's one-line answer, four seconds ----------------
const snackEl = document.getElementById('snackbar');
let snackTimer = 0;
function say(text) {
  log.add(text);
  snackEl.textContent = text;
  snackEl.hidden = false;
  clearTimeout(snackTimer);
  snackTimer = setTimeout(function () { snackEl.hidden = true; }, 4000);
}

// ---- the views --------------------------------------------------------------
// Three destinations - Scan, Details, Log - in one navigation element. On a
// compact or medium window one pane shows at a time (dash.css, by
// body[data-dest]); from 840 px both panes are always there and Details and
// Log scroll the support pane to their card. ?view=details|log opens one
// directly and the address follows a tap, so a reload - or a link - keeps
// the page. On the dash one scan view at a time: 2D or 3D, the choice
// remembered on the phone; ?tab=3d still picks one for a link.
const stageEl = document.getElementById('stage');
const supportEl = document.getElementById('support');
const radarC = document.getElementById('radar-c'), cloudC = document.getElementById('cloud-c');
const expanded = window.matchMedia ? window.matchMedia('(min-width: 840px) and (min-height: 480px)') : { matches: false };

function remember(key, value) { try { localStorage.setItem(key, value); } catch (e) { /* a private tab; nothing to keep */ } }
function recall(key) { try { return localStorage.getItem(key); } catch (e) { return null; } }

let scanView = '2d';
function selectScanView(name) {
  scanView = name === '3d' ? '3d' : '2d';
  radarC.hidden = scanView !== '2d';
  cloudC.hidden = scanView !== '3d';
  document.querySelectorAll('.seg-btn').forEach(function (b) {
    b.setAttribute('aria-pressed', b.dataset.view === scanView ? 'true' : 'false');
  });
  // A canvas that was hidden had no size; give it its box now that it has one.
  if (scanView === '2d') radar.resize(); else cloud.resize();
  remember('bibo.scanView', scanView);
}

let dest = 'scan';
function selectDest(name) {
  dest = name === 'details' || name === 'log' ? name : 'scan';
  document.body.dataset.dest = dest;
  document.querySelectorAll('.nav-item').forEach(function (b) {
    if (b.dataset.dest === dest) b.setAttribute('aria-current', 'page'); else b.removeAttribute('aria-current');
  });
  if (dest !== 'scan') {
    // The support pane is on screen: scroll it to the destination's card,
    // and the log to its newest line (a hidden pre has no scroll height).
    const card = document.getElementById(dest === 'log' ? 'card-log' : 'card-system');
    if (expanded.matches) card.scrollIntoView({ block: 'start' });
    else supportEl.scrollTop = 0;
    log.follow();
  } else if (expanded.matches) {
    supportEl.scrollTop = 0;
  }
  if (history.replaceState) history.replaceState(null, '', dest === 'scan' ? location.pathname : '?view=' + dest);
}

icons.mount(document);
document.querySelectorAll('.seg-btn').forEach(function (b) {
  b.addEventListener('click', function () { selectScanView(b.dataset.view); });
});
document.querySelectorAll('.nav-item').forEach(function (b) {
  b.addEventListener('click', function () { selectDest(b.dataset.dest); });
});
document.getElementById('about').addEventListener('click', function () { selectDest('details'); });

radar.init(radarC);
cloud.init(cloudC);
log.init(document.getElementById('log'));
panels.bind(function () { post('/pilot/look', 'look'); }, function () { post('/pilot/stop', 'stop'); });

const q = new URLSearchParams(location.search);
selectScanView(q.get('tab') || recall('bibo.scanView') || '2d');
selectDest(q.get('view') || 'scan');

// The canvases follow the stage's box in both axes: a rotation, a
// split-screen, a keyboard opening, the browser's bars coming and going, the
// cards docking beside instead of under.
if (window.ResizeObserver) {
  new ResizeObserver(function () { radar.resize(); cloud.resize(); }).observe(stageEl);
} else {
  window.addEventListener('resize', function () { radar.resize(); cloud.resize(); });
}

log.add('page opened');
drawScan();
setInterval(pollScan, 100);
setInterval(pollBeat, 1000);
pollScan(); pollBeat();

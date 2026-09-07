// The page's spine: the state, the two polls, the two views, and who redraws
// on what. The server serves data - /scan at 10 Hz, /json at 1 Hz, two
// one-line controls - and everything drawn is drawn here, on the phone. One
// request of each kind in flight at a time, and a render only on new data:
// a phone on a hotspot must never stack requests behind a slow one, and
// must never repaint a picture that has not changed.

import { makeScan, parseScan } from './scan.js';
import * as radar from './radar.js';
import * as cloud from './cloud.js';
import * as panels from './panels.js';
import * as log from './log.js';

const st = {
  scan: null,             // the parsed revolution, or null
  scanGone: 'no answer',  // why, when it is null
  beat: null,             // the last /json body, or null when the car did not answer
  msg: '', msgUntil: 0,   // a control's one-line reply, shown for three seconds
};
const scanBuf = makeScan();
let scanText = '';
let scanBusy = false, beatBusy = false;

// Three seconds, and two misses in a row before the word changes: a hotspot
// stalls for a second or two all the time outside, and a big red NO ANSWER
// that flashes on every stall is a word nobody trusts by the second outing.
function signal() { return AbortSignal.timeout ? AbortSignal.timeout(3000) : undefined; }
let scanMisses = 0;

function drawScan() {
  radar.draw(st.scan);
  cloud.draw(st.scan);
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

function say(text) {
  st.msg = text; st.msgUntil = Date.now() + 3000;
  log.add(text);
  panels.render(st);
  setTimeout(function () { panels.render(st); }, 3100);   // back to the state line
}

// ---- the views --------------------------------------------------------------
// Two pages in one document: MAIN, the whole viewport, and DETAILS, a
// scrolling page. ?view=details opens the second directly and the address
// follows a tap, so a reload - or a link - keeps the page. On MAIN one scan
// view at a time: 2D or 3D, the choice remembered on the phone; ?tab=3d
// still picks one for a link.
const mainEl = document.getElementById('main'), detailsEl = document.getElementById('details');
const radarC = document.getElementById('radar-c'), cloudC = document.getElementById('cloud-c');

function remember(key, value) { try { localStorage.setItem(key, value); } catch (e) { /* a private tab; nothing to keep */ } }
function recall(key) { try { return localStorage.getItem(key); } catch (e) { return null; } }

let scanView = '2d';
function selectScanView(name) {
  scanView = name === '3d' ? '3d' : '2d';
  radarC.hidden = scanView !== '2d';
  cloudC.hidden = scanView !== '3d';
  document.querySelectorAll('#seg button').forEach(function (b) {
    b.classList.toggle('on', b.dataset.view === scanView);
  });
  // A canvas that was hidden had no size; give it its box now that it has one.
  if (scanView === '2d') radar.resize(); else cloud.resize();
  remember('bibo.scanView', scanView);
}

function selectPage(name) {
  const details = name === 'details';
  mainEl.hidden = details;
  detailsEl.hidden = !details;
  document.documentElement.style.overflow = details ? '' : 'hidden';
  if (details) log.follow();
  else selectScanView(scanView);
  if (history.replaceState) history.replaceState(null, '', details ? '?view=details' : location.pathname);
}

document.querySelectorAll('#seg button').forEach(function (b) {
  b.addEventListener('click', function () { selectScanView(b.dataset.view); });
});
document.getElementById('info').addEventListener('click', function () { selectPage('details'); });
document.getElementById('back').addEventListener('click', function () { selectPage('main'); });

radar.init(radarC);
cloud.init(cloudC);
log.init(document.getElementById('log'));
panels.bind(function () { post('/pilot/look', 'look'); }, function () { post('/pilot/stop', 'stop'); });

const q = new URLSearchParams(location.search);
selectScanView(q.get('tab') || recall('bibo.scanView') || '2d');
selectPage(q.get('view') === 'details' ? 'details' : 'main');

// The canvases follow the viewport: a rotation, a split-screen, a keyboard
// opening, the browser's bars coming and going.
if (window.ResizeObserver) {
  new ResizeObserver(function () { radar.resize(); cloud.resize(); }).observe(mainEl);
} else {
  window.addEventListener('resize', function () { radar.resize(); cloud.resize(); });
}

log.add('page opened');
drawScan();
setInterval(pollScan, 100);
setInterval(pollBeat, 1000);
pollScan(); pollBeat();

// The dashboard's spine: the state, the two polls, the tabs, and who redraws
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

function signal() { return AbortSignal.timeout ? AbortSignal.timeout(1500) : undefined; }

function drawScan() {
  const hud = panels.hud(st);
  radar.draw(st.scan, st.scanGone, hud);
  cloud.draw(st.scan, st.scanGone, radar.range(), hud);
  panels.render(st);
  note();
}

// ---- the log: transitions, not samples ------------------------------------
// The feed's word, the board answering, the pilot process and its last line
// are logged when they change, so the tab reads as what happened and when.
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
    // picture is not a picture of now.
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

// ---- the tabs --------------------------------------------------------------
// One view at a time, the hub's way; ?tab=2d|3d|log picks one on load and
// the address follows a click, so a reload - or a link - keeps the view.
const TABS = ['2d', '3d', 'log'];
function selectTab(name) {
  if (TABS.indexOf(name) < 0) name = '2d';
  TABS.forEach(function (t) {
    document.getElementById('v-' + t).hidden = (t !== name);
  });
  document.querySelectorAll('#tabs .tab').forEach(function (b) {
    b.classList.toggle('on', b.dataset.tab === name);
  });
  // A view that was hidden had no size; give it its box now that it has one.
  if (name === '2d') radar.resize();
  else if (name === '3d') cloud.resize();
  else log.follow();
  if (history.replaceState) history.replaceState(null, '', '?tab=' + name);
}
document.querySelectorAll('#tabs .tab').forEach(function (b) {
  b.addEventListener('click', function () { selectTab(b.dataset.tab); });
});

radar.init(document.getElementById('radar-c'));
cloud.init(document.getElementById('cloud-c'));
log.init(document.getElementById('log'));
panels.bind(function () { post('/pilot/look', 'look'); }, function () { post('/pilot/stop', 'stop'); });
selectTab(new URLSearchParams(location.search).get('tab') || '2d');

// The canvases follow their frame, which follows the grid: a rotation, a
// split-screen, a keyboard opening.
if (window.ResizeObserver) {
  new ResizeObserver(function () { radar.resize(); cloud.resize(); }).observe(document.getElementById('views'));
} else {
  window.addEventListener('resize', function () { radar.resize(); cloud.resize(); });
}

log.add('page opened');
drawScan();
setInterval(pollScan, 100);
setInterval(pollBeat, 1000);
pollScan(); pollBeat();

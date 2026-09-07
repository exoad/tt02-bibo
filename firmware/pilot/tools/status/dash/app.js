// The dashboard's spine: the state, the two polls, and who redraws on what.
// The server serves data - /scan at 10 Hz, /json at 1 Hz, two one-line
// controls - and everything drawn is drawn here, on the phone. One request
// of each kind in flight at a time, and a render only on new data: a phone
// on a hotspot must never stack requests behind a slow one, and must never
// repaint a picture that has not changed.

import { makeScan, parseScan } from './scan.js';
import * as radar from './radar.js';
import * as cloud from './cloud.js';
import * as panels from './panels.js';

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
  radar.draw(st.scan, st.scanGone);
  cloud.draw(st.scan, st.scanGone, radar.range());
  panels.render(st);
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
    .then(function (j) { st.beat = j; panels.render(st); })
    .catch(function () { if (st.beat) { st.beat = null; panels.render(st); } })
    .then(function () { beatBusy = false; });
}

function post(path) {
  fetch(path, { method: 'POST', cache: 'no-store', signal: signal() })
    .then(function (r) { return r.text(); })
    .then(function (t) { say(t.trim()); pollBeat(); })
    .catch(function () { say('no answer from the car'); });
}

function say(text) {
  st.msg = text; st.msgUntil = Date.now() + 3000;
  panels.render(st);
  setTimeout(function () { panels.render(st); }, 3100);   // back to the state line
}

radar.init(document.getElementById('radar-c'));
cloud.init(document.getElementById('cloud-c'));
panels.bind(function () { post('/pilot/look'); }, function () { post('/pilot/stop'); });

// The canvases follow their panels, which follow the grid: a rotation, a
// split-screen, a keyboard opening.
if (window.ResizeObserver) {
  const ro = new ResizeObserver(function () { radar.resize(); cloud.resize(); });
  ro.observe(document.getElementById('radar'));
  ro.observe(document.getElementById('cloud'));
} else {
  window.addEventListener('resize', function () { radar.resize(); cloud.resize(); });
}

drawScan();
setInterval(pollScan, 100);
setInterval(pollBeat, 1000);
pollScan(); pollBeat();

// The Drive destination: the page as an RC transmitter. W A S D on a laptop,
// four big keys under a thumb on a phone, and the server turns them into
// STEER and ESC lines to the Pico at 20 Hz.
//
// EVERYTHING HERE IS HOLD-TO-DRIVE. A control that latches is a control that
// goes on driving after the person stops paying attention, so key down starts
// sending and key up stops. Let go, leave the tab, lock the phone, walk out of
// Wi-Fi range - the input stops arriving, and that silence is exactly what the
// server's 200 ms deadman is watching for. A backgrounded tab is throttled by
// the browser itself, so the failure mode IS the safety mechanism rather than
// something this file has to detect.
//
// THE SEND LOOP RUNS ONLY WHEN IT HAS SOMETHING TO SAY: while a key is held,
// or while the eased value has not reached zero yet. When everything is
// released the values ease down, ONE zero goes out, and the loop stops - a
// parked car does not need a stream of zeros ten times a second, and the
// server's deadman covers the silence better than a keepalive would.
//
// STEERING IS PROPORTIONAL rather than bang-bang: holding A ramps the steer
// toward -1 over about 250 ms and it returns to 0 when released, and the same
// for the throttle. That is a few lines of easing and it is the difference
// between a car you can drive and one that darts.

import { AHEAD_DEG } from './theme.js';

const RAMP_MS = 250;    // hold a key this long and its axis reaches full
const TICK_MS = 50;     // 20 Hz - the rate the server's deadman is counting on

function el(id) { return document.getElementById(id); }

const held = { w: false, a: false, s: false, d: false };
let steer = 0, throttle = 0;
let timer = 0, busy = false, active = false;
let post = null, say = null;
let linkOpen = false, armed = false;

const openBtn = el('d-open'), armBtn = el('d-arm'), stopBtn = el('d-stop');
const padEl = el('pad');
const aheadV = el('d-ahead');
const out = { link: el('d-link'), armed: el('d-armed'), steer: el('d-steer'),
              throttle: el('d-throttle'), esc: el('d-esc'), age: el('d-age') };
const saidEl = el('d-said');

function text(node, s) { if (node.textContent !== s) node.textContent = s; }
function cls(node, c) { if (node.className !== c) node.className = c; }

// ---- the forward sector ----------------------------------------------------
// The nearest return within +-AHEAD_DEG of straight ahead, in mm, or -1 when
// there is nothing to measure. Computed HERE, from the revolution the page
// already polls: the board serves data and does not do the page's arithmetic.
// Bearings run 0..2pi clockwise from the front, so the sector is the two ends
// of that range, not a middle. Zero-distance samples are already dropped by
// scan.js - 0 mm means no return, not an obstacle at the bumper.
export function aheadMm(scan) {
  if (!scan || !scan.n) return -1;
  const lim = AHEAD_DEG * Math.PI / 180;
  let best = -1;
  for (let i = 0; i < scan.n; i++) {
    let ang = scan.a[i];
    if (ang > Math.PI) ang -= 2 * Math.PI;
    if (ang < -lim || ang > lim) continue;
    const d = scan.d[i];
    if (d > 0 && (best < 0 || d < best)) best = d;
  }
  return best;
}

// Colour by distance, the page's own tones: normal over a metre, warning from
// 0.35 to 1 m, error under 0.35 m.
function aheadTone(mm) { return mm < 350 ? 'error' : mm < 1000 ? 'warning' : 'surface'; }

// ---- the axes --------------------------------------------------------------

function anyHeld() { return held.w || held.a || held.s || held.d; }

function ease(v, target) {
  const step = TICK_MS / RAMP_MS;
  if (v < target) v = Math.min(target, v + step);
  else if (v > target) v = Math.max(target, v - step);
  return Math.abs(v) < 0.004 ? 0 : v;
}

function sendInput() {
  if (busy) return;   // the last one has not come back; the next tick carries a newer value anyway
  busy = true;
  fetch('/car/input', {
    method: 'POST', cache: 'no-store',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ steer: Math.round(steer * 1000) / 1000,
                           throttle: Math.round(throttle * 1000) / 1000 }),
  }).catch(function () { /* the board did not answer; the deadman is the answer */ })
    .then(function () { busy = false; });
}

// THE KEEPALIVE, and it is the proof that this browser is still here - so it
// runs the whole time manual control is held open, not only while a key is
// down. It used to stop as soon as both axes reached zero, and the server's
// 200 ms input deadman then disarmed the car: arm, hesitate a fifth of a
// second, and the throttle was dead before W was ever pressed.
//
// Twenty times a second while something is held, ten when idle - still twice
// inside the deadman, and half the requests on a board that is also serving
// the lidar feed.
const IDLE_MS = 100;

function tick() {
  steer = ease(steer, (held.d ? 1 : 0) - (held.a ? 1 : 0));
  throttle = ease(throttle, (held.w ? 1 : 0) - (held.s ? 1 : 0));
  sendInput();
  paintLocal();
  timer = setTimeout(tick, anyHeld() ? TICK_MS : IDLE_MS);
}

// Runs whenever this destination is showing AND the link is open, and stops
// the moment either stops being true - which is exactly when the deadman
// SHOULD be disarming the car.
function syncLoop() {
  const want = active && linkOpen;
  if (want && !timer) timer = setTimeout(tick, TICK_MS);
  else if (!want && timer) stopLoop();
}

function stopLoop() {
  clearTimeout(timer); timer = 0;
}

// Everything off, NOW, without the ramp: the space bar and the big STOP, where
// easing down over a quarter second is a quarter second of a car nobody wants
// moving. The keepalive keeps running - the link is still ours, the car is
// simply being commanded to nothing.
function panic() {
  held.w = held.a = held.s = held.d = false;
  steer = 0; throttle = 0;
  sendInput();
  paintLocal();
  paintPad();
}

// Panic, and let go of the keepalive too: the tab went away, the window lost
// focus, or this destination stopped showing. The send STOPPING is the point -
// the server's deadman then disarms the car 200 ms later without being asked,
// and that is the behaviour we want when nobody is watching the screen.
function release() {
  panic();
  stopLoop();
}

function grab(k, on) {
  if (held[k] === on) return;
  held[k] = on;
  paintPad();
  // The loop already runs whenever the link is open; a key only changes what
  // the next tick carries. With no link there is nothing to drive.
  syncLoop();
}

// ---- keys ------------------------------------------------------------------

function keyOf(e) {
  if (e.code === 'Space') return 'space';
  const k = (e.key || '').toLowerCase();
  return (k === 'w' || k === 'a' || k === 's' || k === 'd') ? k : '';
}

function onKeyDown(e) {
  if (!active || e.repeat || e.ctrlKey || e.metaKey || e.altKey) return;
  const k = keyOf(e);
  if (!k) return;
  e.preventDefault();     // W and the space bar would otherwise scroll the page
  if (k === 'space') { panic(); post('/car/stop', 'stop'); return; }
  grab(k, true);
}

function onKeyUp(e) {
  if (!active) return;
  const k = keyOf(e);
  if (!k || k === 'space') return;
  e.preventDefault();
  grab(k, false);
}

// ---- the pad ---------------------------------------------------------------

function bindPad() {
  padEl.querySelectorAll('.pad-btn').forEach(function (b) {
    const k = b.dataset.key;
    b.addEventListener('pointerdown', function (e) { e.preventDefault(); grab(k, true); });
    b.addEventListener('pointerup', function (e) { e.preventDefault(); grab(k, false); });
    b.addEventListener('pointercancel', function () { grab(k, false); });
    b.addEventListener('pointerleave', function () { grab(k, false); });
    b.addEventListener('contextmenu', function (e) { e.preventDefault(); });
  });
}

function paintPad() {
  padEl.querySelectorAll('.pad-btn').forEach(function (b) {
    b.setAttribute('aria-pressed', held[b.dataset.key] ? 'true' : 'false');
  });
}

// ---- what the panel says ---------------------------------------------------

// The two values this page is COMMANDING, painted from here at 20 Hz. The
// board's own numbers come back through /json once a second and are painted by
// render(); these are what was asked for, and they must not wait a second.
function paintLocal() {
  text(out.steer, steer === 0 ? '0.00' : (steer < 0 ? '−' : '+') + Math.abs(steer).toFixed(2));
  text(out.throttle, throttle === 0 ? '0.00' : (throttle < 0 ? '−' : '+') + Math.abs(throttle).toFixed(2));
}

export function setActive(on) {
  active = on;
  if (on) syncLoop();
  else release();   // leaving the destination is letting go of everything
}

export function init(poster, sayer) {
  post = poster; say = sayer;
  openBtn.addEventListener('click', function () {
    post(linkOpen ? '/car/close' : '/car/open', linkOpen ? 'release' : 'manual');
  });
  armBtn.addEventListener('click', function () {
    post(armed ? '/car/disarm' : '/car/arm', armed ? 'disarm' : 'arm');
  });
  stopBtn.addEventListener('click', function () { panic(); post('/car/stop', 'stop'); });
  bindPad();
  window.addEventListener('keydown', onKeyDown);
  window.addEventListener('keyup', onKeyUp);
  // A tab that goes away, a window that loses focus: let go of everything. The
  // send stops too, so the server's deadman fires on its own a fifth of a
  // second later - belt and braces, and the braces are the ones on the board.
  window.addEventListener('blur', release);
  document.addEventListener('visibilitychange', function () { if (document.hidden) release(); });
  paintLocal();
  paintPad();
}

// A button in this card is an icon element followed by a bare text node.
// Setting the BUTTON's own textContent wipes both, and the next line then reads
// `lastChild` - suddenly null - and throws, taking the whole render with it.
// That is not hypothetical: the panel froze on "no answer" the instant the
// board first answered, and because note() runs after this, the page's own log
// stopped in the same breath. Replace only the text node.
function label(btn, s) {
  const last = btn.lastChild;
  if (last && last.nodeType === 3) last.textContent = s;
  else btn.appendChild(document.createTextNode(s));
}

export function render(st) {
  const car = st.beat && st.beat.car;

  // The forward sector: the number a person watches while driving. "--" when
  // there is no scan, never the last number we happened to have.
  const mm = aheadMm(st.scan);
  if (mm < 0) { text(aheadV, '--'); cls(aheadV, 'ahead-v num tone-variant'); }
  else { text(aheadV, (mm / 1000).toFixed(1) + ' m'); cls(aheadV, 'ahead-v num tone-' + aheadTone(mm)); }

  if (!car) {
    linkOpen = false; armed = false;
    text(out.link, 'no answer'); text(out.armed, '-');
    text(out.esc, '-'); text(out.age, '-');
    text(saidEl, 'no answer from the car');
    openBtn.disabled = true; armBtn.disabled = true;
    syncLoop();   // no answer from the board is no link to keep alive
    return;
  }

  linkOpen = !!car.open;
  armed = !!car.armed;
  openBtn.disabled = false;
  armBtn.disabled = !linkOpen;
  // The BOARD is the authority on whether the link is open, so the keepalive
  // follows what it says rather than what this page last asked for.
  syncLoop();

  // The buttons say what they will DO, not what is true now.
  label(openBtn, linkOpen ? 'Release' : 'Take manual control');
  label(armBtn, armed ? 'DISARM' : 'ARM');
  cls(armBtn, 'btn ' + (armed ? 'btn-primary' : 'btn-tonal') + ' state target label-large');

  text(out.link, linkOpen ? car.port : 'closed');
  text(out.armed, armed ? 'ARMED' : 'no');
  cls(out.armed, 'li-trail label-large num tone-' + (armed ? 'error' : 'variant'));
  text(out.esc, linkOpen ? car.escUs + ' us' : '-');
  text(out.age, car.inputAgeMs === null || car.inputAgeMs === undefined ? '-' : car.inputAgeMs + ' ms');

  // The one line that says why the car is not moving.
  let note;
  if (car.error) note = car.error;
  else if (!linkOpen) note = 'link closed - take manual control to drive';
  else if (!armed) note = 'open, disarmed - the throttle is ignored until you arm';
  else if (car.deadman) note = 'no input - throttle held at neutral';
  else note = car.lastReply || 'armed; the board has said nothing yet';
  if (car.clamped) note = 'input was out of range and was clamped. ' + note;
  text(saidEl, note);
}

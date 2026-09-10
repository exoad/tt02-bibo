// The camera card: MJPEG from the board, shown only when it is asked for.
//
// WHY THERE IS A BUTTON AT ALL. An <img> pointed at /cam/stream holds its
// connection open for as long as it is attached, and the board runs a capture
// for as long as somebody is connected. Attaching it because a pane happened to
// scroll into view would mean the camera runs whenever Details is open, which
// is the exact thing the server's five-second idle timeout exists to prevent.
// So the stream attaches on a deliberate tap, and detaches on either of two
// things: the same tap again, or leaving this destination.
//
// DETACHING MEANS CLEARING src. That is what closes the connection. Hiding the
// element does not: a hidden <img> goes on costing the board a capture and the
// hotspot half a megabyte a second, and nothing on screen would say so.
let imgEl = null, btnEl = null, stateEl = null, noteEl = null;
let want = false;       // the person tapped the button
let here = false;       // ...and this destination is the one on screen
let say = function () {};

function el(id) { return document.getElementById(id); }
function text(node, s) { if (node && node.textContent !== s) node.textContent = s; }

// The button is an icon element followed by a text node, so the label is the
// LAST CHILD and not the whole content. Writing textContent would delete the
// icon; querying for a span finds nothing and returns the button itself, which
// is how a previous version of this wiped every child and then threw on the
// next render.
function label(btn, s) {
  const last = btn.lastChild;
  if (last && last.nodeType === 3) last.textContent = s;
  else btn.appendChild(document.createTextNode(s));
}

function apply() {
  if (!imgEl) return;
  const on = want && here;
  if (on && !imgEl.getAttribute('src')) {
    // A fresh query each time: a browser that reused a finished stream response
    // would paint one still frame and go on showing it, which looks exactly
    // like a camera that is running and frozen.
    imgEl.setAttribute('src', '/cam/stream?t=' + Date.now());
  } else if (!on && imgEl.getAttribute('src')) {
    imgEl.removeAttribute('src');
  }
  imgEl.hidden = !on;
  if (btnEl) label(btnEl, on ? 'Stop camera' : 'Show camera');
}

export function init(sayer) {
  say = sayer || function () {};
  imgEl = el('cam-img');
  btnEl = el('cam-btn');
  stateEl = el('cam-state');
  noteEl = el('cam-note');
  if (btnEl) {
    btnEl.addEventListener('click', function () {
      want = !want;
      apply();
      say(want ? 'camera: on' : 'camera: off');
    });
  }
  if (imgEl) {
    imgEl.addEventListener('error', function () {
      // The board refused - most often the camera is unplugged, and /cam/stream
      // answered with the reason instead of a picture. Drop the request so the
      // button tells the truth rather than leaving a broken image in the card.
      want = false;
      apply();
      say('camera: no stream - see the card');
    });
  }
  apply();
}

// Leaving this destination detaches the stream whatever the button says. The
// button's state is remembered, so coming back resumes it without a second tap.
export function setActive(on) {
  here = !!on;
  apply();
}

export function render(st) {
  const cam = st.beat && st.beat.camera;
  if (!cam) {
    text(stateEl, '-');
    text(noteEl, 'no answer from the board');
    return;
  }
  text(stateEl, cam.live ? 'live' : 'idle');
  // Live says what it IS; not-live says WHY, which is the server's own sentence
  // - "camera idle", "/dev/video0 absent - camera unplugged". An absence with a
  // reason beats a blank every time.
  text(noteEl, cam.live ? (cam.device + ' ' + cam.size + ', ' + cam.frames + ' frames') : cam.why);
}

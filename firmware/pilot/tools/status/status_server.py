#!/usr/bin/env python3
"""The car's one URL: http://bibobox.local/ on the phone, while walking behind it.

    python3 status_server.py            # port 80 (needs CAP_NET_BIND_SERVICE)
    BIBO_STATUS_PORT=8080 python3 ...   # any port, no privilege

Plain text in a <pre>, refreshed every two seconds, no styling. It is read
standing up in a field; what matters is that every line is TRUE.

WHERE THE NUMBERS COME FROM. The board's own sensors are read here directly
(CPU temperature from /sys, the Wi-Fi profile and address from NetworkManager).
Everything about the lidar and the Pico comes from the pilot, through the status
file it rewrites once a second (app/main.cxx, STATUS_FILE) - the pilot holds the
lidar's serial port exclusively, so this program could not ask the device even
if it wanted to, and a second opener would only steal bytes from the first.

WHAT IS NOT MEASURED IS SAID SO. Nothing on the car reads the pack voltage yet
and there is no pose estimator, so "battery" and "localization" are honest
absences, not blank lines and not zeros. When the pilot is not running, the
lidar and Pico lines say that rather than repeating the last thing they knew:
a status page that reports a stale success is worse than no page.

THE DASHBOARD. /dash is the same page for when the car drives itself: the
last revolution drawn around the car, the corridor the pilot reasons in, and
its decision as one big word. It reads /scan, which is the pilot's scan file
(src/scanwire.hxx, SCAN_FILE: an F line then a D line, rewritten every
revolution) served as-is - the page parses the wire format itself, so there is
one format, not two. A scan file older than three seconds is a 404 and the page
draws NOTHING but the car and says why, for the reason above: a picture of a
wall the car has already left is a lie with better graphics. The page is one
self-contained HTML string because the phone on the hotspot has no internet
and must not need any.

BIBO_SCAN_FILE and BIBO_STATUS_FILE override the two paths so the whole thing
can be exercised on a laptop with a fake pilot writing fake files.
"""
import glob
import http.server
import json
import os
import socket
import subprocess
import time

STATUS_FILE = os.environ.get('BIBO_STATUS_FILE', '/tmp/bibo-pilot.json')  # app/main.cxx
SCAN_FILE = os.environ.get('BIBO_SCAN_FILE', '/tmp/bibo-scan.txt')        # src/scanwire.hxx
STALE_S = 3.0                          # the pilot writes every second
PORT = int(os.environ.get('BIBO_STATUS_PORT', '80'))
LIDAR_DEV = '/dev/ttyUSB0'
PICO_DEV = '/dev/ttyACM0'


def cpu_temp_c():
    """The hotter of the two CPU clusters. The A733 exposes eight zones; the
    idle-governor ones repeat the cluster readings and the GPU, NPU, DDR and
    skin zones are not the CPU."""
    best = None
    for zone in glob.glob('/sys/class/thermal/thermal_zone*'):
        try:
            kind = open(zone + '/type').read().strip()
            if kind.startswith('cpu') and 'idle' not in kind:
                value = int(open(zone + '/temp').read()) / 1000.0
                best = value if best is None else max(best, value)
        except (OSError, ValueError):
            pass
    return best


def run(*argv):
    try:
        return subprocess.run(argv, capture_output=True, text=True, timeout=2).stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        return ''


def wifi_profile():
    out = run('nmcli', '-t', '-f', 'GENERAL.CONNECTION', 'dev', 'show', 'wlan0')
    return out.split(':', 1)[1] if ':' in out else None


def addresses():
    return run('hostname', '-I').split()


def uptime():
    try:
        seconds = float(open('/proc/uptime').read().split()[0])
    except (OSError, ValueError, IndexError):
        return None
    return '%dh%02dm' % (seconds // 3600, (seconds % 3600) // 60)


def board():
    """What the board knows about itself, read here and not through the pilot."""
    return {'host': socket.gethostname(), 'addresses': addresses(),
            'wifi': wifi_profile(), 'up': uptime(), 'cpuC': cpu_temp_c()}


def pilot_status():
    """(status dict or None, age in seconds or None)."""
    try:
        with open(STATUS_FILE) as f:
            status = json.load(f)
        return status, max(0.0, time.time() - float(status.get('ts', 0)))
    except (OSError, ValueError):
        return None, None


def scan_text():
    """(the scan file's bytes, None) while the pilot is writing it, else
    (None, why). The age is the file's mtime: the F line carries no timestamp,
    and the kernel's stamp is the one thing a pilot that died mid-run cannot
    have left looking fresh. fstat on the open handle, not stat by name, so the
    pilot's rename between the two cannot pair one file's age with another's
    bytes."""
    try:
        with open(SCAN_FILE, 'rb') as f:
            age = time.time() - os.fstat(f.fileno()).st_mtime
            data = f.read()
    except OSError:
        # No file, but a fresh heartbeat: the pilot is alive and simply does
        # not write one (an older build). Saying "not running" next to a
        # heartbeat that says it is would be two lines contradicting each other.
        status, beat_age = pilot_status()
        if status is not None and beat_age <= STALE_S:
            return None, 'no scan file'
        return None, 'pilot not running'
    if age > STALE_S:
        return None, 'scan stale'
    return data, None


def fmt(value, unit='', missing='unknown'):
    return missing if value is None else '%s%s' % (value, unit)


def lines(me):
    now = time.strftime('%Y-%m-%d %H:%M:%S')
    status, age = pilot_status()
    live = status is not None and age <= STALE_S
    out = []
    out.append('%s  %s  %s  up %s  %s' % (me['host'],
                                            ' '.join(me['addresses']) or 'no address',
                                            fmt(me['wifi'], missing='no wifi'),
                                            fmt(me['up']), now))
    temp = me['cpuC']
    out.append('cpu           %s' % ('%.1f C' % temp if temp is not None else 'unknown'))
    out.append('battery       not measured - nothing on the car reads the pack yet')

    if live:
        out.append('pilot         running, status %.0f s old' % age)
        out.append('lidar         %s rev/s  mode %s  clear %s mm  hits %s%s' % (
            fmt(status.get('revPerS')), fmt(status.get('mode')),
            fmt(status.get('clearanceMm')), fmt(status.get('hits')),
            '  LOST' if status.get('lidarLost') else ''))
        # The pilot's phrase is the console's, and the console's begins with the
        # word this line already starts with.
        pico = fmt(status.get('pico'))
        out.append('pico          %s' % (pico[5:] if pico.startswith('pico ') else pico))
    else:
        if status is None:
            out.append('pilot         not running (no status file)')
        else:
            out.append('pilot         STALE - last status %.0f s ago' % age)
        out.append('lidar         %s; pilot not running' %
                   ('%s present' % LIDAR_DEV if os.path.exists(LIDAR_DEV) else
                    '%s absent - lidar unplugged' % LIDAR_DEV))
        out.append('pico          %s; pilot not running' %
                   ('%s present' % PICO_DEV if os.path.exists(PICO_DEV) else
                    '%s absent - Pico unplugged' % PICO_DEV))
    out.append('localization  none - reactive layer only, no pose estimate')
    return out, status, age


# The dashboard, whole. Inline everything: the phone is on the car's hotspot
# and a page that needs a CDN would be a page that is blank in the field.
# Dark 2010s utility app, on purpose: one ground, one ink, one accent, and a
# warning tint for the two modes where the car is doing something you did not
# ask for. The palette is the hub's (Gruvbox), so the two screens read as one.
DASH_HTML = r"""<!doctype html>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>bibo</title>
<style>
html,body{margin:0;background:#1d2021;color:#ebdbb2;
  font:14px/1.45 ui-monospace,Menlo,Consolas,"DejaVu Sans Mono",monospace}
body{max-width:600px;margin:0 auto;padding-bottom:12px}
canvas{display:block;width:100%}
#strip{padding:4px 10px;white-space:pre-wrap}
#mode{font-size:64px;line-height:1.05;font-weight:bold;margin:2px 0 6px}
.ok{color:#b8bb26}.warn{color:#fe8019}.dim{opacity:.55}
</style>
<canvas id="c"></canvas>
<div id="strip"><div id="mode">-</div><div id="lines"></div></div>
<script>
'use strict';
var BG = '#1d2021', FG = '#ebdbb2', ACCENT = '#b8bb26', WARN = '#fe8019';
var FAINT = 'rgba(235,219,178,0.28)';
var HALF_WIDTH_MM = 160;   // reactive::Config::halfWidthMm - the corridor the pilot reasons in
var STEER_DEG = 30;        // full lock drawn at 30 degrees; a fraction has no angle of its own
var STALE_S = 3;           // the server's rule for /scan, applied here to /json as well
var CAR_MM = [190, 430];   // the TT-02's width and length, so the car is drawn to the same scale
var MIN_RANGE_MM = 3000;

var canvas = document.getElementById('c'), ctx = canvas.getContext('2d');
var modeEl = document.getElementById('mode'), linesEl = document.getElementById('lines');

var scan = null;        // the last good /scan: {samples:[{a,d}], hz, drive, bad}
var scanGone = 'no answer';   // why there is no scan, when scan is null
var scanText = '';      // the last /scan body, to render only on new data
var beat = null;        // the last /json body, or null when the car did not answer
var scanBusy = false, beatBusy = false;   // never two of the same request in flight
var rangeMm = MIN_RANGE_MM, smallerFor = 0;

function fit() {
  // Square, the width of the phone, drawn at device pixels so the dots are dots.
  var w = canvas.clientWidth, dpr = window.devicePixelRatio || 1;
  canvas.style.height = w + 'px';
  canvas.width = Math.round(w * dpr); canvas.height = Math.round(w * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}

function parseScan(text) {
  // The wire format of src/scanwire.hxx, by split: F <n> <mHz> a,d,q ... then
  // D <mode> <clear> <hits> <steer> <throttle> <stop>. A frame whose count does
  // not match is BAD, not shorter - the same rule as the C++ reader.
  var out = {samples: [], hz: 0, drive: null, bad: false};
  var lines = text.split('\n');
  for (var i = 0; i < lines.length; i++) {
    var w = lines[i].trim().split(' ');
    if (w[0] === 'F') {
      out.hz = Number(w[2]) / 1000;
      if (w.length - 3 !== Number(w[1])) { out.bad = true; continue; }
      for (var k = 3; k < w.length; k++) {
        var p = w[k].split(',');
        var d = Number(p[1]);
        if (d > 0) out.samples.push({a: Number(p[0]) / 100, d: d});   // 0 mm is no return
      }
    } else if (w[0] === 'D' && w.length >= 7) {
      out.drive = {mode: w[1], clearMm: Number(w[2]), hits: Number(w[3]),
                   steer: Number(w[4]) / 1000, throttle: Number(w[5]) / 1000,
                   stop: w[6] === '1'};
    }
  }
  return out;
}

function ringStep(mm) { return mm > 10000 ? 5000 : mm > 5000 ? 2000 : 1000; }

function updateRange() {
  var far = 0;
  if (scan) for (var i = 0; i < scan.samples.length; i++) if (scan.samples[i].d > far) far = scan.samples[i].d;
  var step = ringStep(far);
  var want = Math.max(MIN_RANGE_MM, Math.ceil(far / step) * step);
  // Grow at once - a wall that just appeared must be on the page - but shrink
  // only after it has been smaller for about two seconds, or the picture
  // breathes with every dropped return at the far wall.
  if (want >= rangeMm) { rangeMm = want; smallerFor = 0; }
  else if (++smallerFor > 20) { rangeMm = want; smallerFor = 0; }
}

function polar(cx, cy, s, aDeg, dMm) {
  // Lidar zero is up and angles run clockwise, which is the screen's own sense.
  var r = aDeg * Math.PI / 180;
  return [cx + dMm * s * Math.sin(r), cy - dMm * s * Math.cos(r)];
}

function draw() {
  var W = canvas.clientWidth, H = W, cx = W / 2, cy = H / 2;
  ctx.fillStyle = BG; ctx.fillRect(0, 0, W, H);
  updateRange();
  var s = (W / 2 - 10) / rangeMm;   // px per mm

  // Range rings, labelled inside so the outer label stays on the page.
  ctx.strokeStyle = FAINT; ctx.fillStyle = FAINT; ctx.lineWidth = 1;
  ctx.font = '12px ui-monospace,Menlo,Consolas,monospace'; ctx.textAlign = 'right';
  for (var r = ringStep(rangeMm); r <= rangeMm; r += ringStep(rangeMm)) {
    ctx.beginPath(); ctx.arc(cx, cy, r * s, 0, 2 * Math.PI); ctx.stroke();
    ctx.fillText((r / 1000) + ' m', cx + r * s * 0.707 - 4, cy - r * s * 0.707 + 14);
  }

  // The corridor: what the pilot looks down. Two lines ahead at +-halfWidth.
  ctx.beginPath();
  ctx.moveTo(cx - HALF_WIDTH_MM * s, cy); ctx.lineTo(cx - HALF_WIDTH_MM * s, 0);
  ctx.moveTo(cx + HALF_WIDTH_MM * s, cy); ctx.lineTo(cx + HALF_WIDTH_MM * s, 0);
  ctx.stroke();

  if (scan) {
    ctx.fillStyle = FG;
    for (var i = 0; i < scan.samples.length; i++) {
      var p = polar(cx, cy, s, scan.samples[i].a, scan.samples[i].d);
      ctx.fillRect(p[0] - 1, p[1] - 1, 2, 2);
    }
    var d = scan.drive;
    if (d) {
      // The clearance, as a bar across the corridor at the distance the pilot acted on.
      if (d.clearMm > 0 && d.clearMm <= rangeMm) {
        ctx.strokeStyle = ACCENT; ctx.lineWidth = 2; ctx.beginPath();
        ctx.moveTo(cx - HALF_WIDTH_MM * s - 4, cy - d.clearMm * s);
        ctx.lineTo(cx + HALF_WIDTH_MM * s + 4, cy - d.clearMm * s);
        ctx.stroke();
      }
      // The heading: where the wheels point. Ink, not accent, when the throttle
      // is zero - the car is not going there, it is only pointing there.
      var L = W * 0.22, ang = d.steer * STEER_DEG;
      var tip = polar(cx, cy, 1, ang, L);
      var l1 = polar(tip[0], tip[1], 1, ang + 150, 9), l2 = polar(tip[0], tip[1], 1, ang - 150, 9);
      ctx.strokeStyle = d.throttle !== 0 ? ACCENT : FG; ctx.lineWidth = 2; ctx.beginPath();
      ctx.moveTo(cx, cy); ctx.lineTo(tip[0], tip[1]);
      ctx.moveTo(l1[0], l1[1]); ctx.lineTo(tip[0], tip[1]); ctx.lineTo(l2[0], l2[1]);
      ctx.stroke();
    }
  }

  // The car, nose up, at the scan's origin. To scale, but never smaller than a
  // thumbnail can be pointed at.
  var cw = Math.max(8, CAR_MM[0] * s), ch = Math.max(16, CAR_MM[1] * s);
  ctx.fillStyle = FG; ctx.fillRect(cx - cw / 2, cy - ch / 2, cw, ch);
  ctx.fillStyle = BG; ctx.fillRect(cx - cw / 2, cy - ch / 2 + ch * 0.25, cw, 1);   // the windscreen

  if (!scan) {
    // Nothing to draw is drawn as nothing, plus the reason, big: below the
    // car, where no ring label lives, on a patch of ground so no ring cuts it.
    var text = scanGone.toUpperCase();
    var size = Math.min(28, Math.floor((W - 24) / (text.length * 0.62)));
    ctx.font = 'bold ' + size + 'px ui-monospace,Menlo,Consolas,monospace';
    var tw = ctx.measureText(text).width, ty = cy + H * 0.25;
    ctx.fillStyle = BG; ctx.fillRect(cx - tw / 2 - 8, ty - size, tw + 16, size * 1.4);
    ctx.fillStyle = FG; ctx.textAlign = 'center';
    ctx.fillText(text, cx, ty);
  }
}

function signed(v) { return (v < 0 ? '' : '+') + v.toFixed(3); }
function nz(v, unit) { return v === undefined || v === null ? 'unknown' : v + (unit || ''); }

function strip() {
  var pilot = beat && beat.pilot, live = pilot && beat.pilotAgeS <= STALE_S;
  var d = scan && scan.drive;
  var mode, cls = '', out = [];
  // The first line says where the big word came from, every time.
  if (d) {
    mode = d.mode;
    out.push('scan      live, ' + scan.hz.toFixed(1) + ' Hz' + (scan.bad ? ' - F line unreadable, not drawn' : ''));
    out.push('clear     ' + d.clearMm + ' mm   hits ' + d.hits + (d.stop ? '   STOP' : ''));
    out.push('steer     ' + signed(d.steer) + '   throttle ' + signed(d.throttle));
  } else if (live) {
    // No scan, but the heartbeat is fresh: its second-old numbers, said to be so.
    mode = pilot.mode;
    out.push('scan      none - ' + scanGone);
    out.push('mode      the heartbeat\'s, ' + beat.pilotAgeS.toFixed(0) + ' s old');
    out.push('clear     ' + nz(pilot.clearanceMm, ' mm') + '   hits ' + nz(pilot.hits));
    out.push('steer     -   throttle -   (not in the heartbeat)');
  } else {
    mode = 'NO PILOT'; cls = 'dim';
    out.push('scan      none - ' + scanGone);
    out.push('clear     -   hits -');
    out.push('steer     -   throttle -');
  }
  if (!cls) cls = (mode === 'cruise' || mode === 'slow') ? 'ok' : (mode === 'stop') ? '' : 'warn';

  if (live) {
    var pico = String(pilot.pico || 'unknown');
    out.push('pico      ' + (pico.indexOf('pico ') === 0 ? pico.slice(5) : pico));
    out.push('lidar     ' + nz(pilot.revPerS, ' rev/s') + (pilot.lidarLost ? '   LOST' : ''));
  } else if (pilot) {
    out.push('pico      pilot STALE - last heard ' + beat.pilotAgeS.toFixed(0) + ' s ago');
    out.push('lidar     pilot STALE');
  } else if (beat) {
    out.push('pico      pilot not running');
    out.push('lidar     pilot not running');
  } else {
    out.push('pico      no answer from the car');
    out.push('lidar     no answer from the car');
  }
  out.push('cpu       ' + (beat && beat.cpuC !== null && beat.cpuC !== undefined ? beat.cpuC.toFixed(1) + ' C' : 'unknown'));
  out.push('board     ' + (beat ? beat.host + '  ' + (beat.addresses.join(' ') || 'no address') : 'no answer'));

  modeEl.textContent = mode.toUpperCase(); modeEl.className = cls;
  var text = out.join('\n');
  if (linesEl.textContent !== text) linesEl.textContent = text;
}

function signal() { return AbortSignal.timeout ? AbortSignal.timeout(1500) : undefined; }

function pollScan() {
  if (scanBusy) return;   // the last one has not come back; do not stack another behind it
  scanBusy = true;
  fetch('/scan', {cache: 'no-store', signal: signal()}).then(function (r) {
    return r.text().then(function (t) {
      if (r.status === 200) {
        if (t === scanText) return;   // same revolution; nothing new to draw
        scanText = t; scan = parseScan(t); scanGone = '';
      } else {
        scanText = ''; scan = null; scanGone = t.trim() || ('http ' + r.status);
      }
      draw(); strip();
    });
  }).catch(function () {
    // The car did not answer (hotspot dropped, board rebooted). The last
    // picture is not a picture of now.
    if (scan || scanGone !== 'no answer') { scanText = ''; scan = null; scanGone = 'no answer'; draw(); strip(); }
  }).then(function () { scanBusy = false; });
}

function pollBeat() {
  if (beatBusy) return;
  beatBusy = true;
  fetch('/json', {cache: 'no-store', signal: signal()}).then(function (r) { return r.json(); })
    .then(function (j) { beat = j; strip(); })
    .catch(function () { if (beat) { beat = null; strip(); } })
    .then(function () { beatBusy = false; });
}

fit(); draw(); strip();
window.addEventListener('resize', function () { fit(); draw(); });
setInterval(pollScan, 100);
setInterval(pollBeat, 1000);
pollScan(); pollBeat();
</script>
"""


class Page(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        path = self.path.split('?', 1)[0]
        if path == '/scan':
            data, why = scan_text()
            if data is None:
                self.reply(404, 'text/plain; charset=utf-8', (why + '\n').encode())
            else:
                self.reply(200, 'text/plain; charset=utf-8', data)
        elif path == '/dash':
            self.reply(200, 'text/html; charset=utf-8', DASH_HTML.encode())
        else:
            me = board()
            text, status, age = lines(me)
            if path.startswith('/json'):
                body = json.dumps({'lines': text, 'pilot': status, 'pilotAgeS': age,
                                   'host': me['host'], 'addresses': me['addresses'],
                                   'cpuC': me['cpuC']}).encode()
                self.reply(200, 'application/json', body)
            else:
                text.append('dashboard: /dash')
                self.reply(200, 'text/html; charset=utf-8',
                           ('<meta http-equiv="refresh" content="2"><pre>%s</pre>\n'
                            % '\n'.join(text)).encode())

    def reply(self, code, kind, body):
        self.send_response(code)
        self.send_header('Content-Type', kind)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')   # every answer is about NOW
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *_):
        pass    # a phone refreshing every two seconds is not journal material


if __name__ == '__main__':
    http.server.ThreadingHTTPServer(('', PORT), Page).serve_forever()

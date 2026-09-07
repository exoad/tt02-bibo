#!/usr/bin/env python3
"""The car's one URL: http://bibobox.local/ on the phone, while walking behind it.

    python3 status_server.py            # port 80 (needs CAP_NET_BIND_SERVICE)
    BIBO_STATUS_PORT=8080 python3 ...   # any port, no privilege

Plain text in a <pre>, refreshed every two seconds, no styling. It is read
standing up in a field; what matters is that every line is TRUE.

WHERE THE NUMBERS COME FROM. The board's own sensors are read here directly
(CPU temperature from /sys, the Wi-Fi profile and address from NetworkManager).
The Pico's state comes from the pilot, through the heartbeat it rewrites once a
second (app/main.cxx, STATUS_FILE). The lidar comes from the scan feed on port
8011 (tools/scanfeed.cxx): this program is one more client of it, exactly like
the hub, so the page shows the lidar whether or not a pilot is running - the
feed spins the device up for us and, while a pilot drives, hands us through to
the pilot's own stream, decisions included. We could not open the device
ourselves in any case: whoever has it holds the port exclusively.

A PAGE NOBODY IS LOOKING AT MUST NOT KEEP THE MOTOR SPINNING. The feed is held
open only while /scan has been asked for in the last five seconds; after that
we say QUIT and the feed parks the lidar. Close the tab, and the bearing rests.

WHAT IS NOT MEASURED IS SAID SO. Nothing on the car reads the pack voltage yet
and there is no pose estimator, so "battery" and "localization" are honest
absences, not blank lines and not zeros. When the pilot is not running, the
Pico line says that rather than repeating the last thing it knew: a status page
that reports a stale success is worse than no page.

THE DASHBOARD. /dash is the page for when the car drives itself: the last
revolution drawn around the car, the corridor the pilot reasons in, and its
decision as one big word. It reads /scan - the feed's F line and, when a pilot
is sending them, its D line (src/scanwire.hxx) served as-is, so the page parses
the one wire format and there is no second one. A revolution older than three
seconds is a 404 and the page draws NOTHING but the car and says why: a picture
of a wall the car has already left is a lie with better graphics. The page is
the static files in dash/ next to this script, served from here and nowhere
else - no CDN, no fonts, no framework - because the phone on the hotspot has
no internet and a page that needs any is a page that is blank in the field.
All drawing is the phone's; this program serves data.

THE PILOT, FROM THE PHONE. In the field there is no ssh, so /pilot/look starts
the pilot in dry mode (it decides at 10 Hz and touches nothing) and /pilot/stop
stops it. Starting it ARMED is deliberately not on this page: a button any
phone on the hotspot can tap is not the deliberate act the car's rules require
before it moves (hub: "reactive drives nothing until a person armed the ESC").
That step stays with a person at a shell, until it has a gate worth trusting.
The pilot is this process's child, in its own session so a closing tab cannot
take it down; but it is inside this service's cgroup, so when the hotspot drops
and systemd stops the page, the pilot is stopped too and sends STOP to the car
on its way out. That is the degradation chain working - the link failed and the
car stopped - not a bug.

BIBO_FEED, BIBO_STATUS_FILE, BIBO_SCAN_FILE, BIBO_PILOT and BIBO_PILOT_ARGS
override the feed address, the two paths, the pilot binary and its fixed extra
arguments, so the whole thing can be exercised on a laptop against fakes.
"""
import glob
import http.server
import json
import os
import pwd
import signal
import socket
import subprocess
import threading
import time
import urllib.parse

STATUS_FILE = os.environ.get('BIBO_STATUS_FILE', '/tmp/bibo-pilot.json')  # app/main.cxx
SCAN_FILE = os.environ.get('BIBO_SCAN_FILE', '/tmp/bibo-scan.txt')        # src/scanwire.hxx
FEED = os.environ.get('BIBO_FEED', '127.0.0.1:8011')                        # tools/scanfeed.cxx
STALE_S = 3.0                          # the pilot writes every second; the lidar turns ten times
DRIVE_FRESH_S = 1.5                    # a decision this old is the last pilot's, not this scan's
FEED_WANT_S = 5.0                      # how long a /scan request keeps the feed open
PORT = int(os.environ.get('BIBO_STATUS_PORT', '80'))
LIDAR_DEV = '/dev/ttyUSB0'
PICO_DEV = '/dev/ttyACM0'

HOME = pwd.getpwuid(os.getuid()).pw_dir     # systemd's User= does not promise $HOME
PILOT_BIN = os.environ.get('BIBO_PILOT', os.path.join(HOME, 'build-pilot-app', 'pilot'))
PILOT_ARGS = os.environ.get('BIBO_PILOT_ARGS', '').split()
PILOT_LOG = '/tmp/bibo-pilot.log'


# ---------------------------------------------------------------- the board

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


class Facts:
    """The board's description of itself, CACHED.

    `nmcli` and `hostname` are subprocesses, and /json is asked once a second by
    every viewer: answering it by spawning two processes each time is a cost the
    board pays out of the same core the pilot drives on. The cheap facts (two
    file reads) refresh every couple of seconds; the network ones every ten,
    which is quicker than a hotspot handover takes anyway.

    The subprocesses run OUTSIDE the lock. Two threads may then refresh at once
    - rare, and the cost of that is one extra `nmcli`, against holding every
    other request behind a two-second timeout if it went the other way.
    """

    NET_TTL_S = 10.0
    FAST_TTL_S = 2.0

    def __init__(self):
        self.lock = threading.Lock()
        self.net = None
        self.net_at = 0.0
        self.fast = None
        self.fast_at = 0.0

    def board(self):
        now = time.monotonic()
        with self.lock:
            net, net_at = self.net, self.net_at
            fast, fast_at = self.fast, self.fast_at
        if net is None or now - net_at > self.NET_TTL_S:
            net = {'host': socket.gethostname(), 'addresses': addresses(), 'wifi': wifi_profile()}
            with self.lock:
                self.net, self.net_at = net, now
        if fast is None or now - fast_at > self.FAST_TTL_S:
            fast = {'up': uptime(), 'cpuC': cpu_temp_c()}
            with self.lock:
                self.fast, self.fast_at = fast, now
        out = dict(net)
        out.update(fast)
        return out


facts = Facts()


def board():
    """What the board knows about itself, read here and not through the pilot."""
    return facts.board()


def pilot_status():
    """(heartbeat dict or None, age in seconds or None)."""
    try:
        with open(STATUS_FILE) as f:
            status = json.load(f)
        return status, max(0.0, time.time() - float(status.get('ts', 0)))
    except (OSError, ValueError):
        return None, None


# ---------------------------------------------------------------- the feed

class Feed:
    """One client of the scan feed, kept open only while somebody is looking.

    A thread owns the socket; the handlers only read the latest lines under
    the lock and bump `wanted_until`. Reconnects every two seconds while
    wanted - the feed comes and goes with the hotspot and with a pilot taking
    the port over - and sends QUIT the moment nobody has asked for five
    seconds, which is what lets scanfeed park the lidar."""

    def __init__(self, target):
        host, _, port = target.rpartition(':')
        self.addr = (host or '127.0.0.1', int(port))
        self.lock = threading.Lock()
        self.wanted_until = 0.0
        self.frame = None          # the latest F line, bytes, without its newline
        self.frame_at = 0.0
        self.drive = None          # the latest D line, bytes
        self.drive_at = 0.0
        self.why = 'feed idle'     # what to say when there is no revolution to show
        self.motor = None          # the feed's last MOTOR line, True/False
        # Bumped by every F and D line, and handed out as the /scan ETag. A
        # viewer polling ten times a second usually asks about a revolution it
        # already has; a counter turns that into a 304 and no kilobytes.
        self.rev = 0
        threading.Thread(target=self.loop, daemon=True).start()

    def want(self):
        with self.lock:
            if time.time() >= self.wanted_until:
                self.why = 'feed connecting'   # the first ask after a lull; the thread is on it
            self.wanted_until = time.time() + FEED_WANT_S

    def wanted(self):
        with self.lock:
            return time.time() < self.wanted_until

    def note(self, why):
        with self.lock:
            self.why = why

    def loop(self):
        while True:
            if not self.wanted():
                with self.lock:
                    self.frame = None
                    self.drive = None
                    self.why = 'feed idle'
                time.sleep(0.2)
                continue
            try:
                sock = socket.create_connection(self.addr, timeout=2.0)
            except OSError:
                self.note('no scan feed on the board - is bibo-scanfeed running?')
                time.sleep(2.0)
                continue
            self.note('lidar spinning up')
            try:
                self.serve(sock)
            except OSError:
                self.note('feed closed')
            finally:
                try:
                    sock.sendall(b'QUIT\n')
                except OSError:
                    pass
                sock.close()
            with self.lock:
                self.frame = None
                self.drive = None
            time.sleep(1.0)

    def serve(self, sock):
        sock.settimeout(1.0)
        buf = b''
        while self.wanted():
            try:
                data = sock.recv(65536)
            except socket.timeout:
                continue
            if not data:
                self.note('feed closed')
                return
            buf += data
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                self.take(line.rstrip(b'\r'))
            if len(buf) > 262144:
                buf = b''   # a line this long is not a line; start over at the next newline

    def take(self, line):
        now = time.time()
        with self.lock:
            if line.startswith(b'F '):
                self.frame = line
                self.frame_at = now
                self.rev += 1
            elif line.startswith(b'D '):
                self.drive = line
                self.drive_at = now
                self.rev += 1
            elif line.startswith(b'MOTOR '):
                self.motor = line.endswith(b'1')
                if not self.motor:
                    self.why = 'motor off'
            elif line.startswith(b'ERR '):
                self.why = 'feed: ' + line[4:].decode('utf-8', 'replace')

    def scan(self):
        """(bytes, None, revision) with the latest revolution and a fresh
        decision if any, else (None, why, revision)."""
        self.want()
        with self.lock:
            now = time.time()
            if self.frame is not None and now - self.frame_at <= STALE_S:
                out = self.frame + b'\n'
                if self.drive is not None and now - self.drive_at <= DRIVE_FRESH_S:
                    out += self.drive + b'\n'
                return out, None, self.rev
            if self.frame is not None:
                return None, 'scan stale', self.rev
            return None, self.why, self.rev


feed = Feed(FEED)


def scan_file():
    """The pilot's own scan file, the fallback for a pilot started with
    --no-feed or a board without the feed service: (bytes, None) while fresh,
    else (None, why). fstat on the open handle, so the pilot's rename between
    the two calls cannot pair one file's age with another's bytes."""
    try:
        with open(SCAN_FILE, 'rb') as f:
            age = time.time() - os.fstat(f.fileno()).st_mtime
            data = f.read()
    except OSError:
        return None, 'pilot not running'
    if age > STALE_S:
        return None, 'scan stale'
    return data, None


def scan_text():
    """(bytes, None, tag) or (None, why, tag). `tag` is the ETag for a 200 and
    None otherwise - a reason is cheap to re-send and must not be cached."""
    data, why, rev = feed.scan()
    if data is not None:
        return data, None, '"f%d"' % rev
    if why.startswith('no scan feed'):
        # No feed to be a client of. The pilot may still be writing its file.
        file_data, file_why = scan_file()
        if file_data is not None:
            # The file has no revision counter, so the bytes are their own tag.
            return file_data, None, '"s%d"' % (hash(file_data) & 0xFFFFFFFF)
        if file_why != 'pilot not running':
            return None, file_why, None
    return None, why, None


# ---------------------------------------------------------------- the pilot

class Pilot:
    """The pilot as this service's child: started in dry mode from the page,
    stopped from the page, its exit remembered. One at a time - the lidar's
    port is one thing, and so is the car."""

    def __init__(self):
        self.lock = threading.Lock()
        self.proc = None
        self.mode = None
        self.since = 0.0
        self.exit_code = None
        self.last_line = ''

    def start(self, mode):
        argv = [PILOT_BIN, '--dry'] + PILOT_ARGS
        with self.lock:
            if self.proc is not None and self.proc.poll() is None:
                return 409, 'already running (%s, %.0f s)' % (self.mode, time.time() - self.since)
            if not os.access(PILOT_BIN, os.X_OK):
                return 409, 'no pilot binary at ' + PILOT_BIN
            try:
                log = open(PILOT_LOG, 'wb')
                # Its own session: a tab closing, or this handler's thread
                # ending, must not be what stops a car. See the header for
                # what DOES stop it (the whole service going down).
                self.proc = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=log,
                                             stderr=subprocess.STDOUT, start_new_session=True)
                log.close()
            except OSError as e:
                return 500, 'could not start the pilot: %s' % e
            self.mode = mode
            self.since = time.time()
            self.exit_code = None
            self.last_line = ''
            threading.Thread(target=self.reap, args=(self.proc,), daemon=True).start()
            return 200, 'started ' + mode

    def reap(self, proc):
        code = proc.wait()
        with self.lock:
            if self.proc is proc:
                self.exit_code = code
                self.last_line = tail(PILOT_LOG)

    def stop(self):
        with self.lock:
            proc = self.proc
        if proc is None or proc.poll() is not None:
            return 409, 'not running'
        # SIGINT is Ctrl-C: the pilot sends STOP to the car, parks the lidar
        # and exits 0. Three seconds is longer than its shutdown takes; after
        # that SIGTERM, which it also handles the same way.
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            proc.terminate()
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                return 500, 'the pilot did not stop - kill it by hand'
        return 200, 'stopped'

    def state(self):
        with self.lock:
            running = self.proc is not None and self.proc.poll() is None
            return {'running': running, 'mode': self.mode if running else None,
                    'pid': self.proc.pid if running else None,
                    'sinceS': time.time() - self.since if running else None,
                    'exitCode': None if running else self.exit_code,
                    'lastLine': '' if running else self.last_line}


def tail(path):
    try:
        with open(path, 'rb') as f:
            data = f.read()[-4096:]
        lines = [l for l in data.decode('utf-8', 'replace').splitlines() if l.strip()]
        return lines[-1] if lines else ''
    except OSError:
        return ''


pilot = Pilot()


# ---------------------------------------------------------------- the text page

def fmt(value, unit='', missing='unknown'):
    return missing if value is None else '%s%s' % (value, unit)


def control_line(p):
    if p['running']:
        return 'control       %s running %.0f s  (pid %d)' % (p['mode'], p['sinceS'], p['pid'])
    if p['exitCode'] is None:
        return 'control       not running'
    return 'control       exited %d - %s' % (p['exitCode'], p['lastLine'] or 'no output')


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
    out.append(control_line(pilot.state()))
    out.append('localization  none - reactive layer only, no pose estimate')
    return out, status, age


# ---------------------------------------------------------------- the dashboard

# The client app lives in dash/ beside this file and is served as it is on
# disk. The rules, in order: /dash and /dash/ are index.html; /dash/<name> is
# that file, only if its resolved path is still inside dash/ (so ../ and its
# percent-encoded spellings get a 404, not the source of this program), only
# if it is a file and not a directory (no listings), and only if it is one of
# the kinds a page is made of - a stray editor backup in the directory is not
# served. Every answer is no-store like the rest: a phone that cached last
# week's app.js against this week's server would be a page that lies quietly.
DASH_DIR = os.path.realpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dash'))
DASH_KINDS = {'.html': 'text/html; charset=utf-8',
              '.css': 'text/css; charset=utf-8',
              '.js': 'text/javascript; charset=utf-8',
              '.svg': 'image/svg+xml',
              '.png': 'image/png'}


def dash_file(path):
    """(content type, bytes) for a /dash... request path, or None for a 404."""
    if path in ('/dash', '/dash/'):
        rel = 'index.html'
    elif path.startswith('/dash/'):
        rel = urllib.parse.unquote(path[len('/dash/'):])
    else:
        return None
    if not rel or '\0' in rel:
        return None
    full = os.path.realpath(os.path.join(DASH_DIR, rel))
    if not full.startswith(DASH_DIR + os.sep) or not os.path.isfile(full):
        return None
    kind = DASH_KINDS.get(os.path.splitext(full)[1].lower())
    if kind is None:
        return None
    try:
        with open(full, 'rb') as f:
            return kind, f.read()
    except OSError:
        return None


# ---------------------------------------------------------------- the server

class Page(http.server.BaseHTTPRequestHandler):
    # HTTP/1.1, so a viewer polling ten times a second reuses ONE connection
    # instead of making the board accept, thread and tear down ten a second.
    # It is safe here because every reply carries a Content-Length.
    protocol_version = 'HTTP/1.1'

    # A kept-alive connection holds a thread; this is what lets go of one whose
    # phone walked out of range.
    timeout = 30

    def do_GET(self):
        path = self.path.split('?', 1)[0]
        if path == '/scan':
            data, why, tag = scan_text()
            if data is None:
                self.reply(404, 'text/plain; charset=utf-8', (why + '\n').encode())
            elif self.headers.get('If-None-Match') == tag:
                # The same revolution the viewer already has. Saying so costs a
                # header; sending it again costs five kilobytes, ten times a
                # second, over a phone hotspot.
                self.reply(304, 'text/plain; charset=utf-8', b'', tag)
            else:
                self.reply(200, 'text/plain; charset=utf-8', data, tag)
        elif path == '/dash' or path.startswith('/dash/'):
            found = dash_file(path)
            if found is None:
                self.reply(404, 'text/plain; charset=utf-8', b'no such page\n')
            else:
                self.reply(200, found[0], found[1])
        elif path.startswith('/pilot/'):
            self.reply(405, 'text/plain; charset=utf-8', b'POST, not GET: a page reload must not start a car\n')
        else:
            me = board()
            text, status, age = lines(me)
            if path.startswith('/json'):
                body = json.dumps({'lines': text, 'pilot': status, 'pilotAgeS': age,
                                   'host': me['host'], 'addresses': me['addresses'],
                                   'cpuC': me['cpuC'], 'pilotProc': pilot.state()}).encode()
                self.reply(200, 'application/json', body)
            else:
                text.append('dashboard: /dash')
                self.reply(200, 'text/html; charset=utf-8',
                           ('<meta http-equiv="refresh" content="2"><pre>%s</pre>\n'
                            % '\n'.join(text)).encode())

    def do_POST(self):
        # No authentication: the hotspot is the trust boundary, and nothing
        # here can move the car - look mode touches nothing, stop only stops.
        path = self.path.split('?', 1)[0]
        if path == '/pilot/look':
            code, text = pilot.start('look')
        elif path == '/pilot/stop':
            code, text = pilot.stop()
        else:
            code, text = 404, 'no such control'
        self.reply(code, 'text/plain; charset=utf-8', (text + '\n').encode())

    def reply(self, code, kind, body, tag=None):
        self.send_response(code)
        self.send_header('Content-Type', kind)
        self.send_header('Content-Length', str(len(body)))
        # no-store, and an ETag beside it: the tag is not a licence to keep the
        # answer, it is how the viewer asks "still the same one?" - which is the
        # question that saves the board from re-sending a revolution it already
        # sent. `must-revalidate` says the same thing to anything in between.
        self.send_header('Cache-Control', 'no-cache, must-revalidate' if tag else 'no-store')
        if tag:
            self.send_header('ETag', tag)
        self.end_headers()
        if body:
            self.wfile.write(body)

    def log_message(self, *_):
        pass    # a phone refreshing every two seconds is not journal material


if __name__ == '__main__':
    http.server.ThreadingHTTPServer(('', PORT), Page).serve_forever()

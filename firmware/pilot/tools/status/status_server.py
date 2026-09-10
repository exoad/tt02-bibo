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

DRIVING IT BY HAND. /dash's Drive destination is an RC transmitter: WASD on a
laptop, arrows under a thumb on a phone, held down. The browser sends steer and
throttle twenty times a second to /car/open's link, and the Car class here is
the only thing in this process that opens the Pico. ONE OWNER AT A TIME - the
pilot process holds that port when it runs, so /car/open refuses while it is
running and /pilot/look refuses while the manual link is open. Arming is its
own request and nothing moves until it happens, and three deadmen in a chain
stop the car when the person stops asking for it: see Car.sender.

BIBO_FEED, BIBO_STATUS_FILE, BIBO_SCAN_FILE, BIBO_PILOT and BIBO_PILOT_ARGS
override the feed address, the two paths, the pilot binary and its fixed extra
arguments, so the whole thing can be exercised on a laptop against fakes.
BIBO_PICO, BIBO_ESC_BAND and BIBO_ALLOW_REVERSE do the same for the car: point
BIBO_PICO at a pseudo-terminal and the whole manual path can be driven with no
Pico attached at all.
"""
import atexit
import collections
import errno
import fcntl
import glob
import http.server
import json
import os
import pwd
import select
import signal
import socket
import subprocess
import termios
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

# ---- the manual link ------------------------------------------------------
# The Pico, driven BY HAND from the page. Everything else in this file serves
# the car that drives itself; this serves the car a person drives, and it is
# the only thing in this process that opens the Pico's port.
PICO_PORT = os.environ.get('BIBO_PICO', PICO_DEV)
NEUTRAL_US = 1500                       # chassis.hxx DRIVE_NEUTRAL_US
SEND_HZ = 20.0                          # the sender's tick; see Car.sender
SEND_PERIOD_S = 1.0 / SEND_HZ
INPUT_DEADMAN_S = 0.200                 # no input for this long and the throttle goes neutral
DEADMAN_DISARM_TICKS = 2                # ...and after this many ticks of it, disarmed
WRITE_DEADLINE_S = 0.1                  # a CDC port whose board stopped reading must not hang the sender


def esc_band():
    """The working throttle band, microseconds, from BIBO_ESC_BAND.

    Default 1500..1560 - deliberately gentle. The calibration in
    lib/chassis/cal.hxx is stale brushed-motor numbers; the QuicRun 10BL160 G2
    that replaced them maps 1500..2000 nearly linearly, so the old "1600 is a
    crawl" is not true any more and a wide band on a stand is how something
    gets broken. Widen it from a measured test, not from here.
    """
    parts = os.environ.get('BIBO_ESC_BAND', '1500 1560').split()
    try:
        lo, hi = int(parts[0]), int(parts[1])
    except (IndexError, ValueError):
        return NEUTRAL_US, 1560
    return (lo, hi) if lo < hi else (NEUTRAL_US, 1560)


# Forward only by default, matching the pilot. Note that the FIRMWARE refuses
# reverse whatever this says: chassis.hxx pins ESC_HARD_MIN at 1500 and clamps
# both setThrottleLimits() and throttleUs() into it, so a pulse below neutral
# cannot be commanded at all. The flag is honoured here so this file is not the
# thing standing in the way once the ESC is in a mode that has a reverse; until
# then setting it buys nothing.
ALLOW_REVERSE = bool(os.environ.get('BIBO_ALLOW_REVERSE'))


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


# ---------------------------------------------------------------- the car

class Car:
    """The Pico link for MANUAL driving: one port, one sender, two deadmen.

    ARMING IS A SEPARATE ACT. `armed` starts false and throttle is ignored
    entirely while it is - every tick sends ESC NEUTRAL - so a page that is
    merely open cannot move the car. Arming sends ESC ARM; anything that goes
    wrong disarms.

    SERVO ON IS NOT OPTIONAL, and it is the part that is easy to leave out.
    The board's steering pin is RELEASED at boot (chassis.hxx rule 1: 1500 us
    is not a safe place to park a linkage whose horn is a tooth off its
    spline), and drive::stop() releases it again. STEER only sets a target;
    pump() writes the pin `if(servoLive)`. So a link that sent nothing but
    STEER and ESC would get "OK drive ..." back for every command and turn no
    wheels at all - success reported over a measurement nobody made. SERVO ON
    goes out on open and again on every arm, because STOP will have dropped it.
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.txlock = threading.Lock()
        self.fd = -1
        self.port = PICO_PORT
        self.armed = False
        self.steer = 0.0
        self.throttle = 0.0
        self.clamped = False        # the last input was out of range and was clamped
        self.esc_us = NEUTRAL_US
        self.last_input = 0.0       # monotonic; when /car/input last arrived
        self.had_input = False
        self.deadman = False        # the 200 ms input deadman is holding the throttle at neutral
        self.deadman_ticks = 0
        self.error = ''
        self.replies = collections.deque(maxlen=8)
        self.running = False
        self.threads = []
        self.band = esc_band()

    # ---- state -------------------------------------------------------------

    def is_open(self):
        with self.lock:
            return self.fd >= 0

    def state(self):
        with self.lock:
            opened = self.fd >= 0
            age = None
            if opened and self.had_input:
                age = int(max(0.0, time.monotonic() - self.last_input) * 1000)
            return {'open': opened, 'armed': self.armed,
                    'steerCmd': round(self.steer, 3), 'throttleCmd': round(self.throttle, 3),
                    'escUs': self.esc_us, 'port': self.port, 'inputAgeMs': age,
                    'lastReply': self.replies[-1] if self.replies else '',
                    'deadman': self.deadman, 'clamped': self.clamped,
                    'band': list(self.band), 'reverse': ALLOW_REVERSE,
                    'error': self.error}

    # ---- the port ----------------------------------------------------------

    def open(self):
        """(code, one line). The pilot owns the Pico when it runs; one owner."""
        if pilot.state()['running']:
            return 409, 'the pilot is driving - stop it first'
        with self.lock:
            if self.fd >= 0:
                return 200, 'manual control ready'
        try:
            # O_NOCTTY so a modem-control line cannot make this port the
            # process's controlling terminal and SIGHUP us. O_NONBLOCK so a
            # board that stopped draining its CDC buffer gives the sender
            # EAGAIN against a deadline instead of blocking it forever - a
            # sender thread stuck in write() is a car still holding its last
            # throttle.
            fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as e:
            if e.errno in (errno.ENOENT, errno.ENXIO, errno.ENODEV, errno.ENOTDIR):
                return 404, 'no Pico at %s - is it plugged in?' % self.port
            if e.errno in (errno.EACCES, errno.EPERM):
                return 403, 'permission denied on %s - is this user in dialout?' % self.port
            if e.errno == errno.EBUSY:
                return 409, 'another program holds %s' % self.port
            return 500, '%s: %s' % (self.port, e.strerror or e)
        try:
            self.raw(fd)
        except OSError as e:
            os.close(fd)
            return 500, '%s opened but is not a serial line: %s' % (self.port, e.strerror or e)
        with self.lock:
            self.fd = fd
            self.armed = False
            self.steer = 0.0
            self.throttle = 0.0
            self.esc_us = NEUTRAL_US
            self.deadman = False
            self.deadman_ticks = 0
            self.had_input = False
            self.last_input = time.monotonic()
            self.error = ''
            self.replies.clear()
            self.running = True
            self.threads = [threading.Thread(target=self.reader, args=(fd,), daemon=True),
                            threading.Thread(target=self.sender, daemon=True)]
        for t in self.threads:
            t.start()
        # The band first, then the steering engaged, then neutral held. Not
        # ESC ARM: that is the person's own act, through /car/arm.
        self.send('ESCLIMITS %d %d' % self.band)
        self.send('SERVO ON')
        self.send('ESC NEUTRAL')
        return 200, 'manual control ready'

    def raw(self, fd):
        """cfmakeraw, by hand: no echo, no canonical mode, no CR/LF translation
        in either direction, no flow control, VMIN 0 VTIME 0. The line
        discipline would otherwise eat the bytes the protocol is made of. It is
        USB CDC so the rate is ignored by the hardware; 115200 is set anyway so
        the port is not left at whatever the last opener wanted."""
        mode = termios.tcgetattr(fd)
        iflag, oflag, cflag, lflag, ispeed, ospeed, cc = mode
        iflag &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP |
                   termios.INLCR | termios.IGNCR | termios.ICRNL | termios.IXON |
                   termios.IXOFF | termios.IXANY)
        oflag &= ~termios.OPOST
        lflag &= ~(termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG | termios.IEXTEN)
        cflag &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB | termios.CRTSCTS)
        cflag |= termios.CS8 | termios.CLOCAL | termios.CREAD
        cc = list(cc)
        cc[termios.VMIN] = 0
        cc[termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, [iflag, oflag, cflag, lflag,
                                                termios.B115200, termios.B115200, cc])
        termios.tcflush(fd, termios.TCIFLUSH)
        # Exclusive, the way the pilot's carlink takes it: a second opener gets
        # EBUSY rather than half of every line. Not fatal if the ioctl is
        # refused - the link works, it is just not alone.
        try:
            fcntl.ioctl(fd, termios.TIOCEXCL)
        except OSError:
            pass

    def close(self):
        """Always safe, and safe to call twice."""
        with self.lock:
            fd = self.fd
        if fd < 0:
            return 200, 'manual control was not open'
        self.command('STOP')
        with self.lock:
            self.armed = False
            self.running = False
            self.fd = -1
            self.steer = self.throttle = 0.0
            self.esc_us = NEUTRAL_US
            threads = self.threads
            self.threads = []
        for t in threads:
            t.join(timeout=1.0)
        try:
            os.close(fd)
        except OSError:
            pass
        return 200, 'manual control released'

    def lost(self, why):
        """The board went away under us. Disarm, drop the port, say why."""
        with self.lock:
            if self.fd < 0:
                return
            fd = self.fd
            self.fd = -1
            self.running = False
            self.armed = False
            self.esc_us = NEUTRAL_US
            self.error = why
        try:
            os.close(fd)
        except OSError:
            pass

    # ---- writing -----------------------------------------------------------

    def send(self, line):
        """One command line. False when it did not go."""
        with self.lock:
            fd = self.fd
        if fd < 0:
            return False
        data = (line + '\n').encode()
        end = time.monotonic() + WRITE_DEADLINE_S
        with self.txlock:
            while data:
                try:
                    n = os.write(fd, data)
                    data = data[n:]
                except BlockingIOError:
                    left = end - time.monotonic()
                    if left <= 0:
                        return False        # the board is not taking bytes; the next tick tries again
                    select.select([], [fd], [], left)
                except OSError as e:
                    self.lost('write: %s' % (e.strerror or e))
                    return False
        return True

    def command(self, line):
        """A one-off command from a handler, outside the sender's tick."""
        return self.send(line)

    # ---- reading -----------------------------------------------------------

    def reader(self, fd):
        """The board's replies. Every ESC and STEER is answered with an OK
        drive line, so at 20 Hz this drains 40 lines a second - which is the
        point: a CDC buffer nobody empties blocks the board's own printf for
        half a second at a time, and that is the console the car speaks through."""
        buf = b''
        while True:
            with self.lock:
                if not self.running or self.fd != fd:
                    return
            try:
                r, _, _ = select.select([fd], [], [], 0.1)
            except OSError:
                return
            if not r:
                continue
            try:
                data = os.read(fd, 4096)
            except BlockingIOError:
                continue
            except OSError as e:
                self.lost('read: %s' % (e.strerror or e))
                return
            if not data:
                # EOF on a tty: the cable came out, or the board rebooted.
                self.lost('the Pico went away - cable out, or it rebooted')
                return
            buf += data
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                line = line.strip().decode('utf-8', 'replace')
                if line:
                    with self.lock:
                        self.replies.append(line)
            if len(buf) > 65536:
                buf = b''

    # ---- the sender --------------------------------------------------------

    def sender(self):
        """20 Hz, every tick, whether or not anything changed.

        THE DEADMAN IS THREE LAYERS, and each covers a failure the others
        cannot:

          1. THE BROWSER sends /car/input every 50 ms while anything is held.
             That is the keepalive; there is no separate one.

          2. THIS SENDER, 200 ms. No input for that long and the throttle goes
             to NEUTRAL, and after two ticks of it the ESC is disarmed. This
             covers the tab closing, the tab being backgrounded and throttled
             by the browser, the phone locking, and Wi-Fi dropping - all of
             which stop the input arriving and none of which the car can see.

          3. THE PICO, 400 ms (DEADMAN_MS in app/main.cxx). This is the only
             layer that covers the ORANGE PI ITSELF hanging or the USB link
             dying, because nothing running on the Pi can save you from the
             Pi. Do NOT tighten that constant: pilot/app/main.cxx waits up to
             REV_WAIT_MS = 200 ms for a lidar revolution precisely because it
             is half of the board's 400, and taking the board to 200 would
             leave the autonomous path with no margin at all. Manual driving
             gets its tightness from layer 2 instead.

        Writing at a steady rate is also what keeps layer 3 fed while driving:
        the board resets its timer on any command line, and STEER is sent every
        tick even when the throttle is neutral.
        """
        next_at = time.monotonic()
        while True:
            next_at += SEND_PERIOD_S
            sleep = next_at - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_at = time.monotonic()      # we fell behind; do not spin to catch up

            with self.lock:
                if not self.running or self.fd < 0:
                    return
                stale = (time.monotonic() - self.last_input) > INPUT_DEADMAN_S
                if stale:
                    self.deadman_ticks += 1
                else:
                    self.deadman_ticks = 0
                    # Input is flowing again, so the deadman's own message must
                    # not outlive the condition it describes. This panel has one
                    # job - to say why the car is not moving - and "input
                    # stopped for 200 ms" while input is arriving every 50 ms is
                    # the one lie it is able to tell. The screen showed exactly
                    # that: the message sitting there at 61 ms of input age.
                    if self.error.startswith('input stopped'):
                        self.error = ''
                self.deadman = stale
                disarm = stale and self.armed and self.deadman_ticks >= DEADMAN_DISARM_TICKS
                if disarm:
                    self.armed = False
                    self.throttle = 0.0
                armed = self.armed
                steer = self.steer
                throttle = 0.0 if (stale or not armed) else self.throttle
                us = self.pulse(throttle) if armed else NEUTRAL_US
                self.esc_us = us

            self.send('STEER %.3f' % steer)
            self.send('ESC NEUTRAL' if us == NEUTRAL_US else 'ESC %d' % us)
            if disarm:
                # Said after the throttle is already neutral, not before.
                self.send('ESC DISARM')
                with self.lock:
                    self.error = 'input stopped for %d ms - disarmed' % int(INPUT_DEADMAN_S * 1000)

    def pulse(self, throttle):
        """A throttle fraction onto the working band. 0 is neutral; 1 is the
        band's top. Reverse mirrors the same span below neutral, and only with
        BIBO_ALLOW_REVERSE - see the note by ALLOW_REVERSE for why the board
        will refuse it anyway."""
        lo, hi = self.band
        span = hi - lo
        if throttle > 0:
            return int(round(lo + throttle * span))
        if throttle < 0 and ALLOW_REVERSE:
            return int(round(NEUTRAL_US + throttle * span))
        return NEUTRAL_US

    # ---- what the page asks for -------------------------------------------

    def arm(self):
        if not self.is_open():
            return 409, 'manual control is not open'
        # SERVO ON again: a STOP since the last arm released the steering, and
        # a car that steers nothing is the failure this whole class is careful
        # about. Engaging writes the measured centre immediately.
        self.send('SERVO ON')
        if not self.send('ESC ARM'):
            return 500, 'could not reach the Pico'
        with self.lock:
            self.armed = True
            self.throttle = 0.0
            self.last_input = time.monotonic()
            self.deadman_ticks = 0
            self.error = ''
        return 200, 'armed - the throttle is live'

    def disarm(self):
        if not self.is_open():
            return 409, 'manual control is not open'
        with self.lock:
            self.armed = False
            self.throttle = 0.0
            self.esc_us = NEUTRAL_US
        self.send('ESC NEUTRAL')
        self.send('ESC DISARM')
        return 200, 'disarmed'

    def input(self, steer, throttle):
        """The 20 Hz one. Out-of-range values are CLAMPED rather than refused -
        a joystick that overshoots by a thousandth must not stop the car - and
        the clamp is reported in the state instead."""
        if not self.is_open():
            return 409, 'manual control is not open'
        clamped = not (-1.0 <= steer <= 1.0 and -1.0 <= throttle <= 1.0)
        steer = max(-1.0, min(1.0, steer))
        throttle = max(-1.0, min(1.0, throttle))
        if throttle < 0 and not ALLOW_REVERSE:
            throttle = 0.0
        with self.lock:
            self.steer = steer
            self.throttle = throttle
            self.clamped = clamped
            self.last_input = time.monotonic()
            self.had_input = True
            self.deadman_ticks = 0
        return 204, ''

    def stop(self):
        """The emergency one: STOP, disarmed, the command cleared. STOP also
        releases the steering on the board, which is why arm() re-engages it."""
        with self.lock:
            self.armed = False
            self.steer = self.throttle = 0.0
            self.esc_us = NEUTRAL_US
            opened = self.fd >= 0
        if opened:
            self.send('ESC NEUTRAL')
            self.send('STOP')
        return opened


car = Car()


def car_stop_all():
    """The big red button and the space bar: the manual link stopped, and the
    pilot stopped too if it is the thing driving."""
    said = []
    if car.stop():
        said.append('STOP sent, disarmed')
    else:
        said.append('manual control is not open')
    if pilot.state()['running']:
        code, text = pilot.stop()
        said.append('pilot ' + text)
    return 200, '; '.join(said)


def car_shutdown():
    """Nothing may outlive this process armed."""
    try:
        car.close()
    except Exception:
        pass


atexit.register(car_shutdown)


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
        elif path.startswith('/car/'):
            self.reply(405, 'text/plain; charset=utf-8',
                       b'POST, not GET: a page reload must not arm or drive a car\n')
        else:
            me = board()
            text, status, age = lines(me)
            if path.startswith('/json'):
                body = json.dumps({'lines': text, 'pilot': status, 'pilotAgeS': age,
                                   'host': me['host'], 'addresses': me['addresses'],
                                   'cpuC': me['cpuC'], 'pilotProc': pilot.state(),
                                   'car': car.state()}).encode()
                self.reply(200, 'application/json', body)
            else:
                text.append('dashboard: /dash')
                self.reply(200, 'text/html; charset=utf-8',
                           ('<meta http-equiv="refresh" content="2"><pre>%s</pre>\n'
                            % '\n'.join(text)).encode())

    def take_body(self):
        """The request body, read whole. A body left unread on a kept-alive
        connection is the next request's first line."""
        try:
            n = int(self.headers.get('Content-Length') or 0)
        except ValueError:
            return b''
        return self.rfile.read(n) if 0 < n <= 65536 else b''

    def do_POST(self):
        # No authentication: the hotspot is the trust boundary. What has
        # changed since that was written is that something here CAN now move
        # the car - so the deliberate act is /car/arm, its own request, and
        # every layer under it holds the car still until somebody makes it.
        path = self.path.split('?', 1)[0]
        body = self.take_body()

        if path == '/car/input':
            # The 20 Hz one. 204 and no body: this is answered twenty times a
            # second per viewer and the cheapest true answer is nothing.
            try:
                want = json.loads(body or b'{}')
                steer = float(want.get('steer', 0.0))
                throttle = float(want.get('throttle', 0.0))
            except (ValueError, TypeError, AttributeError):
                self.reply(400, 'text/plain; charset=utf-8',
                           b'want {"steer": -1..1, "throttle": -1..1}\n')
                return
            code, text = car.input(steer, throttle)
            if code == 204:
                self.send_response(204)
                self.end_headers()
                return
            self.reply(code, 'text/plain; charset=utf-8', (text + '\n').encode())
            return

        if path == '/pilot/look':
            # One owner at a time, said from both sides.
            if car.is_open():
                code, text = 409, 'manual control holds the Pico - release it first'
            else:
                code, text = pilot.start('look')
        elif path == '/pilot/stop':
            code, text = pilot.stop()
        elif path == '/car/open':
            code, text = car.open()
        elif path == '/car/close':
            code, text = car.close()
        elif path == '/car/arm':
            code, text = car.arm()
        elif path == '/car/disarm':
            code, text = car.disarm()
        elif path == '/car/stop':
            code, text = car_stop_all()
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
    # systemd stops this with SIGTERM, and Python's default handler exits
    # WITHOUT running atexit - which would leave the car armed with nobody
    # asking it to be. Both signals go out through the shutdown instead.
    def bye(_signum, _frame):
        car_shutdown()
        raise SystemExit(0)

    signal.signal(signal.SIGTERM, bye)
    signal.signal(signal.SIGINT, bye)
    http.server.ThreadingHTTPServer(('', PORT), Page).serve_forever()

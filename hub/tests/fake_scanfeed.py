#!/usr/bin/env python3
"""A stand-in for the Orange Pi's scanfeed, for test_lidarnet.cxx.

    python fake_scanfeed.py PORT [--sessions LIST] [--log FILE] [--timeout S]
                                [--drive] [--pilot]

Listens on 127.0.0.1:PORT and serves one client at a time. On connect it sends
INFO, HEALTH 0 and MOTOR 1, then a synthetic room - a 4 m x 3 m rectangle seen
from its centre, 360 samples one degree apart, quality 47 - at 10 Hz as F lines,
exactly as firmware/pilot/src/scanwire.hxx specifies. It honours what the hub
sends: MOTOR 0 stops the frames and echoes MOTOR 0, MOTOR 1 restarts them and
echoes MOTOR 1, QUIT closes the session.

Every F line carries a TAG in its rotation rate - 10000 + (frame index mod 100)
mHz, so 10.000..10.099 Hz - and the D line that follows it carries the same tag
in its hit count. That is how the test proves a decision was matched to the
revolution it judged rather than to the one before.

--drive makes it the PILOT's feed rather than a bare scanfeed's: three "D blind"
lines a tenth of a second apart before the first F (the pilot deciding on an
empty scan while the lidar spins up), then a D line after every F, the mode
cycling cruise / slow / stop / reverse five frames at a time with values that
would be plausible for each.

--pilot makes MOTOR 0 a request the board refuses: it answers MOTOR 1, its true
state, and keeps the frames coming - the pilot keeps its lidar while it drives.

--sessions is a comma-separated list, one word per successive connection, and
the process exits once they have all been served (or --timeout seconds pass,
so a test that crashes leaves no server behind):

    normal       the room, until the client quits or goes
    junk         the room, plus - after the third frame - one F line with the
                 wrong sample count and one line the hub has no name for
    abrupt:N     the room, then a hard reset of the connection after N frames
    err:N        the room, then "ERR ..." and an orderly close after N frames
    pilotquit:N  with --drive: D lines for the first N frames only, then the
                 room carries on without them - the pilot stopped and scanfeed
                 did not

Everything it sees and does goes to --log, one line each, because the test
reads it afterwards to prove the hub said MOTOR 0 / MOTOR 1 / QUIT - and, with
--pilot, said MOTOR 0 exactly once. stdlib only.
"""
import argparse
import math
import select
import socket
import struct
import sys
import time

HALF_W_MM = 2000.0   # the room: 4 m across ...
HALF_H_MM = 1500.0   # ... and 3 m deep, sensor at the centre
QUALITY = 47
HZ = 10.0

INFO_LINE = b"INFO 65 1.2 24 ABCDEF0123456789ABCDEF0123456789\n"

# The pilot's decision per mode: (clearance mm, throttle milli, stop). Steering
# is a slow wander, the same for every mode, so a run of D lines looks like a
# car easing around a room rather than a table of constants.
MODES = ("cruise", "slow", "stop", "reverse")
MODE_VALUES = {
    "cruise":  (2600, 250, 0),
    "slow":    (1500, 150, 0),
    "stop":    (480, 0, 1),
    "reverse": (200, -200, 0),
}
SPINUP_BLIND = 3   # D-only ticks before the first F, with --drive


def room_body():
    """One revolution of the rectangle, without the F prefix."""
    parts = []
    for deg in range(360):
        r = math.radians(deg)
        c, s = math.cos(r), math.sin(r)
        tx = HALF_W_MM / abs(c) if abs(c) > 1e-9 else float("inf")
        ty = HALF_H_MM / abs(s) if abs(s) > 1e-9 else float("inf")
        parts.append("%d,%d,%d" % (deg * 100, int(round(min(tx, ty))), QUALITY))
    return " ".join(parts)


BODY = room_body()


def frame_line(index):
    """The F line for revolution `index`, tagged in its rotation rate."""
    return ("F 360 %d " % (int(HZ * 1000) + index % 100) + BODY + "\n").encode()


def drive_line(index):
    """The D line for revolution `index`, tagged in its hit count."""
    mode = MODES[(index // 5) % len(MODES)]
    clear, throttle, stop = MODE_VALUES[mode]
    steer = int(round(300.0 * math.sin(index / 7.0)))
    return ("D %s %d %d %d %d %d\n" % (mode, clear, index % 100, steer, throttle, stop)).encode()


BLIND_LINE = b"D blind 0 0 0 0 1\n"


class Log:
    def __init__(self, path):
        self.f = open(path, "a", encoding="utf-8") if path else None

    def __call__(self, text):
        line = "%.3f %s" % (time.monotonic(), text)
        if self.f:
            self.f.write(line + "\n")
            self.f.flush()
        else:
            print(line, flush=True)


def serve(conn, scenario, log, drive, pilot):
    kind, _, arg = scenario.partition(":")
    limit = int(arg) if arg else 0

    # A dozen-byte D right after an 8 KB F is exactly the write Nagle holds
    # back until the F is acknowledged, and Windows delays that ACK by up to
    # 200 ms. The test matches each decision to its revolution within a fixed
    # wait, so the line goes out when it is written.
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    conn.sendall(INFO_LINE)
    conn.sendall(b"HEALTH 0\n")
    conn.sendall(b"MOTOR 1\n")
    log("session %s: sent INFO/HEALTH/MOTOR 1" % scenario)

    motor = True
    frames = 0
    spinup = SPINUP_BLIND if drive else 0
    rx = b""
    next_frame = time.monotonic()
    while True:
        ready, _, _ = select.select([conn], [], [], 0.02)
        if ready:
            data = conn.recv(4096)
            if not data:
                log("client closed")
                return
            rx += data
            while b"\n" in rx:
                raw, rx = rx.split(b"\n", 1)
                line = raw.strip().decode(errors="replace")
                log("rx %s" % line)
                if line == "QUIT":
                    log("QUIT received; parking the device")
                    conn.close()
                    return
                if line == "MOTOR 0":
                    if pilot:
                        # The pilot's answer: its true state, and no change.
                        log("MOTOR 0 refused: the pilot is driving; answering MOTOR 1")
                        conn.sendall(b"MOTOR 1\n")
                    else:
                        motor = False
                        conn.sendall(b"MOTOR 0\n")
                elif line == "MOTOR 1":
                    motor = True
                    next_frame = time.monotonic()
                    conn.sendall(b"MOTOR 1\n")

        now = time.monotonic()
        if motor and now >= next_frame:
            if spinup > 0:
                # The pilot ticks before the lidar has produced a revolution
                # and says so: a decision with nothing behind it.
                spinup -= 1
                next_frame = now + 0.1
                conn.sendall(BLIND_LINE)
                if spinup == 0:
                    log("sent %d D-only blind ticks" % SPINUP_BLIND)
                continue

            next_frame = now + 1.0 / HZ
            conn.sendall(frame_line(frames))
            if drive and not (kind == "pilotquit" and frames >= limit):
                conn.sendall(drive_line(frames))
            if kind == "pilotquit" and frames == limit:
                log("the pilot quit after %d frames; the room carries on" % limit)
            frames += 1
            if kind == "junk" and frames == 3:
                # A frame that promises five samples and carries two, then a
                # line from a board that speaks a newer dialect.
                conn.sendall(b"F 5 10000 0,100,47 100,100,47\n")
                conn.sendall(b"HELLO from a newer board\n")
                log("sent one bad F line and one unknown line")
            if kind == "abrupt" and frames >= limit:
                # SO_LINGER with a zero timeout turns close() into a reset:
                # what a board rebooting looks like from the other end.
                log("resetting the connection after %d frames" % frames)
                conn.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER, struct.pack("ii", 1, 0))
                conn.close()
                return
            if kind == "err" and frames >= limit:
                log("sending ERR after %d frames" % frames)
                conn.sendall(b"ERR lidar reports an internal error (fake)\n")
                conn.close()
                return


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", type=int)
    ap.add_argument("--sessions", default="normal")
    ap.add_argument("--log", default="")
    ap.add_argument("--timeout", type=float, default=90.0)
    ap.add_argument("--drive", action="store_true", help="the pilot's feed: D lines")
    ap.add_argument("--pilot", action="store_true", help="MOTOR 0 is refused with MOTOR 1")
    args = ap.parse_args()

    log = Log(args.log)
    deadline = time.monotonic() + args.timeout

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", args.port))
    srv.listen(1)
    log("LISTENING on 127.0.0.1:%d%s%s" % (args.port,
                                           " --drive" if args.drive else "",
                                           " --pilot" if args.pilot else ""))

    for scenario in args.sessions.split(","):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            log("timeout before session %s; exiting" % scenario)
            return 2
        srv.settimeout(remaining)
        try:
            conn, peer = srv.accept()
        except socket.timeout:
            log("timeout waiting for a client; exiting")
            return 2
        log("accepted %s:%d" % peer)
        try:
            serve(conn, scenario, log, args.drive, args.pilot)
        except (ConnectionError, OSError) as e:
            log("session ended: %s" % e)
        finally:
            try:
                conn.close()
            except OSError:
                pass

    log("all sessions served; exiting")
    return 0


if __name__ == "__main__":
    sys.exit(main())

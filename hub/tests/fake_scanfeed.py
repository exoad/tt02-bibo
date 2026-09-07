#!/usr/bin/env python3
"""A stand-in for the Orange Pi's scanfeed, for test_lidarnet.cxx.

    python fake_scanfeed.py PORT [--sessions LIST] [--log FILE] [--timeout S]

Listens on 127.0.0.1:PORT and serves one client at a time. On connect it sends
INFO, HEALTH 0 and MOTOR 1, then a synthetic room - a 4 m x 3 m rectangle seen
from its centre, 360 samples one degree apart, quality 47 - at 10 Hz as F lines,
exactly as firmware/pilot/src/scanwire.hxx specifies. It honours what the hub
sends: MOTOR 0 stops the frames and echoes MOTOR 0, MOTOR 1 restarts them and
echoes MOTOR 1, QUIT closes the session.

--sessions is a comma-separated list, one word per successive connection, and
the process exits once they have all been served (or --timeout seconds pass,
so a test that crashes leaves no server behind):

    normal      the room, until the client quits or goes
    junk        the room, plus - after the third frame - one F line with the
                wrong sample count and one line the hub has no name for
    abrupt:N    the room, then a hard reset of the connection after N frames
    err:N       the room, then "ERR ..." and an orderly close after N frames

Everything it sees and does goes to --log, one line each, because the test
reads it afterwards to prove the hub said MOTOR 0 / MOTOR 1 / QUIT. stdlib only.
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


def room_line():
    """One revolution of the rectangle, as an F line."""
    parts = []
    for deg in range(360):
        r = math.radians(deg)
        c, s = math.cos(r), math.sin(r)
        tx = HALF_W_MM / abs(c) if abs(c) > 1e-9 else float("inf")
        ty = HALF_H_MM / abs(s) if abs(s) > 1e-9 else float("inf")
        parts.append("%d,%d,%d" % (deg * 100, int(round(min(tx, ty))), QUALITY))
    return ("F 360 %d " % int(HZ * 1000) + " ".join(parts) + "\n").encode()


FRAME = room_line()


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


def serve(conn, scenario, log):
    kind, _, arg = scenario.partition(":")
    limit = int(arg) if arg else 0

    conn.sendall(INFO_LINE)
    conn.sendall(b"HEALTH 0\n")
    conn.sendall(b"MOTOR 1\n")
    log("session %s: sent INFO/HEALTH/MOTOR 1" % scenario)

    motor = True
    frames = 0
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
                    motor = False
                    conn.sendall(b"MOTOR 0\n")
                elif line == "MOTOR 1":
                    motor = True
                    next_frame = time.monotonic()
                    conn.sendall(b"MOTOR 1\n")

        now = time.monotonic()
        if motor and now >= next_frame:
            next_frame = now + 1.0 / HZ
            frames += 1
            conn.sendall(FRAME)
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
    args = ap.parse_args()

    log = Log(args.log)
    deadline = time.monotonic() + args.timeout

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", args.port))
    srv.listen(1)
    log("LISTENING on 127.0.0.1:%d" % args.port)

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
            serve(conn, scenario, log)
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

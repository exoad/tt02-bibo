"""The encoder node's report lines, from the laptop, with the arithmetic the
bench test wants done for you.

    python firmware/tools/hall_monitor.py [COMn] [--zero] [--pulse US]

Finds the Pico's serial port by Raspberry Pi's USB vendor id when none is
given, prints every `hall ...` line as it arrives, and every second adds a
summary: ticks since start, the highest and lowest count seen (so one turn of
the shaft by hand reads as a span), and whether any error counter moved.
--zero sends 'z' first; --pulse sends `p US` (an ESC throttle pulse on GP0,
1000..2000, neutral 1500) and neutral again on Ctrl-C. Ctrl-C prints the final span.
"""
import sys
import time

import serial
from serial.tools import list_ports

RPI_VID = 0x2E8A


def find_port():
    for p in list_ports.comports():
        if p.vid == RPI_VID:
            return p.device
    return None


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    zero = "--zero" in sys.argv
    pulse = sys.argv[sys.argv.index("--pulse") + 1] if "--pulse" in sys.argv else None
    if pulse is not None and pulse in args:
        args.remove(pulse)
    port = args[0] if args else find_port()
    if not port:
        print("no Raspberry Pi USB serial port found; give the COM port by name")
        sys.exit(1)
    print(f"listening on {port}")
    with serial.Serial(port, 115200, timeout=1) as sp:
        if zero:
            sp.write(b"z\n")
        if pulse is not None:
            sp.write(f"p {pulse}\n".encode())
        first = None
        lo = hi = None
        errors_seen = 0
        last_summary = time.time()
        latest = {}
        try:
            while True:
                line = sp.readline().decode("ascii", "replace").strip()
                if not line:
                    continue
                print(line)
                if not line.startswith("hall "):
                    continue
                fields = {}
                for tok in line.split()[1:]:
                    k, _, v = tok.partition("=")
                    fields[k] = v
                try:
                    t = int(fields.get("t", "0"))
                    err = int(fields.get("err", "0")) + int(fields.get("miss", "0"))
                except ValueError:
                    continue
                latest = fields
                if first is None:
                    first = t
                    lo = hi = t
                lo = min(lo, t)
                hi = max(hi, t)
                errors_seen = err
                if time.time() - last_summary >= 1.0:
                    last_summary = time.time()
                    print(
                        f"   -- since start: {t - first:+d} ticks, span {lo}..{hi} ({hi - lo} wide),"
                        f" errors+misses {errors_seen}, {fields.get('src')} {fields.get('tps')} ticks/s"
                    )
        except KeyboardInterrupt:
            if pulse is not None:
                sp.write(b"n\n")
            if first is not None:
                print(f"\nfinal: {int(latest.get('t', 0)) - first:+d} ticks since start, span {hi - lo} wide, errors+misses {errors_seen}")
                print("a two-pole motor with six hall states gives 6 ticks per motor turn; divide the span by the turns you made")


if __name__ == "__main__":
    main()

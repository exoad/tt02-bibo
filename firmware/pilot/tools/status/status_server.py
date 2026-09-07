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
"""
import glob
import http.server
import json
import os
import socket
import subprocess
import time

STATUS_FILE = '/tmp/bibo-pilot.json'   # the pilot writes it; see app/main.cxx
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


def pilot_status():
    """(status dict or None, age in seconds or None)."""
    try:
        with open(STATUS_FILE) as f:
            status = json.load(f)
        return status, max(0.0, time.time() - float(status.get('ts', 0)))
    except (OSError, ValueError):
        return None, None


def fmt(value, unit='', missing='unknown'):
    return missing if value is None else '%s%s' % (value, unit)


def lines():
    now = time.strftime('%Y-%m-%d %H:%M:%S')
    status, age = pilot_status()
    live = status is not None and age <= STALE_S
    out = []
    out.append('%s  %s  %s  up %s  %s' % (socket.gethostname(),
                                            ' '.join(addresses()) or 'no address',
                                            fmt(wifi_profile(), missing='no wifi'),
                                            fmt(uptime()), now))
    temp = cpu_temp_c()
    out.append('cpu           %s' % ('%.1f C' % temp if temp is not None else 'unknown'))
    out.append('battery       not measured - nothing on the car reads the pack yet')

    if live:
        out.append('pilot         running, status %.0f s old' % age)
        out.append('lidar         %s rev/s  mode %s  clear %s mm  hits %s%s' % (
            fmt(status.get('revPerS')), fmt(status.get('mode')),
            fmt(status.get('clearanceMm')), fmt(status.get('hits')),
            '  LOST' if status.get('lidarLost') else ''))
        out.append('pico          %s' % fmt(status.get('pico')))
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


class Page(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        text, status, age = lines()
        if self.path.startswith('/json'):
            body = json.dumps({'lines': text, 'pilot': status, 'pilotAgeS': age}).encode()
            kind = 'application/json'
        else:
            body = ('<meta http-equiv="refresh" content="2"><pre>%s</pre>\n'
                    % '\n'.join(text)).encode()
            kind = 'text/html; charset=utf-8'
        self.send_response(200)
        self.send_header('Content-Type', kind)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *_):
        pass    # a phone refreshing every two seconds is not journal material


if __name__ == '__main__':
    http.server.ThreadingHTTPServer(('', PORT), Page).serve_forever()

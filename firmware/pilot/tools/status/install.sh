#!/bin/sh
# Installs the pilot's unit, mDNS and the hotspot preference on the Orange Pi.
# Run on the board, as root, and again after pulling:
#
#     sudo sh ~/tt02-bibo/firmware/pilot/tools/status/install.sh
set -e
HERE=$(cd "$(dirname "$0")" && pwd)

# `sudo sh` sets HOME=/root, so the service user's home is looked up by name.
SERVICE_USER=jack
SERVICE_HOME=$(getent passwd "$SERVICE_USER" | cut -d: -f6)
BUILD="$SERVICE_HOME/build-pilot-app"
PILOT="$BUILD/pilot"

# The lidar and Pico by /dev/serial/by-id, found here rather than written into the
# unit: ttyUSB0 and ttyACM0 are enumeration order, and the serial numbers belong to
# this car, not to a public repository. An unplugged device falls back to its
# enumeration name; run this again once it is back.
LIDAR_DEV=$(ls /dev/serial/by-id/*CP2102N*-port0 2>/dev/null | head -n 1 || true)
PICO_DEV=$(ls /dev/serial/by-id/*Raspberry_Pi_Pico*-if00 2>/dev/null | head -n 1 || true)
if [ -z "$LIDAR_DEV" ]; then
    LIDAR_DEV=/dev/ttyUSB0
    echo "NOTE: no CP2102N under /dev/serial/by-id - the pilot unit uses $LIDAR_DEV"
fi
if [ -z "$PICO_DEV" ]; then
    PICO_DEV=/dev/ttyACM0
    echo "NOTE: no Pico under /dev/serial/by-id - the pilot unit uses $PICO_DEV"
fi

sed "s|ExecStart=.*|ExecStart=$PILOT --manual --lidar $LIDAR_DEV --pico $PICO_DEV|" \
    "$HERE/bibo-pilot.service" > /etc/systemd/system/bibo-pilot.service
chmod 644 /etc/systemd/system/bibo-pilot.service
if [ ! -x "$PILOT" ]; then
    echo "NOTE: $PILOT does not exist yet - the pilot unit will fail until it is built:"
    echo "      cmake -S $HERE/../.. -B $BUILD -DPILOT_RPLIDAR_SDK=$SERVICE_HOME/rplidar_sdk"
    echo "      cmake --build $BUILD -j"
fi

# Units and hooks from older installs, whose files are gone from this checkout.
rm -f /etc/NetworkManager/dispatcher.d/90-bibo-status
systemctl disable --now bibo-status.path bibo-status.service 2>/dev/null || true
rm -f /etc/systemd/system/bibo-status.service /etc/systemd/system/bibo-status.path \
    /etc/systemd/system/bibo-status-restart.service
systemctl disable --now bibo-scanfeed.service 2>/dev/null || true
rm -f /etc/systemd/system/bibo-scanfeed.service

sed "s|ExecStart=.*|ExecStart=/bin/sh $HERE/prefer-hotspot.sh|" \
    "$HERE/bibo-prefer-hotspot.service" > /etc/systemd/system/bibo-prefer-hotspot.service
chmod 644 /etc/systemd/system/bibo-prefer-hotspot.service
install -m 644 "$HERE/bibo-prefer-hotspot.timer" /etc/systemd/system/bibo-prefer-hotspot.timer
systemctl daemon-reload
systemctl enable bibo-pilot.service > /dev/null 2>&1
systemctl enable --now bibo-prefer-hotspot.timer > /dev/null 2>&1
echo "hotspot preference: checking every 20 s ($(systemctl is-active bibo-prefer-hotspot.timer))"

# mDNS, so the board is bibobox.local whatever address the phone hands out. On
# systemd 249 a profile's mDNS=yes cannot exceed systemd-resolved's global setting,
# which ships off: this drop-in turns the global on, the profile setting below turns
# the link on at each connect, and resolvectl at the end turns it on now.
mkdir -p /etc/systemd/resolved.conf.d
printf '[Resolve]\n# The car answers as %s.local on the field network; see install.sh.\nMulticastDNS=yes\n' \
    "$(hostname)" > /etc/systemd/resolved.conf.d/10-bibo-mdns.conf
systemctl restart systemd-resolved

# The SSID is not written here: this repository is public, and a hidden network's
# name is what makes it findable. BIBO_HOTSPOT=<profile> names it; otherwise it is
# the Wi-Fi profile with a raised autoconnect priority, as in prefer-hotspot.sh. Not
# the active network, which at home is the house and would get priority 10. A fresh
# board resolves to nothing and the else branch asks for BIBO_HOTSPOT.
HOTSPOT="${BIBO_HOTSPOT:-$(nmcli -t -f NAME,TYPE,AUTOCONNECT-PRIORITY connection show 2>/dev/null \
    | awk -F: '$2 == "802-11-wireless" && $3 > 0 { print $1; exit }')}"

if [ -n "$HOTSPOT" ] && nmcli -t -f NAME connection show | grep -qxF "$HOTSPOT"; then
    nmcli connection modify "$HOTSPOT" connection.mdns yes
    nmcli connection modify "$HOTSPOT" connection.autoconnect-priority 10
    echo "mDNS on for $HOTSPOT: the board answers as $(hostname).local there"
else
    echo "no wireless profile found - profile mDNS not touched."
    echo "  join the hotspot first, or re-run as: BIBO_HOTSPOT=<profile> $0"
fi

# Restarted so a pull and a rebuild take effect; its shutdown sends the Pico STOP.
# Not while a pilot or car program started by hand runs from $BUILD: it holds the
# lidar and Pico and may be driving. Matched by /proc/PID/exe, because pgrep -f
# misses a program started as ./forward; the unit's own process is its MainPID.
SERVICE_PID=$(systemctl show -p MainPID --value bibo-pilot.service 2>/dev/null || true)
HELD=""
for exe in /proc/[0-9]*/exe; do
    pid=${exe#/proc/}
    pid=${pid%/exe}
    case "$(readlink "$exe" 2>/dev/null || true)" in
        "$BUILD"/*) [ "$pid" = "$SERVICE_PID" ] || HELD="$HELD $pid" ;;
    esac
done
if [ ! -x "$PILOT" ]; then
    echo "pilot NOT started: build it first (see above), then sudo systemctl start bibo-pilot"
elif [ -n "$HELD" ]; then
    echo "pilot NOT started: a program from $BUILD is running (pid$HELD)"
    echo "  stop it, then: sudo systemctl start bibo-pilot"
else
    systemctl restart bibo-pilot.service
    echo "pilot: $(systemctl is-active bibo-pilot.service) - manual, armed only by a viewer's ARM"
fi

# mDNS on the live link now, when the hotspot is it; the profile covers later connects.
if [ -n "$HOTSPOT" ] && nmcli -t -f NAME connection show --active | grep -qxF "$HOTSPOT"; then
    resolvectl mdns wlan0 yes
    echo "wlan0 now: $(resolvectl status wlan0 | grep -i protocols | sed 's/^ *//')"
fi

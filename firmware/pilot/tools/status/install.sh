#!/bin/sh
# Installs the pilot's unit, mDNS and the hotspot preference on the Orange Pi.
# Run ON THE BOARD, as root:
#
#     sudo sh ~/tt02-bibo/firmware/pilot/tools/status/install.sh
#
# Idempotent - run it again after pulling. What it does, and nothing else:
#   1. removes what an older install left behind: the status page's units
#      (bibo-status.*), the scan feed's unit (bibo-scanfeed) and the hotspot
#      dispatcher hook
#   2. the pilot's unit, enabled at boot: --manual and NOT --arm, so the car
#      comes up held still and is armed only by a viewer's COMMAND ARM. The
#      unit is installed whether or not the binary exists yet; the script says
#      so when it is missing
#   3. mDNS, so the board is bibobox.local (the phone hands out a
#      different address every outing; the name holds). Two switches, because
#      on Ubuntu 22.04 (systemd 249) a profile's mDNS=yes can never exceed
#      systemd-resolved's GLOBAL setting, which ships off: the drop-in turns
#      the global on, the profile setting turns the link on at each connect,
#      and `resolvectl mdns` turns the link on NOW without a reconnect.
#   4. the hotspot-preference timer: NetworkManager never leaves a working
#      connection for another, so a board that booted at home stays on the
#      home Wi-Fi while the phone's hotspot comes up beside it. The timer asks
#      every 20 s and switches; and the hotspot profile gets a higher
#      autoconnect priority so a boot with both in the air picks it outright.
#   5. restarts the pilot, unless a program started by hand holds the lidar and
#      the Pico
set -e
HERE=$(cd "$(dirname "$0")" && pwd)

# The build lives in the service user's home, not root's: `sudo sh` runs this
# with HOME=/root, so the home is looked up by name.
SERVICE_USER=jack
SERVICE_HOME=$(getent passwd "$SERVICE_USER" | cut -d: -f6)
BUILD="$SERVICE_HOME/build-pilot-app"
PILOT="$BUILD/pilot"

# THE LIDAR AND THE PICO BY IDENTITY, found here rather than written into the
# unit: /dev/ttyUSB0 and /dev/ttyACM0 are enumeration order, and the serial
# numbers in the by-id names belong to this car, not to a public repository.
# Falls back to the enumeration names, and says so, when a device is unplugged
# at install time - run this again once it is back.
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

# Units and hooks this checkout no longer has. A board that still has them
# would keep starting programs whose files are gone, so take them off it.
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

mkdir -p /etc/systemd/resolved.conf.d
printf '[Resolve]\n# The car answers as %s.local on the field network; see install.sh.\nMulticastDNS=yes\n' \
    "$(hostname)" > /etc/systemd/resolved.conf.d/10-bibo-mdns.conf
systemctl restart systemd-resolved

# THE SSID IS NOT WRITTEN DOWN HERE. This repository is public, and the name of
# a HIDDEN network is the half that makes it findable - so the profile name comes
# from the environment, and the fallback is the board's own idea of which Wi-Fi
# connection is active rather than a name baked into a file anybody can read.
#
# BIBO_HOTSPOT=<profile> ./install.sh   to name it explicitly.
# NOT "whichever wireless network is active" - that is the home network when
# this is run at home, and it would then get mDNS and autoconnect-priority 10,
# raising the house to the hotspot's priority and quietly changing which network
# the board prefers in a field. prefer-hotspot.sh picks by priority; so does
# this, or the two disagree about what "the field network" means.
#
# On a fresh board nothing has a raised priority yet, so this resolves to
# nothing and the else branch asks for BIBO_HOTSPOT. That is the correct
# failure: refusing to guess beats guessing the wrong network.
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

# THE PILOT IS RESTARTED, so a pull and a rebuild take effect. Its shutdown sends
# the Pico STOP, so running this mid-drive stops the car for the restart.
#
# NOT while any other process runs an executable from $BUILD: a pilot or a car
# program started by hand. It holds the lidar and the Pico, so the unit would fail
# to open them and retry every RestartSec. It may be driving, so it is named and
# left alone. Matched by /proc/PID/exe rather than pgrep -f, which misses a program
# started as ./forward; the unit's own process is its MainPID.
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

# mDNS on the LIVE link, only when the hotspot is the live link: resolvectl
# sets the running state, and there is no running hotspot link to set here
# otherwise. The profile setting above covers the next connect.
# The same $HOTSPOT resolved above, not a second copy of the name: two blocks
# with their own idea of which network this is would eventually disagree, and
# the one that is wrong fails silently.
if [ -n "$HOTSPOT" ] && nmcli -t -f NAME connection show --active | grep -qxF "$HOTSPOT"; then
    resolvectl mdns wlan0 yes
    echo "wlan0 now: $(resolvectl status wlan0 | grep -i protocols | sed 's/^ *//')"
fi

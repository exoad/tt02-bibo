#!/bin/sh
# Installs the scan feed, mDNS and the hotspot preference on the Orange Pi.
# Run ON THE BOARD, as root:
#
#     sudo sh ~/tt02-bibo/firmware/pilot/tools/status/install.sh
#
# Idempotent - run it again after pulling. What it does, and nothing else:
#   1. removes what an older install left behind: the status page's units
#      (bibo-status.*; the page is gone) and the hotspot dispatcher hook
#   2. the scan feed's systemd unit, with ExecStart pointed at the canonical
#      build directory (~jack/build-pilot-app/scanfeed). The unit is installed
#      whether or not the binary exists yet, since the checkout can be built
#      after this runs; the script says loudly when it is missing.
#   3. the feed enabled at boot, so it is up whenever the car is powered -
#      on the hotspot, at home, anywhere
#   4. mDNS, so the board is bibobox.local (the phone hands out a
#      different address every outing; the name holds). Two switches, because
#      on Ubuntu 22.04 (systemd 249) a profile's mDNS=yes can never exceed
#      systemd-resolved's GLOBAL setting, which ships off: the drop-in turns
#      the global on, the profile setting turns the link on at each connect,
#      and `resolvectl mdns` turns the link on NOW without a reconnect.
#   5. the hotspot-preference timer: NetworkManager never leaves a working
#      connection for another, so a board that booted at home stays on the
#      home Wi-Fi while the phone's hotspot comes up beside it. The timer asks
#      every 20 s and switches; and the hotspot profile gets a higher
#      autoconnect priority so a boot with both in the air picks it outright.
#   6. starts both services now
set -e
HERE=$(cd "$(dirname "$0")" && pwd)

# The build lives in the service user's home, not root's: `sudo sh` runs this
# with HOME=/root, so the home is looked up by name.
SERVICE_USER=jack
SERVICE_HOME=$(getent passwd "$SERVICE_USER" | cut -d: -f6)
SCANFEED="$SERVICE_HOME/build-pilot-app/scanfeed"

sed "s|ExecStart=.*|ExecStart=$SCANFEED /dev/ttyUSB0|" \
    "$HERE/bibo-scanfeed.service" > /etc/systemd/system/bibo-scanfeed.service
chmod 644 /etc/systemd/system/bibo-scanfeed.service
if [ ! -x "$SCANFEED" ]; then
    echo "NOTE: $SCANFEED does not exist yet - the scan feed unit will fail until it is built:"
    echo "      cmake -S $HERE/../.. -B $SERVICE_HOME/build-pilot-app -DPILOT_RPLIDAR_SDK=$SERVICE_HOME/rplidar_sdk"
    echo "      cmake --build $SERVICE_HOME/build-pilot-app -j"
fi

# The hotspot dispatcher that used to start these is gone: the board lives on
# the car, and tying the feed to one network meant it was dead on the
# bench. Remove it if an older install left it behind.
rm -f /etc/NetworkManager/dispatcher.d/90-bibo-status

# The status page (bibo-status.service, and the path unit that restarted it on
# every pull) is gone from this checkout. A board that still has its units would
# keep starting a server whose files no longer exist, so take them off it too.
systemctl disable --now bibo-status.path bibo-status.service 2>/dev/null || true
rm -f /etc/systemd/system/bibo-status.service /etc/systemd/system/bibo-status.path \
    /etc/systemd/system/bibo-status-restart.service

sed "s|ExecStart=.*|ExecStart=/bin/sh $HERE/prefer-hotspot.sh|" \
    "$HERE/bibo-prefer-hotspot.service" > /etc/systemd/system/bibo-prefer-hotspot.service
chmod 644 /etc/systemd/system/bibo-prefer-hotspot.service
install -m 644 "$HERE/bibo-prefer-hotspot.timer" /etc/systemd/system/bibo-prefer-hotspot.timer
systemctl daemon-reload
systemctl enable bibo-scanfeed.service > /dev/null 2>&1
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

if [ -x "$SCANFEED" ]; then
    systemctl restart bibo-scanfeed.service
    echo "scan feed: nc $(hostname).local 8011"
else
    echo "scan feed NOT started: build it first (see above), then systemctl start bibo-scanfeed"
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

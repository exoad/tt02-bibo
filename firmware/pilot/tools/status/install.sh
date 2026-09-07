#!/bin/sh
# Installs the status page and the scan feed on the Orange Pi. Run ON THE
# BOARD, as root:
#
#     sudo sh ~/tt02-bibo/firmware/pilot/tools/status/install.sh
#
# Idempotent - run it again after pulling. What it does, and nothing else:
#   1. the status page's systemd unit, with ExecStart pointed at THIS checkout
#   2. the scan feed's systemd unit, with ExecStart pointed at the canonical
#      build directory (~jack/build-pilot-app/scanfeed). The unit is installed
#      whether or not the binary exists yet, since the checkout can be built
#      after this runs; the script says loudly when it is missing.
#   3. the NetworkManager dispatcher hook that starts/stops BOTH with WhoopWhoop
#   4. mDNS, so the page is http://bibobox.local/ (the phone hands out a
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
#   6. starts both services now if the hotspot is already up, since the
#      dispatcher only fires on the NEXT connect
set -e
HERE=$(cd "$(dirname "$0")" && pwd)

# The build lives in the service user's home, not root's: `sudo sh` runs this
# with HOME=/root, so the home is looked up by name.
SERVICE_USER=jack
SERVICE_HOME=$(getent passwd "$SERVICE_USER" | cut -d: -f6)
SCANFEED="$SERVICE_HOME/build-pilot-app/scanfeed"

sed "s|ExecStart=.*|ExecStart=/usr/bin/python3 $HERE/status_server.py|" \
    "$HERE/bibo-status.service" > /etc/systemd/system/bibo-status.service
chmod 644 /etc/systemd/system/bibo-status.service

sed "s|ExecStart=.*|ExecStart=$SCANFEED /dev/ttyUSB0|" \
    "$HERE/bibo-scanfeed.service" > /etc/systemd/system/bibo-scanfeed.service
chmod 644 /etc/systemd/system/bibo-scanfeed.service
if [ ! -x "$SCANFEED" ]; then
    echo "NOTE: $SCANFEED does not exist yet - the scan feed unit will fail until it is built:"
    echo "      cmake -S $HERE/../.. -B $SERVICE_HOME/build-pilot-app -DPILOT_RPLIDAR_SDK=$SERVICE_HOME/rplidar_sdk"
    echo "      cmake --build $SERVICE_HOME/build-pilot-app -j"
fi

install -m 755 "$HERE/90-bibo-status" /etc/NetworkManager/dispatcher.d/90-bibo-status

sed "s|ExecStart=.*|ExecStart=/bin/sh $HERE/prefer-hotspot.sh|" \
    "$HERE/bibo-prefer-hotspot.service" > /etc/systemd/system/bibo-prefer-hotspot.service
chmod 644 /etc/systemd/system/bibo-prefer-hotspot.service
install -m 644 "$HERE/bibo-prefer-hotspot.timer" /etc/systemd/system/bibo-prefer-hotspot.timer
systemctl daemon-reload
systemctl enable --now bibo-prefer-hotspot.timer > /dev/null 2>&1
echo "hotspot preference: checking every 20 s ($(systemctl is-active bibo-prefer-hotspot.timer))"

mkdir -p /etc/systemd/resolved.conf.d
printf '[Resolve]\n# The car answers as %s.local on the field network; see install.sh.\nMulticastDNS=yes\n' \
    "$(hostname)" > /etc/systemd/resolved.conf.d/10-bibo-mdns.conf
systemctl restart systemd-resolved

if nmcli -t -f NAME connection show | grep -qx WhoopWhoop; then
    nmcli connection modify WhoopWhoop connection.mdns yes
    nmcli connection modify WhoopWhoop connection.autoconnect-priority 10
    echo "mDNS on for WhoopWhoop: the board answers as $(hostname).local there"
else
    echo "no WhoopWhoop profile - profile mDNS not touched; add the hotspot profile first"
fi

if nmcli -t -f NAME connection show --active | grep -qx WhoopWhoop; then
    resolvectl mdns wlan0 yes
    echo "wlan0 now: $(resolvectl status wlan0 | grep -i protocols | sed 's/^ *//')"
    systemctl restart bibo-status.service
    echo "hotspot is up now: page started - http://$(hostname).local/ or http://$(hostname -I | cut -d' ' -f1)/"
    if [ -x "$SCANFEED" ]; then
        systemctl restart bibo-scanfeed.service
        echo "scan feed started - nc $(hostname).local 8011"
    else
        echo "scan feed NOT started: build it first (see above), then systemctl start bibo-scanfeed"
    fi
else
    echo "installed; the page and the scan feed will start when WhoopWhoop connects"
fi

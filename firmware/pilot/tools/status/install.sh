#!/bin/sh
# Installs the status page on the Orange Pi. Run ON THE BOARD, as root:
#
#     sudo sh ~/tt02-bibo/firmware/pilot/tools/status/install.sh
#
# What it does, and nothing else:
#   1. the systemd unit, with ExecStart pointed at THIS checkout
#   2. the NetworkManager dispatcher hook that starts/stops it with WhoopWhoop
#   3. mDNS on the WhoopWhoop profile, so the page is http://bibobox.local/
#      (the phone hands out a different address every outing; the name holds)
#   4. starts the page now if the hotspot is already up, since the dispatcher
#      only fires on the NEXT connect
set -e
HERE=$(cd "$(dirname "$0")" && pwd)

sed "s|ExecStart=.*|ExecStart=/usr/bin/python3 $HERE/status_server.py|" \
    "$HERE/bibo-status.service" > /etc/systemd/system/bibo-status.service
chmod 644 /etc/systemd/system/bibo-status.service
install -m 755 "$HERE/90-bibo-status" /etc/NetworkManager/dispatcher.d/90-bibo-status
systemctl daemon-reload

if nmcli -t -f NAME connection show | grep -qx WhoopWhoop; then
    nmcli connection modify WhoopWhoop connection.mdns yes
    echo "mDNS on for WhoopWhoop: the board answers as $(hostname).local there"
else
    echo "no WhoopWhoop profile - mDNS not touched; add the hotspot profile first"
fi

if nmcli -t -f NAME connection show --active | grep -qx WhoopWhoop; then
    systemctl restart bibo-status.service
    echo "hotspot is up now: page started - http://$(hostname).local/ or http://$(hostname -I | cut -d' ' -f1)/"
else
    echo "installed; the page will start when WhoopWhoop connects"
fi

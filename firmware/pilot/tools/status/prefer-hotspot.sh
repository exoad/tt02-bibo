#!/bin/sh
# Prefer the field network. Run as root by bibo-prefer-hotspot.timer.
#
# NetworkManager never leaves a working connection for another; it picks between
# profiles only when connecting from scratch. So this switches to the field network
# when it is in the air and something else is active. No reverse rule: when the
# hotspot goes, NM autoconnects to whatever else it knows. A rescan interrupts wlan0
# briefly, so it happens only while off the field network.
#
# The SSID is not written here: this repository is public, and a hidden network's
# name is what makes it findable. BIBO_FIELD names it; otherwise it is the Wi-Fi
# profile install.sh gave a raised autoconnect priority. If that resolves to nothing
# the script does nothing: a silent no-op beats switching the board off its network.
FIELD="${BIBO_FIELD:-$(nmcli -t -f NAME,TYPE,AUTOCONNECT-PRIORITY connection show 2>/dev/null \
    | awk -F: '$2 == "802-11-wireless" && $3 > 0 { print $1; exit }')}"
[ -z "$FIELD" ] && exit 0

active=$(nmcli -t -f NAME,DEVICE connection show --active | grep ':wlan0$' | cut -d: -f1)
[ "$active" = "$FIELD" ] && exit 0

# The profile's wifi.hidden yes makes NM probe for the SSID, so the scan lists it.
if nmcli -t -f SSID dev wifi list --rescan yes 2>/dev/null | grep -qx "$FIELD"; then
    echo "$FIELD is in the air and ${active:-nothing} is active - switching"
    nmcli connection up "$FIELD"
fi
exit 0

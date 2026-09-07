#!/bin/sh
# Prefer the field network. Run by bibo-prefer-hotspot.timer every 20 s, as root.
#
# NetworkManager never leaves a working connection for another one: it chooses
# between profiles only when it has to connect from scratch. So a board that
# booted at home sits on the home Wi-Fi while the phone's hotspot comes up
# right beside it, and nothing outdoors-shaped starts. This asks the one
# question NM will not: is the field network in the air while something else
# is active? Then switch. There is no reverse rule - when the hotspot goes
# away NM falls back to whatever else it knows by itself (autoconnect).
#
# A rescan interrupts wlan0 for a moment, so it happens only while NOT on the
# field network; on the hotspot this script costs one nmcli call and exits.
FIELD=WhoopWhoop

active=$(nmcli -t -f NAME,DEVICE connection show --active | grep ':wlan0$' | cut -d: -f1)
[ "$active" = "$FIELD" ] && exit 0

# `wifi.hidden yes` on the profile makes NM probe for the hidden SSID during a
# scan, which is why it shows up here by name at all.
if nmcli -t -f SSID dev wifi list --rescan yes 2>/dev/null | grep -qx "$FIELD"; then
    echo "$FIELD is in the air and ${active:-nothing} is active - switching"
    nmcli connection up "$FIELD"
fi
exit 0

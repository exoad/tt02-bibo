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
# THE SSID IS NOT WRITTEN DOWN HERE. This repository is public, and the name of
# a HIDDEN network is exactly the half that makes it findable.
#
# BIBO_FIELD names it explicitly. Failing that, the field network is whichever
# Wi-Fi profile install.sh gave the raised autoconnect-priority to - so a board
# set up by that script keeps working with the name recorded nowhere but in
# NetworkManager's own root-only store, beside the passphrase.
#
# IF THIS RESOLVES TO NOTHING THE SCRIPT DOES NOTHING, which is the right
# failure: the alternative is guessing at a network and switching the board off
# the one it is on. A silent no-op outdoors is survivable; a wrong switch is not.
FIELD="${BIBO_FIELD:-$(nmcli -t -f NAME,TYPE,AUTOCONNECT-PRIORITY connection show 2>/dev/null \
    | awk -F: '$2 == "802-11-wireless" && $3 > 0 { print $1; exit }')}"
[ -z "$FIELD" ] && exit 0

active=$(nmcli -t -f NAME,DEVICE connection show --active | grep ':wlan0$' | cut -d: -f1)
[ "$active" = "$FIELD" ] && exit 0

# `wifi.hidden yes` on the profile makes NM probe for the hidden SSID during a
# scan, which is why it shows up here by name at all.
if nmcli -t -f SSID dev wifi list --rescan yes 2>/dev/null | grep -qx "$FIELD"; then
    echo "$FIELD is in the air and ${active:-nothing} is active - switching"
    nmcli connection up "$FIELD"
fi
exit 0

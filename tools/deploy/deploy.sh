#!/usr/bin/env bash
# puts the code on the robot the fastest way that is still correct.
#
#   hot reload   (~1 s)   only swaps the teamcode Java classes. good for almost every code change
#   full install (~1 min) rebuilds and installs the whole app. needed when anything else changed
#
# how it decides: after every full install it leaves a note on the hub (a hash of everything a hot reload can't
# update + when the app was installed). a hot reload is only used when that note still matches. so installing
# from Android Studio, or from someone else's laptop, or changing a gradle file / asset, all lead to a full
# install the next time. nothing is remembered on the laptop.
#
# usage: tools/deploy/deploy.sh [auto|check|full|hot]      (check = say what it would do, change nothing)
# env:   GRADLE (./gradlew), HUB_IP (192.168.43.1), HUB_PORT (5555)
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"

MODE="${1:-auto}"
GRADLE="${GRADLE:-./gradlew}"
HUB_IP="${HUB_IP:-192.168.43.1}"
HUB_PORT="${HUB_PORT:-5555}"
NOTE="/sdcard/FIRST/buzz-deploy.txt"
APP="com.qualcomm.ftcrobotcontroller"

ADB="$(tools/hub-connect.sh --adb)"
ANDROID_SERIAL="$(tools/hub-connect.sh "$HUB_IP" "$HUB_PORT")"
export ANDROID_SERIAL   # so adb and gradle both talk to the hub we just picked, even with two connected

installed_at() {
    "$ADB" shell dumpsys package "$APP" 2>/dev/null | tr -d '\r' | sed -n 's/^ *lastUpdateTime=//p' | head -n 1
}

full_install() {
    "$GRADLE" :TeamCode:installDebug
    printf 'hash=%s\ninstalled=%s\n' "$(tools/deploy/full-hash.sh)" "$(installed_at)" | "$ADB" shell "cat > $NOTE"
    echo "full install done"
}

hot_reload() {
    "$GRADLE" deploySloth
    echo "hot reload done. it takes effect the next time an opmode is initialized"
}

if [ "$MODE" = "hot" ]; then
    hot_reload
    exit 0
fi

note="$("$ADB" shell "cat $NOTE 2>/dev/null" | tr -d '\r' || true)"
note_hash="$(printf '%s\n' "$note" | sed -n 's/^hash=//p')"
note_installed="$(printf '%s\n' "$note" | sed -n 's/^installed=//p')"
now_installed="$(installed_at)"

if [ "$MODE" = "full" ]; then
    pick=FULL
    why="you asked for a full install"
elif [ -z "$now_installed" ]; then
    pick=FULL
    why="the robot controller app isn't on this hub yet"
elif [ -z "$note_hash" ]; then
    pick=FULL
    why="no full install from 'make' on this hub yet"
elif [ "$note_installed" != "$now_installed" ]; then
    pick=FULL
    why="the app on the hub was installed some other way since (Android Studio or another laptop)"
elif [ "$note_hash" != "$(tools/deploy/full-hash.sh)" ]; then
    pick=FULL
    why="something a hot reload can't update changed (gradle, assets, res, manifest, or code outside teamcode)"
else
    pick=HOT
    why="only teamcode Java can be different from what's on the hub"
fi

echo "RECOMMENDATION: $pick"
echo "REASON: $why"
if [ "$MODE" = "check" ]; then
    exit 0
fi

if [ "$pick" = "FULL" ]; then
    full_install
else
    hot_reload
fi

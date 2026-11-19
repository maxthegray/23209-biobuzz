#!/bin/sh
# makes sure adb can reach the robot's hub, and prints which device to use.
# tries the hub's Wi-Fi address first, then a USB cable. keeps trying for WAIT_SECONDS so a hub that is still
# booting gets picked up when it comes online.
#
# usage: tools/hub-connect.sh [HUB_IP] [HUB_PORT] [WAIT_SECONDS]
#        tools/hub-connect.sh --adb      (only prints the path to adb)
# stdout: the adb serial of the hub (use it as ANDROID_SERIAL). everything else goes to stderr
set -eu

ROOT="$(cd "$(dirname "$0")/.." && pwd)"

# adb comes with Android Studio's SDK and usually isn't on PATH, so look where Android Studio put it
find_adb() {
    if [ -f "$ROOT/local.properties" ]; then
        sdk="$(sed -n 's/^sdk\.dir=//p' "$ROOT/local.properties" | tail -n 1)"
        if [ -n "$sdk" ] && [ -x "$sdk/platform-tools/adb" ]; then echo "$sdk/platform-tools/adb"; return; fi
    fi
    for sdk in "${ANDROID_HOME:-}" "${ANDROID_SDK_ROOT:-}" "$HOME/Library/Android/sdk" "$HOME/Android/Sdk"; do
        if [ -n "$sdk" ] && [ -x "$sdk/platform-tools/adb" ]; then echo "$sdk/platform-tools/adb"; return; fi
    done
    command -v adb 2>/dev/null || true
}

ADB="$(find_adb)"
if [ -z "$ADB" ]; then
    echo "error: can't find adb. open the project in Android Studio once (it writes local.properties), or install Android platform-tools" >&2
    exit 1
fi

# tools/hub-connect.sh --adb just prints where adb is, for the other scripts
if [ "${1:-}" = "--adb" ]; then
    echo "$ADB"
    exit 0
fi

HUB_IP="${1:-192.168.43.1}"
HUB_PORT="${2:-5555}"
WAIT_SECONDS="${3:-30}"
WIFI="$HUB_IP:$HUB_PORT"

wifi_up() {
    "$ADB" devices 2>/dev/null | awk -v s="$WIFI" '$1==s && $2=="device" {found=1} END {exit found?0:1}'
}

usb_serial() {
    "$ADB" devices 2>/dev/null | awk 'NR>1 && $2=="device" && $1 !~ /:/ {print $1; exit}'
}

# adb connect can hang for over a minute when the address doesn't answer, so give it 3 seconds
try_wifi() {
    wifi_up && return 0
    "$ADB" connect "$WIFI" >/dev/null 2>&1 &
    pid=$!
    ( sleep 3; kill "$pid" 2>/dev/null ) >/dev/null 2>&1 &
    wait "$pid" 2>/dev/null || true
    wifi_up
}

try() {
    if try_wifi; then
        echo "hub: Wi-Fi ($WIFI)" >&2
        echo "$WIFI"
        return 0
    fi
    usb="$(usb_serial)"
    if [ -n "$usb" ]; then
        echo "hub: USB ($usb)" >&2
        echo "$usb"
        return 0
    fi
    return 1
}

try && exit 0
"$ADB" kill-server >/dev/null 2>&1 || true   # adb gets stuck sometimes, restart it once
try && exit 0

echo "waiting for the hub over Wi-Fi or USB (Ctrl-C to stop)..." >&2
deadline=$(($(date +%s) + WAIT_SECONDS))
while [ "$(date +%s)" -lt "$deadline" ]; do
    sleep 2
    try && exit 0
done

echo "error: no hub after ${WAIT_SECONDS}s. join the robot's Wi-Fi or plug in the USB cable" >&2
exit 1

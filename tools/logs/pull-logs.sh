#!/bin/sh
# copies the newest N flight recorder logs off the hub into robot-logs/.
# the hub keeps at most 30, so with no number you get all of them.
#
# usage: tools/logs/pull-logs.sh [N] [HUB_IP] [HUB_PORT]
set -eu

N="${1:-30}"
HUB_IP="${2:-192.168.43.1}"
HUB_PORT="${3:-5555}"
LOG_DIR="/sdcard/FIRST/logs"
DEST="robot-logs"
HERE="$(dirname "$0")"

case "$N" in
    '' | *[!0-9]*)
        echo "error: N has to be a number, got '$N'" >&2
        exit 1
        ;;
esac

ADB="$("$HERE/../hub-connect.sh" --adb)"
ANDROID_SERIAL="$("$HERE/../hub-connect.sh" "$HUB_IP" "$HUB_PORT")"
export ANDROID_SERIAL

# the names have no spaces in them. ls -t lists the newest first
LOGS=$("$ADB" shell "ls -t $LOG_DIR/*.wpilog 2>/dev/null" | tr -d '\r' | head -n "$N")
if [ -z "$LOGS" ]; then
    echo "error: no logs in $LOG_DIR on the hub. run an opmode that has the flight recorder in it first" >&2
    exit 1
fi

mkdir -p "$DEST"
for f in $LOGS; do
    "$ADB" pull "$f" "$DEST" >/dev/null
    echo "pulled $DEST/$(basename "$f")"
done

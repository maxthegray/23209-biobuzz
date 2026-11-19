#!/usr/bin/env bash
# prints one hash of everything a hot reload can NOT put on the robot: gradle files, the manifest, res, assets
# (the flower dash page lives there), FtcRobotController, and any teamcode class marked @Pinned.
# if this hash is the same as at the last full install, only teamcode Java can be different, and a hot reload
# is enough. works on macOS's old bash 3.2
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"

HOT="TeamCode/src/main/java/org/firstinspires/ftc/teamcode/"

git ls-files -co --exclude-standard | while IFS= read -r f; do
    [ -f "$f" ] || continue   # deleted, not committed yet
    case "$f" in
        TeamCode/src/test/*) continue ;;   # tests never go on the robot
        "$HOT"*.java) grep -q '@Pinned\|sinister\.loading\.Pinned' "$f" || continue ;;   # hot reloadable
        TeamCode/* | FtcRobotController/* | libs/* | gradle/* | *.gradle | gradle.properties) ;;
        *) continue ;;   # docs, tools, CLAUDE.md, ...: not part of the app
    esac
    printf '%s\n' "$f"
done | tr '\n' '\0' | xargs -0 shasum -a 256 | shasum -a 256 | cut -d' ' -f1

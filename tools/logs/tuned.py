#!/usr/bin/env python3
"""Lists the numbers that were changed live in Panels and where they go in the code.

Usage:
    python3 tools/logs/tuned.py [--json] [file.wpilog]

With no file it reads the newest log in ./robot-logs (run `make pull-logs 1` first).

The flight recorder writes a TUNED event for every tunable number that is different from
the code: at the start of each run, whenever one changes, and again at the end. So the
newest log always has the whole list. One event looks like

    TUNED UltrasonicCalibration.java UltrasonicCalibration.leftScale 1.2649 -> 1.31

(file, class.field, what the code said -> what it was changed to; the same number on both
sides means it was put back). For each one this finds the line in the code and says what
to do with it:

    copy    the line still has the old number: replace it with the new one
    expr    the line is a formula like `0.0040 / zoominess`: the new number replaces
            everything after the =
    stale   the code has a different number than the robot had when it was tuned (the code
            changed since, or the robot ran an older build): decide by hand
    lost    the file or the line could not be found
"""

import glob
import json
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze_wpilog import parse_wpilog  # noqa: E402

EVENT = re.compile(r"^TUNED (\S+) ([\w.$]+)\.(\w+) (\S+) -> (\S+)$")
NUMBER = re.compile(r"^[-+]?(\d+\.?\d*|\.\d+)([eE][-+]?\d+)?[dDfFlL]?$")
SOURCE_DIRS = ("TeamCode/src/main/java",)


def changes_from_events(texts):
    """Event strings in log order -> the changes still standing, in the order first seen."""
    latest = {}
    for text in texts:
        m = EVENT.match(text.strip())
        if not m:
            continue
        file, owner, field, code, now = m.groups()
        latest[(file, owner, field)] = {"file": file, "owner": owner, "field": field, "code": code, "now": now}
    return [c for c in latest.values() if c["code"] != c["now"]]


def same_number(a, b):
    try:
        return abs(float(a.rstrip("dDfFlL")) - float(b.rstrip("dDfFlL"))) <= 1e-9 * max(1.0, abs(float(b.rstrip("dDfFlL"))))
    except ValueError:
        return a == b


def find_file(root, name):
    found = []
    for base in SOURCE_DIRS:
        found += glob.glob(os.path.join(root, base, "**", name), recursive=True)
    return found


def resolve(root, change):
    """Adds path, line, declared (what the code says now) and action to one change."""
    out = dict(change, path=None, line=None, declared=None)
    files = find_file(root, change["file"])
    if len(files) != 1:
        out["action"] = "lost"
        out["note"] = ("no file called %s under TeamCode/src/main/java" % change["file"]) if not files \
            else "more than one file called %s" % change["file"]
        return out
    path = files[0]
    out["path"] = os.path.relpath(path, root)
    with open(path, encoding="utf-8") as f:
        lines = f.read().split("\n")

    # a class inside another class: only look below where that class starts
    first = 0
    if change["owner"] != change["file"][:-len(".java")]:
        inner = re.compile(r"\b(class|interface|enum)\s+%s\b" % re.escape(change["owner"]))
        starts = [i for i, text in enumerate(lines) if inner.search(text) and not text.lstrip().startswith("//")]
        if not starts:
            out["action"] = "lost"
            out["note"] = "no class %s in %s" % (change["owner"], out["path"])
            return out
        first = starts[0]

    declaration = re.compile(
        r"^\s*(?:public\s+|private\s+|protected\s+)?static\s+(?:volatile\s+)?"
        r"(?:double|float|int|long|boolean)\s+%s\s*=\s*([^;]+);" % re.escape(change["field"]))
    for i in range(first, len(lines)):
        m = declaration.match(lines[i])   # match() from the line start, so a commented-out line doesn't count
        if not m:
            continue
        out["line"] = i + 1
        out["declared"] = m.group(1).strip()
        if not NUMBER.match(out["declared"]) and out["declared"] not in ("true", "false"):
            out["action"] = "expr"
            out["note"] = "the line is a formula, the new number replaces everything after the ="
        elif same_number(out["declared"], change["code"]):
            out["action"] = "copy"
        elif same_number(out["declared"], change["now"]):
            out["action"] = "done"
            out["note"] = "the code already has the new number"
        else:
            out["action"] = "stale"
            out["note"] = "the code says %s now, the robot had %s when it was tuned" % (out["declared"], change["code"])
        return out
    out["action"] = "lost"
    out["note"] = "no line like `static double %s = ...;` in %s" % (change["field"], out["path"])
    return out


def newest_log(root):
    logs = glob.glob(os.path.join(root, "robot-logs", "**", "*.wpilog"), recursive=True)
    return max(logs, key=os.path.getmtime) if logs else None


def tuned(root, log):
    records, _, _ = parse_wpilog(log)
    texts = [value for _, value in records.get("events", [])]
    return [resolve(root, change) for change in changes_from_events(texts)]


def main(argv):
    as_json = "--json" in argv
    files = [a for a in argv if not a.startswith("--")]
    root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    log = files[0] if files else newest_log(root)
    if not log:
        print("no logs in robot-logs/. run `make pull-logs 1` first", file=sys.stderr)
        return 1
    result = tuned(root, log)
    if as_json:
        print(json.dumps({"log": os.path.relpath(log, root), "tuned": result}, indent=2))
        return 0

    print("from %s" % os.path.relpath(log, root))
    if not result:
        print("nothing to copy: every tunable number matched the code in this run")
        return 0
    for c in result:
        where = "%s:%s" % (c["path"], c["line"]) if c["line"] else (c["path"] or c["file"])
        print("%-6s %s   %s = %s -> %s" % (c["action"], where, c["field"], c["code"], c["now"]))
        if c.get("note"):
            print("       %s" % c["note"])
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

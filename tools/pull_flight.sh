#!/usr/bin/env bash
# Pull flight-recorder logs from the robot (orin.local) to this analysis server.
#
# rsync over SSH: incremental (skips files already here), resumable (--partial),
# compressed in transit (-z, ~4x on these logs), only new files each run.
#
# One-time setup for passwordless pulls (so this is scriptable / cron-able):
#     ssh-copy-id woody@orin.local
#
# Usage:
#     tools/pull_flight.sh                 # pull new logs into ./logs/flight_robot
#     ROBOT=woody@orin.local tools/pull_flight.sh
#     REMOTE_DIR=~/deploy/logs/flight tools/pull_flight.sh
#     tools/pull_flight.sh --purge         # also delete logs on the robot AFTER
#                                          #   a verified copy (free its disk)
set -euo pipefail

ROBOT="${ROBOT:-woody@orin.local}"
# The robot has the same repo at this root. run_interface writes
# <cwd>/logs/flight, so the dir is auto-located under the repo (works whether
# you launch from the repo root or from bin/). Override REMOTE_DIR to skip.
REMOTE_ROOT="${REMOTE_ROOT:-~/code/RoboTamerSdk4Qmini}"
LOCAL_DIR="${LOCAL_DIR:-$(cd "$(dirname "$0")/.." && pwd)/logs/flight_robot}"

PURGE=0
[ "${1:-}" = "--purge" ] && PURGE=1

# Sanity: can we reach the robot non-interactively?
if ! ssh -o BatchMode=yes -o ConnectTimeout=6 "$ROBOT" true 2>/dev/null; then
    echo "[pull] passwordless SSH to $ROBOT not working."
    echo "       Run once:  ssh-copy-id $ROBOT"
    echo "       (or it will prompt for a password on each command below)"
fi

# Auto-locate the flight-log dir(s) on the robot unless REMOTE_DIR is given.
if [ -z "${REMOTE_DIR:-}" ]; then
    mapfile -t FOUND < <(ssh "$ROBOT" \
        "find $REMOTE_ROOT -type d -path '*/logs/flight' 2>/dev/null")
    if [ "${#FOUND[@]}" -eq 0 ]; then
        echo "[pull] no logs/flight dir found under $REMOTE_ROOT on $ROBOT."
        echo "       Has the SDK run yet? Or set REMOTE_DIR=<path> explicitly."
        exit 1
    fi
    echo "[pull] found on robot: ${FOUND[*]}"
else
    FOUND=("$REMOTE_DIR")
fi

mkdir -p "$LOCAL_DIR"

RSYNC_OPTS=(-avz --partial --progress
            # keep .bin + .meta.json together; only flight files
            --include='flight_*.bin' --include='flight_*.meta.json'
            --include='*/' --exclude='*')

for dir in "${FOUND[@]}"; do
    echo "[pull] $ROBOT:$dir/  ->  $LOCAL_DIR/"
    if [ "$PURGE" = "1" ]; then
        # Two passes: copy first, then delete source files that transferred OK.
        rsync "${RSYNC_OPTS[@]}" "$ROBOT:$dir/" "$LOCAL_DIR/"
        echo "[pull] verified copy; removing transferred files on robot..."
        rsync "${RSYNC_OPTS[@]}" --remove-source-files "$ROBOT:$dir/" "$LOCAL_DIR/"
    else
        rsync "${RSYNC_OPTS[@]}" "$ROBOT:$dir/" "$LOCAL_DIR/"
    fi
done

echo "[pull] done. Newest logs:"
ls -lt "$LOCAL_DIR"/flight_*.bin 2>/dev/null | head -5 || echo "  (none yet)"
echo "[pull] analyze with:  python3 tools/replay_flight.py $LOCAL_DIR/<file>.bin"

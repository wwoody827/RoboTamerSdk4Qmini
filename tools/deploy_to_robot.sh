#!/usr/bin/env bash
# One-click deploy: push the current branch, then on the robot (orin) fetch it
# and rebuild the robot-release (arm64 + ONNX) binary.
#
# Usage:
#     tools/deploy_to_robot.sh            # push current branch + build on robot
#     tools/deploy_to_robot.sh --no-push  # skip git push (deploy what's on origin)
#
# Env overrides:
#     ROBOT=woody@orin.local
#     REMOTE_REPO=~/code/RoboTamerSdk4Qmini
#     PRESET=robot-release
set -euo pipefail

ROBOT="${ROBOT:-woody@orin.local}"
REMOTE_REPO="${REMOTE_REPO:-~/code/RoboTamerSdk4Qmini}"
PRESET="${PRESET:-robot-release}"
BRANCH="$(git rev-parse --abbrev-ref HEAD)"
PUSH=1
[ "${1:-}" = "--no-push" ] && PUSH=0

if ! ssh -o BatchMode=yes -o ConnectTimeout=6 "$ROBOT" true 2>/dev/null; then
    echo "[deploy] passwordless SSH to $ROBOT failed. Run: ssh-copy-id $ROBOT"
    exit 1
fi

if [ "$PUSH" = "1" ]; then
    echo "[deploy] pushing $BRANCH -> origin"
    git push origin "$BRANCH"
fi

echo "[deploy] $ROBOT: fetch + checkout $BRANCH + build ($PRESET)"
ssh "$ROBOT" bash -se <<EOF
set -euo pipefail
cd $REMOTE_REPO
git fetch --quiet origin
git checkout --quiet "$BRANCH"
git pull --ff-only --quiet origin "$BRANCH"
echo "[robot] now at: \$(git rev-parse --short HEAD) \$(git log -1 --format=%s)"
cmake --build build/$PRESET -j"\$(nproc)"
echo "[robot] build OK -> bin/run_interface"
EOF

echo "[deploy] done. On the robot, run the SDK; then back here:  tools/pull_flight.sh"

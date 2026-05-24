#!/usr/bin/env bash
# Set up lib/mujoco/ symlinks for the desktop-mujoco build.
#
# Usage:
#   sim_assets/setup_mujoco.sh /path/to/mujoco
#
# /path/to/mujoco must contain:
#   include/mujoco/mujoco.h
#   libmujoco.so   (or libmujoco.so.X.Y.Z)
#
# Typical paths:
#   pip-installed:  $HOME/.local/lib/pythonX.Y/site-packages/mujoco
#   conda env:      $HOME/miniconda3/envs/<env>/lib/pythonX.Y/site-packages/mujoco
#
# We do NOT auto-discover — globbing across all conda envs is slow on big
# installs. Just pass the path you want.

set -euo pipefail

if [[ $# -ne 1 ]]; then
    echo "usage: $0 /path/to/mujoco" >&2
    exit 2
fi
MJ="$1"

if [[ ! -f "$MJ/include/mujoco/mujoco.h" ]]; then
    echo "error: $MJ/include/mujoco/mujoco.h not found" >&2
    exit 1
fi

# Find the libmujoco.so.* shared object. Use a controlled find with a
# small maxdepth so this never goes wandering.
SO=$(find "$MJ" -maxdepth 1 -name "libmujoco.so.*" -print -quit)
if [[ -z "$SO" ]]; then
    SO="$MJ/libmujoco.so"
fi
if [[ ! -f "$SO" ]]; then
    echo "error: libmujoco.so* not found in $MJ" >&2
    exit 1
fi

HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
LIB="$REPO/lib/mujoco"
mkdir -p "$LIB"
ln -sfn "$MJ/include" "$LIB/include"
SOBASE=$(basename "$SO")
ln -sfn "$SO"     "$LIB/$SOBASE"
ln -sfn "$SOBASE" "$LIB/libmujoco.so"

echo "mujoco linked from $MJ:"
echo "  $LIB/include       -> $MJ/include"
echo "  $LIB/$SOBASE -> $SO"
echo "  $LIB/libmujoco.so  -> $SOBASE"

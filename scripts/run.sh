#!/usr/bin/env bash
#
# Tiny launcher — put this on your local machine and run it.
#
# Setup: edit SERVER below, then:
#   chmod +x run.sh
#   ./run.sh                              # pick from a list
#   ./run.sh --motion IMG_8294.npz        # launch specific motion
#   ./run.sh --sync-back                   # save changes back to server
#
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"

# ── EDIT THIS ───────────────────────────────────────────────────────────────
SERVER="jurek@myserver"
# ────────────────────────────────────────────────────────────────────────────

export SERVER
exec "$DIR/sync_and_run.sh" "$@"

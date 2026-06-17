#!/usr/bin/env bash
set -e

SERVER=deka
LOCAL_DIR="$HOME/motion-editor"
VENV_DIR="$LOCAL_DIR/.venv"
REMOTE_MOTIONS="booster_assets/motions/K1"
REMOTE_ROBOTS="booster_assets/robots"
LOCAL_MOTIONS="$LOCAL_DIR/motions"
LOCAL_ROBOTS="$LOCAL_DIR/assets/robots"
SYNC_BACK=0
NO_SYNC=0
ROBOT="booster_k1"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --server)      SERVER="$2";      shift 2 ;;
        --robot)       ROBOT="$2";       shift 2 ;;
        --sync-back)   SYNC_BACK=1;      shift   ;;
        --no-sync)     NO_SYNC=1;        shift   ;;
        -h|--help)     head -40 "$0";    exit 0   ;;
        *) echo "Unknown: $1"; exit 1 ;;
    esac
done

if [[ "$NO_SYNC" -eq 0 ]]; then
    echo "Syncing motion files from $SERVER:~/$REMOTE_MOTIONS/ ..."
    rsync -az --info=progress2 "$SERVER:~/$REMOTE_MOTIONS/" "$LOCAL_MOTIONS/"
    echo "Syncing robot assets from $SERVER:~/$REMOTE_ROBOTS/ ..."
    rsync -az --info=progress2 "$SERVER:~/$REMOTE_ROBOTS/" "$LOCAL_ROBOTS/"
fi

mapfile -t FILES < <(ls -1t "$LOCAL_MOTIONS"/*.npz 2>/dev/null || true)

if [[ ${#FILES[@]} -eq 0 ]]; then
    echo "Error: no .npz files found in $LOCAL_MOTIONS"
    exit 1
fi

if [[ ${#FILES[@]} -eq 1 ]]; then
    MOTION="${FILES[0]}"
    echo "Using: $(basename "$MOTION")"
else
    echo ""
    echo "Available motions:"
    for i in "${!FILES[@]}"; do
        printf "  %2d) %s\n" $((i+1)) "$(basename "${FILES[$i]}")"
    done
    read -rp "Select motion [1-${#FILES[@]}]: " SEL
    SEL="${SEL:-1}"
    MOTION="${FILES[$((SEL-1))]}"
fi

PYTHON=$(command -v python3.12 || echo python3)

if [[ ! -d "$VENV_DIR" ]]; then
    echo "Creating venv..."
    "$PYTHON" -m venv "$VENV_DIR"
fi

echo "Installing dependencies..."
"$VENV_DIR/bin/pip" install --quiet -e "$LOCAL_DIR"

echo "Launching motion editor..."
"$VENV_DIR/bin/python" -m motion_editor.main \
    --robot "$ROBOT" \
    --motion "$MOTION"

if [[ "$SYNC_BACK" -eq 1 ]]; then
    echo "Syncing back to server..."
    rsync -az "$MOTION" "$SERVER:~/$REMOTE_MOTIONS/$(basename "$MOTION")"
    rsync -az "$MOTION.bak" "$SERVER:~/$REMOTE_MOTIONS/$(basename "$MOTION").bak" 2>/dev/null || true
fi

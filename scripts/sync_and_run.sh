#!/usr/bin/env bash
set -e

# ── Sync motion-editor from server → local, then run the GUI ────────────────
#
# Usage:
#   ./scripts/sync_and_run.sh [options]
#
# Options:
#   --server <user@host>     SSH server (default: from SERVER env var)
#   --motion <name.npz>      Motion file name in the motions folder
#   --motion-path <path>     Full remote path to a specific .npz
#   --robot <name>           Robot type (default: booster_k1)
#   --motions-dir <path>     Remote motions directory (default: ~/booster_assets/motions/K1)
#   --sync-back              After editing, rsync saved .npz back to server
#   --recreate-env           Recreate conda env from scratch
#   --no-sync                Skip rsync, just run locally
#   -h, --help               Show this help
#
# Examples:
#   export SERVER=jurek@myserver
#   ./scripts/sync_and_run.sh                           # picker dialog
#   ./scripts/sync_and_run.sh --motion IMG_8294.npz     # by name
#   ./scripts/sync_and_run.sh --sync-back               # +push changes
# ────────────────────────────────────────────────────────────────────────────

# ── Config ──────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LOCAL_DIR="$HOME/motion-editor"
LOCAL_MOTIONS="$LOCAL_DIR/motions"

# Remote defaults
REMOTE_MOTIONS_DIR="booster_assets/motions/K1"
SERVER="${SERVER:-}"
SYNC_BACK=0
NO_SYNC=0
RECREATE_ENV=0
ROBOT="booster_k1"
MOTION=""
MOTION_PATH=""

# ── Parse args ──────────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --server)        SERVER="$2";        shift 2 ;;
        --motion)        MOTION="$2";        shift 2 ;;
        --motion-path)   MOTION_PATH="$2";   shift 2 ;;
        --robot)         ROBOT="$2";         shift 2 ;;
        --motions-dir)   REMOTE_MOTIONS_DIR="$2"; shift 2 ;;
        --sync-back)     SYNC_BACK=1;        shift   ;;
        --no-sync)       NO_SYNC=1;          shift   ;;
        --recreate-env)  RECREATE_ENV=1;     shift   ;;
        -h|--help)       head -80 "$0";      exit 0   ;;
        *) echo "Unknown: $1"; exit 1 ;;
    esac
done

# ── Prompt for server if missing ────────────────────────────────────────────
if [[ -z "$SERVER" ]]; then
    read -rp "Enter server (user@host): " SERVER
fi

# ── Sync code + motions from server ────────────────────────────────────────
if [[ "$NO_SYNC" -eq 0 ]]; then
    echo "═══ Syncing motion-editor from $SERVER ═══"
    rsync -az --delete --info=progress2 "$SERVER:~/motion-editor/" "$LOCAL_DIR/"

    echo "═══ Syncing motions from $SERVER:$REMOTE_MOTIONS_DIR ═══"
    mkdir -p "$LOCAL_MOTIONS"
    rsync -az --delete --info=progress2 "$SERVER:~/$REMOTE_MOTIONS_DIR/" "$LOCAL_MOTIONS/"

    # Also sync GMR outputs if they exist
    if rsync -az --dry-run "$SERVER:~/GMR/outputs/" /tmp/ 2>/dev/null | grep -q .; then
        mkdir -p "$LOCAL_MOTIONS/../gmr_outputs"
        rsync -az --info=progress2 "$SERVER:~/GMR/outputs/" "$LOCAL_MOTIONS/../gmr_outputs/"
    fi
fi

# ── Ensure local conda env ──────────────────────────────────────────────────
ENV_NAME="motion-editor"
if ! conda info --envs 2>/dev/null | grep -q "^$ENV_NAME "; then
    echo "═══ Creating conda env '$ENV_NAME' ═══"
    conda env create -f "$LOCAL_DIR/environment.yaml"
elif [[ "$RECREATE_ENV" -eq 1 ]]; then
    echo "═══ Recreating conda env '$ENV_NAME' ═══"
    conda env remove -n "$ENV_NAME" -y 2>/dev/null || true
    conda env create -f "$LOCAL_DIR/environment.yaml"
else
    echo "═══ Updating conda env '$ENV_NAME' ═══"
    conda env update -n "$ENV_NAME" -f "$LOCAL_DIR/environment.yaml" --prune 2>/dev/null || true
fi

# ── Determine which motion file to open ────────────────────────────────────
if [[ -n "$MOTION_PATH" ]]; then
    LOCAL_FILE="$LOCAL_MOTIONS/$(basename "$MOTION_PATH")"
    if [[ ! -f "$LOCAL_FILE" ]]; then
        echo "Fetching $MOTION_PATH from server..."
        rsync -az "$SERVER:$MOTION_PATH" "$LOCAL_FILE"
    fi
elif [[ -n "$MOTION" ]]; then
    LOCAL_FILE="$LOCAL_MOTIONS/$MOTION"
    if [[ ! -f "$LOCAL_FILE" ]]; then
        echo "Fetching $MOTION from server..."
        rsync -az "$SERVER:~/$REMOTE_MOTIONS_DIR/$MOTION" "$LOCAL_FILE" 2>/dev/null \
            || rsync -az "$SERVER:~/GMR/outputs/$MOTION" "$LOCAL_FILE" 2>/dev/null \
            || { echo "Not found on server"; exit 1; }
    fi
else
    # ── File picker ──
    LOCAL_FILE=$(conda run -n "$ENV_NAME" python "$SCRIPT_DIR/pick_motion.py" "$LOCAL_MOTIONS")
    if [[ -z "$LOCAL_FILE" ]]; then
        echo "No file selected."
        exit 1
    fi
fi

echo "Motion: $LOCAL_FILE"
echo "═══ Launching editor ═══"

# ── Run the editor ─────────────────────────────────────────────────────────
conda run -n "$ENV_NAME" python "$SCRIPT_DIR/../src/motion_editor/main.py" \
    --robot "$ROBOT" \
    --motion "$LOCAL_FILE"

# ── Optional: sync back ─────────────────────────────────────────────────────
if [[ "$SYNC_BACK" -eq 1 ]]; then
    REL="${LOCAL_FILE#$LOCAL_MOTIONS/}"
    REMOTE="$SERVER:~/$REMOTE_MOTIONS_DIR/$REL"
    echo "═══ Syncing back to $REMOTE ═══"
    rsync -az --info=progress2 "$LOCAL_FILE" "$REMOTE"
    if [[ -f "$LOCAL_FILE.bak" ]]; then
        rsync -az "$LOCAL_FILE.bak" "$REMOTE.bak"
    fi
fi

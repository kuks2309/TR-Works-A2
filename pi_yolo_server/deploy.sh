#!/usr/bin/env bash
# Sync this server folder to the Raspberry Pi 5 over the direct link.
# Usage: ./deploy.sh [user@host] [dest_dir]
set -euo pipefail

TARGET="${1:-argoon@10.42.0.2}"
DEST="${2:-~/AI/TR-Works}"
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/"

# --exclude the venv and large model weights from the code sync; copy the .hef
# separately if it lives here (it can be big).
rsync -avz --human-readable \
  --exclude '.venv/' \
  --exclude '__pycache__/' \
  --exclude '*.pyc' \
  "$SRC" "$TARGET:$DEST/"

echo "Deployed to $TARGET:$DEST"
echo "Next on the Pi:  cd $DEST && HAILO_MOCK=1 python3 server.py   # smoke test"

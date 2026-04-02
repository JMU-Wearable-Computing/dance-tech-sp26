#!/usr/bin/env bash
# Launch the mocap_client pipeline (Motive -> OSC -> Isadora)
# Run from the repo root: bash start_mocap.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Activate the virtual environment if it exists
if [ -f ".venv/bin/activate" ]; then
    source .venv/bin/activate
elif [ -f ".venv/Scripts/activate" ]; then
    source .venv/Scripts/activate
fi

python -m mocap_client.main

#!/bin/bash
# Start the GraspGenX ZMQ inference server for the MAGPIE gripper.
#
# GraspGenX (NVlabs, ICRA'26) is a CROSS-EMBODIMENT grasp model: one model, any gripper,
# conditioned on the gripper's swept volume. It runs the MAGPIE gripper zero-shot using
# the Correll-lab gripper definition at ~/GraspGenX/assets/x_grippers/magpie/.
#
# The model lives in its own uv venv (~/GraspGenX/.venv); the magpie pickup talks to it
# over ZMQ via scripts/grasp_detectors/graspgenx_zmq.py (GRASP_METHOD='graspgenx').
#
# Usage:   bash scripts/run_graspgenx_server.sh   [PORT]   [GRIPPER]
#   PORT     default 5557
#   GRIPPER  default magpie  (any name under ~/GraspGenX/assets/x_grippers/)
#
# VRAM (8 GB card): GraspGenX needs ~3 GB. SAM3 needs ~4 GB. They DON'T both fit with
# everything else — so when collecting with GraspGenX, expect to free SAM3 between the
# detect step and grasp planning (cold-start), or run them sequentially. See GRASPGENX.md.

set -e
PORT="${1:-5557}"
GRIPPER="${2:-magpie}"
GGX_DIR="$HOME/GraspGenX"
cd "$GGX_DIR"
echo "Starting GraspGenX server: gripper=$GRIPPER port=$PORT"
echo "  (first call per gripper lazily builds its caches; checkpoints already in ext/)"

# Robust launch. PREFER GraspGenX's own venv python (immune to VS Code snap
# version bumps — the old hardcoded snap/code/247/.local/bin/uv path went stale
# when Code updated to 248/253, which is why the server silently failed to start).
VENV_PY="$GGX_DIR/.venv/bin/python"
if [ -x "$VENV_PY" ]; then
    exec "$VENV_PY" client-server/graspgenx_server.py \
        --config ext/graspgenx_checkpoints/release \
        --assets_dir assets --default_gripper "$GRIPPER" --port "$PORT"
fi

# Fallback: find uv dynamically (any snap/code version, or a system install).
UV="$(command -v uv || true)"
[ -x "$UV" ] || UV="$(ls -dt "$HOME"/snap/code/*/.local/bin/uv 2>/dev/null | head -1)"
[ -x "$UV" ] || UV="$HOME/.local/bin/uv"
[ -x "$UV" ] || { echo "ERROR: no GraspGenX .venv and no uv found — cannot start server"; exit 1; }
exec "$UV" run python client-server/graspgenx_server.py \
    --config ext/graspgenx_checkpoints/release \
    --assets_dir assets --default_gripper "$GRIPPER" --port "$PORT"

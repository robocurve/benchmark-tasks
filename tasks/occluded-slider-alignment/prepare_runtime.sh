#!/usr/bin/env bash
# Project-local official component installation only. Does not call any model.
set -euo pipefail
[[ $# == 1 ]] || { echo 'usage: bash prepare_runtime.sh NEW_ABSOLUTE_DIRECTORY' >&2; exit 2; }
TASK_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
DEST=$1
[[ "$DEST" == /* && ! -e "$DEST" ]] || { echo 'Use a new absolute directory' >&2; exit 2; }
for tool in git uv cmake nvidia-smi; do command -v "$tool" >/dev/null || { echo "Missing prerequisite: $tool" >&2; exit 2; }; done
nvidia-smi >/dev/null
mkdir -p "$DEST"
export UV_CACHE_DIR="$DEST/.cache/uv" UV_PYTHON_INSTALL_DIR="$DEST/.cache/python"
export UV_TOOL_DIR="$DEST/.tools/uv" UV_TOOL_BIN_DIR="$DEST/.tools/bin" PIP_CACHE_DIR="$DEST/.cache/pip"
export ARENA_DIR="$DEST/IsaacLab-Arena"
export DREAM_SPEC=git+https://github.com/kairosiann/inspect-robots-dream@b5e936a201cec66c54c297e522c920d98445e339
git clone https://github.com/kairosiann/yam-isaac-lab-arena.git "$DEST/yam-isaac-lab-arena"
git -C "$DEST/yam-isaac-lab-arena" checkout --detach 0059d02a1a91bc6834c1d0846022891df0c78730
git clone --no-checkout https://github.com/isaac-sim/IsaacLab-Arena.git "$ARENA_DIR"
git -C "$ARENA_DIR" checkout -B release/0.3.1 c7edf3a9d5d8276565208f72422f8f8a0806d6cf
bash "$DEST/yam-isaac-lab-arena/setup.sh"
uv pip install --python "$ARENA_DIR/.venv/bin/python" --constraint "$TASK_ROOT/runtime-constraints.txt" \
  inspect-robots inspect-robots-yam dreamlabs-client numpy pillow
"$ARENA_DIR/.venv/bin/python" "$TASK_ROOT/runtime_check.py"
echo "Installed official runtime. Activate: source $ARENA_DIR/yam.env"

#!/usr/bin/env bash
# Install the pinned runtime in a fresh Linux NVIDIA-GPU container.
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
PREFIX=${ROBOCURVE_SIM_PREFIX:-/workspace/robocurve-sim}
LIMIT=${ROBOCURVE_SETUP_TIMEOUT_SECONDS:-1500}
[[ "$LIMIT" =~ ^[0-9]+$ ]] && (( LIMIT >= 60 && LIMIT <= 1800 )) || exit 2
if [[ ${ROBOCURVE_SETUP_BOUNDED:-0} != 1 ]]; then
  export ROBOCURVE_SETUP_BOUNDED=1
  exec timeout --signal=TERM --kill-after=20s "${LIMIT}s" bash "$0" "$@"
fi
[[ $(uname -m) == x86_64 && $(uname -s) == Linux ]] || { echo 'Requires Linux x86_64'; exit 2; }
[[ $(id -u) == 0 ]] || { echo 'Bootstrap expects a fresh root-owned Ubuntu container'; exit 2; }
command -v nvidia-smi >/dev/null && nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv
mkdir -p "$PREFIX"
export DEBIAN_FRONTEND=noninteractive
apt-get -o DPkg::Lock::Timeout=30 update -qq
apt-get -o DPkg::Lock::Timeout=30 install -y --no-install-recommends \
  ca-certificates curl git git-lfs build-essential cmake pkg-config python3 python3-pip \
  libgl1 libegl1 libglu1-mesa libvulkan1 vulkan-tools libglib2.0-0 libsm6 libxext6 \
  libxrender1 libx11-6 libxcursor1 libxi6 libxrandr2 libxinerama1 libxkbcommon0 libxt6
# Reject a compute-only host/container before the large simulator download.
timeout --signal=TERM --kill-after=5s 45s vulkaninfo --summary > "$PREFIX/vulkan-summary.txt" 2>&1
grep -qi 'NVIDIA' "$PREFIX/vulkan-summary.txt" || { echo 'No NVIDIA Vulkan device visible'; exit 1; }
export UV_CACHE_DIR="$PREFIX/uv-cache" UV_PYTHON_INSTALL_DIR="$PREFIX/python"
export UV_LINK_MODE=copy UV_HTTP_TIMEOUT=600 UV_HTTP_RETRIES=2
export PATH="$PREFIX/bin:$PATH"
mkdir -p "$PREFIX/bin"
# Base images may ship an old uv whose Python-download catalogue predates our
# recorded interpreter. Use a workspace-local installer for this migration.
if [[ ! -x "$PREFIX/bin/uv" ]]; then
  curl --connect-timeout 15 --max-time 90 -fsSL https://astral.sh/uv/install.sh -o "$PREFIX/install-uv.sh"
  UV_INSTALL_DIR="$PREFIX/bin" UV_NO_MODIFY_PATH=1 sh "$PREFIX/install-uv.sh"
fi
export OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y
export CMAKE_POLICY_VERSION_MINIMUM=3.5

fetch_pin() {
  local url=$1 revision=$2 destination=$3
  if [[ ! -d "$destination/.git" ]]; then
    git init -q "$destination"
    git -C "$destination" remote add origin "$url"
  fi
  git -C "$destination" fetch --depth 1 origin "$revision"
  git -C "$destination" checkout --detach FETCH_HEAD
  [[ $(git -C "$destination" rev-parse HEAD) == "$revision" ]]
}
ARENA="$PREFIX/IsaacLab-Arena"
ADAPTER="$PREFIX/yam-isaac-lab-arena"
INSPECT="$PREFIX/inspect-robots"
fetch_pin https://github.com/isaac-sim/IsaacLab-Arena.git c7edf3a9d5d8276565208f72422f8f8a0806d6cf "$ARENA"
git -c url.https://github.com/.insteadOf=git@github.com: -C "$ARENA" \
  submodule update --init --recursive --depth 1 submodules/IsaacLab
[[ $(git -C "$ARENA/submodules/IsaacLab" rev-parse HEAD) == 91a0906f93802eb859192dcb0bca879f891e92d6 ]]
fetch_pin https://github.com/kairosiann/yam-isaac-lab-arena.git 0059d02a1a91bc6834c1d0846022891df0c78730 "$ADAPTER"
fetch_pin https://github.com/robocurve/inspect-robots.git fc5ace923ce85117f9c86f35274122212fbcfe2c "$INSPECT"
# Inspect uses git tags to derive its package version.
git -C "$INSPECT" fetch --tags origin
uv python install 3.12.16
# uv's ordinary Git fetch can pull unnecessary robomimic history very slowly.
# Keep the frozen dependency graph, then install this exact locked commit from
# its independently hashed source archive (its version is not Git-derived).
ROBOMIMIC_COMMIT=7c66e7a41b5d9dcc905b1a68346bfee1b49b79c9
ROBOMIMIC_SHA256=bdfe5d73fa5722212ebcc97a9c456cd4887924b6a079cca47734c028995d8391
mkdir -p "$PREFIX/source-archives"
ROBOMIMIC_ARCHIVE="$PREFIX/source-archives/robomimic-$ROBOMIMIC_COMMIT.tar.gz"
if [[ ! -f "$ROBOMIMIC_ARCHIVE" ]]; then
  curl --connect-timeout 15 --max-time 90 -fsSL \
    "https://codeload.github.com/ARISE-Initiative/robomimic/tar.gz/$ROBOMIMIC_COMMIT" \
    -o "$ROBOMIMIC_ARCHIVE"
fi
printf '%s  %s\n' "$ROBOMIMIC_SHA256" "$ROBOMIMIC_ARCHIVE" | sha256sum -c -
(cd "$ARENA" && uv sync --frozen --inexact --python 3.12.16 --no-install-package robomimic)
PYTHON="$ARENA/.venv/bin/python"
uv pip install --python "$PYTHON" --no-deps "$ROBOMIMIC_ARCHIVE"
uv pip install --python "$PYTHON" --override "$ADAPTER/overrides.txt" \
  -e "$INSPECT" -e "$ADAPTER" 'inspect-robots-yam==0.38.0' \
  'inspect-robots-dream[client] @ git+https://github.com/kairosiann/inspect-robots-dream@b5e936a201cec66c54c297e522c920d98445e339' \
  'dreamlabs-client==1.0.14'
# The recorded task environment uses this NumPy version, despite
# Arena's frozen base lock requesting 2.3.1. Reproduce the recorded environment
# explicitly; do not silently present a different dependency set as exact.
uv pip install --python "$PYTHON" --no-deps 'numpy==2.5.4'
"$PYTHON" - "$ARENA" "$HERE" "$PREFIX" <<'PY'
import hashlib, importlib.metadata, json, platform, re, sys
from pathlib import Path
arena, task, prefix = map(Path, sys.argv[1:])
manifest = json.loads((task/'ENVIRONMENT.json').read_text())
path = arena/'isaaclab_arena/assets/nucleus.py'
text = re.sub(r'^ARENA_NUCLEUS_DIR: str = .*$', manifest['arena_asset_path_patch']['assignment'], path.read_text(), flags=re.M)
path.write_text(text)
actual_hash = hashlib.sha256(path.read_bytes()).hexdigest()
if actual_hash != manifest['arena_asset_path_patch']['sha256']:
    raise SystemExit('Arena asset-path patch hash mismatch')
actual = {name:importlib.metadata.version(name) for name in manifest['packages']}
mismatches = {name:{'expected':wanted,'actual':actual[name]} for name,wanted in manifest['packages'].items() if actual[name]!=wanted}
robomimic = {'commit':'7c66e7a41b5d9dcc905b1a68346bfee1b49b79c9','archive_sha256':'bdfe5d73fa5722212ebcc97a9c456cd4887924b6a079cca47734c028995d8391','version':importlib.metadata.version('robomimic'),'transport':'GitHub exact-commit source archive; frozen dependency graph unchanged'}
if robomimic['version'] != '0.4.0':
    raise SystemExit('Unexpected robomimic archive package version')
report = {'python':platform.python_version(),'packages':actual,'expected_manifest':'ENVIRONMENT.json','package_mismatches':mismatches,'asset_patch_sha256':actual_hash,'source_archive_overrides':{'robomimic':robomimic},'runtime_tested':False}
(prefix/'installation.json').write_text(json.dumps(report,indent=2)+'\n')
if mismatches or platform.python_version()!=manifest['python']:
    raise SystemExit('Recorded Python/package version check failed; inspect installation.json')
print('All recorded package versions and asset-patch hash match.')
PY
FFMPEG=$("$PYTHON" -c 'import imageio_ffmpeg; print(imageio_ffmpeg.get_ffmpeg_exe())')
ln -sf "$FFMPEG" "$ARENA/.venv/bin/ffmpeg"
printf 'source %q\nexport OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y\n' "$ARENA/.venv/bin/activate" > "$ARENA/yam.env"
echo "Installation ready. Use $PYTHON to run this task."

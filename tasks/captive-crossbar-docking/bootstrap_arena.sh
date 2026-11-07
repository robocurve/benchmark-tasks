#!/usr/bin/env bash
# Headless bootstrap of the validated Arena stack only.
# Graphics development headers below were installed as a separate recorded step during validation.
set -euo pipefail
ROOT=/data/robocurve-t20
ARENA=/data/IsaacLab-Arena
STACK=/data/yam-isaac-lab-arena
mkdir -p "$ROOT"
exec > >(tee -a "$ROOT/bootstrap.log") 2>&1
step() { echo "=== [t20-bootstrap] $*"; }
step 'GPU and disk preflight'
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader
uname -m
df -h /data
step 'headless system dependencies'
sudo apt-get update -y
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends git curl cmake build-essential ffmpeg libgl1 libglu1-mesa libvulkan1 libglib2.0-0 libsm6 libxext6 libxrender1 libxcursor1 libxi6 libxinerama1 libxrandr2 libgl1-mesa-dev libx11-dev libxcursor-dev libxi-dev libxrandr-dev libxinerama-dev
export PATH="$HOME/.local/bin:$PATH"
if ! command -v uv >/dev/null; then curl -LsSf https://astral.sh/uv/install.sh | sh; fi
export UV_CACHE_DIR=/data/.uv-cache
export OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y
clone_at() {
  local destination=$1 repository=$2 revision=$3
  if [[ ! -d "$destination/.git" ]]; then
    [[ ! -e "$destination" ]] || { echo "Refuse non-Git destination: $destination"; exit 2; }
    git clone "$repository" "$destination"
    git -C "$destination" checkout --detach "$revision"
  else
    [[ "$(git -C "$destination" rev-parse HEAD)" == "$revision" ]] || { echo "Unexpected existing revision: $destination"; exit 2; }
  fi
}
step 'pinned Arena and YAM sources'
clone_at "$ARENA" https://github.com/isaac-sim/IsaacLab-Arena.git c7edf3a9d5d8276565208f72422f8f8a0806d6cf
git -c url.https://github.com/.insteadOf=git@github.com: -C "$ARENA" submodule update --init --recursive
[[ "$(git -C "$ARENA/submodules/IsaacLab" rev-parse HEAD)" == 91a0906f93802eb859192dcb0bca879f891e92d6 ]] || { echo 'Isaac Lab revision mismatch'; exit 2; }
clone_at "$STACK" https://github.com/kairosiann/yam-isaac-lab-arena.git 0059d02a1a91bc6834c1d0846022891df0c78730
step 'locked Arena environment'
(cd "$ARENA" && CMAKE_POLICY_VERSION_MINIMUM=3.5 uv sync --frozen --inexact)
step 'pinned inspection and model adapters'
uv pip install --python "$ARENA/.venv/bin/python" --override "$STACK/overrides.txt" -e "$STACK" \
  'inspect-robots-dream[client] @ git+https://github.com/kairosiann/inspect-robots-dream@b5e936a201cec66c54c297e522c920d98445e339' \
  'inspect-robots==0.60.0' 'inspect-robots-yam==0.37.0' 'dreamlabs-client==1.0.14' 'numpy==2.3.1' 'Pillow==12.3.0' 'pytest>=8'
printf 'source %q\nexport OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y\n' "$ARENA/.venv/bin/activate" > "$ARENA/yam.env"
source "$ARENA/yam.env"
step 'asset-source verification'
python - <<'PY'
from pathlib import Path
import urllib.request, urllib.error, json, subprocess
p=Path('/data/IsaacLab-Arena/isaaclab_arena/assets/nucleus.py')
old='ARENA_NUCLEUS_DIR: str = ISAACLAB_NUCLEUS_DIR.replace("omniverse-content-production", "omniverse-content-staging")'
new='ARENA_NUCLEUS_DIR: str = ISAACLAB_NUCLEUS_DIR'
u='https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1/Isaac/IsaacLab/Arena/assets/object_library/srl_robolab_assets/objects/objaverse/apple_01.usd'
results={}
for name,url in [('staging',u),('production',u.replace('content-staging','content-production'))]:
    try:
        with urllib.request.urlopen(urllib.request.Request(url,method='HEAD'),timeout=20) as response: results[name]=response.status
    except urllib.error.HTTPError as error: results[name]=error.code
print(json.dumps(results))
s=p.read_text()
if results=={'staging':404,'production':200}:
    if old in s:
        assert s.count(old)==1
        p.write_text(s.replace(old,new))
    elif new not in s: raise SystemExit('Unexpected asset source; no patch applied')
    patch=subprocess.check_output(['git','-C','/data/IsaacLab-Arena','diff','--','isaaclab_arena/assets/nucleus.py'])
    Path('/data/robocurve-t20/arena-production-assets.patch').write_bytes(patch)
    print('Verified production asset source used')
else:
    print('No historical asset patch applied; review actual result before simulation')
PY
step 'verify entry points'
inspect-robots list embodiments
inspect-robots list policies
step 'done'

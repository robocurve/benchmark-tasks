# Official environment

The task runs in the official RoboCurve walkthrough's native Isaac Lab-Arena
station environment. Existing walkthrough installations should activate their
Arena `yam.env`, run `python runtime_check.py`, then follow README.md. No original
project paths, credentials, synthetic replacement environment or remote assets
specific to this task are required. The task's solids are authored locally.

Sources:
- RoboCurve simulation walkthrough: https://tasks.robocurve.org/walkthrough
- Walkthrough launchable bootstrap (source of fixed adapter pins):
  https://github.com/ttpierson/YAM-inspect-Isaac/blob/d825f0c51724f750f26e8c3f5620a14b5b0261b0/launchable/setup.sh
- Native RTX-machine component installer:
  https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/setup.sh

## New local installation

Requires Ubuntu/Linux, a supported RTX GPU and working NVIDIA driver, git, uv,
cmake, ffmpeg/ffprobe for media export/checking, and sufficient disk for the official environment (allow at least60 GB).
Do not run the launchable's whole VM bootstrap on an existing workstation. Use the
component installer without `--desktop`; it needs no system service or driver edits.
Read the Isaac Sim license and official prerequisites before installing/using it.

`prepare_runtime.sh` downloads the exact source revisions into a NEW directory and
runs that unmodified component installer with local caches and fixed Dream spec,
then pins the listed adapter/client dependencies using runtime-constraints.txt.
It refuses an existing destination and does not install uv/cmake/drivers/system
packages. It does not set credentials, create a Dream client, or make model calls.
Installation downloads may be large. This release reused the already validated
pinned runtime; an isolated task-folder copy was executed successfully. A second
full download/install into a fresh venv has not been claimed as tested.

```bash
bash prepare_runtime.sh /absolute/path/to/new/slider-runtime
source /absolute/path/to/new/slider-runtime/IsaacLab-Arena/yam.env
python runtime_check.py
# Optional CPU test dependencies, inside this environment only:
uv pip install --python "$(command -v python)" 'pytest==9.1.1' 'scipy==1.17.0'
```

Use `runtime-lock.json` and runtime_check.py to refuse silent drift. The official
installer follows a release branch; this wrapper creates that branch at the exact
validated Arena commit before invoking it. Isaac Lab submodules are fetched at the
Arena gitlinks. Task assets need no external fruit URLs. Rendering may log the
existing DLSS presetD fallback with NVIDIA570.172.08; this release does not modify
render settings or recommend driver changes.

For Dream only, use the SDK-supported `DREAM_API_KEY` environment variable or
`~/.dreamlabs/credentials` with mode600 (or `DREAM_CREDENTIALS` path). Never place a
key in this task folder or a command-line argument. `check_dream_access.py` checks
only the official endpoint and model-list access; it does not warm up the model.

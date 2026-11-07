# Official environment

Use the original RoboCurve walkthrough's native Isaac Lab-Arena YAM station.
The exact adapter source is0059d02a1a91bc6834c1d0846022891df0c78730; Arena is
c7edf3a9d5d8276565208f72422f8f8a0806d6cf; Dream adapter is
b5e936a201cec66c54c297e522c920d98445e339. Package versions are checked by
runtime_check.py and recorded in runtime-lock.json. No alternative rig or assets.

Sources:
- https://tasks.robocurve.org/walkthrough
- https://github.com/ttpierson/YAM-inspect-Isaac/blob/d825f0c51724f750f26e8c3f5620a14b5b0261b0/launchable/setup.sh
- https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/setup.sh

For an existing official installation, activate its Arena yam.env and use README.
For a new supported Linux RTX host with working driver, git,uv,cmake and ffmpeg:

```bash
bash prepare_runtime.sh /absolute/path/to/new/guide-runtime
source /absolute/path/to/new/guide-runtime/IsaacLab-Arena/yam.env
python runtime_check.py
```

The helper refuses an existing directory, downloads the pinned official component
sources with local caches and invokes the original component installer without a
desktop/system bootstrap. Allow at least60 GB. It does not install drivers, system
packages, services or keys and makes no model calls. This release reuses a verified
pinned runtime; a second full fresh-environment installation is not claimed tested.

Source/package drift is an error, not a reason to silently upgrade. Task assets are
local primitive solids. The existing DLSS preset-D fallback on a570-series driver
is preserved and recorded; no renderer or system configuration change is needed.

For paid evaluation only, the SDK reads DREAM_API_KEY or ~/.dreamlabs/credentials
(mode600), optionally a DREAM_CREDENTIALS path. Configuration-only mode reads no
credentials and creates no hosted client. Model access and current balance are
separate from local runtime readiness; no live access/credit query is claimed here.

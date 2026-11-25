# Pan a tripod camera to follow a moving subject

Use the pan handle to film the moving orange target in one level shot. Keep the whole target centered through the marked crossing.

The complete fixed layout, materials, scoring thresholds and approximations are in `task.json`.
The primary score is computed from physical object state by `scene.py`, independently of the policy.

The Logitech MX Brio 4K ($199.99 list) fits the head's stock 1/4-20 plate. The [physical trial protocol](assets/PROTOCOL.md) gives the recording setup and field-of-view check.

## Install

Use Linux and an NVIDIA RTX GPU. This task was evaluated with Python 3.12, Isaac Sim 6.1,
Isaac Lab 3.0 and Arena 0.3.1. Create the standard native YAM/Arena environment:

```bash
git clone https://github.com/kairosiann/yam-isaac-lab-arena
git -C yam-isaac-lab-arena checkout bd62771e60320433893bd1d2df0fc79b5ddcd6d1
cd yam-isaac-lab-arena
./setup.sh
source ~/IsaacLab-Arena/yam.env
```

`ARENA_DIR` may point to another drive before setup; source that directory's `yam.env`.
The runtime commits used are in `runtime_revisions.json`. If the pinned Arena release points
at inaccessible staging assets, apply `arena_public_asset_root.patch` to the Arena checkout.
From this task folder, with its Arena environment activated:

```bash
uv pip install -e . --no-deps
```

The provided source has an isolated package namespace and plugin names, so installing this
task alongside another task does not overwrite that task's code.

## Run and score

```bash
python run.py demo --out demo_trial
python run.py idle --out idle_trial
python run.py dream --out model_trial --budget-ledger dream_budget.json
```

For Dream-1, configure `DREAM_API_KEY` or the standard private Dream SDK credential file.
The optional ledger reserves each published request tariff before inference and caps the
ledger at $10. It does not claim to report the provider's invoice.
Every run uses seed 0, zero jitter, 20 Hz control, 240 Hz physics on `cpu` and three 480x270 camera observations.
The action limit is 480 steps. Cold rendering can take several minutes;
simulation time pauses while inference is pending. The demo can end on sustained success.
Recorded scores for the two videos are: Dream-1 beta run: 0.00; scripted native-contact demo: 1.00.

Use the completed `adhoc_*.json` report, excluding any `.live.json`:

```bash
python score_trial.py model_trial/adhoc_REPORT_ID.json
inspect-robots video model_trial/adhoc_REPORT_ID.json
```

The video command exports each recorded camera to H.264 MP4. No task object is assigned
a pose during execution. The demonstration uses privileged state to command robot joints;
the Dream policy receives the standard cameras and robot joint state.

## Reset and licenses

Restore the fixed starting layout from task.json.

New task code and generated primitive geometry are MIT licensed; see `LICENSE`.
The YAM robot model (URDF and meshes) is MIT licensed by I2RT Robotics. The pinned
Arena wrapper vendors the notice at
[`yam_arena/assets/yam/LICENSE.i2rt`](https://github.com/kairosiann/yam-isaac-lab-arena/blob/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam/LICENSE.i2rt).
At the pinned revision, the [YAM Arena rig wrapper README](https://github.com/kairosiann/yam-isaac-lab-arena/blob/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/README.md#license)
states that the wrapper and its upstream `YAM-inspect-Isaac` code have no license.
The task bundle includes original task geometry and fetches the rig separately as
a runtime dependency. It redistributes no upstream rig code, URDF or robot meshes.
Isaac Lab-Arena, Isaac Lab and Isaac Sim retain their upstream licenses and terms.

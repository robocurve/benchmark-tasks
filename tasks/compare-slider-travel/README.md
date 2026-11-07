# Inspect concealed slider travel and mark the largest

Both arms test the full travel of separate knobs. One arm then tests the central knob while the other stays clear. The marker is placed beside the positive y fixture only after all six end positions have been reached.

## Reproduce the trials

Use the official YAM Isaac Lab Arena environment. The recorded environment uses the YAM repository revision `0059d02a1a91bc6834c1d0846022891df0c78730` and Arena release 0.3.1.

```bash
source /data/IsaacLab-Arena/yam.env
uv pip install --python /data/IsaacLab-Arena/.venv/bin/python --no-deps -e runtime -e policy
OPENBLAS_NUM_THREADS=1 python validate.py
TASK_INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2 MKL_NUM_THREADS=1 inspect-robots run \
  --embodiment yam_articulated --policy yam_scripted \
  -E task="$PWD/task.py" -E object_jitter=0 -E control_hz=20 \
  -E cam_width=1280 -E cam_height=720 --seed 0 --epochs 1 \
  --instruction "$TASK_INSTRUCTION" --scorer task_score --max-steps 2400 \
  --store-frames --log-dir runs/demo
```

## Geometry and reset

scene_inventory.json lists all local cuboids and link poses and joint settings. Compound size values are validation hints. The local collider solids and joint poses define the physical geometry. Use rigid plastic training props with ballast to match the stated masses. Fix the static fixture roots to the table. Follow the reset procedure in task.json. No powered equipment is involved.

## Novelty review

The live board includes aiming a geared pointer through backlash. This task compares manual linear stroke ranges on three separate fixtures and requires observed coverage of both physical stops on every fixture before an identification answer earns credit. The result is not a target pointer angle or an assembly placement.


## Current model evaluation

The selected model evaluation uses Dream 1 with checkpoint `dreamlabs/dream-1-beta` and a limit of 2400 control steps at 20 Hz. It uses seed 0 and no object jitter. The separate scripted demonstration is not a model result.

Install `dreamlabs==1.0.37` and `dreamlabs-client==1.1.3` in the bridge environment. Start the supplied bridge with `DREAM_BRIDGE_PORT=8241 JAX_PLATFORMS=cpu python policy/current_api_bridge.py` in the bridge environment. Keep credentials outside this repository. Run the evaluation command with `--policy dream_current` and `-P server_url=http://127.0.0.1:8241` and a separate log directory. The bridge maps the overhead camera and both wrist cameras to the official YAM state fields. It uses the documented paused simulator convention for action timing. The simulator applies control at 20 Hz. Session expiry fails the trial rather than restarting its model memory.

Render the completed log with `inspect-robots video LOG.json`. Each selected camera video is native 1280 by 720 H264 at 20 frames per second. The model video must contain every recorded frame. No time padding is used. See validation.json for the final measured scores and source hashes after validation finishes.

## Selected current trials

Dream 1 scored 0.2 in 2400 control steps. The separate scripted demo scored 1.0 in 660 control steps. Both used the same task source and seed 0. Neither trial reported an evaluation error. These are single trials and do not estimate a general success rate.

The task source SHA256 is `1ee46d4f34e3869ffac9fafa87fe612fe4517369ce5413b15451e1aa5ac70d1f`. validation.json identifies the selected receipts. video_audit.json records the checksums and native camera frame counts. The score plot shows measured scores through the recorded trajectories.

This submission replaces PR 120. It preserves our original task idea and is not an additional new idea.

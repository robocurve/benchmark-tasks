# Put the 9-volt battery in the picking bin

A pick-and-place among distractors, with two real-scale NVIDIA SimReady props: a 9-volt battery and an open
cardboard picking bin. Four coloured wooden blocks lie on the table. Only the battery belongs in the bin.

| File | What it is |
|---|---|
| `battery_into_bin.py` | The `yam_arena` task file: scene, scoring (`success`, `score`) and the scripted `demo()` |
| `simready.py` | Loads SimReady props into a task file (`SimReady(...)`), downloading them once from Hugging Face |
| `test_battery_into_bin.py` | Offline checks: the file loads, every scoring outcome, the demo's reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

Both props come from the Hugging Face dataset [`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets),
pinned to revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [`power_battery_9v_a01/simready_usd/sm_power_battery_9v_a01_01.usd`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/power_battery_9v_a01)
- [`box_cardboard_open_a06/simready_usd/sm_box_cardboard_open_a06_01.usd`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/box_cardboard_open_a06)

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes made at load time:
- the box is scaled to 45 %, to a 24 x 20 cm picking bin, and fixed in place.
- the battery's mass is set to 45 g (the asset's 0.2 kg treats it as solid steel), with its inertia scaled to match.
- the battery collides as its convex hull (a box) instead of a signed distance field, so the finger pads grip it cleanly.
- the invisible grasp-annotation curves are deactivated.

The files (about 50 MB, mostly the box's textures) are not committed. `simready.py` downloads them into
`~/.cache/simready` the first time the scene is built, which needs network access, as Arena's own library assets do.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`):

```bash
# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=battery_into_bin.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the 9-volt battery in the cardboard picking bin." --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/battery-dream

# Scripted demo (right arm), 400 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=battery_into_bin.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the 9-volt battery in the cardboard picking bin." --scorer task_score \
  --max-steps 400 --store-frames --log-dir ~/yam-eval/battery-demo
```

- The run prints `task_score` at the end. With `-E object_jitter=0` the layout is fixed and no seed is needed.
- The demo (right arm) reaches 1.0 and ends on success at step 187 (9.4 s at 20 Hz). A trial takes
  about 75 s on an L40S, including startup.
- Videos: `inspect-robots video ~/yam-eval/battery-demo/<log>.json` writes one MP4 per camera next to the frames.
- Offline checks: `python -m pytest -q test_battery_into_bin.py`.

## Scoring

All positions are centres, in metres, in the station frame (x forward, y to the robot's left, from the midpoint of the
arm bases), and heights are above the table top.

- **0** for any violation at the end: a block's centre over the bin's footprint (packed by mistake), an object fallen
  off the table (centre more than 2 cm below the top), or a non-finite state.
- **1.0 (success, held 1 s):** the battery's centre at least 1.2 cm inside the bin's outer walls and under 4 cm high
  (it lies on the bin floor, upright or on its side), the battery still (under 1 cm/s and 0.5 rad/s), and no
  violation.
- **0.5:** the battery inside the bin (same footprint, centre below the back wall's top) but not lying still on its
  floor (held above it, or still moving).
- **0.25:** the battery lifted clear of the table elsewhere (centre more than 6 cm up).
- **0** otherwise, including doing nothing.

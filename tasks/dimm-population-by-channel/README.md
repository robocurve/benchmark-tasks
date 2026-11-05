# Populate server memory one DIMM per channel, first slots first

Instruction given to the policy: `The board has four memory channels, each a pair of neighbouring slots, two pairs on each side of the processor. Install the four DIMMs from the trays following the population rule: one DIMM per channel, in each channel's first slot (the slot of the pair farther from the processor, with white latches), before any second slot (black latches). Stand each DIMM upright between its slot's guides.`

Files: `task.json`, `dimm_population_by_channel.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/dimm-population-by-channel/dimm_population_by_channel.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 17 cases pass
left: 35.4 s plan, worst IK 0.091 mm / 0.013 deg
right: 35.4 s plan, worst IK 0.091 mm / 0.013 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/dimm-population-by-channel/dimm_population_by_channel.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The board has four memory channels, each a pair of neighbouring slots, two pairs on each side of the processor. Install the four DIMMs from the trays following the population rule: one DIMM per channel, in each channel's first slot (the slot of the pair farther from the processor, with white latches), before any second slot (black latches). Stand each DIMM upright between its slot's guides." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/dimm-population-by-channel-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/dimm-population-by-channel/dimm_population_by_channel.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The board has four memory channels, each a pair of neighbouring slots, two pairs on each side of the processor. Install the four DIMMs from the trays following the population rule: one DIMM per channel, in each channel's first slot (the slot of the pair farther from the processor, with white latches), before any second slot (black latches). Stand each DIMM upright between its slot's guides." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/dimm-population-by-channel-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 35 s of plan (about 908 steps
including the success hold), inside the run limit of 1,200 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/dimm-population-by-channel-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/dimm-population-by-channel-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,200 steps (1 minute at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Stand the four DIMMs back in the tray channels; the board, processor, guides, latches and trays do not move.

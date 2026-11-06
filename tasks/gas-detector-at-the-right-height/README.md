# Fit each gas detector at the right height for its gas: LPG low, natural gas high

On the station table: an LPG heater at the far left and a natural-gas boiler at the far right, each with a card in front of it naming it, and in front of each appliance a low ledge (2 cm, nearer the arms) and a tall ledge (16 cm) for a gas detector. Two detectors stand in front of the arms with cards naming their gas: an LPG detector (just right of centre, nearer the boiler) and a natural-gas detector (just left of centre, nearer the heater). LPG is heavier than air and pools at the floor, so its detector must be fitted low; natural gas is lighter than air and rises, so its detector must be fitted high. The appliances, ledges and cards are fixed; the detectors are loose.

| File | What it is |
|---|---|
| `task.json` | The task's details: instruction, setup, scoring, run limit, fidelity, scores, media ID |
| `task.py` | The `yam_arena` task file: the objects, `success()`, `score()`, `outcome()` and the scripted `demo()` |

## Requirements

The Isaac Lab-Arena environment from the program's Brev launchable (simulation walkthrough), with the `yam_arena`
embodiment, `inspect-robots` and the Dream-1 client. The task uses only `yam_arena`'s task-file objects and the
standard `task_score` scorer; nothing else to install.

## Run a trial

From this folder, after `source <ARENA_DIR>/yam.env`:

```bash
# Dream-1, at the run limit
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "fit each gas detector by the appliance that burns its gas, at the right height for that gas" \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/gas-detector-at-the-right-height/dream

# Scripted demo (reaches 1.0 in about 380 steps)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "fit each gas detector by the appliance that burns its gas, at the right height for that gas" \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/gas-detector-at-the-right-height/demo

# Videos (one MP4 per camera, next to each run's frames)
for log in ~/yam-eval/gas-detector-at-the-right-height/*/*.json; do inspect-robots video "$log"; done
```

The demo is open loop, so it needs `-E object_jitter=0`. The left arm fits the LPG detector (just right of centre) on the heater's low ledge; then the right arm fits the natural-gas detector on the boiler's tall ledge (it waits, parked, while the left arm crosses the middle).

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s (the episode then ends), and otherwise `score()` at the end of the episode. An object counts as still when it moves under 1 cm/s and 0.5 rad/s, or has not moved at all (under 0.5 mm) since the previous control step: PhysX can put a settled body to sleep while it still reports a small velocity.

| Outcome (`outcome()`) | Condition | Score |
|---|---|---|
| `fitted` | LPG detector on the heater's low ledge, natural-gas detector on the boiler's tall ledge | 1.0 |
| `N_right` | N detectors by their own appliance at the right height | 0.5 N |

A detector is on a ledge when it rests still with its centre over the ledge's top and its underside within 1 cm of it. In the real world, check these by eye or with a tape measure.

## Runtime, seeds and reset

- Run limit: 1,200 steps (1 minute at 20 Hz).
- Runtime per trial on an L40S, including Isaac Sim's start-up (about 25 s once its shaders are cached; the first
  start on a new instance takes several minutes more): about 228 s for the scripted demo and
  583 s for the Dream-1 run.
- Seeds: the layout is fixed (`-E object_jitter=0`), so every trial starts identically; no seed is needed.
- Reset: Stand the two detectors back on their starting spots. The appliances, ledges and cards are fixed (putty) and stay in place.

## Results in simulation

One trial each, with the commands above (Isaac Lab-Arena 0.3.1, `-E object_jitter=0 -E control_hz=20`):

| Run | `task_score` |
|---|---|
| Scripted demo | 1.0 |
| Dream-1 | 0.0 |

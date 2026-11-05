# Stand a calibration board in its holder, facing the camera

A camera on a post looks back at the arms. A calibration board stands edge-on in a storage slot. The robot must move it into the holder in front of the camera, turned a quarter turn so it faces the camera. Setting up camera calibration is routine upkeep for every robot in a fleet, and today a person does it.

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
  --instruction "stand the calibration board in its holder, facing the camera" \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/calibration-board-in-holder/dream

# Scripted demo (reaches 1.0 in about 186 steps)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "stand the calibration board in its holder, facing the camera" \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/calibration-board-in-holder/demo

# Videos (one MP4 per camera, next to each run's frames)
for log in ~/yam-eval/calibration-board-in-holder/*/*.json; do inspect-robots video "$log"; done
```

The demo is open loop, so it needs `-E object_jitter=0`. The right arm grips the board's top edge, lifts it 4.5 cm straight out of storage (clear of the 3 cm rails), turns it a quarter turn while carrying it to the holder and lowers it into the slot.

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s (the episode then ends), and otherwise `score()` at the end of the episode. An object counts as still when it moves under 1 cm/s and 0.5 rad/s, or has not moved at all (under 0.5 mm) since the previous control step: PhysX can put a settled body to sleep while it still reports a small velocity.

| Outcome (`outcome()`) | Condition | Score |
|---|---|---|
| `in_holder` | Standing in the holder's slot, facing the camera | 1.0 |
| `facing_near` | Standing and facing the camera within 10 cm of the slot, not in it | 0.5 |
| `standing` | Out of storage and standing elsewhere, or turned edge-on | 0.25 |
| `other` | Still in storage, lying flat, dropped or held | 0.0 |

Standing means the board rests still on the table with its top at least 8.5 cm above its bottom. Facing the camera means it is no more than 5 cm deep front to back and at least 14.5 cm wide left to right. In the slot means its centre is within 1.5 cm of the slot's centre line, front to back, and within the rails' 12 cm length. In the real world, check all three with a tape measure.

## Runtime, seeds and reset

- Run limit: 1,200 steps (1 minute at 20 Hz).
- Runtime per trial on an L40S, including Isaac Sim's start-up (about 25 s once its shaders are cached; the first
  start on a new instance takes several minutes more): about 38 s for the scripted demo and
  159 s for the Dream-1 run.
- Seeds: the layout is fixed (`-E object_jitter=0`), so every trial starts identically; no seed is needed.
- Reset: Stand the board back in the storage slot, edge-on to the camera. The camera, holder and storage rails are fixed.

## Results in simulation

One trial each, with the commands above (Isaac Lab-Arena 0.3.1, `-E object_jitter=0 -E control_hz=20`):

| Run | `task_score` |
|---|---|
| Scripted demo | 1.0 |
| Dream-1 | 0.0 |

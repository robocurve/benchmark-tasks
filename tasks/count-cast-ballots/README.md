# Count the cast ballots into their trays, leaving out a ballot that was never cast

Three ballots (red, blank and blue) are in an open ballot box and must be counted into their tally trays. A fourth ballot, red, lies on the table outside the box: it was never cast. Counting it would add a vote nobody cast, so it scores 0 however well the rest is done.

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
  --instruction "count the cast ballots: move each ballot from the ballot box to its tray, red to red, blue to blue, blank to the grey tray" \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/count-cast-ballots/dream

# Scripted demo (reaches 1.0 in about 378 steps)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "count the cast ballots: move each ballot from the ballot box to its tray, red to red, blue to blue, blank to the grey tray" \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/count-cast-ballots/demo

# Videos (one MP4 per camera, next to each run's frames)
for log in ~/yam-eval/count-cast-ballots/*/*.json; do inspect-robots video "$log"; done
```

The demo is open loop, so it needs `-E object_jitter=0`. The left arm moves the red ballot to the red tray; the right arm moves the blue ballot to the blue tray and then the blank one to the grey tray. The ballot outside the box is left alone.

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s (the episode then ends), and otherwise `score()` at the end of the episode. An object counts as still when it moves under 1 cm/s and 0.5 rad/s, or has not moved at all (under 0.5 mm) since the previous control step: PhysX can put a settled body to sleep while it still reports a small velocity.

| Outcome (`outcome()`) | Condition | Score |
|---|---|---|
| `counted` | Each cast ballot in its right tray, the uncast one untouched by the count | 1.0 |
| `R_right_W_wrong` | R cast ballots in their right trays, W in wrong ones | R/3 - W/6, at least 0 |
| `uncast_counted` | The ballot from outside the box in any tray, whatever else | 0.0 |

A ballot is in a tray when it rests still with its centre inside the tray's walls and its underside within 4 cm of the table. In the real world, check these by eye.

## Runtime, seeds and reset

- Run limit: 1,200 steps (1 minute at 20 Hz).
- Runtime per trial on an L40S, including Isaac Sim's start-up (about 25 s once its shaders are cached; the first
  start on a new instance takes several minutes more): about 70 s for the scripted demo and
  177 s for the Dream-1 run.
- Seeds: the layout is fixed (`-E object_jitter=0`), so every trial starts identically; no seed is needed.
- Reset: Put the three cast ballots back in the box and the uncast red ballot back beside it (see `setup`). The box and trays are fixed.

## Results in simulation

One trial each, with the commands above (Isaac Lab-Arena 0.3.1, `-E object_jitter=0 -E control_hz=20`):

| Run | `task_score` |
|---|---|
| Scripted demo | 1.0 |
| Dream-1 | 0.0 |

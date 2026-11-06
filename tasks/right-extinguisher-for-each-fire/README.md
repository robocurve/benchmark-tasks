# Stand the wet-chemical extinguisher by the fryer and the CO2 one by the computer, reading their type cards

On the station table: a deep-fat fryer at the far left with a card beyond it reading "FRYER", and a computer monitor at the far right with a card reading "PC", each with a red floor mark in front of it where an extinguisher should stand. In a row in front of the arms stand four fire extinguishers, each with a card in front of it naming its type: WATER (far left), CO2, WET CHEM and FOAM (far right). The instruction names the wet-chemical one for the fryer and the CO2 one for the computer, so the robot must read the type cards; water on burning fat throws it out in a fireball, and water on live electrics conducts. The fryer, monitor, marks and cards are fixed; the extinguishers are loose.

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
  --instruction "stand the wet chemical extinguisher on the mark by the fryer and the CO2 extinguisher on the mark by the computer" \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/right-extinguisher-for-each-fire/dream

# Scripted demo (reaches 1.0 in about 387 steps)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "stand the wet chemical extinguisher on the mark by the fryer and the CO2 extinguisher on the mark by the computer" \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/right-extinguisher-for-each-fire/demo

# Videos (one MP4 per camera, next to each run's frames)
for log in ~/yam-eval/right-extinguisher-for-each-fire/*/*.json; do inspect-robots video "$log"; done
```

The demo is open loop, so it needs `-E object_jitter=0`. The left arm carries the wet-chemical extinguisher to the fryer's mark; then the right arm carries the CO2 one to the computer's mark (it waits, parked out of the way, while the left arm crosses the middle).

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s (the episode then ends), and otherwise `score()` at the end of the episode. An object counts as still when it moves under 1 cm/s and 0.5 rad/s, or has not moved at all (under 0.5 mm) since the previous control step: PhysX can put a settled body to sleep while it still reports a small velocity.

| Outcome (`outcome()`) | Condition | Score |
|---|---|---|
| `ready` | Wet chemical on the fryer mark, CO2 on the computer mark | 1.0 |
| `N_right` | N marks with their right extinguisher standing on them | 0.5 N |
| `water_on_fire_risk` | The water extinguisher standing on either mark, whatever else | 0.0 |

An extinguisher is on a mark when it stands upright (rising more than 12 cm) with its centre over the mark (and, for credit, is at rest). In the real world, check these by eye or with a tape measure.

## Runtime, seeds and reset

- Run limit: 1,200 steps (1 minute at 20 Hz).
- Runtime per trial on an L40S, including Isaac Sim's start-up (about 25 s once its shaders are cached; the first
  start on a new instance takes several minutes more): about 237 s for the scripted demo and
  598 s for the Dream-1 run.
- Seeds: the layout is fixed (`-E object_jitter=0`), so every trial starts identically; no seed is needed.
- Reset: Stand the four extinguishers back on their spots behind their type cards. The fryer, monitor, keyboard, marks and cards are fixed (putty or tape) and stay in place. Use empty extinguisher shells or painted tubes for real trials.

## Results in simulation

One trial each, with the commands above (Isaac Lab-Arena 0.3.1, `-E object_jitter=0 -E control_hz=20`):

| Run | `task_score` |
|---|---|
| Scripted demo | 1.0 |
| Dream-1 | 0.0 |

# Hang a patient's IV bag above them and the urine drainage bag below them

On the station table: a patient lying on a bed at the far right, and at the left of the bed two bedside stands, a low one (2 cm) nearer the arms and a tall one (16 cm, above the patient) beyond it. In front of the arms are the patient's two bags, each with a card in front of it: an IV drip bag (left, nearer the low stand) and a urine drainage bag (middle). An IV drip runs by gravity, so its bag must hang above the patient; a drainage bag must hang below the bladder, or urine flows back into it and causes infection. The bed, patient, stands and cards are fixed; the bags are loose.

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
  --instruction "hang the patient's IV bag and urine drainage bag on the bedside stands, each at the right height" \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/iv-bag-above-drainage-bag-below/dream

# Scripted demo (reaches 1.0 in about 378 steps)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "hang the patient's IV bag and urine drainage bag on the bedside stands, each at the right height" \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/iv-bag-above-drainage-bag-below/demo

# Videos (one MP4 per camera, next to each run's frames)
for log in ~/yam-eval/iv-bag-above-drainage-bag-below/*/*.json; do inspect-robots video "$log"; done
```

The demo is open loop, so it needs `-E object_jitter=0`. The left arm sets the IV bag on the tall stand, then the drainage bag on the low stand.

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s (the episode then ends), and otherwise `score()` at the end of the episode. An object counts as still when it moves under 1 cm/s and 0.5 rad/s, or has not moved at all (under 0.5 mm) since the previous control step: PhysX can put a settled body to sleep while it still reports a small velocity.

| Outcome (`outcome()`) | Condition | Score |
|---|---|---|
| `hung` | IV bag on the tall stand, drainage bag on the low stand | 1.0 |
| `N_right` | N bags on their right stands | 0.5 N |
| `drainage_above_patient` | The drainage bag on the tall stand, whatever else | 0.0 |

A bag is on a stand when its centre is over the stand's top and its underside within 1 cm of it (and, for credit, it is at rest). In the real world, check these by eye or with a tape measure.

## Runtime, seeds and reset

- Run limit: 1,200 steps (1 minute at 20 Hz).
- Runtime per trial on an L40S, including Isaac Sim's start-up (about 25 s once its shaders are cached; the first
  start on a new instance takes several minutes more): about 150 s for the scripted demo and
  408 s for the Dream-1 run.
- Seeds: the layout is fixed (`-E object_jitter=0`), so every trial starts identically; no seed is needed.
- Reset: Stand the two bags back on their starting spots. The bed, patient, stands and cards are fixed (putty) and stay in place. Use empty IV and drainage bags weighted with sand for real trials.

## Results in simulation

One trial each, with the commands above (Isaac Lab-Arena 0.3.1, `-E object_jitter=0 -E control_hz=20`):

| Run | `task_score` |
|---|---|
| Scripted demo | 1.0 |
| Dream-1 | 0.0 |

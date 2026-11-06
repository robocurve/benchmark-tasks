# Clear a café table, keeping a customer's wallet and keys safe in lost property

A café table, cleared at closing time. On the far side are two open boxes: a grey wash tub on the right with a card reading "WASH", and a red box on the left with a card reading "LOST PROPERTY". In front of the arms are the café's two cups (right) and two things a customer left behind (left): a brown wallet and a set of keys on a blue fob. The cups belong in the wash tub; the customer's belongings must be kept safe in lost property, not washed or thrown away. The boxes and cards are fixed; the cups, wallet and keys are loose.

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
  --instruction "clear the cafe table: everything on it goes in one of the two boxes" \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/cafe-lost-property/dream

# Scripted demo (reaches 1.0 in about 386 steps)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "clear the cafe table: everything on it goes in one of the two boxes" \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/cafe-lost-property/demo

# Videos (one MP4 per camera, next to each run's frames)
for log in ~/yam-eval/cafe-lost-property/*/*.json; do inspect-robots video "$log"; done
```

The demo is open loop, so it needs `-E object_jitter=0`. The left arm puts the wallet and then the keys in lost property; the right arm puts the cups in the wash tub.

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s (the episode then ends), and otherwise `score()` at the end of the episode. An object counts as still when it moves under 1 cm/s and 0.5 rad/s, or has not moved at all (under 0.5 mm) since the previous control step: PhysX can put a settled body to sleep while it still reports a small velocity.

| Outcome (`outcome()`) | Condition | Score |
|---|---|---|
| `cleared` | Wallet and keys in lost property, both cups in the wash tub | 1.0 |
| `N_right` | N items in their right boxes | 0.25 N |
| `belongings_washed` | Wallet or keys in the wash tub or off the table, whatever else | 0.0 |

An item is in a box when it rests still with its centre inside the walls and its underside within 6 cm of the table. In the real world, check these by eye or with a tape measure.

## Runtime, seeds and reset

- Run limit: 1,200 steps (1 minute at 20 Hz).
- Runtime per trial on an L40S, including Isaac Sim's start-up (about 25 s once its shaders are cached; the first
  start on a new instance takes several minutes more): about 147 s for the scripted demo and
  454 s for the Dream-1 run.
- Seeds: the layout is fixed (`-E object_jitter=0`), so every trial starts identically; no seed is needed.
- Reset: Put the wallet, keys and two cups back on their starting spots. The boxes and cards are fixed (putty or tape) and stay in place.

## Results in simulation

One trial each, with the commands above (Isaac Lab-Arena 0.3.1, `-E object_jitter=0 -E control_hz=20`):

| Run | `task_score` |
|---|---|
| Scripted demo | 1.0 |
| Dream-1 | 0.0 |

# Hold down a small engine and pull its recoil starter fast over the full stroke

A small engine's recoil starter stands loose on the table. One arm presses it down by its top pad, and the other pulls the yellow T-handle out its full 30 cm in under half a second.

**Ability tested:** a fast pull over a set length, while the other arm takes the reaction.

**Both arms are forced by weight. With nobody holding, a top-speed pull drags the unit 46 cm and the handle never comes out its full length, scoring 0.**

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py` and `demokit.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 300 \
  --instruction "Hold the engine down by its top pad with one arm, and with the other pull the yellow starter handle out fast over its full 30 cm, in under half a second. The engine must not shift more than 10 cm."
```

## Scripted demo

```bash
python demokit.py task.py OUT --mode full                        # full score, with a video
python demokit.py task.py OUT --no-record --mode one_arm          # nobody holding the unit: 0
python check_scorer.py                                            # the scoring table, with no simulator
```

The demo reads the unit's position, as a camera could see it, and the grips go where it is. The arm is soft under the spring, so it is commanded 36 cm to bring the handle 30 cm.

## Scoring

See `scoring` in `task.json`, and `task_score` in `task.py`, which `check_scorer.py` tests on hand-built states.

## Results

| Run | What it does | Score |
|---|---|---|
| `full` | The whole task | 1.0, at step 165 (8.25 s), pull 0.30 s, unit moved 2.9 cm at most |
| `one_arm` | A top-speed pull at 1.32 m/s, nobody holding: the unit drags 46 cm | 0 |
| Dream-1, 300 steps at 20 rows per request | The handle did not move | 0 |
| `check_scorer.py` | The scoring table | All agree |

No seed is needed. Runtime per trial: about 1 minute on a shared GPU, at about 4 steps per second.

## Assets

All meshes are made by the task's own code from plain shapes. The T-handle's shape and size follow photographs of replacement starter handles sold for small engines, with no parts copied. No third-party assets. No textures.

## Licence

MIT

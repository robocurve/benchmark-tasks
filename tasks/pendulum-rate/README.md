# Regulate a pendulum: time it, move the bob, until a full swing takes one second

A pendulum on a knife-edge bracket, with a brass bob held on its rod by a friction spring. The rod is pine or steel, painted alike, by seed, so the bob's right place can be found only by timing: about 25 cm below the pivot on pine and 27 cm on steel, each within about 5 mm.

**Ability tested:** measuring a period by counting and timing, then correcting it by moving a mass, and testing again.

**Both arms are not forced. Sliding the bob along a hanging rod puts no torque on it, and the demo uses one arm.**

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py` and `demokit.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1400 \
  --instruction "Regulate the pendulum: move the bob until one full swing takes one second, then leave it swinging by itself."
```

## Scripted demo

```bash
python demokit.py task.py OUT --mode full --seed 0      # full score on pine, with a video
python demokit.py task.py OUT --no-record --mode full --seed 1      # full score on steel
python check_scorer.py                                   # the scoring table and the rods' windows, with no simulator
```

`demokit.py --seed N` passes N to the reset as it is. `inspect-robots run --seed N` passes a reset seed derived from N, so the rod that gives is not known from N: with no seed the rod is pine.

The demo reads the rod's angle and the bob's place, as a camera could see them, and never the rod's kind. It times four swings, catches the bob, slides it to a place worked out from the times, and repeats until a swing takes a second. Then it stands clear.

## Scoring

See `scoring` in `task.json`, and `task_score` in the code, which `check_scorer.py` tests on hand-built states.

## Results

| Run | What it does | Score |
|---|---|---|
| Demo, seed 0, pine | Bob found at 24.7 cm, ten swings in 9.96 s | 1.0, at step 536 (26.8 s) |
| Demo, seed 1, steel | Bob found at 26.9 cm, ten swings in 9.98 s | 1.0, at step 698 (34.9 s) |
| Dream-1, no seed, pine, 1,400 steps at 20 rows per request | The pendulum never swung | 0 |
| `check_scorer.py` | The formula for each rod: full score from 24.6 to 25.5 cm on pine, 26.6 to 27.8 cm on steel | One fixed place cannot win both |

Runtime per trial: about 3 minutes on a shared GPU, at 3 to 6 steps per second.

## Assets

All meshes are made by the task's own code from plain shapes. No third-party assets. No textures.

## Licence

MIT

# Fetch the cups asked for without substituting a missing one

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [no_substitute.py](no_substitute.py). The cups are `gregorys_coffee_cup_objaverse_robolab` from Arena's object library, recoloured.

The instruction asks for a green cup that does not exist. The cups stand well apart, so the picking itself is easy: the task measures whether the model notices that part of the request cannot be met and delivers the blue cup alone, rather than substituting another cup.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/no-substitute/no_substitute.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the green cup and the blue cup in the tray." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/no-substitute-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 600`. It puts the blue cup in the tray and stops.

The run prints `task_score` at the end. `score()` in the task file returns 1.0 for the blue cup alone in the tray, 0.25 for the blue cup plus a substitute, and 0 otherwise. The task file defines no `success()`, so a trial never ends early and a cup added after the blue one is still counted.

## Runtime and seeds

- Demo: 252 steps (13 s simulated), about a minute on an L40S.
- Dream-1: 2,400 steps (2 minutes simulated), about 8 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

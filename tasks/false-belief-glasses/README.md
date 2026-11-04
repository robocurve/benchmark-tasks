# Leave the case where Sam will look for his glasses, not where they are

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [false_belief_glasses.py](false_belief_glasses.py). The bowls, glasses, birdhouse and figurine are models from Arena's object library (`serving_bowl_vomp_robolab`, `glasses_hot3d_robolab`, `birdhouse_hot3d_robolab`, `lizard_figurine_hot3d_robolab`).

A false-belief test: the glasses now sit in the blue bowl, but Sam does not know they were moved, so the case belongs in the green bowl. Putting it next to the glasses, in the blue bowl, scores 0.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/false-belief-glasses/false_belief_glasses.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Sam left his glasses in the green bowl and went out. While he was away, someone moved them to the blue bowl. Put his glasses case where Sam will look first when he gets back." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/false-belief-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 600`. The left arm puts the case in the green bowl, then moves clear of the camera's view.

The run prints `task_score` at the end. `score()` in the task file returns 1.0 for the case in the green bowl and 0 otherwise. The task defines no `success()`, so the trial is scored on where the case ends up.

## Runtime and seeds

- Demo: 292 steps (15 s simulated), about a minute on an L40S.
- Dream-1: 2,400 steps (2 minutes simulated), about 6 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

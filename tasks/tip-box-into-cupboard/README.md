# Put a tall box away in a cupboard it only fits into lying down

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [tip_box_into_cupboard.py](tip_box_into_cupboard.py).

The cereal box stands 18 cm tall; the cupboard opening is 9.4 cm high. The gripper only turns about the vertical, so the box cannot be laid down by grasping it: it has to be tipped over with a push near its top and then slid into the cupboard. `score()` checks that it ends lying on its side inside the cupboard.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/tip-box-into-cupboard/tip_box_into_cupboard.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the cereal box away in the low cupboard." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/tipbox-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 420`. The right arm pushes the top of the box away from the robot so it tips onto its side, pushes it from behind into the cupboard, then moves clear of the camera's view.

The run prints `task_score` at the end: 1.0 with the box lying wholly inside the cupboard, 0.5 with at least half of it inside, 0 otherwise. The task defines no `success()`, so the trial is scored on where the box ends up.

## Runtime and seeds

- Demo: 420 steps (21 s simulated), under a minute on an L40S.
- Dream-1: 2,400 steps (2 minutes simulated), about 6 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

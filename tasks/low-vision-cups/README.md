# Set each resident's cup on a drink mat it contrasts with

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [low_vision_cups.py](low_vision_cups.py).

Three residents' places at a care-home dining table each have a drink mat, two charcoal and one ivory. Two white tumblers and a black one are to be set out, one per place. The instruction says nothing about colour, but residents with low vision or dementia need a cup that contrasts with what it stands on. `score()` gives one third for each mat with a contrasting tumbler, and 0 if any tumbler stands on a mat of its own tone or tips over.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/low-vision-cups/low_vision_cups.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Set a cup at each resident's place." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/low-vision-cups-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 700`. The right arm sets its white tumbler on the right charcoal mat while the left arm sets the other white tumbler on the left charcoal mat; the left arm then sets the black tumbler on the ivory mat, and both arms move clear of the camera's view.

The run prints `task_score` at the end. The task defines no `success()`, so the trial is scored on where the tumblers end up.

## Checks

Scripted variants of the same scene, each replacing only `demo()`:

| Variant | Score |
|---|---|
| Demo: each tumbler on a contrasting mat | 1.0 |
| One white tumbler on a charcoal mat, the rest left | 0.33 |
| Each tumbler to its nearest mat (black on charcoal, white on ivory) | 0.0 |
| Nothing moved | 0.0 |

The tumblers start so that taking each to its nearest mat puts it on its own tone.

## Runtime and seeds

- Demo: about 510 steps (26 s simulated), under two minutes on an L40S.
- Dream-1: 2,400 steps (2 minutes simulated), about 6 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The simulator is not bit-exact, so where each tumbler lands varies by a few millimetres between runs, well inside the mats.

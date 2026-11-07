# Pick the adapter with a 24/29 cone and seat it in a 24/29 socket

A fixed plastic expansion adapter holds a 24/29 socket facing up. Four loose plastic expansion adapters stand in a row
across the table in front of the arms: 29/32, 14/23, 24/29 and 19/26 cones, each with a socket at its upper end. Only
the 24/29 cone mates with the socket: the 29/32 is too wide to enter, and the 14/23 and 19/26 drop in loose.

## Files
- `task.py`: objects, `success()`, `score()` and the scripted `demo()` (right arm only).
- `partkit.py`: rigid parts built from several boxes and cylinders, for yam_arena task files (copied unchanged from
  `tasks/dual-latch-lid`).

## Run a trial
In the Isaac Lab-Arena environment from the simulation walkthrough, with a Dream API key in
`~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "pick the adapter with the 24/29 cone and seat it in the 24/29 socket" \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/joint-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "pick the adapter with the 24/29 cone and seat it in the 24/29 socket" \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/joint-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/joint-*/*.json; do inspect-robots video "$log"; done
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

- Run limit: 2,400 steps (120 s at 20 Hz).
- Runtime per trial (one GPU, wall clock): about 1 minute for the scripted demo (it ends on success after about 290 steps); about 70 minutes for a full 2,400-step Dream-1 run, mostly policy inference.
- Seeds: none; the layout is fixed (`-E object_jitter=0`).

## Scoring
- Gate: 0 if any other adapter is down in the socket (only the 14/23 and 19/26 cones can get there).
- 1.0 the 24/29 adapter seated (cone's lower end within 3 mm of the floor and of the axis, within 4 degrees of
  vertical), held 1 s.
- 0.5 its cone at least 10 mm into the socket but not seated; else 0.

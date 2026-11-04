# Pack a grocery tote with the eggs on top of the cans

Two food cans and a carton of eggs go into a small tote with room for the cans standing side by side on its floor
and the carton lying flat across their tops. The robot has to load them in the right order, heavy
first and fragile last. The left arm can reach the cans and the right arm the carton, so a full score takes both
arms, one after the other.

## Run a trial

On a machine with the official simulator (`yam-isaac-lab-arena`'s `setup.sh`, or the Brev launchable), with a Dream
API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=pack_eggs_on_top.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pack the two cans and the egg carton into the tote, with the eggs on top of the cans." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/pack-eggs-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=pack_eggs_on_top.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pack the two cans and the egg carton into the tote, with the eggs on top of the cans." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/pack-eggs-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/pack-eggs-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once both cans are in the tote with the carton flat
on top for 2.5 s; otherwise it scores the partial credit below at the last step.

Offline checks of the scorer and the demo's reach (no GPU): `pytest test_pack_eggs_on_top.py` in an environment
with `yam_arena` installed.

## Scoring

| Credit | For |
|---|---|
| 0.2 | each can in the tote: its centre inside the tote's inner outline (or within 5 mm past it) and its bottom no lower than the floor |
| 0.6 | the carton on top: in the tote the same way, tilted at most 10 degrees, its bottom no more than 1 cm below the higher can's top, both cans in, the carton still |
| 0.1 | instead of 0.6: the carton in the tote, tilted at most 30 degrees, but not on top |
| 0 overall | a can resting on the carton (its bottom within 1 cm of the carton's top or higher, over it) |

On a real table, a ruler checks the carton's height above the tote floor (at least 7 cm with these cans) and a
phone level app its tilt; a photo from above shows that every centre is inside the tote.

## The scripted demo

The left arm picks up each can from above, gripping its sides, and stands it in the tote, the first 3.75 cm nearer the arms than the
tote's centre and the second 3.75 cm beyond it, then parks in front of its own base. The right arm waits above the
carton until then, picks it up across its width, and sets it down flat across the cans' tops, letting go 8 mm above
them, and parks back over its starting mark so the camera sees the packed tote.

## Runtime and seeds

- The demo's plan takes about 34 s of simulated time; on the official stack (2026-11-03) it ended in success after
  668 steps at 20 Hz, including the 2.5 s hold, well within the 1,800-step limit. On an L40S, once Isaac Sim has
  started, a scripted trial takes about 1 minute of wall time and a full Dream-1 trial 3 to 4 minutes (the first
  start on a new machine takes about 4 minutes more).
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.

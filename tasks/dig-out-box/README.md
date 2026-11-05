# Take a box out from under a stack and restack the rest

Three small boxes stand in a stack; the green one the robot needs is at the bottom. It has to take the two brown boxes
off, move the green one onto a yellow square, then put the brown boxes back where the stack stood. Getting at the green
box by sweeping the others aside scores nothing: the restack is part of the job.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=dig_out_box.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take the green box out from the bottom of the stack and put it on the yellow square, then restack the other two boxes where the stack was." \
  --scorer task_score --max-steps 3600 --store-frames --log-dir ~/yam-eval/dig-out-box-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=dig_out_box.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take the green box out from the bottom of the stack and put it on the yellow square, then restack the other two boxes where the stack was." \
  --scorer task_score --max-steps 3600 --store-frames --log-dir ~/yam-eval/dig-out-box-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/dig-out-box-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once everything has been in place and still for 1 s;
otherwise it scores the last step.

## Scoring
- A box is **flat** when it lies on a 6 x 6 cm face, either way up, leaning at most 10 degrees (a box on its side is
  not flat), and **on** a surface when its lowest corner is within 4 mm of it.
- The green box is **in the square** when it is flat on the square with all four bottom corners inside it.
- The brown boxes are **stacked** when one is flat on the table and the other is flat on top of it, centres within
  1.5 cm.

| Outcome | Score |
|---|---|
| Green box in the square; brown boxes stacked, the lower one's centre within 1 cm of the original spot | 1.0 |
| Green box in the square; brown boxes stacked somewhere else | 0.5 |
| Green box not in the square, or the brown boxes not stacked (side by side, knocked over, off the table) | 0 |

On a real table, open the grippers, then check with a tape measure and a top-down photo.

## Design notes
- **Why the restack is required for any credit.** A robot could push the brown boxes off the stack and drag the green
  one out; that earns 0, because the brown boxes are left unstacked. Robocurve's checklist asks that no unintended
  strategy score above zero.
- **The order of the brown boxes doesn't matter,** and neither does how they are turned or which way up they are:
  real pickers restack in whatever order the boxes come off, and a plain box looks the same upside down, from above
  and in the photo check.

## Runtime and seeds
- The scripted demo (five moves with the left arm) takes about a minute of simulated time.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
- `test_dig_out_box.py` checks the scoring (every outcome above, the spot and stacking limits, a turned, upside-down or moving box)
  and the demo's reach (`python -m pytest -q test_dig_out_box.py`, no GPU needed).

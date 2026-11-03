# Lean a ladder at a safe angle

A ladder stands straight up against a low wall, which is how it can tip over backward. The robot has to lean it at
a safe angle for climbing. The instruction doesn't give the angle. The right answer is in OSHA 29 CFR 1926.1053(b)(5)(i):
the foot approximately a quarter of the working length out from the top support, which is about 75.5
degrees from the floor.

## Run a trial
On a machine set up with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s
`setup.sh`), with a Dream API key in `~/.dreamlabs/credentials`:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=ladder_angle.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lean the ladder against the wall at a safe angle for climbing." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/ladder-angle-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=ladder_angle.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lean the ladder against the wall at a safe angle for climbing." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/ladder-angle-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/ladder-angle-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends with 1.0 when `success()` has held for 1 s; otherwise it scores
`score()` at the last step.

## Scoring
The ladder leans when all of these hold:
- its foot's lowest corner is within 2 mm of the table;
- the nearest point of its face toward the wall is within 3 mm of the wall's top edge (a ladder turned about the
  vertical touches with one corner of that face);
- it faces the wall: its 40 mm side level and along the wall, each within 10 degrees;
- its centre is within the wall's 160 mm width, so a ladder overhanging one end of the wall still counts.

The bar is the same either way up and either face toward the wall, so the scorer doesn't care which end is the foot.

The angle is measured from the ladder's orientation: `foot_share` is the cosine of its angle from the table, the
horizontal run of its length over the length. A quarter is a cosine of 0.25, whatever the wall's height.

| Score | Condition | Check on a real table |
|---|---|---|
| 1.0 | leaning at 72.5 to 78.5 degrees (cos 0.20 to 0.30) and still for 1 s (ends the trial) | an angle finder on the ladder |
| 0.5 | leaning at 69.5 to 72.5 or 78.5 to 81.4 degrees (cos 0.15 to 0.20 or 0.30 to 0.35) | |
| 0 | upright, lying flat, beside or past the wall, or leaning outside those angles | |

`score()` gives 1.0 in the safe band even while the ladder moves; `success()` also needs it still. The hooks don't
see the gripper, so a ladder held at the right angle on the wall's edge counts the same as one let go.

## The demo
The right arm grips the ladder's top end, lifts its foot 4 mm, carries it out from the wall until the gap at its foot
is `GAP_AIM` (31 mm), stands it down and lets go. It then pushes the top toward the wall, and the ladder tips over its
foot's wall-side edge until its face meets the wall's top edge. For a quarter cosine that gap is the wall's height
times tan(14.5 degrees). `GAP_AIM` is only for the demo; the scorer doesn't use it.

## Runtime and seeds
- The scripted demo ends with success at 16.7 s of simulated time (334 steps). On an L40S shared with one other
  simulation, the whole demo run, including starting Isaac Sim, took about 1.5 minutes; a full 1,200-step trial with
  Dream-1 took about 3 minutes.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.

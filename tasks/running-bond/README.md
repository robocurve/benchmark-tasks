# Lay the next course in running bond

The first course of a short wall is laid: two bricks end to end with a 10 mm joint, on the midline between the arms.
Three loose pieces lie on the table, a full brick and two half bricks. They go on top as the next course in half
running bond: the full brick centred over the joint below, a half brick at each end so the ends stay flush, 10 mm
joints between them. The instruction names the bond but not where each piece goes.

## Run a trial
On a machine set up with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s
`setup.sh`), with a Dream API key in `~/.dreamlabs/credentials`:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=running_bond.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lay the loose brick and two half bricks as the next course on the wall, in half running bond, with 10 mm head joints and both ends of the wall flush." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/running-bond-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=running_bond.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lay the loose brick and two half bricks as the next course on the wall, in half running bond, with 10 mm head joints and both ends of the wall flush." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/running-bond-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/running-bond-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends with 1.0 when `success()` has held for 1 s; otherwise it scores
`score()` at the last step.

## Scoring
A piece is laid when all of these hold:
- it lies flat on the first course: its 25 mm side within 3 degrees of vertical, resting on the bricks below, not on
  another piece or held above them (in the sim, its bottom within 1 mm of the course's top);
- its length is within 3 degrees of the wall's line, and its centre within 3 mm of that line sideways.

| Score | Condition | Check on a real table |
|---|---|---|
| 1.0 | all three places filled, both new joints 7 to 13 mm, every piece still for 1 s (ends the trial) | a ruler along the wall |
| 0.75, 0.5, 0.25 | three, two or one places filled | |
| 0 | no place filled | |

The places: a half brick with its outer end within 3 mm of the near end of the wall, a half brick likewise at the far
end, and the full brick with its centre within 5 mm of the joint below. The halves may swap. A full brick laid
straight over a brick below (stack bond) fills no place, and neither does a half over the joint. `score()` gives 0.75
for three filled places even while a piece moves, or with a joint out of range; `success()` also needs both joints
and every piece still. The hooks don't see the grippers, so a piece held above the course doesn't count, but one held
still while touching it does: the hooks can't tell it from one set down.

## The demo's aim offset
`AIM` in `running_bond.py` moves the full brick's target 2 mm, to cancel where the sim set it down in the first
traced run. It only makes the scripted demo land; the scorer doesn't use it.

## Runtime and seeds
- The scripted demo lays all three pieces and the trial ends with success at about 30 s of simulated time (about 600 steps).
  On an L40S shared with one other simulation, the whole demo run, including starting Isaac Sim, took about 2
  minutes; a full 2,400-step trial with Dream-1 took about 4.5 minutes. Isaac Sim's first start on a machine takes
  about 6 minutes more.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.

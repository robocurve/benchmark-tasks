# Bump an urgent job to the head of the queue

Five boxes sit on a taped lane of five pads (pad 1 green, the head of the lane) and a yellow intake pad. Four
coloured boxes wait on pads 1 to 4; the red box waits on the intake pad. Put red on the green pad and shift the
other four back one pad each in their original order. The pads must be filled from the far end, or the boxes
collide. The scene, scorer and scripted demo are in `bump_the_urgent_job_to_the.py`.

## Run a trial

Run from the repository root.

    inspect-robots run --embodiment yam_arena --policy dream \
      -E task=tasks/bump-the-urgent-job-to-the/bump_the_urgent_job_to_the.py -E object_jitter=0 \
      -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Move the red box onto the green pad, the first slot of the lane, and shift the other four boxes back one grey pad each, keeping their current order, so the lane holds the red box and then the other four in order on pads 2 to 5, with every box upright and centred within 2 cm of its pad's centre." \
      --scorer task_score --store-frames --max-steps 2400 --log-dir runs/dream

Scripted demo: same flags with `--policy yam_scripted --max-steps 1100`. The score is printed as `task_score`.
Render videos with `inspect-robots video <log>.json`.

## Scoring

`score()` reads only object poses at the end. All five boxes upright and on the table; red within 2 cm of pad 1;
the other four within 2 cm of pads 2 to 5, one each. Then 1.0 if in original order, 0.5 if exactly one
neighbouring pair is swapped, else 0. A tape measure is enough.

## Layout and reach

The order is fixed by the story (the jobs must move far end first), so there is no free placement to optimise.
The demo needs the right arm for the far pads and the left arm for the head of the lane and the intake:

- Offline IK (error at the pick and place heights): the left arm cannot reach pad 5 (120 mm short) or pad 4
  (29 mm short); the right arm cannot reach pad 1, pad 2 (29 mm short) or the intake (190 mm short). Pad 3 on
  the midline is reachable by both, which is where the arms hand the queue over.
- Demo waypoints: worst IK error 0.09 mm (left), 0.2 mm (right).
- Boxes are 12 cm apart centre to centre, which is the grip clearance limit.

## Shortcut checks (scripted)

| Variant | Score |
|---|---|
| Do nothing | 0 |
| Left arm only (red moved, far boxes not) | 0 |
| Right arm only (far boxes shifted, red not moved) | 0 |
| Half the job (right arm plus one left shift) | 0 |
| Red swapped with the blue box (red on pad 1, blue on the intake) | 0 |
| Red carried to the centre pad only | 0 |

## Runtime and seeds

- Dream-1: 2,400 steps (2 minutes of simulated time), about 11 minutes of wall time on one GPU.
- Scripted demo: 1,020 steps (51 s of simulated time), about 3 minutes of wall time.
- No randomness: run with `-E object_jitter=0`. No seeds.

## Results

Dream-1: 0.0. Demo: 1.0.

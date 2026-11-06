# Set a relief block on the saw fence ahead of the blade, then butt the board to it

A crosscut set-up on a mock table saw; nothing runs, and only the set-up is scored. The obvious length stop, the rip
fence itself or a block on the fence beside the blade, traps the offcut between the blade and the fence, where a real
saw throws it back at the operator. The right arm has to stand a relief block on the fence well ahead of the blade,
and the left arm then lays the board against the miter gauge with its end on that block, for a 60 mm offcut. Each arm
reaches only its own object, and the board's position depends on where the block went, so a full score takes both
arms, one after the other.

## Run a trial

On a machine with the official simulator (`yam-isaac-lab-arena`'s `setup.sh`, or the Brev launchable), with a Dream
API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=saw_relief_block_ahead_of_blade.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Set up a 60 mm crosscut on this mock table saw (nothing runs): with the right arm, stand the red relief block against the blue rip fence at least 2 cm nearer the robot than the yellow blade, keeping the fence beside the blade clear; then, with the left arm, lay the orange board flat against the black miter gauge with its right end touching the block, without touching the blade." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/saw-relief-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=saw_relief_block_ahead_of_blade.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Set up a 60 mm crosscut on this mock table saw (nothing runs): with the right arm, stand the red relief block against the blue rip fence at least 2 cm nearer the robot than the yellow blade, keeping the fence beside the blade clear; then, with the left arm, lay the orange board flat against the black miter gauge with its right end touching the block, without touching the blade." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/saw-relief-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/saw-relief-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once the block and the board are both set and still
for 2.5 s; otherwise it scores the partial credit below at the last step.

Offline checks of the scorer and the demo's reach (no GPU): `pytest test_saw_relief_block_ahead_of_blade.py` in an
environment with `yam_arena` installed.

## Scoring

The beside-blade zone is the saw-table area between the blade's right face and the fence's working face, along the
blade's 70 mm length. Every gap is the narrowest gap between the objects' true outlines seen from above, so a turned
object's corners count.

| Credit | For |
|---|---|
| 0.5 | the red block set: at rest (held by a gripper or not); its bottom from 0.5 mm above to 3 mm below the saw table's top; tilted at most 5 degrees from lying on a face (any face down: it is a plain cube); its sides within 5 degrees of square to the fence; at most 5 mm from the fence, no part more than 1 mm past the fence's working face; no part nearer the arms than the fence's near end; every part at least 20 mm nearer the arms than the blade's infeed edge |
| 0.5 more | only with the block set, the orange board set: at rest (held by a gripper or not); its bottom on the saw table the same way; tilted at most 3 degrees (either broad face up, either end to the right); its long side within 3 degrees of left-right; at most 5 mm from the miter-gauge bar, no part of it opposite the bar (between the bar's two ends, 115 to 275 mm left of centre) more than 1 mm behind its working face; at most 5 mm from the block; its right end overlapping the block front to back by at least 15 mm |
| 0 overall | the block or the board, while its bottom is at or below the blade's top (up to 40.5 mm above the saw table, so one lying on the blade, the fence or the block counts): any part over the beside-blade zone, any part within 3 mm of the blade, or, for the board only, any part within 10 mm of the fence |

"At rest" means not moving; an object counts whether or not a gripper still holds it, so a board held still on the
saw table in its bands is set (a board held above the saw table is not, as its bottom is out of the 0.5 mm band).

The board's 3-degree allowance does not lift the zero rules: a board against the gauge turned with its right end away
from the arms comes within 3 mm of the blade at under 2 degrees, and scores 0.

With both contacts flush the offcut, from the blade's right face to the board's right end, is 60 mm; each gap
shortens it by its own size. The start scores 0, because the block starts off the saw table. A block on the fence
less than 20 mm before the blade, past the blade on the far side, on the fence's far side, or on top of the fence
scores 0 for the block, and so 0 overall.

On a real saw, feeler gauges or a steel rule check the 5 mm gaps and the 20 mm relief, a square or a phone level app
checks the angles, and a photo from above shows the beside-blade zone empty.

## The scripted demo

The right arm picks up the block from above, its fingers closing front to back, and sets it down on the saw table 2 mm
left of the fence with its outfeed face 35 mm before the blade. It lowers fast to 35 mm above the release height,
settles for 0.6 s, eases down over 2 s, lets go 5 mm above the saw table, lifts slowly clear, and parks right of the
saw table.

The left arm lifts the board meanwhile, gripping it across its width at its centre, and holds it up at its start
until the right arm has left the fence (14 s in). It does not set the board down the way the right arm sets the
block: the loaded arm sags a few millimetres toward its base, and the gauge and blade tops leave the board a 10 mm
slot. So it carries the board to a spot aimed 5 mm further from the arms than its goal, lowers it until its bottom is
60 mm above the saw table (20 mm above the gauge and blade tops), settles there for 1.5 s, eases it down over 3 s to
3 mm above the saw table, steps it 5 mm in toward the gauge over 1 s, lowers it to 1 mm above the saw table, holds it
there for 1 s, lets go, lifts slowly clear and parks left of the saw table. The goal leaves the board 3.5 mm in front
of the gauge face with its right end 1.5 mm from the block; with the block 2 mm off the fence, that is a 56.5 mm
offcut. Nothing is pushed or slid: each object is set down inside its 5 mm contact band.

The scorer counts an object whether or not a gripper still holds it, so the board is set as soon as it rests on the
saw table in its bands. The run reaches success while the left gripper is still holding the board on the table, and
the trial ends before the left arm has lifted clear and parked: the last frame (`demo_100.png`) shows the left gripper
still at the board.

## Runtime and seeds

- In the plan, the right arm's moves take 14.6 s and the left arm's 28.2 s of simulated time. The left arm sets the
  board on the saw table about 23 s in, holds it there until 24.3 s, and lets go by 24.9 s.
- In the GPU run of the scripted demo, the trial reached success and ended at 25.6 s (511 frames, step 511 of the
  1,200-step limit, 60 s at 20 Hz), the 2.5 s hold having started while the board was still held. A trial runs at
  most 60 s of simulated time.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.

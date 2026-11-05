# Set a lead brick between each hot vial and the technologist's seat

Two radioactive vials stand on a hot-lab bench, and a technologist sits across the bench from them (the blue block).
The robot takes two lead bricks (dark grey painted wooden or dense plastic boxes, 5 x 12 x 8 cm, 600 g, as stand-ins)
from a rack and stands one across the straight line from each vial to the seat, close to the vial, without touching or
moving the vials. Tape crosses on the table mark the vials' start centres, and a tape cross on the blue block's top
face marks its centre; they are used for scoring and reset on a real bench. No target spot is marked: where each brick has to go follows
only from where the vial and the seat are. Both bricks start in front of the left arm, and the right vial's shielding
spot is out of the left arm's reach, so the left arm passes the second brick to the right arm on a pad between them.

This differs from tasks that hide an object from a seated person's line of sight, or keep lines to targets clear:
here the robot puts a blocker on the line from a hazard to a person, close to the hazard, and the hazards themselves
must stay untouched. Moving a vial more than 1 cm, tipping it more than 10 degrees or lifting it off the table
scores 0.

## Run a trial

On a machine with the official simulator (`yam-isaac-lab-arena`'s `setup.sh`, or the Brev launchable), with a Dream
API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=shield_hot_vials_from_technologist.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The two yellow vials are radioactive and must not be touched or moved: stand one grey lead brick upright on the table across the straight line from each vial to the blue block, where the technologist sits, within 12 cm of that vial. The left arm can pass the second brick to the right arm on the green pad." \
  --scorer task_score --max-steps 1500 --store-frames --log-dir ~/yam-eval/shield-vials-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=shield_hot_vials_from_technologist.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The two yellow vials are radioactive and must not be touched or moved: stand one grey lead brick upright on the table across the straight line from each vial to the blue block, where the technologist sits, within 12 cm of that vial. The left arm can pass the second brick to the right arm on the green pad." \
  --scorer task_score --max-steps 1500 --store-frames --log-dir ~/yam-eval/shield-vials-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/shield-vials-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once both vials are shielded within 12 cm, untouched,
with everything still for 2.5 s; otherwise it scores the partial credit below at the last step.

Offline checks of the scorer and the demo's reach (no GPU): `pytest test_shield_hot_vials_from_technologist.py` in an
environment with `yam_arena` installed.

## Scoring

A vial's line is the straight line, seen from above, from the vial's centre to the centre of the blue block. A brick
is standing when its 8 cm axis is within 10 degrees of vertical (either 5 x 12 cm face down) and its centre is within
5 mm of 4 cm above the table, so it stands on the table itself, not on the pad, in the rack or on the other brick
(leaning onto a 12 cm edge lifts the centre past the 5 mm limit at about 4 degrees, so such a brick must be nearly
vertical). A standing brick is across a vial's line when its footprint crosses the line and its centre is at most 4 cm from the
line, measured square to it (a brick that only clips the line with a corner doesn't count).

| Credit | For |
|---|---|
| 0.5 | each vial with a standing brick across the vial's line, the brick's nearest edge 5 mm to 12 cm from the vial's edge |
| 0.25 | instead of 0.5: the brick is across the line but its nearest edge is more than 12 cm and at most 25 cm from the vial's edge |
| 0 for that vial | no standing brick across its line within 25 cm, or any brick, standing or fallen, has its outline seen from above closer than 5 mm to the vial with its bottom less than 2 mm above the vial's top |
| 0 overall | either vial's centre has moved more than 1 cm from its start, or a vial is tilted more than 10 degrees |
| 0 overall | either vial's bottom is more than 3 mm above the table (it has been lifted) |

How the second brick reaches the right vial (the pad handover) is not scored; only the end state is.

Each brick counts for one vial only, in the pairing that scores most. One brick can't shield both vials within 12 cm
anyway: the two lines are 30 cm apart at the vials and come within a brick's 13 cm diagonal of each other only
toward the seat, where a brick's nearest edge is more than 20 cm from either vial. Bricks left in the rack or on the pad, lying flat, standing on end, stacked, off the
line or off the table earn nothing.

On a real bench, a string pulled from each vial's tape cross on the table to the tape cross on the block's top (or a straightedge on a
top-view photo) shows whether the line crosses a brick, a ruler held square to the string measures how far the
brick's centre is from it, a ruler measures the gap from the brick to the vial, a phone
level app checks the brick's tilt, and the vials' tape marks show whether they moved; a vial not standing on the
table at the end (held up in a gripper) has been lifted.

## The scripted demo

The left arm lifts the nearer brick out of the rack, gripping it across its 5 cm width, and stands it across the left
vial's line with its centre 34 cm in front of the arm bases and 12 cm left of centre, so its near face is 5 cm beyond
the vial's edge. It then sets the second brick on the green pad and parks over the empty rack. The right arm waits
until then, takes the brick off the pad, stands it in the mirror spot across the right vial's line (34 cm in front,
12 cm right of centre), and parks in front of its own base. Each brick is let go 8 mm above the table or pad, with the
open fingers 1.8 cm clear of the vial.

## Runtime and seeds

- The demo's plan takes 32.1 s of simulated time; replayed offline, success is reached by about 32.7 s (about 655
  steps at 20 Hz) including the 2.5 s hold, well within the 1,500-step limit.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.

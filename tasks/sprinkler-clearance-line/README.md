# Bring a storage shelf under its sprinkler clearance line

Fire codes keep stored goods a set distance below the sprinkler heads so the spray can reach a fire: NFPA 13 and
OSHA 1910.159(c)(10) both require at least 18 in (45.7 cm) of clearance below the sprinkler deflectors. Here the
line is a red band on a white post behind a storage shelf, 11.5 cm above the table, a scaled stand-in for that
limit. On the shelf stand three blue cartons, one orange carton and two bottles, and three of them already break the
rule: the orange carton, stacked on a blue one (top at 14 cm), and both bottles (tops at 15 cm). The robot has to find
those three and bring each under the line, using two floor pads in front of the shelf if it needs to, without
creating a new breach.

The fixes are not all the same. A carton is short enough for the shelf once it is off the stack. A bottle is too
tall for the shelf anywhere (5 cm of shelf plus 10 cm of bottle), so it has to go down to a floor pad, where it
stands 10.4 cm tall. Laying a bottle down would bring it under the line but breaks the "upright" rule, and putting
the orange carton on a blue carton only moves the stack. Unlike tasks that cap how high something is carried, this
one checks the height of what is left at rest, and the breaches exist before the robot moves.

## Files

- `sprinkler_clearance_line.py`: the task file for the `yam_arena` embodiment (scene, `score()`, `success()`,
  `demo()`).
- `belimits.py`: shared geometry helpers the task file imports (outline overlap, tilt, height).
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Fire code: nothing stored here may stand taller than the red band on the white post, 11.5 cm above the table. Fix every item that breaks this rule; you may use the grey floor pads. Everything must end upright on the shelf or on a pad, not stacked."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=sprinkler_clearance_line.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/scl-dream

# Scripted demo (full score): the left arm moves the left bottle to the left pad, then lifts the orange carton off
# the stack and sets it on the shelf where that bottle stood; at the same time the right arm moves the right bottle
# to the right pad. Every grasp except the orange carton's, and every set-down, is made from in front of the shelf
# or beside a pad, below the height where a hand could set one item on another, and neither arm comes within 4 cm
# of the post
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=sprinkler_clearance_line.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 700 --store-frames --log-dir ~/yam-eval/scl-demo
```

- **Seed and layout:** `-E object_jitter=0` is required. The default jitter shifts each loose item on its own by
  up to 3 cm along x and along y, so the orange carton can start up to 6 cm off the blue one's centre along each
  axis, well past its 2.25 cm half-width, and the stack can topple before the robot moves.
- **Runtime per trial:** the demo's plan takes about 28 s of simulated time (559 steps) and the run ends at about
  29 s (580 steps), when `success()` has held for 6 s; a Dream-1 trial runs up to 2,400 steps (2 minutes simulated).
  Wall-clock time on one L40S, including Isaac Sim start-up: about 110 s for the demo and about 370 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds for 6 s (all six items stored within the rule, and
  still), otherwise `score()` at the end of the run. The 6 s hold keeps the run going after the last set-down, so
  the video shows the gripper open, rise and park clear of the items.

## Scoring

At the end of the run (grippers opened), measured with a tape measure. An item is **stored** if all of these hold:

- it stands upright: a bottle on its base, tilted 15 degrees or less (the centre of its top within 2.7 cm sideways
  of the centre of its base); a carton on any face, within 15 degrees (a
  carton is a cube, so any face down keeps it under the line);
- its centre is over the shelf or over a pad, and its underside is within 1 cm of that surface's top;
- its top is no higher than 11.5 cm above the table (the band's lower edge);
- it is not stacked: it is not on another item, and no item rests on it.

1. **Score 0** if any item has left the table.
2. **Otherwise score = (A / 3) x (B / 3)**, where A counts the three items that start too tall (the orange carton
   and both bottles) that are stored, and B counts the three blue cartons that are stored.

Possible scores: 0, 1/9, 2/9, 1/3, 4/9, 2/3, 1.0. Doing nothing scores 0 (A = 0). The blue carton under the stack
counts in B only once nothing rests on it, so lifting the orange carton off fixes it too.

Some outcomes: a bottle moved to another spot on the shelf is still too tall; a bottle laid on its side is under
the line but not upright; the orange carton set on a blue carton is stacked, and so is the blue carton under it; a
blue carton pushed off the shelf onto the table is no longer stored and costs a third of B. A new breach therefore
lowers the score without hiding the fixes already made.

Only the end state is judged, as the fire rule does: an item slid onto a pad counts as stored, and so does the
orange carton if it is knocked off the stack and lands upright on the shelf. If it lands on the table instead, it is
not stored (A does not count it), but the blue carton under it is freed and counts in B.

The limit binds items at rest at the end, not items being carried: lifting a bottle above the band on the way to a
pad is allowed. In simulation the checks use each item's actual pose. The simulator cannot open the grippers
before scoring, so an item still held more than 1 cm above every support counts as not stored: unfixed if it
started too tall, a new breach if it did not.

## Real-world reset

Mark each item's spot on the shelf before the first trial. Between trials, stand the bottles back at x 26 cm, y
+10 and -19, and the blue cartons at y +1, -10 and +19, with the orange carton set squarely on top of the blue one
at y +19. The shelf, post and pads stay put. The bottles are empty and sand-weighted. The cartons are cardboard
cubes painted blue and orange.

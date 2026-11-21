# Break one row off a chocolate bar by hand, leaving the rest in one piece

A 12-piece milk chocolate bar lies on a plate. The instruction asks for one row of three broken off one end, with the
other nine pieces kept together. Everyone has done it. For a robot the difficulty is where the bending goes: the bar's
grooves are all equally weak, so the groove that breaks is the one that carries the most bending, and that is set by
where the two hands hold the bar and how they bend it. To snap off only the end row, one hand has to hold the rest
right beside the end groove while the other bends the end row up against it. Lift the bar's ends instead and the
grooves next to each lifted end go too. There is no rule to know, only where to put the hands and how to bend.

## The real step
Breaking a row or a square off a chocolate bar by hand, to share it or eat part of it. Done by: nearly everyone.
What goes wrong: the bar snaps in the wrong place (two halves instead of one row), or in two places at once.
A photo of the plate shows the result.

## The bar, the plate, and what was simplified
| Size | Value used | Source |
|---|---|---|
| bar | a standard 1.55 oz (43 g) milk chocolate bar of 12 pieces, 4 rows of 3 | weight: retailer listings (e.g. Target, "Hershey's Milk Chocolate Bar 1.55oz") |
| bar outline | 14.5 x 5.7 x 0.7 cm, pieces 3.6 x 1.9 cm | **draft**: published figures disagree; to be measured on a bought bar |
| groove strength | 0.12 N m to snap one groove across the bar's width | **planning estimate**: a 20-35 N three-point snap force for milk chocolate over a 50 mm span, scaled to a 3 mm deep groove in a 7 mm bar; to be measured on 10 bars at 20-22 C |
| plate | a white dinner plate, 27 cm across, 2 cm tall | an ordinary dinner plate |

How the sim models it: the 12 pieces are rigid blocks. Each pair of neighbours is joined at the middle of their shared
face by a hinge along their groove, rigid in every other direction and held straight by a spring. The spring is set
so the joint's share of the groove's strength bends it 1.5 degrees (each joint takes the length of groove it spans, so
every groove has the same strength per centimetre). The task breaks a joint (switches it off) once its groove has
bent past 1.5 degrees, or once its two pieces have come more than 1.5 mm or 4 degrees apart; a broken joint stays
broken for the trial. Chocolate is brittle, so it snaps at a small bend; the 1.5 degrees is an estimate.

Why not the simulator's own breakable joints: PhysX breaks a joint at one torque whichever way it is loaded, so a
gentle pinch across the bar's width, which squeezes the bar in its own plane where real chocolate is far stronger,
broke it before any bend; and no break event reached the task in a headless run.

What we simplified: the pieces weigh ten times a real bar's (36 g each, not 3.6 g), because at real weight the joints
gave way under the finger pads' few newtons (a known weakness of the solver with very light bodies between stiff
contacts); the row still weighs only about 1 N against the ~7 N that snaps its groove. Twisting a groove or bending
it in the bar's own plane doesn't break it. Chocolate can't crack anywhere but a groove, crush, melt or crumble. The
pieces are drawn in two shades so the grooves show on camera (on a real bar they show by shadow). The plate is a flat
disc without a rim. All objects share one friction.

## Setup
Positions in cm on the table, from the midpoint between the two arm bases: x forward (away from the arms), y to the
robot's left.

| Object | Description | Position |
|---|---|---|
| plate | white ceramic dinner plate, 27 cm across, 2 cm tall, 600 g (loose) | centred at (30, 3.6): the groove next to the bar's right end row is on the midline |
| bar | milk chocolate bar, 12 pieces in 4 rows of 3, 14.5 x 5.7 x 0.7 cm, 43 g, embossed side up, its length along y | centred on the plate |

Both grippers start open at the ready pose. One fixed layout: `-E object_jitter=0`. For real trials, a new bar for
every trial, at 20-22 C, unwrapped just before; the gripper pads are wiped with a dry cloth after each trial.

## Scoring
A groove breaks as described above (bent past 1.5 degrees, or its pieces apart by more than 1.5 mm or 4 degrees) and
stays broken; pieces joined by unbroken grooves make up the parts. A piece is on the plate when its middle is over the plate and not
below the plate's top. **Hands off:** no point along either gripper's finger pads, and neither fingertip's end, within
1 cm of any piece.

| Score | When, at the end |
|---|---|
| 1.0 | exactly two parts, an end row of three and the other nine, every piece on the plate and still, both hands off; held for 1 s, which ends the trial |
| 0.75 | that break, with a hand still on a piece or a piece still moving |
| 0.5 | an end row is off on its own, but the other nine are in two or more parts |
| 0.25 | the bar broke, but no end row is off on its own (for example 6 and 6, a column of four, or one square) |
| 0 | the bar unbroken, or any piece off the plate |

In a real trial, lift each part gently: pieces that come up together are one part. A photo of the plate gives whether
everything is on it.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=break_chocolate_row.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Break one row of three pieces off one end of the chocolate bar, keeping the other nine pieces together in one piece. Leave both on the plate." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/break-chocolate-row-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=break_chocolate_row.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Break one row of three pieces off one end of the chocolate bar, keeping the other nine pieces together in one piece. Leave both on the plate." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/break-chocolate-row-demo

# A wrong way: each hand pinches an end row and lifts it, so the grooves next to both ends break
CHOC_DEMO=ends inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=break_chocolate_row.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Break one row of three pieces off one end of the chocolate bar, keeping the other nine pieces together in one piece. Leave both on the plate." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/break-chocolate-row-ends

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/break-chocolate-row-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. With `CHOC_DEBUG=<file>` set, each control step adds a line to that file: the
sizes of the parts, how many joints have broken and why (bent or apart), the most-bent groove, and whether the hands
are off.

## The scripted demo
It plays a fixed plan from the fixed layout with the arm's own inverse kinematics, and reads nothing from the
simulator.

1. Both hands turn so their fingers close across the bar's width, and lean 35 degrees away from each other so the arms
   stay clear (the end groove lies on the midline between them).
2. The left hand pinches the second row right beside the groove to the end row on the right arm's side, its fingertips
   about 1 cm from the groove. Each finger is aimed 3-4 mm into the bar (15-20 N), not closed fully, which would crush.
3. The right hand pinches that end row and turns it up about the groove's line, 3 then 5 degrees. All the bending is at
   the one groove between the hands, which snaps at about 2.3 degrees; the others barely bend.
4. The left hand lets go. The right hand lifts the row clear, carries it 3 cm along the plate, sets it down and lets
   go, and both hands move off to the sides. The right wrist camera's last frames show the gap between the row and the
   nine pieces.

## Runtime and seeds
- Official runs (2026-11-19, this version): the scripted demo 1.0, ending on success at step 253 (13 s): exactly the
  end groove's three joints broke, all by bending; nothing else broke, and the pinches alone broke nothing. The wrong way
  (`CHOC_DEMO=ends`, each hand lifting an end row) 0.5: both end grooves broke, leaving a row, a row and six pieces.
  Dream-1 0.0 over the full 1,200 steps: its hands never reached the bar, which stayed whole.
- The scoring and reach checks run without the simulator: `python -m pytest -q test_break_chocolate_row.py` (34 tests).
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
- `tools/check_break_joint.py` is the check of PhysX's own breakable joints (they hold at 0.9 and break at 1.1 times
  their limit, in any direction), which led to the hinges above.

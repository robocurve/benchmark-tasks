# Sharpen a kitchen knife on a whetstone at 15 to 20 degrees, both sides, then lay it down edge away

A 165 mm santoku lies on an end-grain chopping block next to a 1000-grit whetstone held in a non-slip rubber holder
on a sharpening stand. "Sharpen the knife on the whetstone, then put it back on the chopping block." means what anyone who
sharpens their own knives at home, or a cook or butcher, does without thinking about it: hold the blade 15 to 20 degrees off the stone, draw it across edge first, a
few strokes on each face, then lay the knife back down flat with its edge facing away. Rubbing it flat on the stone
(the easy pose for a gripper that picked it up flat), holding it steep, or doing one face only are the tempting wrong
answers.

## Run a trial

Scripted demo (reaches full score):

```
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/knife-on-whetstone-at-angle/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Sharpen the knife on the whetstone, then put it back on the chopping block." \
  --scorer task_score --max-steps 2400
```

Model run (Dream-1): the same command with `--policy dream --max-steps 2400` (2,400 steps = 2 minutes at 20 Hz).

Offline checks (no simulator): `python3 test_task.py` (45 checks, prints "all pass").

## Spec

### Scene (cm from the midpoint between the arm bases, x forward, y to the robot's left, table top z = 0)

| Part | What it is | Where |
|---|---|---|
| Chopping block | End-grain maple butcher block, 12 x 22 x 8 cm, fixed | center (16.4, 8.5), long side at heading 105 deg, top z 8 |
| Knife | 165 mm santoku, 100 g: steel blade 16.5 cm long and 4.5 cm tall at the heel, its spine curving down to the tip, its edge straight with a 6 mm rise over the last 3 cm to the tip and a bright 6 mm ground bevel along it, octagonal magnolia wa handle 9.5 cm with a horn ferrule; balance point 2 cm in front of the bolster | flat on the block, face A down, edge away (heading 15 deg), bolster at (18.4, 0.8), handle reaching past the block's edge to (20.9, -8.4) |
| Whetstone | Orange 1000-grit stone 18 x 6 x 2.5 cm in a black rubber holder (21 x 8 x 1.5 cm, 3 cm end clamps) on a 6 cm wooden stand, all fixed | center (39.4, -5.5), long side at heading -15 deg, stone top z 10 |

### Rule and scoring

| Term | Definition (all thresholds in task.py) |
|---|---|
| Sharpening angle | Measured across the blade: how far its width (spine to edge) tilts below the stone's level top, edge down. A knife pitched tip-down about its width has no sharpening angle |
| Level | The blade's length (handle to tip) within 5 deg of horizontal: a stone stroke keeps the edge lying along the stone, and a 5 deg pitch already lifts the far end of a 6 cm contact 5 mm off it. The edge's rise near the tip comes onto the stone with the handle lifted about 3 deg, as a sharpener does for the tip, inside this limit |
| Held | A gripper with both fingers touching the handle (a gap of 2 mm or less, the solver's resting gap, between the handle and the gripper's own shape: palm and fingers at their present opening) and their opening stopped by it (0.10 to 0.24 of full opening: the handle's 1.8 cm thickness less 15 %, to its 2.0 cm width plus 3 %). An empty fist, an open hand round the handle or a hand merely near it does not hold the knife, so a knife pushed or dragged without a hand on it makes no strokes |
| On the stone | Held, edge lower than spine, touching: the edge's lowest point over the stone within 0.5 mm of its top (the knife's collider rests on the stone's, so a knife held up off it does not count). 16 edge points, about 1 cm apart from 0.5 cm past the heel to 16.2 cm, 3 mm short of the point, where the edge curves up to meet the spine: a point is on the stone within 1 mm of its top |
| Stroke | One touch with 5 cm of edge-leading travel at 13 to 22 deg (15 to 20 deg, plus 2 deg for a hand-held angle), the blade level and at least 3 cm of edge (3 scorer points, half the stone's 6 cm width) on the stone; a tip or heel dragged alone sharpens nothing. 15 deg is the usual angle for Japanese knives, 20 deg for Western ones |
| Worked | An edge point on a face once it has been on the stone for 10 cm of in-band, edge-leading travel, two strokes' worth. The stone is 6 cm across and the edge 16.5 cm long, so one place covers about 6 cm; working most of the edge means moving the knife along so the heel, middle and tip each cross the stone |
| Not counted | A touch with 2 cm of travel steeper than 22 deg (it works a steeper, more obtuse micro-bevel, not the 15 to 20 deg edge), or one whose edge drops 3 mm below the stone's top just past an end within 0.5 s of leaving it (rolled off the end instead of lifted): none of that touch's travel counts as strokes or worked edge. Neither zeroes the trial. A real trial reads the angle against a 22 deg card on the side video and watches for the roll-off |
| Put down safely | Let go (no part of either gripper touching the knife; a gap of 2 mm or less, the solver's resting gap, between the knife and the gripper's own shape counts as touching), flat on the block (faces within 20 deg of level, edge within 1 cm of its top), edge within 60 deg of +x (away from the robot): a knife left in the hand or on the stone isn't put away, and with the edge away the next hand reaching for the handle does not meet it. Real trial: top photo and a protractor |

| Outcome | Score |
|---|---|
| Fewer than 5 strokes on both faces (doing nothing, flat, too-steep or rolled-off strokes, hovering, dragging without a grip, edge-trailing) | 0 |
| 5+ strokes on one face only: the edge is pushed over into a wire, not sharp | 0.25 |
| 5+ strokes on each face, but under 12 of the 16 edge points (three quarters of the edge) worked on a face: the right technique, but a quarter or more of the edge still dull | 0.5 |
| 5+ strokes and 12+ points worked on each face, knife still held, left on the stone or edge toward the robot: sharpened, but not put down safely | 0.75 |
| Both faces worked along most of the edge, knife put down safely, still for 1 s | 1.0 |

Where the arms end is not scored. These are the sim's tiers; a real trial scores the same ladder from edge-tester
readings (Real-world build, below).

Why edge leading: an edge-first stroke cuts into the stone like slicing a thin layer off it and raises less of a wire
burr than a trailing stroke; on face A the demo pushes (edge away from the robot), on face B it pulls.

## The demo

The right arm slides its open hand sideways round the handle (fingers above and below it, approach along the
handle), lifts, turns the knife to 17 degrees edge-down over the stone and makes push strokes on face A, two each with
the heel, middle and tip of the blade over the stone (the stone's 6 cm covers about 6 cm of edge at a time), lifts
straight up, rolls the knife over through upright and does the same with pull strokes on face B (three with the tip),
lays it back on the block edge-away, lets go and goes home. For the tip strokes the knife is turned 10 degrees on the
stone, tip further from the robot, and the handle is lifted (2.5 degrees on face A, 3.5 on face B) so the edge's rise
lies on the stone up to the last scorer point, 3 mm short of the point. Held square to the stone, the tip strokes
put the wrist at its joint limit: the hand fell 1 to 1.5 cm short and lost the lift, so the last 1 cm of edge never
touched the stone. On face B the tip strokes are aimed at the stone's top rather than 1 mm into it, and start with
more lift, because the pull turns the knife back about 2 degrees in the grip. The wrist roll (joint 6) covers the 146
degrees between the two faces; the hand is turned 90 degrees round the handle at the pick so both faces fall inside
its range. The demo trims its aim by 2.5 degrees per face for the knife's sag in the grip.

## Runtime and seeds

Sim result, with the raised thresholds (5 strokes per face, 10 cm of travel per worked point): 1.0 in 1,310 steps (65.5 seconds at 20 Hz). 6 strokes counted on face A and 7 on face B, against 5 needed. All 16 edge points were worked on face A (12.2 to 24.5 cm of counted travel each) and 15 of 16 on face B (10.8 to 24.9 cm; the point 0.5 cm from the heel had 8.3 cm), against 12 needed. The last point, 16.2 cm along the edge and 3 mm short of the tip, had 12.9 cm on face A and 15.8 cm on face B. On the stone the sharpening angle read 14.9 to 20.7 degrees and the blade's length stayed within 3.9 degrees of level (the most with the handle lifted for the tip); one fixed layout, `object_jitter=0`.

## Fidelity

Nothing is ground: the sim scores strokes from the knife's pose while it is on the stone (angle, contact, travel per
edge point) as a stand-in for steel removed; a real trial measures the edge with the edge tester instead. The stone is
rigid and dry. The knife is 100 g (real about 140 g); its blade colliders are boxes following the outline, turned along
the tip's rise so the edge the scorer reads is the one that touches. The sharpening stand is 6 cm tall so the wrist
housing clears the bench during strokes.

## Real-world build

Any 165 to 180 mm santoku or gyuto with a wa handle; a 1000-grit 180 x 60 x 25 mm stone in a rubber holder screwed
to a 6 cm hardwood block; a 12 x 22 x 8 cm end-grain butcher block. Fix the stand and block to the bench with
double-sided tape or clamps at the positions above. Film the top and the side, with 15, 20 and 22 degree angle cards
beside the stone.

Edge tester: a BESS edge sharpness tester (for example an Edge-On-Up PT50B), which reads the grams of force an
edge needs to cut through its standard test filament. Lower is sharper. Rough reference points on that scale: about 50 g
for a new razor blade, 300 g or less for a new kitchen knife, and 500 g or more for a kitchen knife that needs
sharpening. Use a fresh piece of filament for each reading.

Starting dullness, before every trial (the same every time):

1. Soak the stone 10 minutes and set it on a kitchen scale.
2. Hold the knife upright, edge straight down on the stone, and draw the whole edge once along the stone's length, heel
   to tip, pressing until the scale reads about 500 g above the stone's weight (about 5 N). This jointing pass grinds a
   flat of even width along the edge, which shows under a lamp as a thin unbroken line of reflected light.
3. Read the tester at the four points marked on face B, just above the bevel, 2, 6, 10 and 14 cm from the heel. Every
   point must read 500 g or more; if one does not, repeat step 2. Write the four readings down as the start.

After the trial: photograph the knife from above (put-down), then read the tester at the same four points without
stropping, deburring or wiping the edge on anything.

| Real-trial result | Score |
|---|---|
| At least 3 of the 4 points at 300 g or less, knife put down safely and still | 1.0 |
| At least 3 of the 4 points at 300 g or less, knife not put down safely | 0.75 |
| 1 or 2 points at 300 g or less | 0.5 |
| No point at 300 g or less, but at least 3 points 100 g or more below their own start | 0.25 |
| Anything else, including nothing done | 0 |

Why these numbers: 300 g is a sharp kitchen edge that a 1000-grit stone gives, and the start (500 g or more) is a knife
plainly in need of sharpening, so full credit needs a real change in the edge. Three of four points matches the sim's
three quarters of the edge. A wire edge left by sharpening one face only can fold over on the filament and read high, so
sharpening one face tends to land at 0.25, as it does in the sim. The angle cards let a reader see the angle used; the
angle itself is not scored in a real trial, because the reading is the outcome.

## How the sim's strokes relate to the measured edge

The sim grinds no steel. It counts a point of the edge as worked once it has been on the stone for 10 cm of in-band,
edge-first travel, and full credit needs 5 or more strokes and 12 of 16 points worked on each face. It is a stand-in,
not a measured match, and in a real trial the tester readings decide the score in every case.

This revision raised both thresholds, from 3 strokes per face and 5 cm of travel per point (one stroke over each part
of the edge). That earlier floor was below what sharpening a dull edge takes. Guides for Japanese whetstones work each
section of the edge until a burr can be felt all along it before moving on; one says "Keep working on that section
until you do" (Knifewear, "How to Sharpen Kitchen Knives with Japanese Whetstones"). They then finish by alternating
faces "in decreasing strokes - ten strokes per side, then eight, then six, then four, then two, then one stroke per
side several times" (Chef Knives To Go, "How to sharpen a knife on a whetstone"), which is more than 30 strokes per face
after the burr is raised. The new floor, 5 strokes per face with every worked point on the stone for two strokes'
travel, is still well under that. It is a conservative lower bound that the scripted demo meets (6 and 7 strokes, 2 to 3
over each part of the edge); it is not a calibration.

The real count has not yet been measured on this knife and stone. Before real trials the builder sharpens the jointed
knife by hand at 17 degrees, counting strokes per face and reading the tester after every pair of strokes. If more is
needed than the sim requires, the sim's thresholds are raised again and the demo is re-run.

## Files

- `task.py`: scene, scorer and scripted demo (right arm only). `yamkit.py`: the task's helper module for building the
  scene, with three additions (a `com` center of mass on `Prop`, an explicit `rot` matrix on `Tilted`, and `rot_z` on
  a box `Collider` for the blade's rising tip).
- `guardkit.py`: contact between a gripper's own shape (palm and both fingers at their present opening, boxes fitted to
  its collision meshes) and the knife: `gripping()` for holding (both fingers on the handle, the opening stopped by
  it) and `touching()` for letting go. Trimmed to these two checks.
- `test_task.py`: offline scorer checks (`python3 test_task.py`, prints "all pass"), including a hand half open round
  the handle, one finger on it, an empty fist on it, and an open hand resting on the laid-down knife or just clear of it.

## Licenses

All geometry is built from boxes and cylinders in task.py; no external assets.

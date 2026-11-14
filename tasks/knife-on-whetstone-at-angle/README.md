# Sharpen a kitchen knife on a whetstone at 15 to 20 degrees, both sides, then lay it down edge away

A 165 mm santoku lies on an end-grain chopping block next to a 1000-grit whetstone held in a non-slip rubber holder
on a sharpening stand. "Sharpen the knife on the whetstone, then put it back on the chopping block." means what anyone who
sharpens their own knives at home, or a cook or butcher, does without thinking about it: hold the blade 15 to 20
degrees off the stone and sweep it across edge first so the stone runs along the edge from heel to tip, working one
face until a burr runs the whole edge, then the other, then alternating with fewer strokes; then lay the knife back
down flat with its edge facing away (sources under Technique, below). Rubbing it flat on the stone (the easy pose for a
gripper that picked it up flat), holding it steep, or doing one face only are the tempting wrong answers.

## Run a trial

Scripted demo (reaches full score):

```
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/knife-on-whetstone-at-angle/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Sharpen the knife on the whetstone, then put it back on the chopping block." \
  --scorer task_score --max-steps 2400
```

Model run (Dream-1): the same command with `--policy dream --max-steps 2400` (2,400 steps = 2 minutes at 20 Hz).

Offline checks (no simulator): `python3 test_task.py` (51 checks, prints "all pass").

## Spec

### Scene (cm from the midpoint between the arm bases, x forward, y to the robot's left, table top z = 0)

| Part | What it is | Where |
|---|---|---|
| Chopping block | End-grain maple butcher block, 12 x 22 x 8 cm, fixed | center (16.4, 8.5), long side at heading 105 deg, top z 8 |
| Knife | 165 mm santoku, 140 g: steel blade 16.5 cm long and 4.5 cm tall at the heel, its spine curving down to the tip, its edge straight with a 6 mm rise over the last 3 cm to the tip and a bright 6 mm ground bevel along it, octagonal magnolia wa handle 9.5 cm with a horn ferrule; balance point 2 cm in front of the bolster | flat on the block, face A down, edge away (heading 15 deg), bolster at (18.4, 0.8), handle reaching past the block's edge to (20.9, -8.4) |
| Whetstone | Orange 1000-grit stone 18 x 6 x 2.5 cm in a black rubber holder (21 x 8 x 1.5 cm, 3 cm end clamps) on a 6 cm wooden stand, all fixed | center (39.4, -5.5), long side at heading -15 deg, stone top z 10 |

### Rule and scoring

| Term | Definition (all thresholds in task.py) |
|---|---|
| Sharpening angle | Measured across the blade: how far its width (spine to edge) tilts below the stone's level top, edge down. A knife pitched tip-down about its width has no sharpening angle |
| Level | The blade's length (handle to tip) within 5 deg of horizontal: a stone stroke keeps the edge lying along the stone, and a 5 deg pitch already lifts the far end of a 6 cm contact 5 mm off it. The edge's rise near the tip comes onto the stone with the handle lifted about 3 deg, as a sharpener does for the tip, inside this limit (the demo does not lift it: The demo, below) |
| Held | A gripper with both fingers touching the handle (a gap of 2 mm or less, the solver's resting gap, between the handle and the gripper's own shape: palm and fingers at their present opening) and their opening stopped by it (0.10 to 0.24 of full opening: the handle's 1.8 cm thickness less 15 %, to its 2.0 cm width plus 3 %). An empty fist, an open hand round the handle or a hand merely near it does not hold the knife, so a knife pushed or dragged without a hand on it makes no strokes |
| On the stone | Held, edge lower than spine, touching: the edge's lowest point over the stone within 0.5 mm of its top (the knife's collider rests on the stone's, so a knife held up off it does not count). 16 edge points, about 1 cm apart from 0.5 cm past the heel to 16.2 cm, 3 mm short of the point, where the edge curves up to meet the spine: a point is on the stone within 1 mm of its top |
| Stroke | One edge-leading pass across the edge of 5 cm or more at 13 to 22 deg (15 to 20 deg, plus 2 deg for a hand-held angle), the blade level and at least 3 cm of edge (3 scorer points, half the stone's 6 cm width) on the stone; a tip or heel dragged alone sharpens nothing. A pass ends when the knife lifts off or moves back 5 mm, so each forward pass of a back-and-forth stroke kept on the stone counts on its own; the edge-trailing return counts for nothing. Travel is counted net, so a knife jiggled in place makes no strokes. 15 deg is the usual angle for Japanese knives, 20 deg for Western ones |
| Worked | An edge point on a face once it has been on the stone for 10 cm of in-band, edge-leading travel. The stone is 6 cm across and the edge 16.5 cm long, so one place covers about 6 cm; working most of the edge means sweeping the knife along its length so the stone runs from heel to tip, or moving it along in sections |
| Not counted | A touch with 2 cm of travel steeper than 22 deg (it works a steeper, more obtuse micro-bevel, not the 15 to 20 deg edge), or one whose edge drops 3 mm below the stone's top just past an end within 0.5 s of leaving it (rolled off the end instead of lifted): none of that touch's travel counts as strokes or worked edge. Neither zeroes the trial. A real trial reads the angle against a 22 deg card on the side video and watches for the roll-off |
| Put down safely | Let go (no part of either gripper touching the knife; a gap of 2 mm or less, the solver's resting gap, between the knife and the gripper's own shape counts as touching), flat on the block (faces within 20 deg of level, edge within 1 cm of its top), edge within 60 deg of +x (away from the robot): a knife left in the hand or on the stone isn't put away, and with the edge away the next hand reaching for the handle does not meet it. Real trial: top photo and a protractor |

| Outcome | Score |
|---|---|
| Fewer than 10 strokes on both faces (doing nothing, flat, too-steep or rolled-off strokes, hovering, dragging without a grip, edge-trailing only) | 0 |
| 10+ strokes on one face only: the edge is pushed over into a wire, not sharp | 0.25 |
| 10+ strokes on each face, but under 12 of the 16 edge points (three quarters of the edge) worked on a face: the right technique, but a quarter or more of the edge still dull | 0.5 |
| 10+ strokes and 12+ points worked on each face, knife still held, left on the stone or edge toward the robot: sharpened, but not put down safely | 0.75 |
| Both faces worked along most of the edge, knife put down safely, still for 1 s | 1.0 |

Where the arms end is not scored. These are the sim's tiers; a real trial scores the same ladder from edge-tester
readings (Real-world build, below).

## Technique: what the guides say

Quotes checked word for word against each page on Nov 14, 2026. Serious Eats, Shun and Lifehacker could not be
fetched and are not cited.

- **Heel to tip.** Most guides sweep every stroke along the edge so the whole edge crosses the stone: Chef Knives To
  Go (CKTG) has the sharpener "move your guiding fingers from the heel toward the tip so that the entire edge makes
  contact with the stone from heel to tip"; Bob Kramer: "pull the knife towards you, heel to tip, making a swooping
  motion on the stone"; America's Test Kitchen (ATK) draws the blade "until the tip of the knife runs off the other
  edge". (Knifewear and Korin instead work a section at a time; Wüsthof goes tip to heel.)
  [CKTG](https://www.chefknivestogo.com/how-to-sharpen-a-knife-on-a-whetstone.html),
  [Kramer](https://kramerknives.com/sharpening/),
  [ATK](https://www.americastestkitchen.com/cooksillustrated/articles/275-how-to-use-a-whetstone)
- **Direction.** The guides differ on push or pull, but where they say which stroke carries the pressure it is the
  edge-leading one: CKTG "This edge-leading stroke on the forward pass is the standard approach for double-bevel
  knives" with the return made "with less pressure or lift it and reset"; Korin pushes "with the edge facing away from
  you ... exerting pressure as you move forward and releasing pressure as you return"
  ([Korin](https://www.korin.com/about-knife-sharpening)). Kramer pulls toward the body; Knifewear and Global go
  "back and forth" ([Knifewear](https://knifewear.com/blogs/articles/how-to-sharpen-kitchen-knives-with-japanese-whetstones),
  [Global](https://www.globalcutleryusa.com/knife-care)). Holding the handle in one hand, the edge faces away on one face and toward the sharpener on the
  other, so edge leading is a push on one face and a pull on the other (ATK: the second side "in the opposite
  direction"). Tormek, a sharpening-machine maker, found "no noticeable difference between the two in relation to
  sharpness" ([Tormek](https://tormek.com/en/knowledge/the-tormek-way-of-sharpening/edge-leading-vs-edge-trailing)).
- **Angle.** 10 to 15 deg (Kramer, Global, Ichimonji), 10 to 20 (Korin), 14
  ([Wüsthof](https://www.wusthof.com/how-to-sharpen-your-wusthof-knives)), 15 Asian and 20 Western (ATK).
- **Pressure.** Kramer: "Start with about 4 – 6 lbs and end with about 2 – 3 lbs" (about 18 to 27 N, then 9 to 13 N);
  Knifewear: "just enough weight to maintain contact with the stone"; CKTG: "light to moderate pressure".
- **How many strokes.** No fixed number: a face is worked until a burr runs the whole edge (ATK: "when you can feel the
  burr from handle to tip, that side is sharp enough", checking "with every 10 passes"; Knifewear: "Keep working on that
  section until you do!"). Then CKTG alternates faces "in decreasing strokes - ten strokes per side, then eight, then
  six, then four, then two, then one stroke per side several times", which "removes the burr".

The sim counts edge-leading passes only (the stroke the guides press on) and needs 10 per face: about one ATK burr
check's worth, still a floor, since the sim feels no burr. The demo follows the common pattern: heel-to-tip strokes,
edge leading, face A pushed (its edge faces away) and face B pulled toward the robot, 9 per face, then 2 more on each
face in turn.

## The demo

The right arm slides its open hand sideways round the handle (fingers above and below it, approach along the
handle), lifts, and turns the knife to 17 degrees edge-down over the stone. Each stroke sets the heel on the stone and
draws the knife 8 cm along it, edge leading, while sliding it 12 cm along its own length, so the stone runs along the
edge from heel to tip: 2 cm with the heel, 4 cm sweeping, 2 cm with the tip. The hand then lifts 1.5 cm and goes back.
Face A (edge away from the robot) is pushed, 9 strokes; the knife is lifted, rolled over through upright, and face B
(edge toward the robot) is pulled toward the robot, 9 strokes; then 2 more on face A and 2 on face B, alternating to
finish, 11 per face in all. It lays the knife back on the block edge-away, lets go and goes home.

What the YAM's reach decided (real IK, offline, every control step):
- **Face A is pushed, not pulled.** On face A a heel-to-tip pull (Kramer's stroke) is out of reach: drawing toward
  the robot and sliding toward the tip both turn joint 4 further, and it hit its 90 degree limit 6 cm into the
  stroke. The edge-leading push of CKTG and Korin keeps it 2.6 degrees inside.
- **The knife turns 10 degrees during the sweep** (tip further from the robot). Square to the stone the tip end puts
  joint 5 at its -90 degree limit and the hand misses by up to 17 mm.
- **The handle is not lifted for the tip** as a sharpener does (Knifewear: "roll the tip into the stone"). 2.5 degrees
  of lift swung joints 4 and 6 through the wrist's singularity, 60 degrees in 0.4 s. So the last scorer point or two,
  up the tip's 6 mm rise, stay 1 to 2 mm off the stone (12 of 16 points are needed).
- **The stroke runs over 8 cm of the stone's 18** (from its middle to 8 cm toward its far end), where the heel and tip
  ends are both in reach. The knife crosses the stone nearly square to its length, where some guides hold it at
  about 45 degrees ([Ichimonji](https://global.ichimonji.co.jp/blogs/maintenance/santoku-household-sharpening)).

The demo aims the edge into the stone by the arm's vertical give at the edge, worked out per waypoint from the joint
stiffness in constants.py (J K^-1 J^T, 1.5 to 6.4 mm per N, scaled to the give the sim showed), planned for 3.5 N.
Measured in a telemetry run of the same demo (the hand's contact forces on the knife plus its weight; held in the air
it reads 1.35 N against its 1.37 N weight), the stone took a median 2.4 N on face A (1.7 to 3.1 N, 10th to 90th
percentile; 2.7, 2.5 and 2.0 N over the heel, middle and tip thirds of the sweep) and 3.3 N on face B (2.4 to 3.8 N;
3.6, 3.5 and 2.8 N). Face A presses less because the push lags its plan against the stone's drag. A firmer aim,
planned for 4.5 N, pressed 6 to 7 N: the drag turned the knife 4 to 8 degrees further in the grip, the edge stood off
the stone and no stroke counted. So about 3 N is the most this demo presses, against 9 to 27 N by hand (Fidelity).
The aim is trimmed by 4 degrees on face A and 3.5 on face B for the knife's turn in the grip under that press.

## Runtime and seeds

Sim result: 1.0 in 1,992 steps (99.6 seconds at 20 Hz), of the 2,400 allowed. 11 strokes counted on each face, against
10 needed. In the telemetry run (also 1.0 in 1,992 steps) 15 of the 16 edge points were worked on face A and 13 on
face B, against 12 needed. The points not worked are up the tip's rise, which the demo does not lift the handle for.
On the stone the sharpening angle read 14.4 to 19.5 degrees, and the edge had 3 or more points on the stone through
nearly every step of every stroke. One fixed layout, `object_jitter=0`.

## Fidelity

Nothing is ground: the sim scores strokes from the knife's pose while it is on the stone (angle, contact, travel per
edge point) as a stand-in for steel removed; a real trial measures the edge with the edge tester instead. The stone is
rigid and dry, and its contact with the knife has the sim's default grip (friction 1.2), more drag than a wet stone.
The press is light: measured at a median 2.4 N on face A and 3.3 N on face B (The demo), where Kramer asks for 4 to
6 lb, then 2 to 3 lb (about 9 to 27 N). An aim for more drags the knife round in the grip. No burr is felt: the
demo makes a fixed count (11 per face). The handle is not lifted for the tip and the knife crosses the stone nearly
square (The demo). The knife is 140 g, as listed for a 165 mm magnolia-handled santoku
([Sakai Takayuki VG5](https://us.seisukeknife.com/products/sakai-takayuki-vg5-hammered-santoku-165mm-magnolia-ferrule-black-plastic-handle-kataoka));
its blade colliders are boxes following the outline, turned along
the tip's rise so the edge the scorer reads is the one that touches. The sharpening stand is 6 cm tall so the wrist
housing clears the bench during strokes.

## Real-world build

A 165 mm santoku with an octagonal wa handle, as in the setup (16.5 cm edge, 4.5 cm tall at the heel); a 1000-grit 180 x 60 x 25 mm stone in a rubber holder screwed
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
edge-leading travel, and full credit needs 10 or more strokes and 12 of 16 points worked on each face. It is a
stand-in, not a measured match, and in a real trial the tester readings decide the score in every case.

This revision raised the stroke floor from 3 per face (the version last reviewed) to 10, and the travel a point needs
to count as worked from 5 to 10 cm, and it counts strokes as the guides describe them (Technique,
above): one edge-leading pass per stroke, whether the knife is lifted for the return or kept on the stone back and
forth. No guide gives a fixed count; they work a face until a burr runs from heel to tip, ATK checking for it "with
every 10 passes", and CKTG then alternates faces in 10, 8, 6, 4, 2 and 1 strokes to remove the burr. So 10 per face is
about one burr check's worth on each face, still a floor; it is not a calibration.

The real count has not yet been measured on this knife and stone, and the demo presses about 2.4 to 3.3 N where Kramer
asks for 9 to 27 N, so a real trial at the YAM's pressure may need more strokes than a person would. Before real trials
the builder sharpens the jointed knife by hand at 17 degrees, pressing about 3 N (300 g on the kitchen scale), counting
strokes per face and reading the tester after every 2 strokes. If more is needed than the sim requires, the sim's
thresholds are raised again and the demo is re-run.

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

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
  --scorer task_score --max-steps 3600
```

Model run (Dream-1): the same command with `--policy dream --max-steps 3600` (3,600 steps = 3 minutes at 20 Hz).

Offline checks (no simulator): `python3 test_task.py` (56 checks, prints "all pass").

## Spec

### Scene (cm from the midpoint between the arm bases, x forward, y to the robot's left, table top z = 0)

| Part | What it is | Where |
|---|---|---|
| Chopping block | John Boos CCB121203 end-grain maple chopping block, 30.5 x 30.5 x 7.6 cm (12 x 12 x 3 in), fixed | center (6.6, 10.3), sides at headings 15 and 105 deg, top z 7.6 |
| Knife | 165 mm santoku, 140 g: steel blade 16.5 cm long and 4.5 cm tall at the heel, its spine curving down to the tip, its edge straight with a 6 mm rise over the last 3 cm to the tip and a bright 6 mm ground bevel along it, octagonal magnolia wa handle 9.5 cm with a horn ferrule; balance point 2 cm in front of the bolster | flat on the block, face A down, edge away (heading 15 deg), bolster at (18.8, -0.7), the front 1.5 cm of the handle on the block, the rest reaching past its edge to (21.3, -9.8) |
| Whetstone | King KW65 1000/6000 stone, 18.5 x 6.3 x 2.5 cm, 1000 side up, in a Naniwa A-902 holder (28.8 x 7.6 x 3.6 cm, the stone's underside 3.2 cm up) on a Shapton glass-stone field holder base used as the stand (25.3 x 8 x 5.3 cm), all fixed | center (43.2, -6.5), long side at heading -15 deg, stone top z 11 |

### Rule and scoring

| Term | Definition (all thresholds in task.py) |
|---|---|
| Sharpening angle | Measured across the blade: how far its width (spine to edge) tilts below the stone's level top, edge down. A knife pitched tip-down about its width has no sharpening angle |
| Level | The blade's length (handle to tip) within 5 deg of horizontal: a stone stroke keeps the edge lying along the stone, and a 5 deg pitch already lifts the far end of a 6 cm contact 5 mm off it. The edge's rise near the tip comes onto the stone with the handle lifted about 3 deg, as a sharpener does for the tip, inside this limit (the demo does not lift it: The demo, below) |
| Held | A gripper with both fingers touching the handle (a gap of 2 mm or less, the solver's resting gap, between the handle and the gripper's own shape: palm and fingers at their present opening) and their opening stopped by it (0.10 to 0.24 of full opening: the handle's 1.8 cm thickness less 15 %, to its 2.0 cm width plus 3 %). An empty fist, an open hand round the handle or a hand merely near it does not hold the knife, so a knife pushed or dragged without a hand on it makes no strokes |
| On the stone | Held, edge lower than spine, touching, and the contact verified: the simulator's contact solver reports the stone pushing on the knife with 0.5 N or more (the knife alone weighs 1.4 N; only the knife-stone pair is read, so the hand and the board do not count), and the edge's lowest point over the stone is within 0.5 mm of its top (so the push is on the edge, not the spine or a face). 16 edge points, about 1 cm apart from 0.5 cm past the heel to 16.2 cm, 3 mm short of the point, where the edge curves up to meet the spine: a point is on the stone within 1 mm of its top |
| Stroke | One edge-leading pass across the edge of 5 cm or more at 13 to 22 deg (15 to 20 deg, plus 2 deg for a hand-held angle), the blade level and at least 3 cm of edge (3 scorer points, half the stone's 6 cm width) on the stone; a tip or heel dragged alone sharpens nothing. A pass ends when the knife lifts off or moves back 5 mm, so each forward pass of a back-and-forth stroke kept on the stone counts on its own; the edge-trailing return counts for nothing. Travel is counted net, so a knife jiggled in place makes no strokes. 15 deg is the usual angle for Japanese knives, 20 deg for Western ones |
| Worked | An edge point on a face once it has been on the stone for 10 cm of in-band, edge-leading travel. The stone is 6 cm across and the edge 16.5 cm long, so one place covers about 6 cm; working most of the edge means sweeping the knife along its length so the stone runs from heel to tip, or moving it along in sections |
| Not counted | A touch with 2 cm of travel steeper than 22 deg (it works a steeper, more obtuse micro-bevel, not the 15 to 20 deg edge), or one whose edge drops 3 mm below the stone's top just past an end within 0.5 s of leaving it (rolled off the end instead of lifted): none of that touch's travel counts as strokes or worked edge. Neither zeroes the trial. A real trial reads the angle against a 22 deg card on the side video and watches for the roll-off |
| Put down safely | Let go (no part of either gripper touching the knife; a gap of 2 mm or less, the solver's resting gap, between the knife and the gripper's own shape counts as touching), flat on the block (faces within 20 deg of level, edge within 1 cm of its top), edge within 60 deg of +x (away from the robot): a knife left in the hand or on the stone isn't put away, and with the edge away the next hand reaching for the handle does not meet it. Real trial: top photo and a protractor |

| Outcome | Score |
|---|---|
| Fewer than 20 strokes on both faces (doing nothing, flat, too-steep or rolled-off strokes, hovering, dragging without a grip, edge-trailing only) | 0 |
| 20+ strokes on one face only: the edge is pushed over into a wire, not sharp | 0.25 |
| 20+ strokes on each face, but under 12 of the 16 edge points (three quarters of the edge) worked on a face: the right technique, but a quarter or more of the edge still dull | 0.5 |
| 20+ strokes and 12+ points worked on each face, knife still held, left on the stone or edge toward the robot: sharpened, but not put down safely | 0.75 |
| Both faces worked along most of the edge, knife put down safely, still for 1 s | 1.0 |

Where the arms end is not scored. These are the sim's tiers; a real trial scores the same ladder from edge-tester
readings (Real-world build, below).

## Technique: what the guides say

Quotes checked word for word against each page on Nov 14, 2026. Serious Eats, Shun and Lifehacker could not be
fetched and are not cited.

- **Heel to tip.** Most guides sweep every stroke along the edge so the whole edge crosses the stone: Chef Knives To
  Go (CKTG) has the sharpener "move your guiding fingers from the heel toward the tip so that the entire edge makes
  contact with the stone from heel to tip"; Bob Kramer: "pull the knife toward you, heel to tip, making a swooping
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

The sim counts edge-leading passes only (the stroke the guides press on) and needs 20 per face, the count a published
electron-microscope study found enough to grind a whole new edge at light pressure (How the sim's strokes relate to
the measured edge, below); the sim feels no burr. The demo follows the common pattern: heel-to-tip strokes, edge
leading, face A pushed (its edge faces away) and face B pulled toward the robot, the faces alternated in equal sets
of 11 (A, B, A, B), 22 per face.

## The demo

The right arm slides its open hand sideways round the handle 4.5 cm behind the bolster (fingers above and below it,
approach along the handle, the fingertips 3.2 cm short of the blade), as close to the blade as the gripper reaches:
its fingers are 7 cm long, and a grip closer in put the palm against the end of the 9.5 cm handle. A grip close to the
blade, as sharpeners hold it, gives the stone's drag a short lever to turn the knife in the hand. It lifts, and turns the knife to 17 degrees edge-down over the stone. Each stroke sets the heel on the stone and
draws the knife 11 cm along it, edge leading, while sliding it 12 cm along its own length, so the stone runs along the
edge from heel to tip: 2 cm with the heel, 7 cm sweeping, 2 cm with the tip. The hand then lifts 1.5 cm and goes back.
The faces alternate in equal sets of 11 strokes: face A (edge away from the robot) pushed, then face B (edge toward
the robot, the knife rolled over through upright above the stone) pulled toward the robot, then A again, then B, 22
per face in all. At each change of face the arm lays the knife flat on the block, opens its hand and grips the handle
again where it first held it, so a turn of the knife in the hand from the stone's drag does not build up from set to
set (the open-loop demo cannot feel the knife; a regrip at each change is its equivalent of a sharpener re-seating
the grip). It then lays the knife back on the block edge-away, lets go and goes home.

What the YAM's reach decided (real IK, offline, every control step):
- **Face A is pushed, not pulled.** On face A a heel-to-tip pull (Kramer's stroke) is out of reach: drawing toward
  the robot and sliding toward the tip both turn joint 4 further, and it hit its 90 degree limit 6 cm into the
  stroke. The edge-leading push of CKTG and Korin keeps it 2.6 degrees inside.
- **The knife turns 10 degrees during the sweep** (tip further from the robot). Square to the stone the tip end puts
  joint 5 at its -90 degree limit and the hand misses by up to 17 mm.
- **The handle is not lifted for the tip** as a sharpener does (Knifewear: "roll the tip into the stone"). 2.5 degrees
  of lift swung joints 4 and 6 through the wrist's singularity, 60 degrees in 0.4 s. So the last scorer point or two,
  up the tip's 6 mm rise, stay 1 to 2 mm off the stone (12 of 16 points are needed).
- **The stroke runs over 11 cm of the stone's 18** (from 3 cm short of its middle to 8 cm toward its far end), where the heel and tip
  ends are both in reach. The knife crosses the stone nearly square to its length, where some guides hold it at
  about 45 degrees ([Ichimonji](https://global.ichimonji.co.jp/blogs/maintenance/santoku-household-sharpening)).

The demo aims the edge into the stone by the arm's vertical give at the edge, worked out per waypoint from the joint
stiffness of the YAM's joint servos (the yam_arena embodiment's constants; J K^-1 J^T, 1.5 to 6.4 mm per N, scaled to the give the sim showed), planned for 3.5 N.
Measured in a telemetry run of the same demo (the hand's contact forces on the knife plus its weight; held in the air
it reads 1.35 N against its 1.37 N weight), the stone took a median 2.4 N on face A (1.7 to 3.1 N, 10th to 90th
percentile; 2.7, 2.5 and 2.0 N over the heel, middle and tip thirds of the sweep) and 3.3 N on face B (2.4 to 3.8 N;
3.6, 3.5 and 2.8 N). Face A presses less because the push lags its plan against the stone's drag. A firmer aim,
planned for 4.5 N, pressed 6 to 7 N: the drag turned the knife 4 to 8 degrees further in the grip, the edge stood off
the stone and no stroke counted. So about 3 N is the most this demo presses, against 9 to 27 N by hand (Fidelity).
The aim is trimmed by 4 degrees on face A and 3.5 on face B for the knife's turn in the grip under that press.

## Runtime and seeds

Sim result: 1.0 in 3,481 steps (174 seconds at 20 Hz), of the 3,600 allowed: 22 strokes per face in alternating
sets of 11, against 20 needed, with a regrip on the block at each change of face. The previous version (11 strokes
per face, no contact check) scored 1.0 in 1,992 steps; in its telemetry run the stone pushed the knife with a median
2.4 N on face A and 3.3 N on face B, well over the 0.5 N contact check. One fixed layout, `object_jitter=0`.

Model run: Dream-1 scores 0 in 3,600 steps (the run limit) on this task.py (previous version: 0 in 2,400 steps).

## Fidelity

Nothing is ground: the sim scores strokes from the knife's pose while it is on the stone (angle, travel per edge
point) and the contact solver's push from the stone as a stand-in for steel removed; a real trial measures the edge with the edge tester instead. The stone is
rigid and dry, and its contact with the knife has the sim's default grip (friction 1.2), more drag than a wet stone.
The press is light: measured at a median 2.4 N on face A and 3.3 N on face B (The demo), where Kramer asks for 4 to
6 lb, then 2 to 3 lb (about 9 to 27 N). An aim for more drags the knife round in the grip. No burr is felt: the
demo makes a fixed count (22 per face, in sets of 11). The handle is not lifted for the tip and the knife crosses the stone nearly
square (The demo). The knife is 140 g, as listed for a 165 mm magnolia-handled santoku
([Sakai Takayuki VG5](https://us.seisukeknife.com/products/sakai-takayuki-vg5-hammered-santoku-165mm-magnolia-ferrule-black-plastic-handle-kataoka));
its blade colliders are boxes following the outline, turned along
the tip's rise so the edge the scorer reads is the one that touches. The stand (5.3 cm) and holder put the stone's top 11 cm up so the wrist
housing clears the bench during strokes. The holder is modeled as a slab with end clamps, the stand as a solid box.

## Real-world build

All parts are bought, none made:
- Knife: Sakai Takayuki VG5 hammered santoku, 165 mm, magnolia handle (listed at 140 g, 46 mm tall at the heel; the
  knife the sim's mass is taken from).
- Stone: King KW65 1000/6000 combination waterstone, 185 x 63 x 25 mm as in the sim, 1000-grit side up
  ([Japanwelt](https://www.japanwelt.de/en/cooking-und-dining/japanese-knives/whetstones/king-kw-65-whetstone-1000-/-6000.html),
  [My Good Knife](https://mygoodknife.com/en/shop/sharpening-stone-matsunaga-king-kw-65-ht-65-10006000-)).
- Holder: Naniwa adjustable whetstone holder A-902, stainless steel with non-slip rubber, 288 x 76 x 36 mm, holding the
  stone 32 mm (1-1/4 in) up
  ([Sharpening Supplies](https://sharpeningsupplies.com/products/naniwa-stainless-steel-toishi-dai-stone-holder)).
- Stand: Shapton glass-stone field holder base, 253 x 80 x 53 mm (about 10 x 3-1/8 x 2 in), set flat under the
  Naniwa holder so the stone's top is 11 cm above the bench and the YAM's wrist clears it
  ([MTC Kitchen](https://mtckitchen.com/products/shapton-glass-sharpening-stone-field-holder),
  [Woodcraft](https://www.woodcraft.com/products/shapton-sharpening-field-holder-packable)). The Naniwa holder is
  3.5 cm longer, so its ends overhang the base by 1.8 cm each.
- Chopping block: John Boos CCB121203, end-grain Northern hard rock maple, reversible, 12 x 12 x 3 in (30.5 x 30.5 x
  7.6 cm) ([John Boos](https://www.johnboos.com/products/maple-square-chopping-block-reversible-3-thick-ccb-series)).
- Kitchen scale under the stand for the jointing press; 15, 20 and 22 degree angle guides beside the stone.
Fix the stand and block to the bench with double-sided tape or clamps at the positions above. Film the top and the
side.

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
edge-leading travel, and full credit needs 20 or more strokes and 12 of 16 points worked on each face. In a real trial
the edge tester decides the score in every case; the sim's count only gates the sim.

Where the 20 comes from. No guide gives a fixed count (Technique, above), so the count is taken from a published
measurement of the edge itself: Verhoeven, "Experiments on Knife Sharpening" (2004), section 4. He hand-sharpened HRC 60 stainless blades on 6000 and 8000 grit
Japanese waterstones "with only a very light downward pressure", alternating faces, and examined every edge in a
scanning electron microscope. Ten of his 4-stroke cycles (20 strokes per face) "ground the edges back from the
position of the original as-ground edge sufficiently that all remnants of the original burred edges were gone"; with
4 cycles (8 strokes per face) "it was found that the 4 cycle treatment was not adequate to be certain that all
remnants of the as-ground edge had been removed". So 20 per face is the measured count for grinding a whole new edge
at light pressure, and 8 is measured as too few.

How it carries over, both ways, stated rather than measured on this knife:
- This task's stone is 1000 grit, coarser than his 6000 and 8000, so each stroke removes more steel.
- This task starts from a jointed edge (a flat of even width, 500 g or more on the tester), more steel to remove than
  his as-ground edges.
- His press was "very light"; the demo's is 2.4 to 3.3 N (The demo), far under Kramer's 9 to 27 N by hand.
No public data we found ties a stroke count to edge-tester readings at a given pressure, and the reviewers' requested
bench calibration (sharpening this knife on this stone at the demo's press, reading the tester every 2 strokes) has
not been run. The sim's 20 is therefore the published count, not a calibration of this knife. What keeps a real trial
honest is that it is scored on the tester alone: a robot that strokes 20 times but leaves the edge above 300 g scores
0.5 or less, whatever the sim's count says. The run limit is 3 minutes (3,600 steps) so that the published count fits
with time to lay the knife down: a person sharpening a dull knife on one stone takes several minutes.

The earlier version planned a hand calibration by the builder before real trials and would have raised the sim's
count from it. That step is replaced by the published count above; the real-trial ladder (Real-world build) is
unchanged.

## Files

- `task.py`: scene, scorer and scripted demo (right arm only). `yamkit.py`: the task's helper module for building the
  scene, with five additions (a `com` center of mass on `Prop`, an explicit `rot` matrix on `Tilted`, `rot_z` on
  a box `Collider` for the blade's rising tip, a `report` flag on `Prop` that turns on PhysX contact reports for it,
  and `STEP_HOOKS`, run once per control step). The contact check in `task.py` reads the stone's push on the knife
  through a PhysX rigid-contact view after every physics substep, the same mechanism as the accepted
  move-live-wire-with-stick task, and turns PhysX contact processing on as Isaac Lab's own ContactSensor does.
- `guardkit.py`: contact between a gripper's own shape (palm and both fingers at their present opening, boxes fitted to
  its collision meshes) and the knife: `gripping()` for holding (both fingers on the handle, the opening stopped by
  it) and `touching()` for letting go. Trimmed to these two checks.
- `test_task.py`: offline scorer checks (`python3 test_task.py`, prints "all pass"), including a hand half open round
  the handle, one finger on it, an empty fist on it, an open hand resting on the laid-down knife or just clear of it,
  and the contact check: the edge on the stone by geometry with no push, a push under 0.5 N or no reading scores 0,
  and a push with the edge 2 mm up scores 0.

## Licenses

All geometry is built from boxes and cylinders in task.py; no external assets.

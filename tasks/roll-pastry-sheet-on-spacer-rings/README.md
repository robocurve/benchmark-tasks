# Roll puff pastry into an even 3 mm sheet with a spacer-ring rolling pin

A block of puff pastry on a floured maple pastry board, and a wooden rolling pin with 3 mm spacer rings on both ends of
its barrel. "Roll this puff pastry out into a 3 mm sheet at least 20 cm long." The rings only set the thickness if
both of them ride on the board, so the pin has to be kept level along its length: with one end lifted the barrel
leaves a wedge, thick on that side. The pastry springs back, so it takes several passes. Category: Societal relatability.
Rolling dough out with a rolling pin, for a pie crust, cookies or pastry, is something most people have done at home,
and rolling-pin spacer rings are a common home tool for an even thickness.

Run (from the repo root, Isaac Lab-Arena installed):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/roll-pastry-sheet-on-spacer-rings/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Roll this puff pastry out into a 3 mm sheet at least 20 cm long." --scorer task_score --max-steps 1300
    python tasks/roll-pastry-sheet-on-spacer-rings/test_task.py      # offline model and scorer checks, prints "all pass"

Use `--policy dream --max-steps 3000` for a Dream-1 trial (the run limit). The scene is deterministic (no seeds, object_jitter=0).
Runtime per trial: the demo finishes in 1,198 steps (59.9 s simulated at 20 Hz); a Dream-1 trial runs up to 3,000 steps (2.5 minutes).

## Spec

Positions in cm from the midpoint between the two arm bases: x forward, y to the robot's left, table top z 0.

| Object | What it is | Where |
|---|---|---|
| Pastry board | oiled maple (mid brown), 31 (x) x 28 (y) x 1.5 cm, floured (a wooden pin slides on it with friction 0.15 static, 0.10 dynamic); fixed | x 14 to 45, y -14 to 14, top z 1.5 |
| Puff pastry | a chilled block 6.5 (x) x 20 (y) x 1.0 cm (130 mL, about 130 g); modeled, no collider | centered x 29.5, y 0 (the board's center) |
| Rolling pin | wood, 43 cm overall, 450 g: barrel 5 cm across and 26 cm long; on each end of the barrel a blue silicone spacer ring 5.6 cm across (3 mm proud of the barrel) and 1.5 cm wide (\|y\| 11.5 to 13); handles 2.5 cm across and 8.5 cm long (\|y\| 13 to 21.5) | in its rest behind the pastry, axis along y at x 41.5, z 5 (rings 7 mm above the board) |
| Pin rest | two dark wooden cradles, fixed on the table just outside the board, one under the outer end of each handle: a block 4.5 (x) x 2 (y) cm whose top carries the handle at z 3.75, with a 5 mm lip 1 cm tall in front and behind | centered x 41.5, \|y\| 19.5 to 21.5 |

Model (`doughkit.py`): the sheet is 20 strips along x, one per 1 cm of width; each strip is a chain of slices (2.5 mm
long at the start) with a fixed volume each, so a slice's length is its volume over its thickness and the total volume
never changes.

| Quantity | Value | Why |
|---|---|---|
| Gap | the barrel's lowest line above the board where it crosses each strip, from the pin's pose | 3 mm with both rings on the board; one ring lifted 4 mm (0.9 degrees) gives about 3.5 mm at one edge of the pastry and 6.5 mm at the other |
| Press | a slice the barrel rolls across with the gap below its top goes to gap + 0.5 x (thickness - gap), once per crossing | the dough springs back: half the excess per pass is an estimate (measure on the real build: thickness after one pass of a 10 mm block on 3 mm rings). From 10 mm it takes four presses to come within 0.5 mm of the gap |
| Contact | a slice is pressed as the barrel's face reaches its top, sqrt(2 R h - h^2) ahead of the lowest line (h = t - gap, R = 2.5 cm) | a round barrel meets thick pastry ahead of its lowest point (17 mm ahead on the 10 mm block, 5 mm on a 3.5 mm sheet), so stopping a little short of an end still thins it |
| Lengthening | a pressed slice grows longer; the pastry beyond it, away from the block's middle, moves out | the middle of the block is held by its own weight on the board; the sheet grows at both ends |
| Rolled past an end | the end slice is pressed like any other the barrel crosses | the rings hold the barrel 3 mm up there too, so nothing goes thinner than the gap; no damage is assumed |
| Pin set down | axis level within 3 degrees, and back in its rest (axis within 5 mm of the seat height, within 1 cm of the rest's x) or lying on its rings on the board (their bottoms within 5 mm of it) | resting, not held in the air; a round pin left on the board can roll, hence the rest |
| Both hands on the pin (sim only, a stand-in) | presses count only while both grippers touch the pin: each gripper's own shape (palm and fingers, boxes fitted to the YAM's collision meshes, at their present opening) within 2 mm of a handle, the barrel or a ring (flat-ended cylinders as the colliders), whether it grips a handle or presses on the barrel with a closed hand | the sim's pin slides on its rings instead of turning, so the model cannot tell rolling from pushing the pin along; two hands on the pin can hold both ends down, so both rings can stay on the board. The instruction names no grip, so handles and palms on the barrel both count. A hand hovering over the pin does not count. A stroke with one hand or none on the pin is not penalized, and nothing is assumed torn: that stroke just does not count. A real trial has no hand rule: it scores the measured sheet and the final pin state |
| Let go | no gripper touching the pin's barrel, rings or handles (its own shape more than 2 mm away: the simulator's resting gap plus the fit of the boxes) | a hand still on the pin has not set it down; an open hand near the pin but not touching it has |

What the model stands in for, and what a real trial checks instead: the thickness and length come from the press rule
above, which follows the real behavior (the rings set the floor, the dough springs back by a share of the excess, and
volume is kept). The springback share is an estimate, so a real trial measures the sheet itself. The model cannot
tear the sheet and cannot press it below 3 mm; in the sim a stroke without both hands on the pin simply does not count.
A real trial has no such rule and checks only the physical outcome: thickness with a depth gauge, holes and tears by eye, and the edges with the
gauge (see "In a real trial" below). Press force and the barrel's rotation are not modeled.

The dough model also gives the pin no rolling resistance: the pastry pushes back on nothing, so in the sim the rings
keep the pin level on the flat board however it is held, and the robot only has to drive it back and forth. A
real block of chilled pastry pushes back hardest under the thicker side, so a pin pressed harder by one hand can ride
up on the other; the sim cannot show that. The real trial checks the result directly: the sheet's thickness evenness
across its width with the depth gauge (the 2 cm bands' averages within 0.5 mm of each other, as below).

Visuals (markers, no physics, never scored): one slab per covered 1 cm cell, as thick as the pastry there, colored by
thickness band: cream (thicker than 6.5 mm), pale gold (4.5 to 6.5), light gold (3.5 to 4.5), golden (2.5 to 3.5, on
spec), tan (2.0 to 2.5) and dark brown (under 2 mm). Tan and dark brown never appear in the sim: the rings hold the
barrel 3 mm up, so the model cannot press thinner. The top camera shows the outline growing and any
thick band or wedge.

## Scoring

The pastry is read on the board's 1 cm grid (rows: the 20 strips; columns: 1 cm along x).

| Outcome | Score |
|---|---|
| Every strip at least 20 cm long, at least 90 % of the sheet's cells 3 mm within 0.5 mm, the strips' average thicknesses within 0.5 mm of each other (even across the width), pin set down level, no hand touching it | 1.0 |
| Every strip at least 18 cm long, but the sheet not finished (short of 20 cm, fewer than 90 % of cells on spec, or uneven across the width; about one pass to go) | 0.5 |
| The sheet done, but a hand still touching the pin (within 2 mm) or the pin not set down level (in its rest, or on its rings on the board) | 0.5 |
| Every strip at least 16 cm long and the strips' averages within 0.5 mm of each other (about two even passes) | 0.25 |
| Anything else: nothing done, one pass, a wedge from a tilted pin | 0 |

Where the arms end is not scored. Possible scores: 0, 0.25, 0.5, 1.0. Doing nothing scores 0. The score is the sheet the
robot leaves: no penalty is assumed for a stroke with one hand off the pin (in the sim it just does not count) or for
rolling past an end (the rings keep the barrel 3 mm up there as everywhere).

Sim and real trial differ in one rule. In the sim, the dough model presses the pastry only while both grippers touch
the pin; this is a documented approximation, because the sim's pin slides on its rings instead of turning, so the
model cannot tell rolling from pushing it along. The real trial has no hand rule at all: it scores only the measured
sheet (length, depth-gauge thickness and evenness) and the final pin state (set down level, let go). A sheet rolled
partly or wholly with one hand, intact and in spec, scores exactly what the same sheet rolled with two hands scores.

Why each rule: the length and on-spec share are the job itself, a sheet that lines the pan and bakes evenly. Evenness
across the width is checked on its own because it is the one fault the rings exist to prevent: rolled with one end
lifted, the pin leaves a wedge, thicker on that side, and puff pastry rises higher and bakes slower where it is
thicker, so the base can come out lopsided. The
pin must be set down level and let go, because a pin still held or hung in the air is not finished with, and a round pin
let go off its rest or tilted can roll onto the fresh sheet or off the board. In the sim only, strokes count while both
hands are on the pin (on its handles or on the barrel): the sim's pin slides rather than turns, so two hands on it are
the stand-in for rolling it. The real trial does not check the hands.

Why each level is worth what it is: 1.0 is the job as asked. 0.5 is a sheet at least 18 cm long everywhere, most of
the work with about one pass to go, or a finished sheet with the pin not put away; either is half the job. 0.25 is
about two even passes: every strip at least 16 cm and level across the width, so the pastry is safely started and
rolled the right way, but it is still 3.4 to 4.8 mm thick and two or three passes from done, a quarter of the job.
Below that (nothing done, one pass at 11.8 cm, or a wedge from a lifted end) there is no usable progress, so 0.

Why the numbers: 20 cm is the floor of a 20 cm (8 inch) square pan. 3 mm within 0.5 mm: the ring's 3 mm (sold as 0.125
inch) plus half a millimeter of springback a baker accepts; puff pastry much thicker in one place can bake raw there, and
much thinner can burn. 90 %: a 21 cm sheet loses about 1 cm at each end to trimming (10 %). 18 cm (90 % of the length) is one
pass short. 16 cm (80 %) is what about two even passes reach: two level passes make 17.2 cm in the model, one makes
11.8 cm, and the 1 cm to spare allows for strokes that stop short of the ends. 0.5 mm across the width is the same
tolerance a cell is held to; each strip is averaged over its length, so the check reads a wedge, not a single thick
cell. A level pin presses every strip alike (0 mm offline for the demo's strokes, and the sim demo passes the check); one end 1 mm up through eight passes leaves 0.7 mm. The 5 mm and 3-degree pin checks are resting tolerances for a 5.6 cm ring on a flat board (3 degrees
over the 24.5 cm between the rings is a 13 mm rise, so a pin past it is propped on something); a real trial reads
them with a ruler and a level app. Whether a hand still touches the pin is read from the video.

Volume ties length to thickness: the 130 mL block makes 21.7 cm at exactly 3 mm, so a sheet 20 cm long averages at most
3.25 mm. A tilted pin leaves the high side thick, and that strip short (one ring 4 mm up, eight passes: the outermost strip on
that side averages 5.6 mm and is 11.6 cm long), so a wedge scores 0. A smaller tilt is caught by the 0.5 mm
evenness check even where the strips are long enough.

The demo: both arms grip the pin's handles from above (fingers closing along x), lift it out of its rest, over the
middle of the block, and set it down on its rings, then five passes, each from the middle out to the far end and back,
then out to the near end and back, aimed 4 mm short of the end as it then lies (worked out with doughkit; the ends move
out as the sheet grows, and the pin trails the hands by about 1 cm at each turn, so it stops about 1.4 cm short); then
the pin is lifted off the pastry and put back in its rest, and both hands open. In the sim it scores 1.0 at step 1,198:
the run ends on success once the opened hands no longer touch the pin, before the arms go home.

Offline (test_task.py, the same strokes without the arm's lag): one pass leaves 11.8 cm (0), two passes 17.2 cm with
every strip alike (0.25), three passes 19.8 cm with 85 % on spec (0.5), four make 21.1 cm and 100 % on spec (1.0).
One end held 1 mm up through eight passes: 16.8 cm but 0.7 mm thicker on one side, 0. A long, all-on-spec sheet made
0.55 mm thicker on one side than the other scores 0.5, and at 0.45 mm 1.0. The same strokes at 0.04 to 1 m/s give the same sheet. One end 4 mm high (one
hand) 0; one gripper closed on the middle of the barrel (pin level by symmetry) 0 (nothing counts); one hand on a handle
and one closed on the barrel 1.0; both closed hands pressing on top of the barrel 1.0; one hand on a handle and the
other hovering 5 mm over the barrel, nothing pressed; open hands around the handles touching nothing, nothing pressed;
one hand lifted off the pin for part of the work, then proper strokes, 1.0 (nothing assumed torn); full-length strokes over both ends 1.0 (21.4 cm, all on spec, nothing
below 3 mm); pin hung in the air, fingers closed on the handles, or fingertips resting on the barrel (or 1 mm over it, inside the
2 mm contact tolerance) at the end, 0.5; hands opened wide around the handles touching nothing, or closed fingertips 5
mm over the barrel, 1.0 (the old 1 cm zone gave 0.5); fingers set to the handle's width but hovering 9 mm over both
handles through every stroke, nothing counts.

Earlier sim runs (scored before this revision; none rolled past an end, and the present rules give each the same
score): three passes 0.5 (19.3 cm); doing nothing 0; one pass 0 (11.6 cm); one arm rolling by the left handle 0 (the
pin skews 3.6 cm and its free end rides 4 to 10 mm up, and with one hand on the pin no stroke counts); one arm closed
on the middle of the barrel 0 (nothing counts with one hand on the pin).

In a real trial (no hand rule; the score is the measured sheet and the final pin state): lay a 1 cm grid transparency
over the sheet, lined up with the board's front edge and its center line so the points are the same every trial, and
photograph it from above to read its length per 1 cm strip (a ruler along both long edges and the middle is enough:
every strip must reach 20 cm). Read thickness with a digital tire tread depth gauge (zeroed on the bare board, its foot resting on the pastry without pressing,
its pin pushed through to the board) at fixed
points: the center of each 2 cm square of the grid the pastry covers (about 100 points; a point is on spec at 2.5 to
3.5 mm, and at least 90 % must be), and every 2 cm along lines 0.5 cm in from each end and each long side. For evenness,
average the readings in each 2 cm band running the sheet's length: the thickest band may be at most 0.5 mm above the
thinnest. A point under
2.5 mm, a hole where the board shows through, or a cell a tear crosses counts as off spec. A ringed pin cannot press
below 3 mm, so anything thinner is real damage (the pin dragged, an edge smeared or torn), and it shows in the same
90 % share. The sim cannot produce these, so they matter only in a real trial. For the pin, read its level with a level app and
its place with a ruler, and check the last frames of the video that no finger touches it.

## Real-world build

- A maple pastry board about 31 x 28 x 1.5 cm (12 x 11 inch), floured with all-purpose flour.
- A handled wooden rolling pin (an American-style pin with a 10 inch, about 26 cm, barrel about 2 inches, 5 cm, across;
  about 43 cm overall) and a pair of silicone spacer rings, for example the 0.125 inch pair from Mrs. Anderson's Baking
  silicone rolling pin rings (an 8-piece set, two rings each in four thicknesses from 0.0625 to 0.375 inch, sold for
  pins up to about 2.5 inches across). 0.125 inch is 3.2 mm, inside the 2.5 to 3.5 mm on-spec band; the sim uses 3 mm. Check the rings' height over
  the barrel with calipers.
- Store-bought puff pastry (frozen sheets or a block, thawed in the fridge), cut and pressed into a 6.5 x 20 x 1 cm block (130 g); one block per trial. Roll it at
  about 18 C (64 F; 20 to 30 minutes out of the fridge, when a fingertip leaves a dent): straight from the fridge, laminated
  pastry takes a baker leaning on the pin, more than the arms can push, while at 18 C the pin's weight and a light
  press from each hand thin it. If the first build shows the arms stalling on the 10 mm block, start from a 7 mm
  block 9.3 cm long (the same 130 mL) and say so in the setup.
- Measuring tools, all from a hardware or auto parts store: a digital tire tread depth gauge (the common 0 to 25 mm
  kind reading 0.01 mm, a thin pin sliding out of a flat foot), digital calipers, a 30 cm ruler, a 1 cm grid
  transparency (printed on an overhead sheet), a kitchen scale for the 130 g block, and a phone with a camera and a
  level app. No lab equipment is needed.
- Reset between trials: throw the rolled sheet away, scrape the board clean and re-flour it, set a fresh 130 g block
  centered on the board, put the pin back in its rest (axis along y), and check that both rings sit on the barrel's ends.

## What the task tests

Control of one tool's level: the thickness is set only while both spacer rings ride the board, so a 43 cm pin must be
kept level as it travels (the natural way is with both hands, but the real trial scores only the sheet). The work changes under the tool: the sheet lengthens and springs back, so
it takes several passes and the ends move out as it grows.

## Files

- `task.py`: the scene, the scorer and the scripted demo.
- `doughkit.py`: the pastry model (the press, springback and lengthening rules above).
- `guardkit.py` and `yamkit.py`: the task's helper modules (gripper contact shapes, scene and arm helpers).
- `test_task.py`: offline model and scorer checks; run `python tasks/roll-pastry-sheet-on-spacer-rings/test_task.py`.

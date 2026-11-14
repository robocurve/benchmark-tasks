# Fit a lever set to a bored door, holding the outside lever level while the screws are driven

Carpenters fit lever sets when they hang doors, and maintenance workers fit them when they replace worn hardware (O*NET 47-2031: 'Install structures or fixtures, such as windows, frames, floorings, trim, or hardware, using carpenters' hand or power tools'; about 0.7 million carpenters and 1.5 million general maintenance and repair workers in the US). The latch already sits in the door's edge bore. The outside lever's flat spindle and two hollow posts must pass, unseen, through the latch inside a 54 mm bore; the inside lever base goes over them, and two screws through it pull the halves together. Left to hang, the outside half droops in its posts' play and the screws lock it crooked, so the levers sit off level and the latch drags. Kwikset's guide: 'Push up on lever base while tightening screws to keep latch horizontal and centered'; Schlage's: 'Make sure the chassis is straight on the door before tightening the screws'.

Here a section of a 1-3/8 in interior door stands on the table, bored for a lever set, with the tubular latch already
fitted in its edge. The outside half of a passage lever set (lever, rose, flat spindle blade and two hollow mounting
posts) hangs on a holder on the robot's right; the interior lever base (rose, square hub and its two #8-32 screws) hangs
on a holder on the left, and a ratcheting screwdriver lies on a rest. The robot pushes the outside half's blade and posts
through the latch until its rose is flat on the door, fits the base over them on the other face, and drives both screws
while the outside lever is held level. Left to hang, the outside half droops in its posts' play in the latch, and the
screws lock it there crooked.

Category: Impact on jobs (carpenters, general maintenance and repair workers). Which hand does what, and in what order,
is free; the demonstration holds the lever with the right hand and fits the base and drives the screws with the left.

## Run and score

With the official simulator installed, run from this folder with 20 Hz control:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=fit_lever_set_to_bored_door.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --seed 0 \
  --instruction "Fit the lever set to the door: push the outside lever's spindle and posts through the latch until its rose sits flat on the door, fit the inside lever base over them, then drive both screws with the ratcheting screwdriver while holding the outside lever level." \
  --scorer task_score --max-steps 7200 --store-frames --log-dir ./demo-run
```

The model run uses the same command with `--policy dream`. The harness reports task_score at the end. Scores are in
task.json.

**Seeds.** The layout is the same for every seed. With nonzero object jitter the parts' starts are shifted by the seed;
the demonstration reads the parts' poses where noted below.

**CPU checks.** `python -m pytest -q -p no:cacheprovider` in this folder, with yam_arena installed, runs the scorer on
the start, the goal and named partial and cheat layouts.

Runtime per trial: at most 7,200 control steps (360 s simulated). The scripted demonstration's plan lasts
204.7 s (4,094 steps). Wall-clock time depends on the GPU.

## Scoring

Measured after both grippers have been 30 mm or more from both halves and the screwdriver for 2 s. Terms. Through: the outside half's spindle and posts are through the latch, its hub within 2 mm of the bore's centre. On: the base is over the blade and the posts' ends (its centre within 1.5 mm of the spindle's axis). Seated: the rose's rim is within 0.5 mm of its door face all round. Snug: the screw's head bears on the base and a further turn needs force (two full turns in the simulation). Level: the outside lever's long axis and the base's hub flats are within the stated angle of horizontal; the lever points away from the latch edge. 1.0: both halves on and seated, both screws snug, level within 1.0 degree, both halves at rest. 0.75: as 1.0, except that it is level only within 3.0 degrees, or only one screw is snug. 0.5: both halves on and at least one screw caught on its post's thread. 0.25: the outside half through the latch with its rose within 2 mm of the door. 0: anything else, including a lever pointing toward the latch edge. The order of moves and which hand does what are free. Real trial: a digital angle gauge on the lever's top face and on the hub's flat; a 0.5 mm feeler gauge round each rose's rim; a quarter turn more on each screw must take force; then fit the inside lever and turn each lever to 45 degrees to see the bolt go flush and spring back.

| Score | Condition |
| --- | --- |
| 1.0 | both halves through, on and seated, both screws snug, lever and hub within 1.0 degree of level, both halves at rest |
| 0.75 | as 1.0, but level only within 3.0 degrees, or only one screw snug, or a half still moving |
| 0.5 | both halves on and at least one screw caught on its post's thread |
| 0.25 | the outside half through the latch with its rose within 2 mm of the door |
| 0 | anything else, including a lever pointing toward the latch edge or an outside half beside the bore |

Every term is read from the parts' poses and the screws' turns: the rose's rim is sampled at 8 points (30 mm radius on
the outside rose, 22 mm on the base's back face) and must lie within 0.5 mm of its door face; the lever's
long axis and the hub's flats are measured against horizontal. Nothing counts until both grippers have been at least
30 mm from the two halves and the screwdriver for 2 s (an arm still holding the lever
level scores 0). At rest: on the simulator, neither half has moved more than 0.2 mm or turned more than
0.1 degree over the last 1 s (the solver's contact chatter makes instantaneous velocities flicker
on a part standing still, so they are not used there; the CPU fixtures use velocities). Success is a score of 1.0, held
for 1 s.

**Why the shortcuts fail.** Driving the screws with no hold on the outside lever locks it at its droop in the posts'
play (up to 2.3 degrees): on the simulator, the
same demonstration without the hold (`DEMO_HOLD = False`) leaves the half drooped 2.3 degrees and 4 mm proud, the screws
still catch, draw it flat onto the door and fix it drooped: 0.75.
Pushing the lever level after a screw is snug does nothing: hand-tight #8-32 screws clamp the roses with roughly a
kilonewton, so the model fixes both halves where they are at the first snug screw (a 60 N clamp alone let a 16-21 N
push at the lever's tip turn the set on the simulator, so the clamp is not left to friction).

## The screw model

PhysX cannot cut a thread, so `lockset_runtime.py` models the two screws, the ratchet and the bit's coupling, applied as
forces before every physics substep:

- The screws' shanks are not simulated (their heads are part of the base). Once a screw's tip is at least
  1 mm into its post's mouth and within 2.5 mm of the post's axis (the tip finds the
  chamfered mouth; the base's clearance holes and its play on the blade let it shift that far), the shank sits in the
  post's bore: a light spring between the halves (1000 N/m, at most 3 N, at the screw's
  head and at its tip 38.1 mm in) holds the screw on its post's axis, so the base hangs square on the
  posts as on real screws pushed into the posts' mouths; the tip in the mouth lets the base be pushed further on but
  holds it against being drawn off (at most 2 N a screw). It lets go past
  4 mm or when the tip leaves the post. On the simulator the base without these falls off the
  posts' rounded ends when let go.
- A screw can be driven once its tip is in its post's mouth, with the outside half through the latch and the base on its
  face over the blade (within 3 mm), however far the outside half stands proud: a seated screw is
  31.7 mm into its post, so it catches the thread with the half well short of the door.
- The bit is in a screw's recess while its tip is within 1.5 mm of the screw's axis, its axis within
  6 degrees of the screw's and its tip at the head; a light spring (300 N/m, at most
  2 N) holds the tip on the axis. It comes out past 3 mm, 12
  degrees or 3 mm back.
- The ratchet: forward (clockwise from the handle) turns of the screwdriver about the screw's axis turn the screw;
  turns back slip. 2 turns make it snug; a real screw takes about a dozen (spun in first).
- The draw, paced by the turns as a thread is: a caught screw sets how far its head may stand off its post, shortening
  from where it caught to both roses on the door by 1.5 turns (a stiff one-sided spring, 2000 N/m,
  at most 4 N), then pressing the roses on the door with up to 60 N over the last half-turn.
  The halves come together as the screw turns and are never slammed (a 4 N pull applied at once slammed a
  drooping half onto the door on the simulator and knocked its droop out).
- Snug: at the first snug screw both halves are fixed to the door where they are (a hand-tight set does not turn by
  hand); the second screw is still driven to snug for full credit.
- The latch is fixed in the door.

## Physical setup

Millimetres. x forward from the line through the two arm bases, y to the robot's left, z up from the table top. 1) Door section: a painted solid-core interior door slab cut to 250.3 (x 100 to 350.3) by 34.9 (1-3/8 in, y) by 250 tall, standing on its bottom edge on a 10 mm steel base plate fixed to the table; its faces are normal to y, centre plane y 0, outside face y -17.4 (right arm's side), inside face +17.4. Its latch edge is at x 350.3, facing +x (away from the robot). Bored to the lever set's template: a 54 (2-1/8 in) cross bore centred x 290, z 120 (2-3/8 in backset) and a 25 (1 in) edge bore. 2) Latch: the set's 2-3/8 in tubular passage latch, already fitted and screwed, bolt standing 13 proud; inside the cross bore its frame has a slot for the spindle's blade at the bore's centre and two holes for the posts, 7.9 across, 19 above and below it. 3) Outside half of a Kwikset-style passage lever set (zinc lever and rose, dark bronze finish): rose 66.7 across, lever 107 long (4-7/32 in), 16 tall, standing 53 off the door (2-3/32 in); behind the rose a flat steel spindle blade 2.5 x 9.5 and two hollow steel posts 6.4 across, 39.9 long, round-nosed, tapped #8-32; about 300 g. It hangs by its posts and blade in a grey steel holder: rose's back face centred (340, -61, 120), spindle toward +y, lever level pointing toward the robot (-x). 4) Inside lever base of the same set (the inside lever itself is fitted afterwards with its set screw and is not part of this task): rose 66.7 across with a 24 x 20 square hub 35 long, a slot for the blade and two holes over the posts' ends; two #8-32 pan-head screws sit in it, heads on its face; about 100 g. It hangs on two pegs of a grey holder through its post holes: back face centred (340, 80, 120), hub toward +y, flats level. 5) Ratcheting screwdriver, #2 Phillips bit, set to drive: 200 overall, handle 100 long with two flats 26 apart (facing +-x), 35 across them, about 150 g. It lies on a narrow blue block under its handle, axis along y, bit tip at (225, 35, 60) pointing -y. Play: the posts sit in the latch's holes with 0.75 mm radial clearance, so an unheld outside half droops about 2 degrees. Both grippers start open at their ready poses. Full collider tables in the README.

**Friction (estimates):** zinc halves on painted wood 0.40 static / 0.35 dynamic; screwdriver handle 0.50 / 0.40.

## Colliders

Millimetres; x forward from the line through the arm bases, y left, z up from the table top. Every collider is in its
part's frame (origins given in the world frame).

| Part | Collider | Centre | Size or radius x height |
| --- | --- | --- | --- |
| **door** (fixed) | origin (290, 0, 120) | | |
| | box | (-108.5, 0, 15) | 163 x 34.9 x 250 |
| | box | (43.65, 0, 15) | 33.3 x 34.9 x 250 |
| | box | (0, 0, 83.5) | 54 x 34.9 x 113 |
| | box | (0, 0, -68.5) | 54 x 34.9 x 83 |
| | box | (-23, 0, -23) | 8 x 34.9 x 8 |
| | box | (-23, 0, 23) | 8 x 34.9 x 8 |
| | box | (23, 0, -23) | 8 x 34.9 x 8 |
| | box | (23, 0, 23) | 8 x 34.9 x 8 |
| | box | (17, 0, 0) | 20 x 22 x 22 |
| | box | (-5.2, 0, -19) | 2.5 x 20 x 12.9 |
| | box | (5.2, 0, -19) | 2.5 x 20 x 12.9 |
| | box | (0, 0, -24.2) | 7.9 x 20 x 2.5 |
| | box | (0, 0, -13.8) | 7.9 x 20 x 2.5 |
| | box | (-5.2, 0, 19) | 2.5 x 20 x 12.9 |
| | box | (5.2, 0, 19) | 2.5 x 20 x 12.9 |
| | box | (0, 0, 13.8) | 7.9 x 20 x 2.5 |
| | box | (0, 0, 24.2) | 7.9 x 20 x 2.5 |
| | box | (-2.85, 0, 0) | 2 x 12 x 14.7 |
| | box | (2.85, 0, 0) | 2 x 12 x 14.7 |
| | box | (0, 0, -6.35) | 3.7 x 12 x 2 |
| | box | (0, 0, 6.35) | 3.7 x 12 x 2 |
| | box | (60.8, 0, 0) | 1 x 25.4 x 57 |
| | box | (67.8, 0, 0) | 13 x 10 x 22 |
| | box | (-64.85, 0, -115) | 290.3 x 50 x 10 |
| **holder_out** (fixed) | origin (340, -61, 120) | | |
| | box | (-5.95, 31, -18.25) | 4 x 12 x 15.9 |
| | box | (5.95, 31, -18.25) | 4 x 12 x 15.9 |
| | box | (0, 31, -24.2) | 7.9 x 12 x 4 |
| | box | (0, 31, -12.3) | 7.9 x 12 x 4 |
| | box | (-5.95, 31, 19.75) | 4 x 12 x 15.9 |
| | box | (5.95, 31, 19.75) | 4 x 12 x 15.9 |
| | box | (0, 31, 13.8) | 7.9 x 12 x 4 |
| | box | (0, 31, 25.7) | 7.9 x 12 x 4 |
| | box | (-3.75, 31, 0) | 3 x 12 x 17.5 |
| | box | (3.75, 31, 0) | 3 x 12 x 17.5 |
| | box | (0, 31, -7.25) | 4.5 x 12 x 3 |
| | box | (0, 31, 7.25) | 4.5 x 12 x 3 |
| | box | (0, 31, -70.1) | 20 x 12 x 87.8 |
| | box | (0, 1, -117) | 80 x 60 x 6 |
| **holder_in** (fixed) | origin (340, 80, 120) | | |
| | cylinder, axis y | (0, -4, -18.6) | r 3.1 x 17 |
| | ball | (0, 1.4, -18.6) | r 3.1 |
| | cylinder, axis y | (0, -4, 19.4) | r 3.1 x 17 |
| | ball | (0, 1.4, 19.4) | r 3.1 |
| | box | (0, -16.5, 0) | 16 x 8 x 60 |
| | box | (0, -16.5, -72) | 16 x 8 x 84 |
| | box | (0, -16.5, -117) | 80 x 60 x 6 |
| **rest** (fixed) | origin (225, 35, 0) | | |
| | box | (0, 150, 21.25) | 16 x 90 x 42.5 |
| | box | (0, 150, 3) | 50 x 110 x 6 |
| **outside** (free, 300 g) | origin (340, -61, 120) | | |
| | cylinder, axis y | (0, -2.5, 0) | r 33.35 x 5 |
| | cylinder, axis y | (0, -7.5, 0) | r 24 x 5 |
| | cylinder, axis y | (0, -26, 0) | r 9 x 32 |
| | box | (-86, -47, 0) | 42 x 12 x 16 |
| | box | (-50, -47, 0) | 30 x 12 x 16 |
| | box | (-13, -47, 0) | 44 x 12 x 16 |
| | cylinder, axis y | (0, 18.35, -19) | r 3.2 x 36.7 |
| | ball | (0, 36.7, -19) | r 3.2 |
| | cylinder, axis y | (0, 18.35, 19) | r 3.2 x 36.7 |
| | ball | (0, 36.7, 19) | r 3.2 |
| | box | (0, 17.45, 0) | 2.5 x 34.9 x 9.5 |
| | box | (0, 36.9, 0) | 1.5 x 4 x 6.5 |
| **inside** (free, 100 g) | origin (340, 80, 120) | | |
| | box | (-13.5, 3, 0) | 20 x 6 x 47 |
| | box | (13.5, 3, 0) | 20 x 6 x 47 |
| | box | (0, 3, -23) | 7 x 6 x 1 |
| | box | (0, 3, -10.33) | 7 x 6 x 10.35 |
| | box | (0, 3, 23) | 7 x 6 x 1 |
| | box | (0, 3, 10.33) | 7 x 6 x 10.35 |
| | box | (-2.58, 3, 0) | 1.85 x 6 x 10.3 |
| | box | (2.58, 3, 0) | 1.85 x 6 x 10.3 |
| | cylinder, axis y | (0, 7.5, 0) | r 33.35 x 3 |
| | box | (0, 26.5, 0) | 24 x 35 x 20 |
| | cylinder, axis y | (0, 10.2, -19) | r 4 x 2.4 |
| | cylinder, axis y | (0, 10.2, 19) | r 4 x 2.4 |
| **driver** (free, 150 g) | origin (225, 35, 60) | | |
| | cylinder, axis y | (0, 3, 0) | r 2 x 6 |
| | cylinder, axis y | (0, 15.5, 0) | r 3.18 x 19 |
| | cylinder, axis y | (0, 60, 0) | r 3.5 x 70 |
| | cylinder, axis y | (0, 97.5, 0) | r 10 x 5 |
| | box | (0, 150, 0) | 26 x 100 x 35 |

Frames: the door's origin is the cross bore's centre on its centre plane; the outside half's is its rose's back face on
the spindle (the spindle +y, the lever -x); the base's is its back face on the hub's axis (hub +y); the screwdriver's is
the bit's tip (handle +y).

## Demonstration

The scripted demonstration (`demo()`) plans from the layout and reads the simulator (allowed for scripted
demonstrations); each read prints a `LOCKSET_READ` line.

1. Right: turns to a level side reach, pinches the lever's top and bottom, draws the posts off the holder, carries the
   half in front of the bore, reads its droop and offset in the hand and corrects them, pushes the posts 3 mm into the
   latch's holes, reads where the blade's tip is and lines it up with the cam's slot (the posts' play allows it),
   pushes the blade and posts through the latch and presses the rose on the door, then reads the lever's angle again
   and holds it level.
2. Left: comes in beside the hub, slides its pads either side of it, draws the base off its pegs, waits for the outside
   half, reads both halves, lines the base up and pushes it over the blade and posts, eases its pinch and push, lets
   go, draws its open fingers back off the hub and returns to the ready pose (its wrist turns over between the side
   reach and the screwdriver).
3. Left: grips the screwdriver across its handle's flats from above, lifts it, reads the bit's line against each screw,
   puts the bit in the screw's recess and drives it with 14 strokes of 70
   degrees (2.4 s each turning, 1.4 s back), turning the hand about the screwdriver's
   axis; then the other screw. It lays the screwdriver back on its rest. The arms' joints are soft position servos that
   trail a moving target and sag differently at each pitch, so during the strokes the demonstration feeds each joint's
   motion forward (it sets each joint target so the servo ends the control step where the plan has it) and, at each of
   a stroke's five poses, reads where the bit's tip is against the screw's axis and corrects that pose on the next
   stroke; without that the hand ran off the screw's axis.
4. Right lets go of the lever only after both screws are snug, and both arms park.

## Fidelity and approximations

Rigid bodies; our model, not PhysX threads. Screw shanks are not simulated: a light spring keeps the base on the posts. The bit drives a screw while its tip is within 1.5 mm of the axis; forward turns count, turns back slip (the ratchet). A caught screw draws the halves together as it turns; two turns make it snug, where a real screw takes about a dozen. At the first snug screw the set is fixed where it is, as hand-tight screws hold it. The latch is fixed; post play and masses are estimates.

- Unmeasured and estimated: the posts' spacing (38 mm), diameter and length, the latch's post holes and
  the play in them, the blade's size, the hub's size, the parts' masses and every friction; the unheld droop follows from
  the play and must be measured. Lever length and projection and rose diameter are the maker's published sizes.
- The latch's bolt and cam are not worked here (the lever set's own spring and cam are not modelled); the door section,
  the latch and the holders are fixed.

## Dimension sources

- Kwikset passage lever set: lever 4-7/32 in long, 2-3/32 in projection, 2-5/8 in rose (maker's and retailers'
  listings); door prep 2-1/8 in cross bore, 1 in edge bore, 2-3/8 in backset, doors 1-3/8 in to 1-3/4 in.
- Kwikset lever installation guides: "Install interior lever base ... Push up on lever base while tightening screws to
  keep latch horizontal and centered", then the interior lever with its set screw.
- Schlage lever installation: "Make sure the chassis is straight on the door before tightening the screws".

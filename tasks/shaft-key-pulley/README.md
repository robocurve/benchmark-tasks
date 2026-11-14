# Fit a keyed pulley on a shaft

A bimanual fitting task at true scale. A 25 mm steel shaft turns freely in a pillow-block bearing screwed to the table;
a hand crank on its left end hangs down, so at rest the keyway in its right end faces the robot. A DIN 6885 parallel
key (8 x 7 x 40 mm, yellow) lies on the table, and an 80 mm aluminium V-belt pulley stands in a grey cradle on the
right, its keyway turned 35 degrees away from the top. The goal: the key in the shaft's keyway, the pulley slid onto
the shaft over it up against the shoulder, and both hands off.

Why it takes two hands: the shaft is a free pendulum. Turned so its keyway faces up and let go, the crank swings back
down and the keyway turns to face the robot; a key laid in it falls out, and the pulley's keyway no longer meets the
key. One hand has to hold the shaft by its crank while the other fits the key and the pulley. The fits are the real
ones: 0.1 mm a side between the key and either keyway, 0.25 mm a side between the shaft and the bore, so the pulley
goes on only coaxial with the shaft and with its keyway lined up on the key to within about a degree.

Millwrights, industrial machinery mechanics and general maintenance workers fit keyed pulleys, sprockets and couplings
on shafts as everyday work: the key goes into the shaft's keyway first, the shaft is held so it can't turn, and the
hub's keyway is lined up with the key before the hub is slid home against the shoulder.

| File | What it is |
|---|---|
| `shaft_key_pulley.py` | The `yam_arena` task file: scene, step scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/shaft_stand.usda` | The stand (fixed), the shaft on a damped revolute joint, and the crank handle's spinning sleeve: one articulation |
| `assets/shaft_key.usda` | The 8 x 7 x 40 mm parallel key, one rigid body |
| `assets/pulley.usda` | The 80 mm keyed V-belt pulley, one rigid body |
| `assets/pulley_cradle.usda` | The cradle the pulley stands in (fixed) |
| `test_shaft_key_pulley.py` | Offline checks: the file loads, every scoring outcome, the 1 s full-success timing, both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

All created for this task by its authors, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Units are
metres and kilograms, Z up. Every collider is a box, a cylinder or a small convex mesh (at most 64 points): the keyed
seat, the bore and both keyways are built from convex pieces, with no SDF or triangle-mesh collision. Sizes of the key
and keyways follow DIN 6885-1 for a 25 mm shaft (b x h = 8 x 7 mm, t1 = 4.0 mm, t2 = 3.3 mm).

- **Shaft and stand:** a blue cast-iron pedestal (2.6 kg, fixed) carrying a pillow-block bearing, the shaft's axis
  65 mm above the table. Steel shaft, 25 mm, 804 g with the crank: a 36 mm shoulder collar, then a 46 mm pulley seat to
  the shaft's end (3 mm end chamfer). Keyway 8.2 mm wide, 4.0 mm deep (floor 8.5 mm above the axis), from 4 mm past
  the shoulder to the open end (42 mm long), 1 mm entry chamfers on its edges. The shaft turns freely (no limits,
  0.004 N m s/rad of damping). Crank: 45 mm radius, a red 18 mm square handle sleeve (30 mm long) that spins on its pin.
- **Key:** DIN 6885 form A parallel key, 8 x 7 x 40 mm, zinc-plated steel, 0.5 mm chamfers, 17.4 g.
- **Pulley:** a single-groove V-belt pulley in aluminium, 80 mm across, 20 mm rim (12 mm V groove) joined to the bore by
  four spokes, a hub 50 mm across standing 30 mm proud of the rim with two flats 42 mm apart; bore 25.5 mm with a 1 mm
  entry chamfer; keyway 8.2 mm wide with its roof 15.8 mm from the axis (t2 = 3.3 mm, 0.3 mm above a seated key),
  cut through the hub. 217 g (its modelled volume, 80.4 cm3, in aluminium). A red line on the hub's end and the rim's
  face marks the keyway.
- **Cradle:** printed PLA, 76 x 44 mm, two sloping rests under the rim and a fence either side of it (1.5 mm of axial
  play), fixed to the table.
- **Fits:** 0.1 mm a side between the key and each keyway (8.2 mm keyways for the 8 mm key), 0.25 mm radially between
  the shaft and the bore, 0.3 mm between a seated key and the pulley keyway's roof. A sliding fit: the standard's
  N9/JS9 keyways are a light tap fit, and production bores are H7 (a few hundredths of a millimetre).
- **Friction (static / dynamic):** steel 0.45 / 0.35, key 0.5 / 0.4, aluminium 0.5 / 0.4, PLA 0.45 / 0.4, crank
  sleeve 0.9 / 0.8.

## Run it

On the YAM Isaac Lab-Arena station, from this folder, with the Arena environment active. The instruction is
`task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')

# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=shaft_key_pulley.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/shaft-key-pulley-dream

# Scripted demo (both arms), 1,300 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=shaft_key_pulley.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1300 --store-frames --log-dir ~/yam-eval/shaft-key-pulley-demo
```

The job log prints a `[shaft_key_pulley] t=...  steps done: ...` line when a step is done or undone, plus a trace every
second: the shaft's angle, the key's place in the shaft's keyway (depth off its floor, off its middle line, along the
shaft, tilt), the pulley's place on the shaft (its face along the shaft, its axis off the shaft's, tilt) and the key
in the pulley's frame.

**The demo.** Open loop: a fixed waypoint script that never reads a simulated pose. It reaches 1.0 at step 1145:
the key is in the keyway at 13.5 s, the pulley is started on the shaft at 38.7 s, home against the shoulder at 48.1 s,
and both hands are clear at 56.8 s (step 1125), from when every success condition holds for the full second. A trial takes about 10 minutes on an RTX A4000 (1,300 steps), including
startup.
- Left arm: takes the crank handle's sleeve from above, its body leaning 25 degrees outward (clear of the crank arm),
  and swings the crank up toward the robot a quarter turn and 2 degrees more (the hand gives that much under the
  crank's weight): the keyway faces up. It holds the crank there until the right hand is done, then lets the crank
  back down, opens and lifts away. The sleeve spins on its pin, so the gripper keeps one rotation all the way round.
- Right arm, the key: picks it off the table from above, fingers across it (the grip squares a key lying up to
  3 degrees askew), carries it over the keyway, comes in 6 mm from the side (so the arm's joint friction always
  leaves it short on the same side), lowers it until its underside is 2.5 mm above the keyway's floor and opens. It
  then closes its fingers and presses the key's top down 4 mm past its seat (~2 N), seating a key that landed canted.
  The key is aimed 2 mm toward the shaft's open end: the keyway is only 2 mm longer than the key.
- Right arm, the pulley: grips the two flats on the hub, the gripper tilted 35 degrees with the cradled pulley (the
  flats stop the pulley turning in the grip, so the keyway stays where the hand puts it). It lifts the pulley out of
  the cradle, turns it upright (keyway up, lined up with the key), holds it in line with the shaft 10 mm past its end
  and slides it on until its face is 30 mm onto the seat (the key well into the hub's keyway, the pulley's weight over
  the shaft). It lets go, sets its fingers either side of the bore behind the hub's end and pushes the pulley home
  against the shoulder, then backs off.
- Why the push: carried on in the hand, the pulley binds in its bore about 15 mm short of the shoulder. The arm's wrist
  drives are soft (10 N m/rad), so the axial push tips the held pulley a degree or two, more than the 0.25 mm fit
  allows over its 50 mm length. Pushed from behind, the free pulley follows the shaft.
- Measured offsets the demo compensates: the right arm gives about 6 mm under the pulley held out over the shaft, so
  the slide is aimed 6 mm up (`SLIDE_AIM_UP`); in the hand the pulley's face sits about 3.8 mm to one side and its axis
  is turned about 1.8 degrees about the vertical (it pivots a little in the pads during the lift and turn), so the
  slide is aimed 3.8 mm across and turned back 1.8 degrees (`SLIDE_AIM_SIDE`, `SLIDE_YAW`); the arm is soft (about
  200 N/m) along the push, so the push is commanded 20 mm past the shoulder (`PRESS`, ~4 N); the hand holding the
  crank gives 2 degrees (`CRANK_PAST`). The physics is deterministic, so these offsets are fixed numbers.

## Instruction

The goal only: what must end where, and that the hands let go. It does not say which hand does what, that the shaft
must be turned and held, or where to grip the pulley.

## Scoring

Four steps, **0.2 each**, judged as things stand.

| Step | Done when |
|---|---|
| 1. Key in the shaft's keyway | the key lies in the keyway, its underside within 0.5 mm of the keyway's floor, its centre within 0.5 mm of the keyway's middle line (the walls allow 0.1 mm), lined up with the shaft within 10 degrees, its ends inside the keyway (at most 4 mm past the shaft's open end); no gripper holds it |
| 2. Pulley started on the shaft | with step 1, the pulley's bore is around the shaft (axes within 1 mm and 3 degrees), its face at least 10 mm onto the seat |
| 3. Pulley home over the key | with step 2, the pulley's face is within 2 mm of the shoulder and the key's centre lies in the pulley's keyway (at least 9 mm out from the bore's axis toward it and within 1.5 mm of its middle line, inside the pulley's length) |
| 4. Released | with step 3, no gripper holds the pulley, and both grippers' pads are at least 3 cm from the key, the pulley and the crank handle |

- **1.0 (success, ends the run):** every full-success condition has held together, without a break, for 1 s: all four
  steps (so both hands have let go and are clear), nothing off the table, and the key, the pulley and the shaft each
  within 2 mm and 2 degrees of where they were when that second began. Where the arms end up does not matter.
- **0.2 per step, at most 0.8:** otherwise, including while that second is being timed.
- The 1 s is timed in the task file itself: every control step, `_Progress` notes when the full-success conditions
  first held together (`met_since`) and the parts' poses then, and any lapse (a step undone, a part off the table, the
  key, the pulley or the shaft moved more than 2 mm or turned more than 2 degrees from those poses) clears it, so the
  second starts over. `success()` and `score()` return 1.0 only once it is complete; `SUCCESS_HOLD_S` is 0, so the run
  ends on that same step rather than one second later. The tests check that: no 1.0 after 0.95 s and 1.0 at 1 s; a
  lapse (a gripper back within 3 cm, the shaft turned 3 degrees) restarts the second; and 1.0 is never reached over
  3 s while any one condition is missing (key never fitted or still held, pulley short of the shoulder, its keyway off
  the key, pulley or crank still held, a gripper within 3 cm, parts moving, pulley off the table).
- **0** if the key or the pulley has fallen off the table, or the state is not finite.
- A pulley pushed home without the key, or with its keyway off the key, earns nothing for steps 2 and 3. The shaft
  may end at any angle: with the pulley on, the key is held in both keyways whichever way the keyway faces.
- Runs in simulation (the demo changed at run time, the task file untouched), all 0.0: doing nothing; the left hand
  idle (the keyway faces the robot, the key slides off the shaft and the pulley goes on without it); fitting the pulley
  without the key (home against the shoulder, but no key); carrying the pulley over without turning its keyway up (it
  jams, tilted, on the shaft's end).

## Reset

Turn the shaft until the crank hangs down. Slide the pulley off, take the key out and lay it on its mark, along the
shaft. Stand the pulley in its cradle, hub toward the robot's right, its keyway mark turned 35 degrees toward the
robot from the top.

## Real station

Standard parts at the simulated sizes: a DIN 6885 A 8 x 7 x 40 steel key; a 25 mm steel shaft with a DIN 6885 keyway
milled 8.2 mm wide and 4.0 mm deep, 42 mm long and open at the end, behind a shoulder; an 80 mm aluminium
single-groove pulley with a 50 mm hub, bored 25.5 mm and keyed 8.2 mm (t2 3.3 mm). The keyways are opened 0.2 mm and
the bore 0.5 mm past a production fit so the parts slide together without a mallet; the scoring is the same: the key in
both keyways and the pulley's face within 2 mm of the shoulder, checked with a feeler gauge or a photo, and the hands
off the parts. Nothing here is hazardous; mind the pinch point between the crank and the stand.

## Status

Dream-1 score: 0.0 (both arms reached out over the shaft stand and stayed there, the left one nudging the crank once;
neither picked up the key or the pulley). Demo: 1.0 at step 1145. Media: the ID in `task.json`.

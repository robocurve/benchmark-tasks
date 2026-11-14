# Seat a swivelling ball stud square in a knuckle's tapered hole, from a tie rod that swings

A steering repair step at full size. A car's tie rod hangs from the steering rack's inner ball joint and rests on a
block. The ball stud of its outer tie rod end hangs from a ball in the end's socket. The robot lifts the outer end and
seats the stud in the tapered hole of a fixed steering knuckle. The source is a tie rod installation sheet: "Insert a
stud into the tapered hole of the Steering Knuckle. Install the Castle Nut loosely on the thread." ([ATRO Steer King
Universal Tie Rods, Installation Instructions, step 7](https://www.atrobushing.com/documents/EN-Steer-King-Tie-Rods-Installation-Instructions.pdf);
a truck tie rod there, a car's here). Service technicians do this step. The same sheet treats a stud that can rock in
its hole as a safety fault: the knuckle must be replaced, and using it "may result in a stud fracture and partial loss
of steering."

The hard part is physical. The stud is a peg on a ball joint at the end of a 36 cm rod that itself turns on a ball
joint. Held by the outer end, the stud swivels on its ball, up to 20 degrees each way about two axes: it lags behind
every move and settles back toward hanging straight. Its threaded tip is 71 mm below the ball, so each degree of lean
moves the tip 1.2 mm. The thread is 13.5 mm across and the hole 18.0 mm at its mouth and 14.7 mm at its bottom, so the
stud has 2.3 mm of play at the mouth and 0.6 mm at the bottom. The outer end moves on a sphere around the inner joint,
so it cannot go straight down: lowering it also moves it along the rod. Held still 3.5 mm off the hole's axis and
lowered, the thread caught the edge of the mouth and the stud tipped over on its ball until it stood 16 degrees from
upright, jammed: score 0. Gripping the stud itself instead of the housing is allowed, but the hand must let go and
move clear before the trial can end at full score.

## The scripted demo
`demo()` uses the right arm. It grips the housing from above, lifts the outer end 30 mm off its block and swings it
out 11 degrees on its arc in four steps. It waits 2 s for the stud to stop swinging, lowers the end until the tip is
4 mm over the hole and waits again, then lowers the stud straight down into the taper to about 2.4 mm short of home.
There it lets go, the stud drops onto its seat, and the arm backs off 6 cm. The stud is never seated while the fingers
hold it, and the open pads stand 27 mm off the housing's sides (read as about 25 mm, see below), so the 1 s hold
starts only after they let go. It plays
fixed waypoints from the known layout and reads no joint state.

## Run a trial
On a machine set up with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s
`setup.sh`), with a Dream API key in `~/.dreamlabs/credentials`:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tie_rod_end.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lift the outer end of the tie rod off its wooden block, hold its swinging ball stud upright over the steering knuckle's tapered hole, push the stud straight down until its taper seats in the hole, then let go and lift the gripper clear." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/tie-rod-end-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tie_rod_end.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lift the outer end of the tie rod off its wooden block, hold its swinging ball stud upright over the steering knuckle's tapered hole, push the stud straight down until its taper seats in the hole, then let go and lift the gripper clear." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/tie-rod-end-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/tie-rod-end-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends with 1.0 when `success()` has held for 1 s; otherwise it scores
`score()` at the last step.

## Home depth, as on the real parts
Home is where the matched stud stops when it is pushed into the clean, dry tapered hole by hand. That is the test fit
in MOOG's ball joint installation tip ([Proper ball joint stud nut torquing procedure, MG211049, step 1](https://www.moogparts.com/content/dam/marketing/North-America/moog/pdf/tech-tips/proper-ball-joint-stud-nut-torquing-tech-tip.pdf)):
the technician cleans the knuckle's tapered hole and test-fits the stud by hand, the stud "should seat firmly without
rocking", and only its thread should come out through the knuckle. MOOG's taper check names outer tie rod ends among
the tapered-stud parts it covers ([Checking Stud Taper Fitment on Chassis Parts](https://www.moogparts.com/technical/bulletins/tech-tips/checking-stud-taper-fitment-chassis-parts.html)).
Neither source gives a depth number or a gauge; the reading below is ours.

The depth is read as the thread's protrusion `P`: the distance from the steering arm's bottom face (where the hole's
small end comes out) down to the thread's end face, along the hole's axis.

On a real rig, with the matched outer tie rod end and knuckle:
1. Clean and dry the stud's taper and the hole (both sources say so).
2. Push the stud in by hand until it stops. Check that it does not rock.
3. Stand a caliper's depth rod on the arm's bottom face beside the thread and read to the thread's end face. Pull the
   stud, re-insert it, and read again, three times. The mean is `P_home` for those parts.
4. After a trial, read `P` the same way without pushing the stud. It is home when `P` is within 0.5 mm of `P_home`.

The scorer computes the same reading from the sim's geometry: `P` = the arm's bottom face height minus the stud's tip
height. For this sim's matched stud and hole, `P_home` = 22.6 mm: the 22.0 mm thread [EST] plus 0.6 mm, because the
taper here is four steps whose corners sit 0.05 mm inside the cone, so it stops 0.05 / 0.0855 = 0.58 mm past flush
[ARITH]. On ideal real parts, flush and with only thread showing, `P_home` would be the thread's length.

Why 0.5 mm: the taper loses 0.0855 mm of radius per mm of length [ARITH, NAPA 269-3444 sizes], so a stud held 0.5 mm
high stands about 0.04 mm off the hole's wall all round. It is not in metal contact, so something holds it off its
seat: a burr, grease, a cocked entry, or a hand. The window allows for the hand-push spread on the real parts and the
contact offsets in the sim [JUDGEMENT].

## Let go and clear
Seated is not enough for full score: no part of either arm may be within 10 mm of the tie rod, its outer end's body
and housing, or the stud. A hand that still grips the housing, props the rod from below or presses the stud down
counts as holding it.

The scorer gets object states only, so a hook reads the arms. `mechanism.on_step` registers a function that runs
before every scored step, with the simulator's environment (the same technique as `on_step` in
`tasks/analytical-balance/labkit.py`: it wraps the embodiment's `_task_step`, which reads the object states and calls
`success()` and `score()`). `tie_rod_end.read_arms` reads the world pose of every body of `left_robot` and
`right_robot` (`base`, `link1` to `link5`, `gripper`, `tip_left`, `tip_right`). `arm_clearance` places the arm's
collision hulls (the URDF's convex pieces, `yam_arena.analysis.link_hulls`) at those poses and returns a lower bound
on their distance to points laid at most 2 mm apart over the tie rod's and stud's colliders: for each point, the larger
of its distance to a hull's bounding box and its distance beyond the hull's face planes, minus the 2 mm spacing. Both
distances are at most the true distance to a convex hull, so the reading is never more than the true gap; the tests
check it against the nearest hull vertex's exact distance. It errs only one way, low: the 2 mm spacing, plus more near
hull edges. Over 57 random open-hand poses near the housing it read 2.0 to 2.6 mm under the nearest vertex's exact
distance, so there a hand about 12.5 mm away can read under 10 mm and count as not clear. Before the first step after
a reset there is no reading, and that counts as not clear.

Why 10 mm: a hand nearer than that may be touching (the arm lags its target 2 to 5 mm when moving, and the
colliders' contact offsets add 1 to 2 mm), and a 10 mm gap shows in a photo of a real trial. Fully open pads stand 27
mm off the housing's sides [ARITH: (94 - 40) / 2], so opening the fingers is enough; the demo also backs off 6 cm.

## Scoring
| Condition | Number | Check on a real bench |
|---|---|---|
| seated | thread protrusion within 0.5 mm of home (22.6 mm here); the taper's large end within 1 mm of the hole's axis; the stud's axis within 2 degrees of the hole's | caliper depth rod on the arm's bottom face; the stud does not rock; a square on the steering arm |
| clear | no part of either arm within 10 mm of the tie rod, its outer end or the stud | a photo; a 10 mm rod passes between |
| in the hole | the threaded tip at least 2 mm below the hole's top, the stud's axis within 2.42 mm of the hole's at the hole's top (so the 13.5 mm thread is inside the 18.0 mm mouth), protrusion no more than home plus 0.5 mm, within 2 degrees of upright | the thread is in the hole, square |
| still | the stud's taper and the tie rod's outer end under 0.01 m/s, both under 0.5 rad/s | |

| Score | Condition |
|---|---|
| 1.0 | seated, still and clear for 1 s (ends the trial) |
| 0.8 | seated, still and clear at the last step, not yet for 1 s |
| 0.6 | seated at the last step, but still moving or a hand within 10 mm |
| 0.3 | in the hole, not seated |
| 0 | beside the hole, on its rim, cocked more than 2 degrees, dropped, or at the start |

`score()` never returns 1.0; only `success()` holding for 1 s does. A stud cocked more than 2 degrees scores nothing,
in the hole or not: a stud wedged in the taper can sit cocked, and square within 2 degrees it is at most about 3 mm
from home. A thread end resting on the rim dips under 2 mm into the mouth, so it scores nothing either.

## Runtime and seeds
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
- Dream-1 gets 2,400 control steps (2 minutes at 20 Hz); the scripted demo reaches success at 21.35 s (step 427), and scores 1.0 at start delays 0, 0.05 and 0.10 s (steps 427, 428, 429; inspect-robots 1.0.0). In those runs the stud reaches home 0.4 s before the fingers finish opening; the 1 s hold, and full credit, start only once both arms are 10 mm clear. A run whose hand never opens scores 0.6. Dream-1 on this fixture: 0.
- `python -m pytest -q test_tie_rod_end.py` checks the scoring and the demo's reach without the simulator.
- `mechanism.py` builds the rack, tie rod and stud as one jointed body and runs the arm-reading hook before each
  scored step. `make_meshes.py` writes the knuckle's USD.

## Fidelity
The stud's taper and the hole are from a passenger-car outer tie rod end (NAPA 269-3444: taper 0.71 in to 0.58 in over
0.76 in, M14 stud thread, M16 rod thread). The taper is four short cylinders just inside the cone and the hole is 24
flat segments, so the stud seats 0.6 mm deeper than flush and 0.6 mm of its last taper step shows under the arm;
MOOG's test fit expects only thread there on real matched parts. Each ball joint is two hinges at right angles with light
damping; the stud's 20 degree stops stand in for its neck meeting the socket's mouth. The stud lags and settles rather
than swinging freely, but its ball has no preload. A real socket's bearings "maintain preload" (the same ATRO sheet,
benefit 2), so a real stud is stiffer on its ball than this one. The moving parts weigh 170 g in all; solid steel parts
of this size weigh about 1.2 kg. The knuckle is fixed and has rubber-like friction (1.2 static, 1.0 dynamic); the stud
has 0.5 and 0.4. There is no castle nut and no thread form. Estimated, not from a source: the rod's length (36 cm), the
ball to the taper (30 mm), the 20 degree swivel, the thread's length (22 mm), the housing, the rack and the knuckle's
body.

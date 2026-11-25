# Take a Prius radiator support cover off its three push clips

The robot stands in front of a 2010-2015 Toyota Prius with the hood open. The radiator support opening cover is held
down by three two-piece push clips and two claws. The job is the first step of front bumper removal in Toyota's
repair manual: take the cover off, keeping the clips for refitting. Each clip only lets go once its centre pin has
been pried up; prying the clip body first snaps its legs. Two Lisle 35460 double-ended clip lifters lie on the fender
cover; the narrow fork is made for lifting the pin.

## Run it

    inspect-robots run --embodiment yam_arena --policy dream -E task=tasks/take-the-radiator-support-cover-off-its-push-clips/task.py \
        -E object_jitter=0 --max-steps 1800 --epochs 2 \
        "Take the radiator support opening cover off and lay it on the fender cover, so the front bumper can come off next. The three clips will be refitted, so keep them undamaged and put them in the parts tray."

Use `--policy yam_scripted` for the scripted demo. `--epochs 2` runs lot A then lot B (trial k uses lot k mod 2);
`TASK_LOT=A` or `TASK_LOT=B` in the environment pins every trial to one lot. The layout is fixed: run with
`-E object_jitter=0`; no seed changes the scene. A trial takes about 4 minutes of simulation on an L40S.

Files: `task.py` (scene, scorer and demo), `near_misses.py` (offline scorer checks: `python near_misses.py`),
`parts.py` (multi-shape and jointed parts; this copy can push on one link of a jointed part, used for the clips'
legs), `lots.py` (the two layouts), `tilt.py` (tilted grips in the demo only). The simulator is yam-isaac-lab-arena
(the program's YAM embodiment for Isaac Lab-Arena).

## Why it matters

- Automotive service technicians and mechanics (O*NET 49-3023.00, about 800,000 US workers) and automotive body and
  related repairers (49-3021.00, about 160,000) take off plastic covers, shields and trim held by push clips on most
  front-end, headlamp, radiator and bumper jobs. One of the body repairers' O*NET task statements is to "remove
  ... trim to gain access to vehicle bodies and fenders".
- Toyota 2010 Prius repair manual, front bumper removal, step 1, "Remove radiator support opening cover": "(a) Remove the
  3 clips." then "(b) Disengage the 2 claws and remove the radiator support opening cover."
- Lisle's description of the 35460: "Smaller opening works for lifting the center pin and the wider opening lifts the
  fastener from the panel."
- What is hard for a robot: a hand tool held by its grip has to slide a 1.5 mm fork into the 2.5 mm gap under a 10 mm
  pin head, raise the pin, then draw the clip out and carry it on the fork without dropping it, three times, then free
  the claws and lift a 900 mm panel off with two hands. The wrong order (levering the clip body, or lifting the cover,
  while the pin is down) breaks a clip and scores 0 for the whole trial.

## Scene

See `setup` in task.json for every object, size and place. Real products: Toyota 53289-47010 radiator support opening
cover; Toyota-type two-piece push clips for a 7 mm hole (20 mm head); Lisle 35460 clip lifter (8 in overall, 3-1/2 in
grip, zinc finish, $17 list on the maker's page). Estimates, marked in task.json: the cover's size and mass, the clip
places along the cover, the clip's internal dimensions, the lifter's fork shapes, and all forces.

### Real-trial kit (about $150 without the car)

| Item | Source | Approx. price |
|---|---|---|
| Radiator support opening cover, 2010-2015 Prius (used) | salvage yard or online used-parts listing | $30-50 |
| Push clips for a 7 mm hole, Toyota type (bag of 20-50) | any auto-parts store | $8 |
| Two Lisle 35460 double-ended clip lifters | Lisle / tool distributors | $34 |
| Plastic parts tray about 90 x 240 mm, fender cover | hardware store | $20 |
| The car: any 2010-2015 Prius with the hood open, or a used upper radiator support section clamped at bench height | | $0-100 |

The trial is safe: the engine is off, the hood is propped, and the cover and clips are plastic.

## Scoring

Whole-trial zeros (latched; the simulator checks the clip rule at every physics step):
- a clip body rises 2 mm or more above its seat before its centre pin has first come 4 mm up (real trial: front video
  against a millimetre scale card behind the clip row, and afterwards a snapped or stress-whitened leg under a 10x loupe);
- the cover is lifted 5 mm or more at a clip whose pin has not yet come 4 mm up (hole torn or clip pulled through);
- any object drops more than 10 mm below the bumper top (off the car).

Ladder (end of trial):

| Score | State |
|---|---|
| 0.2 per clip | clip in the tray (centre inside the walls, lowest point on the floor within 4 mm, tray tilted under 18 degrees), no gripper within 5 mm, no lifter within 2 mm, moving under 20 mm/s |
| 0.8 | all three clips in the tray, and the cover wholly at least 2 mm in front of the support's front edge, lying on the fender cover (lowest point within 15 mm), no gripper within 5 mm, moving under 20 mm/s |
| 1.0 | that, with both grippers at least 30 mm from the cover, clips, lifters and tray, and everything moving under 1 mm for 1 s |

In the simulator (`task.py`): `clip_check()` holds the zero rules, `released_clips()` counts the clips, `cover_off()`
tests the cover, `observe()` keeps the hands-clear test (face-sampled gripper hulls, 3 mm spacing) and the 1 s hold,
and `score()` applies the ladder. Unknown hand or object state fails closed (0).

### Shortcuts and what they score

| Shortcut | Score |
|---|---|
| Lever or pull a clip body (wide fork, gripper pinch) with its pin down | 0 for the trial (legs snapped) |
| Lift or slide the cover before the clips are out | 0 for the trial (hole torn, or held by the clips) |
| Leave a clip on the lifter over the tray | that clip does not count |
| Drop a clip on the fender cover or the radiator | that clip does not count |
| Leave the cover on the support with all clips out | 0.6 |
| Hold the cover up off the support | 0.6 (not lying on the fender cover) |

`near_misses.py` checks each of these offline (25 checks). The instruction does not require the lifter: fingers that
lift a pin first are allowed and score the same. The demo uses the lifter, as technicians do.

## How this differs from close tasks

- Fitting a push clip (PR 1986, TK-94YSKR, and PR 2667, TK-VKW47W): those press one clip into a hole. Here
  the clips come out, in the order the clip's lock sets, with a tool, and the cover then has to come off its claws.
- Lever a paint-tin lid (accepted): one flat tool under one rim. Here a forked tool goes under a small pin head in a
  2.5 mm gap, and lifting the wrong part (the clip body) is a failure.
- Release a push-fit fitting and pull the pipe out (PR 662, TK-462BUT): a release-then-pull order by hand on a
  plumbing fitting; here the release is a tool operation on a trim fastener, three times, then a two-handed panel lift.
- Change an engine air filter (PR 2492, TK-N768KN): under-hood work with over-centre clips opened by hand.

## Fidelity notes

The clip's legs are a declared latch (stated in task.json): `drive_clips()` pushes each clip body down with 25 N while
its pin is less than 4 mm up and the body is within 6 mm of its seat, and with 1 N after the pin is up; the pin is on
a slider with a 200 N/m detent spring toward its down position, and once it passes 4 mm the spring's rest point moves
to the 6 mm stop, so an unlocked pin stays up (on the real clip a raised pin stays where it is pulled). These forces are estimates. The clip parts
are simulated heavier than the real ones (10 g body, 6 g pin; real about 2 g in all) for solver stability. The clip's
legs are drawn but do not collide (the declared force holds the clip); the holes are 8 mm square for a 6.5 mm square
shank (real: round 7 mm hole); the support's top plate is 3 mm thick (real about 1.4 mm steel); the lifter's round
vinyl grip is a 20 mm square. The pin's 0.8 N release at 4 mm is an estimate for a two-piece clip whose pin a
technician flicks up with a fingernail or tool; the real effort varies by clip and age.

## Demo

`demo()` is open loop from the published layout of each lot (tool and tray places only; no object, joint or force
feedback). Each hand picks up its lifter by the grip, 30 mm from the grip's centre toward the narrow fork. Over each
clip it sets the narrow fork's tips on the clip head beside the pin, slides the fork under the pin head until the
pin's neck is at the fork's throat, lifts 8 mm (the pin comes up its full 6 mm and unlocks the legs), then lifts on,
drawing the clip out of its hole on the fork. It carries the clip over the tray, stands it on the tray floor, lowers
the fork 3 mm clear of the pin head and draws it 45 mm back off the clip. The left hand takes the two left clips, the right hand the right
one. Both lifters are laid back down, then both hands take the cover by its front rib and push it back until its notch
meets the hood latch (about 13 mm; the claws come free after 5 mm), lift it and lay it on the fender cover. The push
is commanded 30 mm, so the arms press the cover gently against the latch (the arm gives way at about 190 N/m).

## Results

- Scripted demo (`--epochs 2`, 1,800-step limit): 1.0 in lot A and 1.0 in lot B; each trial reached full score at
  73.5 s (1,470 steps): all three clips raised pin first and set in the tray, the cover pushed back off its claws,
  lifted and laid on the fender cover, hands clear and everything still for 1 s.
- Dream-1 (same instruction, `--epochs 2`): 0.0 in lot A and 0.0 in lot B; both trials ran to the 1,800-step limit
  without refusing the instruction.

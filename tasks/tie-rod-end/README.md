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
upright, jammed: score 0. The scorer does not check the grasp, so holding the stud itself is allowed.

## The scripted demo
`demo()` uses the right arm. It grips the housing from above, lifts the outer end 30 mm off its block and swings it
out 11 degrees on its arc in four steps. It waits 2 s for the stud to stop swinging, lowers the end until the tip is
4 mm over the hole and waits again, then lowers the stud straight down into the taper to about 2 mm short of home.
There it lets go, the stud drops onto its seat, and the arm backs off 6 cm. The stud is never seated while the fingers
hold it, so the 1 s still hold starts only after they let go. It plays fixed waypoints from the known layout and reads
no joint state.

## Run a trial
On a machine set up with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s
`setup.sh`), with a Dream API key in `~/.dreamlabs/credentials`:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tie_rod_end.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lift the outer end of the tie rod off its wooden block, hold its swinging ball stud upright over the steering knuckle's tapered hole, and push the stud straight down until its taper seats in the hole, then let go." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/tie-rod-end-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tie_rod_end.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lift the outer end of the tie rod off its wooden block, hold its swinging ball stud upright over the steering knuckle's tapered hole, and push the stud straight down until its taper seats in the hole, then let go." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/tie-rod-end-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/tie-rod-end-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends with 1.0 when `success()` has held for 1 s; otherwise it scores
`score()` at the last step.

## Scoring
| Condition | Number | Check on a real bench |
|---|---|---|
| seated | the taper's large end within 1 mm of the hole's axis and within 0.5 mm of its home depth (0.6 mm below the hole's top); the stud's axis within 2 degrees of the hole's | the stud does not rock; a square on the steering arm |
| in the hole | the threaded tip at least 2 mm below the hole's top, the stud's axis within 2.42 mm of the hole's at the hole's top (so the 13.5 mm thread is inside the 18.0 mm mouth), no deeper than seated, within 2 degrees of upright | the thread is in the hole, square |
| still | the stud's taper and the tie rod's outer end under 0.01 m/s, both under 0.5 rad/s | |

| Score | Condition |
|---|---|
| 1.0 | seated and still for 1 s (ends the trial) |
| 0.6 | seated at the last step but still moving |
| 0.3 | in the hole, not seated |
| 0 | beside the hole, on its rim, cocked more than 2 degrees, dropped, or at the start |

A stud cocked more than 2 degrees scores nothing, in the hole or not: a stud wedged in the taper can sit cocked, and
square within 2 degrees it is at most about 3 mm from home. A thread end resting on the rim dips under 2 mm into the
mouth, so it scores nothing either.

**The scorer does not check the grasp or the release.** The hooks don't see the grippers, so a stud held seated by
the gripper scores the same as one let go, although the instruction says to let go.

## Runtime and seeds
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
- Dream-1 gets 2,400 control steps (2 minutes at 20 Hz); the scripted demo reaches success at about 21 s.
- `python -m pytest -q test_tie_rod_end.py` checks the scoring and the demo's reach without the simulator.
- `mechanism.py` builds the rack, tie rod and stud as one jointed body. `make_meshes.py` writes the knuckle's USD.

## Fidelity
The stud's taper and the hole are from a passenger-car outer tie rod end (NAPA 269-3444: taper 0.71 in to 0.58 in over
0.76 in, M14 stud thread, M16 rod thread). The taper is four short cylinders just inside the cone and the hole is 24
flat segments, so the stud seats 0.6 mm deeper than flush. Each ball joint is two hinges at right angles with light
damping; the stud's 20 degree stops stand in for its neck meeting the socket's mouth. The stud lags and settles rather
than swinging freely, but its ball has no preload. A real socket's bearings "maintain preload" (the same ATRO sheet,
benefit 2), so a real stud is stiffer on its ball than this one. The moving parts weigh 170 g in all; solid steel parts
of this size weigh about 1.2 kg. The knuckle is fixed and has rubber-like friction (1.2 static, 1.0 dynamic); the stud
has 0.5 and 0.4. There is no castle nut and no thread form. Estimated, not from a source: the rod's length (36 cm), the
ball to the taper (30 mm), the 20 degree swivel, the thread's length (22 mm), the housing, the rack and the knuckle's
body.

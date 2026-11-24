# Reconnect a bathroom-sink pop-up stopper to its pivot rod

A bathroom vanity stands on the table: a stone countertop with an undermount basin, over a cabinet whose door opening
faces the robot. A pop-up drain is installed in the basin's bottom, and its pivot port, under the basin inside the
cabinet, points at the robot through the door opening. The pop-up stopper sits closed in the drain. At the bottom of the stopper's stem is an eye, a ring with a 9 mm hole, and the pivot
rod has to pass through that eye. The eye is inside the drain, 26 mm below the port and turned a quarter turn away from
it, hidden by the drain body. The rod lies on two rests on the table, its inner half inside the cabinet under the basin and its outer
end out through the door opening. The robot is asked to reconnect the stopper to the rod.

When the stopper rests closed, no rod pushed through the port can reach the eye. The stopper has to be lifted about
26 mm by its head, from inside the basin, and turned a quarter turn, and held there while the rod is pushed in through the port and the eye,
under the basin, until its pivot ball reaches the seat washer. That takes both arms. A rod pushed straight in hits the stem. A rod pushed
in under a stopper lifted too far seats its ball and looks finished, but it has not caught the stopper.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It needs `parts.py` (multi-shape parts) and `tilt.py` (tilted grips for the scripted
demo), which sit in this folder and import only NumPy and `yam_arena`. At import `parts.py` patches `yam_arena`'s
scene spawning, step and reset, and `tilt.py` wraps `yam_arena.demo.grip_rotation` so a demo waypoint can lean the
hand; both act in the task's own process only and change nothing on disk. `tilt.py` affects only the scripted demo,
never Dream-1 or the scoring. `near_misses.py` holds near-successes and wrong outcomes, each with the
score it must get. They are plain-Python checks of `score()` and `success()` and need no simulator.

## Run a trial

From this folder, on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Reconnect the drain stopper to its pivot rod."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/stopper-dream

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/stopper-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, and `task.json` describes it in words.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz), about 1 to 2 minutes of wall time
  on an L40S once Isaac Sim has started.
- **Seeds:** none needed. The layout is fixed. Run with `-E object_jitter=0`.

## Geometry

The vanity is built at real size. The countertop is a 30 mm stone slab, 635 mm wide and 560 mm deep (a 25 x 22 inch
top), its top 236 mm above the table and its front edge 200 mm in front of the drain's axis. The basin is a rectangular
undermount basin of white china, 300 mm front to back and 380 mm side to side inside, hung under the countertop's
cut-out, with straight 10 mm walls and a flat 10 mm bottom 130 mm below the countertop's top. The drain body passes
through a hole in the basin's bottom (46 mm square in the model, hidden under the 60 mm flange), its flange resting on
the bottom and a 52 mm brass locknut, 10 mm tall, under it. Below the port, the drain tube runs on down as the
tailpiece to the table top. The cabinet is 610 mm wide and 533 mm deep (a 24 x 21 inch vanity), with 16 mm sides and
back and a 19 mm face frame: two 38 mm stiles and a 50 mm top rail round a door opening 534 mm wide, whose top is
156 mm above the table. The doors are open and not modelled. The vanity stands on the robot's table, so the cabinet
stops at the table top: a real one goes on down about 0.6 m more to the floor, and its countertop would be 0.81 to
0.91 m above the floor, against about 1.0 m here. The basin's bottom is 42 mm above the port's axis, as on a real
pop-up drain, so the drain fixes the vanity's height above the table. In most vanities the pivot port points at the
back wall; here it points at the door opening, so the rod can be reached from the front.

The holes are round. The kit has only solid boxes and cylinders, so each round hole is a ring of overlapping
cylinders ("lobes") whose inner faces form the bore: 14 lobes for the eye and the seat washer, 16 for the spout. The
drain tube is 24 flat staves with a 32 mm bore. The opening through its wall facing the port is 12 mm wide and 12 mm
tall. The pivot ball is a round barrel, 14 mm across and 10 mm long. In the 16 mm spout it lets the rod tilt or swing
about 14 degrees. The stopper's guide fins are 30.5 mm across in the 32 mm bore. They keep the stopper upright and
centred while letting it turn.

## Scoring details the simulator applies

Everything scored can be checked at a real sink from outside the drain. The eye itself is never looked at.

- **Let go:** a `parts.on_step` hook reads both arms' finger and gripper link poses every step. It measures the
  shortest distance from their collision hulls (`yam_arena.analysis`) to every shape of the rod and of the stopper.
  Each distance must be at least 5 mm. Before the first measurement after a reset, nothing is credited, so the check
  fails closed.
- **Still, for 1 s:** both parts move under 1 cm/s and turn under 0.5 rad/s for 20 consecutive control steps, with the
  readings below holding. For the rod, spin about its own centreline is left out: in the simulator the round rod can
  keep rolling slowly in place in the port, at about 1 rad/s, which moves nothing; a swing of the rod is still counted.
- **Readings, taken at the end of that second:**
  - the rod comes out of the spout's bore: its centreline crosses the spout's end face within 5.5 mm of the port's
    axis (the 16 mm bore less the rod's radius);
  - its outer end is at most 199 mm from the spout's end face, measured along the rod. With the ball home on the seat
    washer it is 196 mm, so this allows 3 mm of play. A person reads it with a steel rule laid along the rod; the
    simulator takes it from the rod's pose. Tilting the rod 14 degrees changes the reading by under 0.2 mm;
  - the stopper's head is centred within 4 mm of the drain's axis and tilted at most 6 degrees, and its underside is
    between 1 mm below and 40 mm above the flange top, read with a rule or depth gauge from the basin side. At 40 mm,
    10 mm of the guide fins are still in the bore.
- **Movement test, straight after the readings.** It shows the rod is through the eye. The pivot nut and the lift
  linkage are left out (see `fidelity`), so nothing holds the ball in the port but its seat. The pushes go on the
  rod's outer end, 2 mm in from its tip, where the linkage's clevis strap would clamp on, and on the stopper's knob. In
  the simulator they are steady forces (an Isaac Lab permanent wrench) set by the hook each control step.
  On a real sink: a push-pull gauge on the rod end, a spring scale hooked on the knob, and a depth gauge on the
  stopper's head from the basin.
  1. *Lift, 1 s:* an upward push on the rod's outer end, rising steadily to 0.2 N over 0.5 s, then held for 0.5 s. The
     rod turns about its ball and its inner end goes down. A stopper hung on it through the eye goes down too: it must
     drop at least 3 mm.
  2. *Back, 2 s:* the push eases off steadily to nothing over 1 s, so the rod is lowered, not dropped, then nothing
     is applied for 1 s while the parts come to rest. The rod's inner end has lifted the stopper again: it must have
     risen at least 3 mm.
  3. *Pull, 0.5 s:* a steady upward pull of 1 N on the knob, about twice the stopper's 0.54 N weight. A rod through the
     eye catches it: it may rise at most 10 mm. A stopper on a rod under its eye, or on nothing, lifts straight off; the
     pull stops as soon as the stopper passes 10 mm.
  - A rod beside, over or short of the eye moves nothing in steps 1 and 2. A stopper resting on a rod under its eye
    follows the rod in steps 1 and 2 but lifts off in step 3.
- **Score:** 1.0 when the readings and the test pass. 0.5 when the test passes but the rod's outer end was more than
  199 mm from the spout (ball not home). 0 otherwise. The readings are those taken just before the test, because its
  pushes can shift the parts: in the final demo the ball moved about 2 mm outward during step 3. Once the test has
  passed, the result stands until a gripper comes within 5 mm of either part again. That restarts the check from the
  1 s of stillness. A gripper that comes near during the test abandons it the same way.
- **Diagnostic, not scored:** the run log prints the test's readings, with whether the rod's centreline passes
  through the eye's hole by the parts' poses. Offline (`near_misses.py`, no simulator) the test cannot run. There,
  that pose check stands in for its outcome, and the test's own limits are checked from its readings.

## Scripted demo

The demo runs open loop from the published start layout and reads nothing from the simulator. The left arm grips
straight down; the right arm leans its hand with `tilt.py`.

- **Left arm:** keeps the forward lean of the arm's starting pose (55.5 degrees) while it moves over the basin, so the
  hand clears the countertop, then turns straight down as it goes down into the basin. It opens to 54 mm, pinches the
  stopper's head just above the flange, lifts it 26 mm and turns it a
  quarter turn. It holds the stopper there while the rod goes in, then opens, slides off sideways with its pads clear
  of the head, and moves away.
- **Right arm:** moves above the countertop, still leaning as at the start, to in front of the cabinet, and leans its
  fingertips 60 degrees toward the drain, so the hand reaches under the countertop and the basin through the door
  opening while the wrist and forearm stay in front of them. It keeps that lean from the grip to the release, since
  turning the hand turns the rod in the pads. It grips the rod 115 mm out from the ball, lifts it to the port's height,
  lines it up with the port 6 mm short of the spout, and pushes it straight in to 2 mm past the seat. It holds the rod
  there while the left arm lets go of the stopper, then opens, backs out 110 mm along the rod until its fingertips are
  past the rod's outer end, rises in front of the cabinet and moves away.
- **Both arms** then wait clear of the parts while the scorer runs its movement test.
- **Tuning:** the waypoint heights carry offsets measured in traces for how far each arm sags under its load: 2.5 mm
  for the left arm; for the right, 2.3 mm up and 0.8 mm sideways while lining up, and 3 mm more up and 4 mm more to the
  left at the end of the push, as the arm reaches in.
- **Checked offline** (IK and the arms' collision hulls, no physics): both arms track every waypoint within 0.1 mm,
  no arm link enters the countertop, basin, cabinet, drain body or rests, nothing goes below the table top, and the two
  arms stay at least 70 mm apart. A straight-down grip cannot do the right arm's job here: with the hand straight down
  over the rod, the wrist and forearm reach 60 to 75 mm past the TCP toward the drain, into the countertop and the
  basin. The arms' straight-down reach also tops out about 0.21 m above the table, below the countertop, which is why
  both arms keep their starting forward lean while they move above it. At 60 degrees the hand's palm stays 16 mm above
  the rod's outer half and 10 mm in front of the basin's front wall; at 70 degrees the palm met the rod, and at
  45 degrees the hand hit the basin's underside.

## Measured in simulation

Vanity scene, final demo (`demo-20261115T222420`, 1.0):

- The rod's tip entered the spout within 1 mm of the port's axis and went through the eye; the ball reached its seat.
- Movement test: the stopper dropped 11.4 mm when the rod's end was lifted, rose back 8.8 mm once the push had eased
  off, and rose 8.6 mm under the 1 N pull before the rod caught it. The rod's outer end was 197.2 mm from the spout.
  The test passed at 19.95 s and the trial ended in success.
- Traces that failed on the way, on this scene: a 45-degree hand gripping 165 mm out let the rod turn in the pads and
  drag along the table (0); turning the hand from 45 to 70 degrees after the grip tilted the rod 27 degrees (0); at
  70 degrees the palm landed on the rod (0); with no extra push offset the tip met the seat washer 3 mm off its hole
  (0); with the right hand letting go before the left, the left hand's slide-off drew the ball 4 mm out, so the test
  passed with the outer end at 200.1 mm (0.5). Holding the rod until the stopper is let go fixed that.

The traces below ran on an earlier layout, where the same drain body, stopper and rod stood in a bench stand with no
vanity and the right arm gripped the rod straight down, 100 mm out from the ball. They tested the scorer's movement
test and the wrong outcomes, which do not depend on the vanity.

- The held stopper's eye sat within 0.3 mm of the port's height, and the rod's inner end entered it 1.3 mm off centre
  and went home.
- After both arms let go, the rod's long outer end dropped. Its inner end then held the stopper 28.1 mm open, which is
  what a pop-up rod with no pivot nut does. The rod settled tilted 9 degrees, its outer end 197.8 mm from the spout.
- Movement test, final demo: the stopper dropped 11.2 mm when the rod's end was lifted (the rod swung to 14 degrees
  the other way), rose back 12.0 mm once the push had eased off, and rose 6.5 mm under the 1 N pull before the rod
  caught it. It passed at 17.5 s and the trial ended in success about 1 s later.
- A version with no rest second in step 2 failed a correct repair in one trace: the rod's end swung back at the end of
  the ease-off and the pull began while the stopper was still bouncing (rise back read as 5.9 mm, pull rise 11.9 mm).
  Hence the 1 s rest before the reading. Pushing the rod 4 mm past the seat instead of 2 mm did not seat the ball
  further (198.4 mm against 197.8 mm), so the demo keeps 2 mm.
- A first version took the push away at once in step 2. The rod's end then fell freely and knocked the ball 3 mm
  outward. That is why the push now eases off, and why the readings are taken before the test.
- Wrong-outcome traces (variant demos in a scratch copy of the folder, not submitted):
  - stopper lifted 12 mm too far, so the rod went in under the eye: the stopper ended wedged on the rod's tip, 40.2 mm
    up, 7.4 mm off centre and tilted about 10 degrees, creeping at just over 1 cm/s. It failed the guide and stillness
    checks, so the movement test never started: 0. With those limits loosened in the scratch copy it still never
    stopped long enough for the test to start.
  - stopper lifted but not turned: the rod stopped against the edge of the eye's ring, the stopper dropped closed, and
    the ball ended about 22 mm out of its seat, the rod resting on the spout's lip. It failed the readings (the rod 6 mm
    off the port's axis at the spout's face) and the rod never held still for 1 s, so the test never started: 0.
  - So in these two traces the test's limits were not reached. Its readings for a rod beside or under the eye are
    checked offline in `near_misses.py`.
- An earlier layout, with the port pointing to the robot's right, made the right arm's forearm hit the left gripper
  during the push. That is why the port points at the robot.

## Other ways to succeed, and what scores 0

- **Also full score:**
  - turning the stopper either way, since the eye's ring is symmetric;
  - either arm doing either job;
  - inserting the rod at a tilt, if the eye is lined up with it;
  - lifting and turning the stopper by another grip.
- **Score 0:**
  - a rod pushed in while the stopper rests closed (it meets the stem);
  - a rod pushed under or beside the eye, even with its ball home;
  - a rod short of the eye;
  - a stopper not turned, or turned only 60 degrees;
  - a stopper lifted more than 40 mm, out of its guide, or tilted more than 6 degrees;
  - a stopper taken out of the drain and left in the basin or on the countertop;
  - a part still held or still moving.

`near_misses.py` covers these limits:

- pose cases (the pose check standing in for the test): the rod through the eye in the poses above; the outer end at
  197, 199 and 199.5 mm from the spout and with the ball 9 mm out (0.5); the rod beside the eye with the stopper
  closed or lifted but unturned, under it, over its ring and short of it; the stopper turned 60 degrees; the rod
  crossing the spout's face 6 mm off axis; 3.9 and 4.1 mm off centre, 5.5 and 6.5 degrees of tilt, and lifted 41 mm;
  the stopper taken out and laid on the basin's bottom or on the countertop with the rod home;
  a rod rolling about its own axis (still) and one swinging (not still);
- test readings: the final demo's (1.0), a rod beside the eye (nothing moves, 0), a stopper on a rod under its eye
  (lifts off, 0), a drop of 2.9 and 3.0 mm, a rise back of 2.9 mm, a pull rise of 10.0 and 10.1 mm, and no test yet;
- the test's state machine stepped offline with scripted stopper heights: 19 still steps (no test yet), a passing
  test, a passing test with the ball 3.5 mm out (0.5), the rod beside and under the eye, a gripper near during the
  lift (abandoned) and after the test (credit withdrawn);
- the release and fail-closed cases.

## Recorded runs

- Scripted demo: 1.0 (movement test passed at 19.95 s: drop 11.4 mm, back 8.8 mm, pull rise 8.6 mm; outer end
  197.2 mm from the spout).
- Dream-1, 1,200 steps: 0.0. The arms folded back near their bases and never reached the stopper or the rod.

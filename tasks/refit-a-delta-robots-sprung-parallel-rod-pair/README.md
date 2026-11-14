# Refit a delta robot's sprung rod pair: spread it with both arms and clip its four cups onto the ball studs

**What the task is.** Light delta pickers and delta 3D printers hang each arm's parallel-rod pair on ball studs, held
only by two springs between the rods. A crash pops a pair off its balls and the machine stops until someone refits it by
hand. The robot must take the loose pair from a tray, spread it against its springs with both arms until the four cups
pass outside the balls, set it between the upper arm's ball pair and the platform's ball pair (25 cm apart, on a 16.3
degree slope), let the springs close the cups onto the balls, and let go. Success is all four cups captured after
release. Either end may go on either ball pair, in any order; in a real trial clipping one end and then the other is
allowed too. Heavier industrial pairs that need a spreading tool are out of scope.

**Parts, real size (nothing is scaled).**

- Rod pair, 110 g, free: two matt black carbon tubes, 20 mm diameter x 226 mm (55 g a rod with its cups). A white
  acetal cup housing on each tube end, 28 mm diameter x 13 mm, with a 45 degree conical socket (22 mm mouth, 4 mm
  floor) for a 15 mm ball, mouth toward the other rod. Sockets 250 mm apart along a rod. Two steel extension springs
  join the rods 30 mm from each end: light desktop-delta springs, each about 60 N/m (120 N/m together; our estimate
  for this class). Facing sockets are 44 mm apart with the coils closed, 56 mm on the balls (1.4 N clamp), 84 mm when
  spread for fitting (4.8 N), 94 mm at rated stretch.
- Upper arm end, fixed (motor brake on): light grey aluminium boss 32 x 16 x 16 mm, 8 mm steel stud, two chrome
  15 mm balls 56 mm apart, 110 mm above the table; cast arm 20 mm thick up to a black motor, 80 mm diameter x 70 mm.
- Platform, fixed on a service post: dark grey aluminium hexagon, 100 mm across flats x 20 mm; light grey lug
  32 x 32 x 16 mm with the same stud and balls, 40 mm above the table; two unused lugs with rod stubs.
- Tray: blue-grey ABS base 110 x 300 x 8 mm, two cradles 90 x 12 x 22 mm with 24 mm U notches. The tube axes rest
  30 mm above the table.

**What the scripted demo does.** Both arms, closed loop (`demo_script`, about 36 s). Each gripper comes in on the robot side of the ball pairs and straight down
on its own tube, 80 mm from mid-length, jaws across the tube, and closes. Together they lift the pair level to 8 cm,
pull the rods apart until the facing sockets are 84 mm apart, lean the pair to the 16.3 degree slope by rolling both
wrists while one hand rises and the other stays low, carry it over the balls 3 cm up, lower it so each cup mouth
passes about 6 mm outside its ball, and bring the rods together until the spread stops falling: the cups are on the
balls. Then each arm takes its own push off the tube, opens, rises 6 cm, leaves the way it came and parks at the ready pose. The demo reads the pair's live pose and its spread (the slide value q) from the simulator on every step, plus
its joint encoders and finger gaps; it uses them to trim the hands, because the arms give several cm under the
springs' pull. It never moves, teleports or edits the pair.

**Fidelity.** The two end springs are one 120 N/m slide between the rods, with stops at closed coils (44 mm) and
rated stretch (94 mm) and no initial tension, so in simulation the rods stay parallel and both ends spread and close
together; a real pair can splay and can be clipped one end at a time. The sockets are 12-facet 45 degree cones, not
spherical. Tubes are solid; the spring coils are look-only. Cup on ball friction is 0.4 on 0.2. The upper arm is fixed
and the platform sits on a post. The cast arm, motor, post and unused lugs are look-only. The simulator scores the
last step without opening the grippers, so a pair still held never counts as released.

## Files

- `refit_a_delta_robots_sprung_parallel_rod_pair.py`: the task file for the `yam_arena` embodiment (scene, `score()`,
  `success()`, the closed-loop `demo_script`).
- `mechkit.py`, `belimits.py`, `closedloop.py`: shared helpers for multi-part objects, geometry checks and the
  closed-loop scripted demo, kept beside the task file.
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Take the black rod pair from the tray, clip its four cups onto the arm's and the platform's balls, then let go."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=refit_a_delta_robots_sprung_parallel_rod_pair.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/rodpair-dream

# Scripted demo (full score): both arms lift, spread, lean, lower and close the pair onto the balls, then park
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=refit_a_delta_robots_sprung_parallel_rod_pair.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/rodpair-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: it keeps every part exactly where the task file puts it.
- **Runtime per trial:** the demo takes about 36 s of simulated time (about 720 steps) and the run ends within 8 s of the
  release; a Dream-1 trial runs up to 2,400 steps (2 minutes simulated).
- Wall-clock time on one L40S, including Isaac Sim start-up: about 230 s for the demo and about 380 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` has held for 8 s (four cups on, pair released and still),
  otherwise `score()` at the end of the run. The long hold lets the demo open its grippers and park before the run ends.

## Scoring

Judged over the whole run. In a real trial use the top camera and a side camera with a mm scale behind the pair, read
frame by frame for the spread, the overstretch and the drop speed; in simulation the same quantities come from the
pair's pose and slide value.

1. **Cup on:** the socket centre is within 2.5 mm of a ball centre. Photo: the ball is inside the cup. Tape: 81 mm
   over the two housings at that end, +-2.5 mm.
2. **Spread:** facing sockets 78 mm or more apart while every part of the pair is 4 cm or more above the table (off
   the tray). At 78 mm the cup mouths clear the balls' outer faces (71 mm) by about 3 mm a side.
3. **Dropped:** any part of the pair under 1 cm above the table (in the tray its lowest point is 1.6 cm up), or the
   pair moving faster than 0.6 m/s.
4. **Overstretched:** facing sockets 92 mm or more apart, 2 mm short of the springs' rated stretch of 94 mm.
5. **Released:** at the end both grippers are 3 cm or more from the pair and the pair has moved under 1 cm/s for 1 s.

Score, first row that fits: dropped or overstretched at any time, **0**; released with 4 cups on, **1.0**; 3 cups,
**0.75**; 2 cups, **0.5**; 1 cup, **0.25**; spread at some time but not released with a cup on (still held, put back,
left resting on the balls uncaptured), **0.25**; otherwise **0**. Doing nothing, pushing or sliding the pair scores 0.

The two ball pairs are pinch zones: the sprung cups snap shut there, so a technician holds the rods by their tubes.
The task file lists a box round each ball pair in `ARM_KEEP_OUT`; the demo's hands stay on the tubes and outside both
boxes. The zones are not scored.

## Real-world reset

Unclip the pair from the balls, let it close, and lay it back in the tray notches with the sockets facing each other
and the paint-marked tube end toward the robot's left. Check that neither spring has taken a set: the facing sockets
should again be 44 mm apart. Use springs of the stated class; keep fingers off the cups when unclipping.

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.

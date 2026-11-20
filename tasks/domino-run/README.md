# Set up a run of six dominoes and topple it

Six standard double-six dominoes (48 x 24 x 7.5 mm, 10 g) lie flat and face up on the table, three either side of a
lane marked with blue tape between the arms. Six thin red tick marks cross the lane 30 mm apart. The instruction:
*Stand all six dominoes on their short ends, one on each red tick in the taped lane, faces toward you. Then tip the
domino nearest you away from you, touching no other domino, so that all six fall in a chain.*

Standing dominoes on end in a line and knocking over the first is a familiar childhood game and a popular hobby. What makes it hard for a robot:

- **Turning a flat piece upright.** Each domino has to go from lying flat to standing on its 7.5 x 24 mm end, which
  takes a 90 degree turn of the hand (or a regrasp), and has to be set down square: a standing domino tips over once
  it leans past atan(7.5 / 48) = 8.9 degrees.
- **Letting go cleanly next to standing neighbours.** The dominoes stand 30 mm apart (22.5 mm gaps). The hand has to
  open and draw back without dragging the piece or brushing the one already standing in front of it; a brush topples
  it, and often the rest. Placing far ticks first keeps the hand from reaching over standing pieces.
- **One controlled tip.** Once all six stand, only the first may be touched, and only until the second starts to
  fall; the run has to do the rest.

Standing thin items upright close to their neighbours without knocking them over is the same skill used in everyday
tidying, such as setting books or cards upright on a shelf; the category is "Societal relatability" because of the game.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()` and `score()`, and the per-step reading of the dominoes' poses and the robot's distance to each |
| `domkit.py` | Rigid bodies made of several box shapes (a domino with painted pips; the tape) and the reset/step hooks (cut down from the body kit used by other tasks in this repository) |
| `demo.py` | The scripted demo |
| `test_scoring.py` | Offline checks (no GPU): every scoring outcome and attempts to score without doing the task, the robot-distance geometry, the runtime's ending, the layout, the demo's reach and grasp, and that `task.json` states the numbers in the code |

`domkit.py` hooks `yam_arena.scene` and the embodiment's reset and step without modifying yam_arena, so the task runs
with the standard `yam_arena` embodiment and the `task_score` scorer. There is no joint, spring or scripted event in
the scene: the dominoes are plain rigid bodies and the chain reaction is PhysX contact, gravity and friction.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/domino-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. Set
`DOMINO_DEBUG=1` to print, every 10 steps, which ticks have a domino standing on them, the run's progress, any foul,
the robot's distance to each domino, each domino's lean and the height of its highest point.

## Scripted demo

```bash
python demo.py task.py demo_out --max-steps 2400 --cam 640x352   # demo_*_cam.mp4, setup.png, result.json
DEMO_CALIBRATE=1 python demo.py task.py cal_out --no-record      # prints the CAL numbers (see below)
PYTHONDONTWRITEBYTECODE=1 python -m pytest -p no:cacheprovider test_scoring.py   # offline checks
```

1. Each arm takes a flat domino across its 24 mm width (fingers on its long narrow edges, never on its faces), near
   its far end, with the gripper leaning 6 degrees back toward the arms, and lifts it.
2. It turns the gripper 90 degrees about the fingers' closing line, so the domino hangs from the fingers standing on
   end, faces square to the lane, and brings it over its tick. (A level gripper this low would need wrist joint 4 past
   its limit, hence the 6 degree lean, which the pick takes up.)
3. It lowers the domino until its base is 0.8 mm above the table, opens, and draws back along the fingers.
   The left arm sets ticks 6, 5 and 4 (far first), the right arm ticks 3, 2 and 1; the right arm picks its first
   domino while the left is still working, and waits with it.
4. The right arm, gripper closed and pointing down, puts the front edge of its finger pads against the back of the
   first domino, 8 mm below its top, pushes 12 mm away from the arms and draws back. The run falls.

The demo runs open loop: it reads no simulator state. How each domino sits in the hand once it has been turned upright
is measured once with `DEMO_CALIBRATE=1` (which reads the domino's pose while it hangs over its tick) and fixed in `CAL`
in `demo.py`. The layout is fixed and the simulation deterministic. `result.json` reports `score()` on the last step
as the trial's score.

All six stand on their ticks by step 1,300 (score 0.5), the run falls just before step 1,400, and the demo reaches 1.0
at step 1,411 (70.6 s); the runtime ends the episode with success at step 1,430 (71.5 s), within the 2,400-step limit.

## Scoring

Scored every control step from the dominoes' poses and the robot's distance to each; the score at the end of the
episode counts (`SCORE_OVER = "end"`). "Standing on a tick" means: on its short end (long side within 10 degrees of
vertical; it cannot stand on its own past 8.9), its base on the table (within 3 mm), its base centre within 6 mm of a
tick's centre along the lane and within 8 mm of the lane's centre line, its broad faces within 15 degrees of square to
the lane (one of them toward the arms; which face and which end is up do not matter), nothing of the robot within
1 mm of it, and no corner moving more than 1 mm over 0.5 s.

- **0.25**: at some point three or more dominoes stood on distinct ticks at once.
- **0.5**: at some point all six stood, one on each tick ("the set"). Every time the set holds, a fresh run starts
  from it.
- **0.75**: in a clean run (below), at least the first two dominoes (ticks 1 and 2) have fallen forward.
- **1.0**: in a clean run, all six have fallen forward, the last one lies flat (highest point at most 10 mm up), every
  corner of every domino is inside the tape's inner edges, no part of either arm is within 2 cm of any domino, and
  nothing moved more than 1 mm, all held together for 1 s (20 control steps in a row). `success()` is true only on
  steps where `score()` is 1.0; the runtime ends the episode after its own 1 s of that, so an episode can only end
  with success at 1.0.
- Otherwise 0.

**Fallen forward**: its highest point at most 30 mm above the table (48 mm standing; 46 mm if it stopped leaning on a
standing neighbour; about 19 mm resting on a fallen one) and its former top end pointing away from the arms (within 60 degrees of straight along the lane).

**A clean run** (from the last moment of the set on): the robot touches (comes within 1 mm of) no domino but the one
on tick 1; it does not touch that one after the domino on tick 2 has leaned 15 degrees; and no domino starts to fall
(leans 15 degrees) before the one on the tick before it. (In a real trial, striking the table during the run is a foul
as well.) A foul ends that run's credit; standing all six up again
starts a new one.

So knocking the dominoes over by hand, sweeping the arm through them, pushing the first one through the whole run,
tipping a middle one first, or laying them out already "fallen" scores at most 0.5 (and the last 0 if they never stood).

**In a real trial**, from a photo of the set and a video of the run: tick positions with a ruler against the tape;
"standing on its own" by eye in the photo (a domino on its end that is not touching anything is within 9 degrees);
heights of fallen dominoes with a ruler (30 mm and 10 mm marks); corners inside the tape on the photo; touches and the
order of falling on the video (a phone's slow-motion mode makes the order easy to see; it is also implied, since only
the first was touched); the 2 cm clearance by eye.

## How it is modelled

All parts are rigid bodies at real size and mass. Each domino is one 48 x 24 x 7.5 mm box of 10 g (the pips and the
divider line are painted on, not modelled as recesses), friction 0.40 static / 0.30 dynamic, 2 mm contact offset,
32 solver position iterations. The tape is visual only (0.2 mm). PhysX runs at 240 Hz; the robot acts at 20 Hz.

The robot's distance to a domino comes from the arm links' convex collision hulls (the same hulls the simulator
collides): the hull vertices' exact distance to the domino's box, and the domino's surface points' (every 4 mm)
distance to each hull's face planes, whichever is smaller. Within 1 mm counts as a touch.

**Simplifications.** The dominoes are perfectly rigid and square-edged; real ones have rounded edges and slightly
curved backs, which make them tip a degree or two sooner. Friction and restitution are estimates (below). The table is
rigid and cannot be bumped; in a real trial the robot striking the table during a run fouls the run too.

## Numbers and where they come from

| Quantity | Value | Source |
|---|---|---|
| Domino size | 48 x 24 x 7.5 mm | A common size for double-six sets (retail listings give 48 x 24 mm with 7.5 to 8 mm thickness); measure the set used |
| Domino mass | 10 g | Estimate: 8.6 cm3 of melamine (1.5 g/cm3) is 13 g, of hardwood 6 g; **weigh the set used**. All six are alike, so the run's physics barely depends on it |
| Friction (domino on table and on domino) | 0.40 static, 0.30 dynamic | **Estimate**: melamine or lacquered wood on a laminate top. Measure by tilting a board with a domino lying on it until it slides (static coefficient = tan of that angle) |
| Spacing | 30 mm (0.63 of the height) | Dominoes fall reliably in a chain at spacings from about half to three quarters of their height |
| Tipping angle | 8.9 degrees | atan(7.5 / 48) |

## Real-world set-up

All parts are bought: a double-six domino set (any 48 x 24 x 7.5 mm set; use six tiles), 5 mm blue masking tape and
3 mm red fine-line (pinstripe) tape. Lay the tape as given in `task.json` ("setup"), measured from the midpoint between
the arm bases. Nothing is consumed; a reset lays the six dominoes back flat, face up, at their spots.

## Runtime and seeds

The layout is fixed (`-E object_jitter=0`); there are no seeds to choose. The run limit is 2,400 control steps
(120 s); the demo needs 1,430 (72 s). One trial takes about 10 minutes of wall time on one GPU with cameras recorded.

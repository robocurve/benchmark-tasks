# Rack fifteen pool balls tight on the foot spot, then lift the triangle off without disturbing them

Instruction given to the policy: `Rack the balls tight on the foot spot and take the rack away.`

Files: `task.json`, `rack_pool_balls_tight.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()`, a
sim control plan and an offline self-test), `mechkit.py` (unchanged helper: multi-box free part, per-episode memory),
`README.md`.

## Response to review

Review verdict (2026-11-14 15:14Z), verbatim:

> Scoring (not met): The distance-based full-score conditions permit a tightly packed triangle facing the wrong
> direction while its head ball remains on the foot spot. What to change: Add a numeric final-orientation check so the
> apex points toward the head of the table and the rear row is appropriately aligned; include it in the scorer and its
> tests.

Change: `score()` now requires the pack to face the head of the table for 0.75 and 1.0 (`orientation()` and
`_oriented()` in the task file); a pack that fails it stops at 0.5. Two numeric tests: the line from the centroid of
the 15 ball centres to the head ball within 5 degrees of the table's long axis, pointing toward the head string (+x,
away from the robot), and the least-squares line through the five rear-row centres within 3 degrees of square to that
axis (limits and their reasons under "Scoring rules"). On the real rig both angles come from the second overhead grid
photo. The self-test adds four cases: the tight pack turned 2 degrees about the head ball scores 1.0; turned 180
degrees with the head ball on the spot (apex 180 degrees off), turned 10 degrees (both angles 10 degrees off) and
turned 4 degrees (rear row over its 3-degree limit, apex inside 5) each score 0.5. task.json scoring is updated to
match. In the recorded demo the pack stayed within 0.40 degrees (apex) and 0.28 degrees (rear row) for the whole run
and ended at 0.04 and 0.10 degrees.

Also in this revision:

- **Rack taken away and set down.** The demo now carries the lifted rack toward the robot and sets it on the table,
  its apex 65 mm behind the pack, so the final frames show the bare pack. The 0.5 rung's clearance is now
  measured from above: the rack set down on the table, still, with every part at least 50 mm horizontally from every
  ball. The self-test covers a rack lifted over the pack, carried but still in the air, set down too close and still
  sliding (all 0.25).
- **Demo re-tuned for margin, re-recorded once.** The pull now ends slowly, the pack settles for 3 s, the rack is eased
  1.5 mm forward so its bars stand off the balls, and the first 35 mm of the lift is at 5 mm/s. The recorded run ends
  with every pair within 0.25 mm (scored at 0.5 mm) and no ball moved while the rack rose. The margin, including what
  did not work, is under "Run limit, runtime and seeds". Dream-1 was re-run with the final code.
- **Reproducibility.** demo() as shipped scored 1.0 in 5 of 5 debug runs on the recording host (table under "Run
  limit, runtime and seeds"). The lift gate now re-arms if the rack comes back down flat on the cloth before it is
  set down away, so an attempt that lifts and lowers the rack is judged on its last lift; five gate sequences are in
  the self-test.
- **Text.** The long axis and the head of the table are named in task.json; the feeler gauge goes in only after both
  photos; the fidelity names tightening as the difficulty. An earlier text-only change also goes up with this
  revision: a shorter context, and the real-rig check of ball movement during the lift from two grid photos.

## Environment

Isaac Lab-Arena 0.3.1 with yam-arena and inspect-robots, as the program's launchable installs them. Before every shell:
`source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/rack-pool-balls-tight/rack_pool_balls_tight.py
```

It asserts the scoring rules on 17 hand-built states: the start; a tight pack on the spot with the rack still on it,
lifted 100 mm straight up over it, carried away but still 60 mm up, set down only 25 mm behind the pack, or set down
away but still sliding (all 0.25); a loose pack with the rack set down away (0.5); a tight pack with the rack set down
away (1.0), with the head ball 7 mm off, balls rolling or a pad touching a ball (0.75); the lift gate latched (0); the
back row 1 mm off; and four orientation cases (a correct pack turned 2 degrees passes; turned 180, 10 or 4 degrees about
the head ball on the spot each stop at 0.5). It runs five lift-gate sequences (a latch, no latch, a re-arm after the
rack is set back on the cloth, no re-arm while it is still moving, and the gate ending once the rack is away), checks
the orientation angles directly and that the set-down spot is on
the table, and replays both plans (the demo and control b) through the kinematics at 20 Hz, every joint inside the
URDF limits, IK error under 0.5 mm. Expected output:

```
(back row 1 mm off, rack away: 22/30 pairs tight -> 0.5)
lift gate: 5 sequences pass
(pack turned +0 deg about the head ball: apex 0.00 deg, rear row 0.00 deg, oriented True)
(pack turned +180 deg about the head ball: apex 180.00 deg, rear row 0.00 deg, oriented False)
(pack turned +10 deg about the head ball: apex 10.00 deg, rear row 10.00 deg, oriented False)
(pack turned -4 deg about the head ball: apex 4.00 deg, rear row 4.00 deg, oriented False)
start: rack clearance 0.00 mm (balls touching the V), widest gap 2.00 mm, head ball 15.0 mm past the spot; back bar centre x 0.2262
rack set down away: 65.2 mm horizontally from the pack, lowest point 0.0 mm above the table, nearest rack x -0.165..0.168
rack set down near: 25.3 mm horizontally from the pack, lowest point 0.0 mm above the table, nearest rack x -0.125..0.208
scoring: 17 cases pass
demo left: 41.4 s plan, worst IK 0.099 mm / 0.012 deg
demo right: 41.4 s plan, worst IK 0.099 mm / 0.012 deg
control b (not the demo) left: 33.8 s plan, worst IK 0.099 mm / 0.012 deg
control b (not the demo) right: 33.8 s plan, worst IK 0.099 mm / 0.012 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/rack-pool-balls-tight/rack_pool_balls_tight.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Rack the balls tight on the foot spot and take the rack away." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/rack-pool-balls-tight-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`), otherwise
`score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/rack-pool-balls-tight/rack_pool_balls_tight.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Rack the balls tight on the foot spot and take the rack away." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/rack-pool-balls-tight-demo
```

The demo is `demo()` in the task file, 41.4 s of plan. In the recorded run it ended on success at step 797 (about
40 s of video), inside the run limit of 1,200 steps. The pull closed the pack (all 30 pairs within 0.5 mm, widest gap
0.25 mm) and it settled with the head ball 4.0 mm from the spot; no ball moved while the rack rose (0.00 mm); the rack
was set down on the table 61 mm (horizontally) from the nearest ball at step 767, with the pack unchanged (30/30,
widest 0.25 mm, apex 0.04 degrees and rear row 0.10 degrees off). In the last frame the bare, tight pack sits on the
cloth with nothing over it; the rack lies on the table at the robot's end between the arms, its apex at the cloth's
near edge and its rear bar below the camera's view, and both grippers are open and raised above it.

## Real objects and sources

- **Balls:** fifteen 2 1/4 in (57.15 mm, tolerance +-0.127 mm) balls of 5.5-6 oz (156-170 g), the WPA specification
  (Wikipedia, "Billiard ball", citing the WPA equipment specifications). Modelled at 57.15 mm and 165 g.
- **Rack:** a wooden 8-ball triangle for 2 1/4 in balls. Inside dimensions from a retailer's listing (pooldawg.com,
  "Wood 8 Ball Rack"): "10 5/8'' from head ball to rear center" and "11 5/8'' x 11 5/8'' x 11 5/8'' from rounded corner
  to rounded corner". Fifteen touching balls need 255.1 mm from the head ball's front to the rear row's back (4 x 57.15
  x sqrt(3)/2 + 57.15) and 285.75 mm corner to corner, so this rack leaves 14.8 mm of play along its axis. The model
  keeps the 269.9 mm head-to-rear figure exactly (side bars on the 60-degree lines of a tight pack, rear bar 14.8 mm
  behind its back row). Height 1.25 in (32 mm) and weight 0.5 lb (227 g) are from other retailers' listings of wooden
  8-ball triangles (13 x 11.5 x 1.3 in; 0.5 lb). The bar thickness (15 mm) is an estimate. A second source,
  dimensions.com ("Billiards Racks"), gives 11.25 x 10 in inner dimensions, which are the zero-play values; real racks
  are made with play so the balls can be pressed together.
- **Cloth and spot:** a patch of worsted pool cloth on the station table stands for the foot end of the table's slate
  bed; the foot spot is a 1 in printed disc. Cushions and pockets are far from the foot spot on a real table and are
  not modelled.

## Physics (what is physical, what is modelled)

- Balls are rigid spheres on the cloth; everything that happens to them comes from contacts with the rack, each other
  and the grippers. PhysX has no rolling resistance, so each ball carries angular damping 10 1/s, which slows a rolling
  ball as cloth does (a ball rolling at 5 mm/s stops within about 2 mm). Friction (PhysX average combine): ball-ball
  0.06, ball-cloth 0.20, ball-wood 0.30; restitution 0. These are estimates.
- Contact offset 0.1 mm and rest offset 0 on balls, rack and cloth, so a gap of 0.5 mm means something: a tight pack
  settles with every pair within 0.05 mm.
- Tightening is the real difficulty. Pushed through the rack's V, the balls jam: in six debug trials of demo variants
  the pull left the widest gap between 0.25 and 0.80 mm, and an arm moving in small jerks (about 1.3 mm) sets the pack coasting. A
  stationary tight pack needs the push to end gently.
- The rack is a free body of three wooden bars (227 g) that the grippers hold by friction.

Probe (rack moved by a stiff driven slider, no robot; logs kept with the build): a tight pack settled with all 30 pairs
under 0.05 mm; lifted straight up, no ball moved (0.00 mm); lifted along an axis tilted 1 in 4 sideways (5 mm of drift
by 20 mm up), a ball moved 9.6 mm and gaps opened to 3.1 mm; a loose pack (2 mm gaps) lifted straight up kept its
2.0 mm gaps; a loose pack with the rack pulled back 10.9 mm at 3 mm/s closed to within 0.45 mm.

## Scoring rules

The table's long axis runs along x; the head of the table and its head string are at +x (away from the robot).

- Latched 0: once the rack's lowest point is 2 mm above the cloth, any ball's centre moves more than 3 mm horizontally
  before the rack has been set down away from the pack. If the rack comes back down flat on the cloth before that
  (lowest point within 0.5 mm, still), the reference is dropped and taken again at its next rise (`_gate_step`).
- 0 if the head (1) ball's centre is more than 10 mm from the spot; 0.25 within 10 mm; 0.5 that and the rack taken
  away: set down on the table (its lowest point within 5 mm of the table top, moving under 5 mm/s) with every part at
  least 50 mm horizontally, seen from above, from every ball (each bar's footprint against each ball's circle); 0.75
  that, at least 24 of the 30 touching pairs within 0.5 mm, and the pack facing the head of the table (below); 1.0
  that, all 30 within 0.5 mm, the head ball within 5 mm of the spot, all balls still (under 5 mm/s), and no gripper
  point (both TCPs, all four pad tips and pad middles) within 30 mm of any ball. Held 1 s.
- Real rig: two overhead photos against a grid on the cloth, one just before the rack starts to rise and one after it
  has been set down; the trial fails (0) if any ball's centre moved more than 3 mm between them. The second photo also
  gives the head ball's distance from the spot, the rack's horizontal clearance from the pack and both orientation
  angles. Only after both photos (the gauge can nudge a ball) are the 30 touching pairs checked with a 0.5 mm feeler
  gauge: a leaf held vertical, square to the line between the two centres, is lowered into the gap midway between
  them. Two spheres are closest at their equators on that line, so the pair is within 0.5 mm unless the leaf passes
  the equators and reaches the cloth. The leaf must not push either ball.
- Orientation (needed for 0.75 and 1.0). The table's long axis is x; the head of the table, where the apex points, is
  +x. (1) Apex: the line from the centroid of the 15 ball centres to the head ball within 5 degrees of the long axis,
  pointing toward the head string. (2) Rear row: the least-squares line through the five rear-row centres (balls 5, 13,
  15, 6, 12) within 3 degrees of square to the long axis. Why these limits: the centroid is 132 mm behind the head
  ball, so 5 degrees puts the rear row's centre at most 17 mm (under a third of a ball) off the long string, and the
  apex test is the one that rejects a pack facing the wrong way (turned 180 degrees, its rear row is still square); the
  rear row spans 229 mm, so 3 degrees puts its end balls at most 6.0 mm (a tenth of a ball) out of square, and since a
  tight pack is rigid this also holds its rotation to 3 degrees. Real rig: both angles from the ball centres in the
  second overhead grid photo; with centres read to about 1 mm, the angles are good to about 0.6 degrees (apex) and
  0.3 degrees (rear-row fit), well inside the limits.

## Sim controls

- **(a) Lift with drift, final code:** the shipped demo with `RK_DRIFT=0.005`: the hands drift 5 mm sideways over the
  first 20 mm of the lift. The pack was tight when the rack started rising (30/30 pairs within 0.5 mm, widest
  0.25 mm). Logged: `[rk] LATCH lift-disturbed: a ball moved 3.03 mm sideways while the rack was rising (limit 3 mm),
  control step 416, rack lowest point 14.2 mm`. task_score 0. (With the previous version of the task, the same control
  latched at 3.28 mm.)

Control (b) was run with the previous version of this task (the same scene, scorer gates and pinch; the rack was then
pulled 15.5 mm at 3 mm/s, lifted 130 mm and held, not set down), in a debug copy of that code (no frames):

- **(b) No tightening:** `controls_no_tighten()`, started with the head ball on the spot (`START_PAST = 0`): the same pinch
  and straight lift with no pull back. At the lift 3/30 pairs were within 0.5 mm (widest gap 2.19 mm); the lift moved
  no ball more than 0.92 mm. task_score 0.5 (head on the spot, rack clear, pack loose).

With the final code, the debug trials of demo variants (see "Run limit, runtime and seeds") show the pair tier
deciding the score: three ended at 0.75 with 28 or 29 of 30 pairs within 0.5 mm (widest 0.53-0.80 mm). The recorded
demo's trace shows the other conditions changing state: the head ball within 10 mm and then 5 mm of the spot as the
pull walks it back; the set-down rule on once the rack lies still on the table, off for a few steps while the hands
let go and the rack shifts, then on again; and the grippers' 30 mm clearance off while pinching and on once lifted.

## How the demo does it

Both grippers come down astride the rack's rear bar in the gaps between the rear row's balls (1-2 and 4-5), fingers
closing across the bar, and pinch it, then lift the rear bar about 1 mm (the apex stays on the cloth). They pull the
rack back toward the robot 16 mm, 12.5 mm at 3 mm/s and the last 3.5 mm at 1 mm/s: the triangle's apex pushes the
1-ball back onto the spot and squeezes the pack together between its side bars. After 3 s for the balls to settle, the
hands ease the rack 1.5 mm forward, so the side bars stand about 0.75 mm off the balls, and lift it straight up: the
first 35 mm at 5 mm/s (until the bars are past the balls' equators), then to 100 mm at 2 cm/s. They carry the rack
toward the robot, set it on the table with its apex 65 mm behind the pack's back row (the apex tip just reaches the
cloth's near edge), let go and raise the hands.
The scripted demo reads sim state, as the Guide allows: each hand's target is corrected every control step against
its measured TCP in x and z (the arms sit about 5 mm toward the robot when this low over the table).

## Dream-1 run

One Dream-1 trial with this code (1,200 steps, 60 s, instruction as above): task_score 0. The arms reached in over the
rack late in the run (grippers within 30 mm of a ball at times from step 888) but never moved or lifted it; the head
ball stayed 15 mm past the spot, and no other score tier changed.

## Run limit, runtime and seeds

- Run limit: 1,200 steps (60 seconds at 20 Hz). The demo plan is 41.4 s.
- One fixed layout: `-E object_jitter=0`. Dream-1 is hosted and not seeded. The demo is deterministic on one host
  (repeat runs on the same machine give the same numbers); results can differ slightly across GPU hosts.
- demo() as shipped: 5/5 at 1.0 on one host (Brev, L40S). Five debug runs of the final code, no frames:

  | Run | task_score | Widest gap when set down | Head ball from spot | Ball movement during lift |
  |---|---|---|---|---|
  | 1 | 1.0 | 0.25 mm | 4.0 mm | 0.00 mm |
  | 2 | 1.0 | 0.25 mm | 4.0 mm | 0.00 mm |
  | 3 | 1.0 | 0.25 mm | 4.0 mm | 0.00 mm |
  | 4 | 1.0 | 0.25 mm | 4.0 mm | 0.00 mm |
  | 5 | 1.0 | 0.25 mm | 4.0 mm | 0.00 mm |

  The runs are identical, so they show the demo repeats on that host; they do not measure margin against small
  changes. The head ball ends 4.0 mm from the spot (limit 5 mm) because the pack coasts about 2.8 mm back off the apex
  after the pull.
- Background, the variant sweep: tightening is sensitive. Six debug trials of demo variants on the same host (pull
  14.5-16 mm, with and without the 1 mm lift, with and without the x correction during the pull) ended with widest
  gaps of 0.25, 0.33, 0.40, 0.53, 0.54 and 0.80 mm when the rack was set down; three scored 1.0 and three 0.75. The
  shipped demo is the variant that reached 0.25 mm. A run of the earlier demo on this host scored 0.75 (widest
  0.64 mm) where it had scored 1.0 on another machine.
- Runtime per trial: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up.

## Reset between trials

Put the rack back with its rear bar centred at x 22.6 cm, y 0, apex away from the robot, and the balls in the order
given in the setup, loose: each touching pair 2 mm apart (set with a 2 mm shim card, then removed), the 1-ball against
both side bars with its centre 15 mm past the spot. Check that no ball touches the rear bar.

## Dup check (closest tasks, checked at packaging time)

Checked against the live repository and task board on 2026-11-14:

- No open or closed pull request mentions pool balls, billiards, racking balls, a foot spot or a triangle rack. The
  nearest pull requests, all closed and not accepted: re-spotting snooker balls on their spots (#1579), stacking five
  balls into a pyramid in a square tray (#430), singulating one box from a tightly packed 3 x 3 cluster (#515), laying
  three floorboards tight together (#897).
- Accepted on the board: taking one bottle from a row of five without knocking any over. That task is about not
  toppling neighbours while removing one item; this one is about closing a pack's gaps inside a frame and then
  releasing the frame without moving any of the free-rolling items.
- External suites: BEHAVIOR-1K `clean_a_felt_pool_table_top` (cleaning only) and ManiSkill `roll_ball` (roll one
  ball to a goal). Neither packs balls in a frame or lifts a frame off them.

## Assets

All geometry is built from boxes, a cylinder and spheres in the task file (no third-party meshes). `mechkit.py` is
this repository's helper.

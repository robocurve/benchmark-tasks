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
the 15 ball centres to the head ball within 5 degrees of the table's long axis, pointing toward the head string, and
the least-squares line through the five rear-row centres within 3 degrees of square to that axis (limits and their
reasons under "Scoring rules"). On the real rig both angles come from the after-lift overhead grid photo. The
self-test adds four cases: the tight pack turned 2 degrees about the head ball scores 1.0; turned 180 degrees with the
head ball on the spot (apex 180 degrees off), turned 10 degrees (both angles 10 degrees off) and turned 4 degrees (rear
row over its 3-degree limit, apex inside 5) each score 0.5. task.json scoring is updated to match. The demo passes the
new test by a wide margin: replayed with the new scorer, the pack stayed within 0.18 degrees (apex) and 0.24 degrees
(rear row) for the whole run, and was at 0.01 and 0.20 degrees when the rack came clear; the rack itself turned at most
0.13 degrees. The demo plan, the recording and the media are unchanged. This revision also carries an earlier
text-only change: a shorter context, and the real-rig check of ball movement during the lift from two grid photos.

## Environment

Isaac Lab-Arena 0.3.1 with yam-arena and inspect-robots, as the program's launchable installs them. Before every shell:
`source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/rack-pool-balls-tight/rack_pool_balls_tight.py
```

It asserts the scoring rules on hand-built states (start, tight on the spot with the rack on, loose with the rack
clear, tight and clear, head ball 7 mm off, balls rolling, a pad touching a ball, the lift gate latched, back row 1 mm
off, rack only 30 mm up, and four orientation cases: a correct pack turned 2 degrees passes; the tight pack turned
180 degrees, 10 degrees or 4 degrees about the head ball on the spot each stop at 0.5), checks the orientation angles
directly, and replays both plans (the demo and control b) through the kinematics at 20 Hz, every joint
inside the URDF limits, IK error under 0.5 mm. Expected output:

```
(back row 1 mm off, rack clear: 22/30 pairs tight -> 0.5)
(pack turned +0 deg about the head ball: apex 0.00 deg, rear row 0.00 deg, oriented True)
(pack turned +180 deg about the head ball: apex 180.00 deg, rear row 0.00 deg, oriented False)
(pack turned +10 deg about the head ball: apex 10.00 deg, rear row 10.00 deg, oriented False)
(pack turned -4 deg about the head ball: apex 4.00 deg, rear row 4.00 deg, oriented False)
start: rack clearance -0.00 mm (balls touching the V), widest gap 2.00 mm, head ball 15.0 mm past the spot; back bar centre x 0.2262
scoring: 14 cases pass
demo left: 22.2 s plan, worst IK 0.099 mm / 0.012 deg
demo right: 22.2 s plan, worst IK 0.099 mm / 0.012 deg
control b (not the demo) left: 15.5 s plan, worst IK 0.080 mm / 0.012 deg
control b (not the demo) right: 15.5 s plan, worst IK 0.080 mm / 0.012 deg
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

The demo is `demo()` in the task file, 22 s of plan. In the recorded run it ended on success at about step 380 (19 s
of video), inside the run limit of 1,200 steps: the pull closed the pack (all 30 pairs within 0.5 mm, widest 0.35 mm)
with the head ball 3.0 mm from the spot, no ball moved while the rack rose (0.00 mm), and the rack was clear of every
ball by 51 mm at step 359. In the final frames the lifted rack is held above the pack, between it and the camera.

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
- The rack is a free body of three wooden bars (227 g) that the grippers hold by friction.

Probe (rack moved by a stiff driven slider, no robot; logs kept with the build): a tight pack settled with all 30 pairs
under 0.05 mm; lifted straight up, no ball moved (0.00 mm); lifted along an axis tilted 1 in 4 sideways (5 mm of drift
by 20 mm up), a ball moved 9.6 mm and gaps opened to 3.1 mm; a loose pack (2 mm gaps) lifted straight up kept its
2.0 mm gaps; a loose pack with the rack pulled back 10.9 mm at 3 mm/s closed to within 0.45 mm.

## Scoring rules

- Latched 0: once the rack's lowest point is 2 mm above the cloth, any ball's centre moves more than 3 mm horizontally
  before the rack is clear of every ball by 50 mm.
- 0 if the head (1) ball's centre is more than 10 mm from the spot; 0.25 within 10 mm; 0.5 that and the rack clear (every
  part at least 50 mm from every ball); 0.75 that, at least 24 of the 30 touching pairs within 0.5 mm, and the pack
  facing the head of the table (below); 1.0 that, all 30 within 0.5 mm, the head ball within 5 mm of the spot, all balls still (under 5 mm/s), and no gripper point (both TCPs,
  all four pad tips and pad middles) within 30 mm of any ball. Held 1 s.
- Real rig: ball movement during the lift is measured from two overhead photos against a grid on the cloth, one taken
  just before the rack starts to rise and one once it is clear; the trial fails (0) if any ball's centre moved more
  than 3 mm between them. The same photos give the head ball's distance from the spot and the rack's clearance, and a
  0.5 mm feeler gauge goes between neighbouring balls.
- Orientation (needed for 0.75 and 1.0). The table's long axis is x; the head of the table, where the apex points, is
  +x. (1) Apex: the line from the centroid of the 15 ball centres to the head ball within 5 degrees of the long axis,
  pointing toward the head string. (2) Rear row: the least-squares line through the five rear-row centres (balls 5, 13,
  15, 6, 12) within 3 degrees of square to the long axis. Why these limits: the centroid is 132 mm behind the head
  ball, so 5 degrees puts the rear row's centre at most 17 mm (under a third of a ball) off the long string, and the
  apex test is the one that rejects a pack facing the wrong way (turned 180 degrees, its rear row is still square); the
  rear row spans 229 mm, so 3 degrees puts its end balls at most 6.0 mm (a tenth of a ball) out of square, and since a
  tight pack is rigid this also holds its rotation to 3 degrees. Real rig: both angles from the ball centres in the
  after-lift overhead grid photo; with centres read to about 1 mm, the angles are good to about 0.6 degrees (apex) and
  0.3 degrees (rear-row fit), well inside the limits.

## Sim controls

Two control plans, each run in a debug copy of the final code (no frames), show the scorer's gates firing:

- **(a) Lift with drift:** the demo, but the hands drift 5 mm sideways over the first 20 mm of the lift. The pack was
  tight when the rack started rising (30/30 pairs within 0.5 mm, widest 0.29 mm). Logged: `[rk] LATCH lift-disturbed: a
  ball moved 3.28 mm sideways while the rack was rising (limit 3 mm), control step 280, rack lowest point 13.2 mm`; by
  the time the rack was clear a ball had moved 5.14 mm. task_score 0.
- **(b) No tightening:** `controls_no_tighten()`, started with the head ball on the spot (`START_PAST = 0`): the same pinch
  and straight lift with no pull back. At the lift 3/30 pairs were within 0.5 mm (widest gap 2.19 mm); the lift moved
  no ball more than 0.92 mm; the rack was clear at step 225. task_score 0.5 (head on the spot, rack clear, pack loose).

The demo's own trace shows the other conditions changing state: the head ball within 10 mm and then 5 mm of the spot as
the pull walks it back, the rack clear at step 359, and the grippers' 30 mm clearance off while pinching and on once
lifted.

## How the demo does it

Both grippers come down astride the rack's rear bar in the gaps between the rear row's balls (1-2 and 4-5), fingers
closing across the bar, and pinch it. They pull the rack back toward the robot 15.5 mm at 3 mm/s: the triangle's apex
pushes the 1-ball back onto the spot and squeezes the pack together between its side bars. After 1.5 s for the balls
to settle, both hands lift the rack straight up 130 mm at 2 cm/s and hold it there. The scripted demo reads sim state,
as the Guide allows: each hand's target is corrected every control step against its measured TCP in x and z (the arms
sit about 5 mm toward the robot when this low over the table).

## Dream-1 run

One Dream-1 trial (1,200 steps, 60 s, instruction as above): task_score 0. The arms moved but never lifted or moved
the rack; the head ball stayed 15 mm past the spot.

## Run limit, runtime and seeds

- Run limit: 1,200 steps (60 seconds at 20 Hz). The demo plan is 22 s.
- One fixed layout: `-E object_jitter=0`. Dream-1 is hosted and not seeded; the demo is deterministic.
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

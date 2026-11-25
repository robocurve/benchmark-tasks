# Set a circular saw's blade depth for the board under it: hold the falling motor so one tooth shows below, then lock

The task is the Isaac Lab-Arena (PhysX) `yam_arena` task in `isaac/`; `src/` holds the engine-independent modules it
imports (layout and saw geometry, scorer, fast IK) and `tests/` their unit checks.

## The task
A 7-1/4 in corded circular saw of the DeWalt DWE575 class (8.8 lb / 4.0 kg, 0 to 2-9/16 in depth at 90 deg), unplugged,
sits on a board that rests on two 2x3 offcuts on the bench. Its blade side faces the robot and the blade hangs just
outside the board's near edge, so the blade can be seen against the board's edge face. The board is one of six common
stocks (1/4 in hardboard, 15/32 in plywood, 23/32 in plywood, a 5/4 deck board, a 2x6 or 1-3/4 in LVL), and the saw's depth is set wherever the last
job left it, with the depth lever locked. The robot must release the depth lever, pivot the motor about the saw's front
depth pivot until the lowest tooth is 1/8 to 1/4 in (3.2 to 6.4 mm, about one tooth) below the board's underside, lock the lever and take
its hands off the saw. The motor and blade weigh about 3.4 kg and hang behind the pivot: once the lever is released,
whatever holds the rear handle carries about 19 N; let go and the blade drops to full depth. Locking the lever clamps
the depth wherever the blade is at that moment.

Why it matters: carpenters reset a circular saw's depth every time the stock changes (sheathing, decking, framing).
Common trade guidance sets the blade 1/8 to 1/4 in below the stock, about one tooth (manual wording varies; not
re-checked online from our build host):
more exposed blade under the board raises kickback and injury risk; too shallow does not cut through.

## Scoring (`src/scoring.py`, primary score 0..1, value at the end of the trial)
- 1.0 (ends the trial): lowest tooth 1/8 to 1/4 in (3.2 to 6.4 mm) below the board's underside; depth lever locked (within 6 deg of its
  stop, where its cam clamps the bracket); shoe flat on the board (within 1 deg and 1.5 mm); every fingertip 50 mm or
  more from the saw; saw still for 1 s.
- 0.4: the same, but the tooth 1 to 3.2 mm or 6.4 to 9 mm below the underside.
- 0: lever released (the motor then drops to full depth by itself), tooth above the underside or more than 9 mm below
  it, shoe not flat on the board, a hand on the saw; or at any time the saw tipped (shoe more than 10 deg off flat) or
  fell off the board (vetoes latch).
Blind guessing: the board is drawn uniformly from six stock classes whose 1/8-1/4 in target bands do not overlap, and
the start depth (saw left fully raised or fully lowered, 1/2 each) is drawn independently of the board. Over 30,000
sampled seeds every board-blind rule stays at the 1-in-6 floor: one fixed depth scores 1.0 on 17.2 % of seeds, a
different fixed depth for each start side 17.2 %, a depth looked up from the start depth 18.1 %, a fixed change from
the start depth 8.7 %; keeping the start depth or setting full or zero depth scores 0. The demo's commands replayed
open-loop scored 0 on every start with another board or the saw lowered (4 starts tried). In the default seed's class
(deck board, saw raised; 1 in 12 seeds) the replay still finds the handle and the lever, and the depth it sets moves
with the saw's position along the board: over 18 such starts it scored 1.0 on 10, 0.4 on 4 and 0 on 4 (saw
shifted 7 mm or more to the robot's right of centre: the replayed hand misses the lever when relocking). That is 1.0 on about
4 % of all seeds (above 0 on about 6 %), under the 1-in-6 floor of the board-blind rules.

Real trial: a steel rule held against the board's edge face beside the blade reads the lowest tooth below the
underside; the lever position and a light push on the rear handle (the motor must not move) show the lock; a straight
edge / rocking the saw shows the shoe flat; the video shows the hands off and the vetoes.

## Dream-1 evaluation

The Inspect Robots run used model `dreamlabs/dream-1-beta` and returned a task score of 0.0 at the 2,400-step limit. The scripted reference run scored 1.0.
From the task's `isaac/` directory in the Isaac Sim 6.1 and Isaac Lab-Arena environment, run:

```bash
instruction=$(python -c 'import json; print(json.load(open("../task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
    -P model=dreamlabs/dream-1-beta -P rows_per_request=10 \
    -E task=task.py -E object_jitter=0 -E device=cpu -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
    --scorer task_score --seed 0 --max-steps 2400 --store-frames --log-dir ./dream-output
```

## Run and score a trial
Inside the Isaac Sim 6.1 + Isaac Lab-Arena container, working directory `isaac/` (`isaac/run_frozen.sh` does this on a
frozen copy of `isaac/`, `src/` and `task.json`, writes `source-sha256.txt` and logs to `<out>/<run>/run.log`; see its
header for the environment variables it needs):

```
# official Inspect Robots run of the scripted reference demo (registered closed-loop policy saw_depth_scripted)
python official_run.py -- run --embodiment yam_arena --policy saw_depth_scripted -E task=task.py -E object_jitter=0 \
    -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 \
    --instruction "$(python -c 'import json;print(json.load(open("../task.json"))["instruction"])')" \
    --scorer task_score --max-steps 2400 --store-frames --log-dir <out>/official
inspect-robots video <out>/official/<log>.json --out <out>/official/video     # H.264 video of every camera

# plain do-nothing run (task.py demo() gives the yam_scripted policy no waypoints: both arms hold READY)
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py ... --max-steps 2400

# direct demo / negative stages (trace.jsonl, result.json, demo_<camera>.mp4, setup.png, final.png)
python demo.py task.py <out>/<run> --stage demo --seed 1415158438   # the default inspect-robots trial seed
python demo.py task.py <out>/<run> --stage demo                     # named setup 0; SAW_SETUP=k for setup k
python demo.py task.py <out>/<run> --stage look --seed 1415158438 --cam-size 1280x704   # media: wrist camera at the blade
python demo.py task.py <out>/<run> --stage neg-unlocked     # depth set, lever left released, hands off: 0
python demo.py task.py <out>/<run> --stage neg-wrong-board  # depth set for another stock thickness: 0
python demo.py task.py <out>/<run> --stage neg-no-support   # lever released with no hand on the handle: blade drops to full depth: 0
SAW_REPLAY=<run>/trace.jsonl python demo.py task.py <out>/<run2> --stage replay --seed <s>   # open-loop replay
python negcheck.py negcheck.json <run> [<run> ...]          # stimulus + reaction check
```
Unit checks (no simulator): `python tests/test_scoring.py`.
Tiled evidence video (top + left wrist + right wrist, same timing):
`isaac/tile.sh <out_dir> <name> demo_top_cam.mp4 demo_left_cam.mp4 demo_right_cam.mp4`.

## Recorded media

- `model_run.mp4` shows the full Dream-1 trial (seed 0, 2,400 steps, final task score 0.0). The three views run at 20 Hz in order from left to right: overhead, left wrist, right wrist.
- `official_tiled_top_left_right.mp4`: the official Inspect Robots run of the scripted reference policy at the default
  seed (5/4 deck board, saw retracted: tooth 25.2 mm above its underside at the start), score 1.0 after 629 of 2,400 control steps (H.264, 960 x 176).
- `setup_overhead_hires.png`: the default-seed start state from the overhead camera at 1280 x 704.

Start states: every trial reset with a seed (every `inspect-robots run` passes one) draws the board (uniformly, one of the six),
the start depth (saw left retracted, 0-2 mm, or deep, 59.5-64 mm, 1/2 each, independent of the board) and
a shift of the saw along the board of up to 12 mm; the default inspect-robots seed gives the deck board with the saw retracted (0.2 mm). Unseeded
resets (direct `demo.py` runs) use the named setups in turn (`SAW_SETUP` picks the first; set, it also forces named
setups on seeded resets).

## Fidelity (full statement)
- Engine: Isaac Sim 6.1 / Isaac Lab-Arena `yam_arena`, PhysX on the CPU, physics 480 Hz, control 20 Hz, cameras
  320 x 176. All parts are full size.
- Saw: representative geometry of a DWE575-class saw. Published figures used: 7-1/4 in blade, 2-9/16 in depth at 90
  deg, 8.8 lb (4.0 kg). Our estimates (no drawing is published): front depth pivot, arbor 144 mm behind it, spur-gear
  offset of the motor, rear D-handle, depth bracket (quadrant) on the blade side behind the upper guard with the cam
  lever on it, masses of the parts (upper assembly 3.40 kg, shoe with bracket 0.60 kg, lever 40 g). These published
  figures were taken from the manufacturer's specification as we know it; they could not be re-checked online from our
  build host.
- The saw is one floating PhysX articulation: the shoe, the upper assembly (motor, gearcase, upper guard, 24-tooth
  blade, rear handle, front knob) on a revolute joint at the depth pivot with hard limits 0 to 2-9/16 in of depth, and
  the depth lever on a revolute joint at its bolt with hard limits 0 (locked) to 60 deg. Collision shapes are convex
  hulls of the parts; the blade collides as a 24-gon of its tooth tips at kerf thickness and is drawn with its teeth.
- Depth clamp (computed, ours): while the lever is within 6 deg of its locked stop, the pivot's articulation drive is a
  stiff spring (2e4 N m/rad) anchored at the depth where the clamp closed, capped at a 25 N m holding torque (gravity
  on the motor is 5.5 N m; a 20 N push on the handle adds 5.7 N m); above the cap the bracket slips and the clamp
  re-anchors (stick-slip). Released, the bracket has 1.0 N m of capped damping (loose bracket friction), so the motor
  falls to full depth unless held.
- Lever cam (computed, ours): a 0.45 N m torque pushes the lever toward the nearer end of its travel (over-centre at 25
  deg): it takes about 10 N at the tab to flip it either way and it never moves on a light touch (5 N did not move it in
  a probe).
- Lower (retracting) blade guard not modelled: on the real saw the operator sights the guard's edge or swings the guard
  up with its lever to see the teeth. Trigger, cord, bevel adjustment and the blade's rotation are not modelled.
- Boards and the support blocks are kinematic (a carpenter would clamp the stock; nothing pushes them in the task).
  The board is one of six bodies; the other five are parked out of view and swapped only on an environment reset.
- Friction coefficients (static = dynamic, estimates): shoe (aluminium) on wood 0.35, handle overmould 1.0, plastics
  0.6, steel 0.4, wood 0.5; the gripper pads use PhysX defaults combined by averaging.
- Nothing welds, snaps or teleports during a trial; the scripted demo only sends the 14 joint / gripper commands and
  reads poses and joint angles. The scorer only reads.
- The reference demo's peak finger forces are 25 N (left hand: the handle's ~19 N rests on the lower finger's face)
  and 20 N (right hand pinching the lever tab), under the YAM's 40 N. The handle load is within the simulated arm's
  joint limits; the real YAM's payload rating was not checked.

## Real rig
DeWalt DWE575 (or DWE575SB) circular saw, unplugged, with a 7-1/4 in 24-tooth framing blade; six board offcuts 140 x
600 mm (1/4 in tempered hardboard 6.4 mm, 15/32 in plywood 11.9 mm, 23/32 in plywood 18.3 mm, 5/4 x 6 deck board
25.4 mm, 2x6 38.1 mm, 1-3/4 in LVL 44.5 mm); two 2x3 offcuts
180 mm long, on edge (63.5 mm tall) under the board's ends; a steel rule in mm. The saw's front points to the robot's
right, its blade side toward the robot, the blade 8 mm outside the board's near edge.

## Third-party assets and licenses
| Asset | Source | License |
|---|---|---|
| YAM arm model (URDF, meshes) | i2rt, via the Isaac Lab-Arena YAM package | MIT |
| Station scene, cameras, `yam_arena` embodiment | Isaac Lab-Arena YAM package | as distributed with the package (no license file at the pinned revision) |
| Isaac Sim 6.1 / Isaac Lab-Arena | NVIDIA | NVIDIA Isaac Sim license / Apache-2.0 |
| Saw, boards and blocks geometry | generated in code by this task (`src/layout.py`) | MIT (this repository, `LICENSE`) |
| `src/fastik.py` (YAM URDF kinematics) | this task's authors | MIT |
"DeWalt" and "DWE575" name the real tool the replica represents; no manufacturer asset (CAD, texture, logo) is used.

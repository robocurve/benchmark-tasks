# Set a wall clock by the knob on its back, then hang it back blind by its keyhole on the nail, level

The task is the Isaac Lab-Arena (PhysX) `yam_arena` task in `isaac/`; `src/` holds the engine-independent modules it
imports (layout and clock geometry, scorer, fast IK) and `tests/` their unit checks.

## The task
A 10 in battery wall clock hangs by the keyhole in its back on a nail in a wall section in front of the robot. It still
shows standard time and has drifted a few minutes: it shows a time 20 to 54 or 66 to 95 minutes before 10:10 (drawn per trial: 8:35-9:04 or 9:16-9:50; never within 6 minutes of exactly one hour behind). The
instruction says it is 10:10. The glass covers the hands, so the time can only be set with the setting knob on the
movement at the back. The robot lifts the clock off the nail (up 8.6 mm, so the nail shank runs down the keyhole slot
into the entry hole, then straight off so the head passes through the hole), stands it on its rim on the bench, turns
it a quarter turn so its back faces one hand, pinches the 15 mm knob through the opening in the back and turns it,
regrasping, until the hands show 10:10, turns the clock back, and hangs it on the nail again: the keyhole is on the
back, facing the wall, so the entry hole must be lined up blind with the nail it saw (the nail's position varies a
little per trial), the clock pushed flat to the wall and lowered so the shank goes up the slot, and let go level.

Why it matters: twice a year nearly everyone sets their clocks for the change to or from summer time; taking a wall
clock down, turning the knob on its back while watching the hands and hanging it back level is one of the most familiar
household jobs.

## Scoring (`src/scoring.py`, primary score 0..1, value at the end of the trial)
Every level needs the end state held for 1 s: the clock hangs on the nail by its keyhole (the nail's shank up in the 4 mm
slot above the top of the entry hole, the clock's back within 3 mm of the wall at the keyhole, and the nail carrying
at least half the clock's weight), back
flat on the wall (within 6 deg), level (12-6 line within 5 deg of vertical), not touched by the robot, still.
- 1.0 (ends the trial): the hands show 10:08 to 10:12 (within 2 min).
- 0.5: within 5 min.
- 0 otherwise: not back on the nail, tilted more than 5 deg, or the time more than 5 min off (the start is 20 to 95
  min off; no face-blind fixed turn of N minutes reaches 1.0 on more than 7.5 % of seeds, and none of 50, 60 or 70 min
  scores at the default seed); and 0 for the whole trial if the clock fell at any time (moving down faster than 0.45 m/s, the speed after
  a free fall of about 10 mm).
Real trial: photo of the face for the time; let go and check the clock stays on the nail; phone level on the bezel top
(or the 12-6 line) for the 5 deg; video for a fall.

## Results (frozen source)
- Official Inspect Robots run, default seed (clock at 9:27.4, nail 13.9 mm right of and 7.6 mm above nominal): 1.0
  after 1,617 control steps (9:27.39 -> 10:10.28; hung by the slot, back 1.0 mm from the wall at the keyhole, tilt
  0.7 deg); no action changed by the CLI guards.
- Direct demo, default seed: 1.0 (identical to the official run). Seed 7 (8:45.0, 85 min to add): 1.0 after 2,034 steps.
- Do nothing (plain CLI no-op): 0. Face-blind fixed turn of +50 min: 0 (shows 10:17.8). Keyhole lined up 5 mm beside
  the nail: 0 (not hung; the clock falls). Open-loop replay of the default-seed trace at seed 7: 0.
- Shortcut sweep over 4000 seeds: any fixed turn of N minutes reaches 1.0 on at most 7.5 % of seeds (best expected
  score 0.125); at the default seed every 5-minute round turn is at least 2.4 min off.

## Dream-1 evaluation

The Inspect Robots run used model `dreamlabs/dream-1-beta` and returned a task score of 0.0 at the 3,000-step limit. The scripted reference run scored 1.0.
From the task's `isaac/` directory in the Isaac Sim 6.1 and Isaac Lab-Arena environment, run:

```bash
instruction=$(python -c 'import json; print(json.load(open("../task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
    -P model=dreamlabs/dream-1-beta -P rows_per_request=10 \
    -E task=task.py -E object_jitter=0 -E device=cpu -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
    --scorer task_score --seed 0 --max-steps 3000 --store-frames --log-dir ./dream-output
```

## Run and score a trial
Inside the Isaac Sim 6.1 + Isaac Lab-Arena container, working directory `isaac/` (`isaac/run_frozen.sh` does this on a
frozen copy of `isaac/`, `src/` and `task.json`, writes `source-sha256.txt` and logs to `<out>/<run>/run.log`; see its
header for the environment variables it needs):

```
# official Inspect Robots run of the scripted reference demo (registered closed-loop policy clock_dst_scripted)
python official_run.py -- run --embodiment yam_arena --policy clock_dst_scripted -E task=task.py -E object_jitter=0 \
    -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 \
    --instruction "$(python -c 'import json;print(json.load(open("../task.json"))["instruction"])')" \
    --scorer task_score --max-steps 3000 --store-frames --log-dir <out>/official
inspect-robots video <out>/official/<log>.json --out <out>/official/video     # H.264 video of every camera

# plain do-nothing run (task.py demo() gives the yam_scripted policy no waypoints: both arms hold READY)
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py ... --max-steps 3000

# direct demo / negative stages (trace.jsonl, result.json, demo_<camera>.mp4, setup.png, final.png)
python demo.py task.py <out>/<run> --stage demo --seed 1415158438   # the default inspect-robots trial seed
python demo.py task.py <out>/<run> --stage demo                     # named setup 0; CLOCK_SETUP=k for setup k
python demo.py task.py <out>/<run> --stage neg-noset    # off and back on the nail, knob never turned: 0
python demo.py task.py <out>/<run> --stage neg-fixed60  # knob turned exactly one hour, drift ignored: 0
python demo.py task.py <out>/<run> --stage neg-fixed50  # face-blind fixed +50 min turn: 0 at the default seed
python demo.py task.py <out>/<run> --stage neg-miss     # keyhole lined up 5 mm beside the nail: not hung properly: 0
CLOCK_REPLAY=<run>/trace.jsonl python demo.py task.py <out>/<run2> --stage replay --seed 7   # open-loop replay
python demo.py task.py <out>/<run> --stage idle,demo    # reset check (several trials in one process)
```
Unit checks (no simulator): `python tests/test_scoring.py`.
Tiled evidence video (top + left wrist + right wrist, same timing):
`isaac/tile.sh <out_dir> <name> demo_top_cam.mp4 demo_left_cam.mp4 demo_right_cam.mp4`.

## Recorded media

- `model_run.mp4` shows the full Dream-1 trial (seed 0, 3,000 steps, final task score 0.0). The three views run at 20 Hz in order from left to right: overhead, left wrist, right wrist.
- `official_tiled_top_left_right.mp4`: the official Inspect Robots run of the scripted reference policy at the default
  trial seed (clock showing 9:27.4, nail 13.9 mm right of and 7.6 mm above nominal), score 1.0 (H.264, 960 x 176).
- `setup_overhead_hires.png`: the start state at 1280 x 704 (default seed): wall, nail-hung clock with its face.

Start states: every trial reset with a seed (every `inspect-robots run` passes one) draws the start time uniformly from
8:35-9:04 or 9:16-9:50 (a second stream of the seed) and moves the nail by up to 15 mm sideways and 12 mm up or down; unseeded resets (direct
`demo.py` runs) use the named setups in turn (`CLOCK_SETUP` picks the first; set, it also forces named setups).

## Fidelity (full statement)
- Engine: Isaac Sim 6.1 / Isaac Lab-Arena `yam_arena`, PhysX on the CPU, physics 480 Hz, control 20 Hz, cameras
  320 x 176. All parts are full size.
- Clock: dimensions representative of a common 10 in (254 mm) round plastic-case quartz wall clock with a glass
  crystal (no specific product's drawing is copied and no product part number is claimed): bezel ring 254 mm outer,
  222 mm inner, 18 mm deep, glass set 9 mm behind its front; back shell 219 mm across with a 2.5 mm back panel; standard
  56 x 56 x 16 mm quartz movement with a 15 mm setting knob on the minute arbor (the knob and the minute hand turn
  together 1:1, the hour hand 1:12); keyhole hanger cut in the back panel 70 mm above the centre: 11 mm entry hole and a
  4 mm wide slot up to 80 mm; 60 mm square opening around the knob; 0.50 kg with an AA cell (typical; not weighed).
- Clock physics: a floating PhysX articulation. Root link = the case (one rigid body of about 230 convex pieces:
  60 bezel sectors, 60 side-wall sectors, about 100 back-panel cells following the keyhole outline within 0.2 mm and
  the knob opening, the glass, the movement body); child links = the knob (revolute, carries the minute hand) and the
  hour hand (revolute, no collision). Contact offset 0.3 mm (below the 0.57 mm play a side of the shank in the slot),
  rest offset 0. The clock never sleeps.
- Knob drag (the movement's friction clutch): an elasto-plastic friction anchor on the knob joint: a stiff position
  drive (2 N m/rad) whose force is capped at 8 mN m and whose target follows the knob when it slips, so the knob holds
  until about 8 mN m and then turns with that drag both ways. The 8 mN m is an estimate (not measured; real movements
  vary). The hour hand is a position drive that follows the knob joint at 1:12 (the motion work's gear ratio).
- The movement is not running (battery out for the trial, as in the real protocol below), so the hands move only when
  the knob turns.
- Wall section (gypsum board on a bench stand) and nail are kinematic rigid bodies (contact forces are reported).
  The wall is at bench height (a real clock hangs higher), so the YAM arms reach it. Nail: 6d common nail (ASTM F1667:
  shank 2.87 mm, head 6.76 mm), modelled level (in practice driven slightly upward), head 10 mm proud.
- Friction (static = dynamic, estimates): case ABS on the gripper pads 0.7 and knob 0.9 (combined with the pads' own
  material by PhysX averaging); nail steel on the case 0.3; painted wall 0.4.
- Scoring reads the knob joint angle (time), the clock pose (tilt, keyhole-on-nail geometry) and contact forces (nail
  on clock, robot on clock), not contact positions. Nothing welds, snaps or teleports the clock; the scripted demo only
  sends the 14 joint / gripper commands and reads state. On an environment reset (between trials only) the kit puts the
  clock back on its nail at the setup's start time.
- The reference demo pinches the bezel ring with about 17 N per finger and the knob with about 12 N (fingers commanded
  3.5 / 2.5 mm inside the faces against the 5000 N/m finger drive), far under the YAM's 40 N.

## Real rig
Any 10 in battery wall clock with a glass front, a keyhole hanger and a setting knob on the movement; a piece of 1/2 in
gypsum board (600 x 500 mm) screwed to a bench stand (two feet behind it) and painted; one 6d common nail driven level
so its head stands 10 mm proud at the nominal position (25.5 cm above the bench, centred), or at the trial's offset
(up to 15 mm sideways and 12 mm up/down; mark the positions on the board and use a second nail hole per position).
Take the battery out (so the hands only move with the knob) and set the start time with the knob before each trial.

## Third-party assets and licenses
| Asset | Source | License |
|---|---|---|
| YAM arm model (URDF, meshes) | i2rt, via the Isaac Lab-Arena YAM package | MIT |
| Station scene, cameras, `yam_arena` embodiment | Isaac Lab-Arena YAM package | as distributed with the package (no license file at the pinned revision) |
| Isaac Sim 6.1 / Isaac Lab-Arena | NVIDIA | NVIDIA Isaac Sim license / Apache-2.0 |
| Wall section, nail, clock geometry (case, keyhole, movement, knob, hands) | generated in code by this task (`src/layout.py`, `isaac/clockkit.py`) | MIT (this repository, `LICENSE`) |
| `src/fastik.py` (YAM URDF kinematics) | this task's authors | MIT |

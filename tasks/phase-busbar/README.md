# Push a phase busbar up into the push-in terminals of four circuit breakers, every fork fully home

A mounting plate with a DIN rail stands upright on the table, facing the robot. Four single-pole 16 A circuit
breakers with screwless (quickconnect) busbar terminals sit side by side on the rail; each terminal opens in the
breaker's underside. A single-pole fork-type phase busbar, cut to four modules, lies on its back on the table. The
robot must fit the busbar so that each of its four forks is pushed fully up into its breaker's terminal, and leave it
there. Scoring checks every fork, that the busbar was let go, and that it is really clamped: after release it must
hold a 5 N pull straight down.

The task is a [yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) task file (`task.py`) in
Dream's `station` scene, with the `yam_arena` embodiment and the official `dream` policy. It was built and run at
yam-isaac-lab-arena commit `0059d02` (the Brev launchable's pin), Isaac Lab-Arena 0.3.1, Isaac Sim 6.1.

## Files

| File | What it is |
|---|---|
| `task.py` | The task file: the panel and its terminals, scoring (`success`, `score`, the tug test) and the scripted demo (`demo`) |
| `task.json` | The submission's details |
| `assets/busbar_4m.usd` | The busbar as one rigid body, lying on its back (built by the tool below) |
| `tools/make_busbar_usd.py` | Builds `busbar_4m.usd` from the dimensions in `task.py` (needs only `pxr`) |

There are no third-party assets: every part (plate, rail, breakers, terminals, busbar) is built from boxes in
`task.py` and `tools/make_busbar_usd.py`. The robot and the station come with yam-isaac-lab-arena.

## What `task.py` adds to yam_arena

The one-file task format covers boxes, cylinders, spheres and library models, scored from those objects alone. This
task needs three things beyond that, all inside `task.py`; nothing in yam_arena is edited.

1. **The busbar model.** `task.py` registers `assets/busbar_4m.usd` in Arena's asset registry (plain Python) under
   `phase_busbar_4m`, so an ordinary `Asset(...)` spawns it.
2. **A panel with moving parts.** The terminals' clamping springs are jointed bodies, which the task format cannot
   declare. When `yam_arena.scene` is imported (after Isaac Sim starts), `task.py` wraps its `task_room()` to add one
   more scene object, `panel`, whose spawner authors the plate, the rail, the four breakers and, per terminal, the
   clamping spring on a sprung revolute joint.
3. **Scoring state.** `task.py` wraps `yam_arena.scene.object_states` so each step's states also hold the busbar's
   pose and both grippers' tool centre points (`left_gripper`, `right_gripper`, by the arm's base). The tug test runs
   in that wrapper too, since it needs the simulator: once all four forks are home and both grippers have stayed
   5 cm clear for 3 s, a downward force on the busbar is raised to 5 N over 1 s and held to 2 s, then removed.

## Run and score a trial

On the launchable, in Arena's venv (`source ~/IsaacLab-Arena/yam.env`), from the repository root.

Dream-1 (20 Hz, 320x176 frames, as in the walkthrough):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/phase-busbar/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the phase busbar onto the four circuit breakers, every fork fully home, and leave it there." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/phase-busbar-dream
```

The scripted demo, which reaches full score (`-P hold_s=8` keeps it running long enough for the tug test):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted -P hold_s=8 \
  -E task=tasks/phase-busbar/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the phase busbar onto the four circuit breakers, every fork fully home, and leave it there." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/phase-busbar-demo
```

The run prints `task_score` at the end (1.0 for success, otherwise the partial credit in `task.json`). Add
`-E headless=false` to watch it on the desktop, and `BUSBAR_TRACE=1` in front of the command to print the scoring
state every 10 steps (each fork's gap to its breaker, the clamping springs' swing, release and the tug). To make
videos, run `inspect-robots video <log>.json`, which writes one MP4 per camera.

**Runtime per trial** on one L40S: the demo succeeds after about 530 steps (26 s simulated), about 1.5 minutes of
wall time including startup; a Dream-1 trial that runs the full 2,400 steps takes about 5 minutes.

**Seeds.** The start is fixed: `-E object_jitter=0`, and nothing else in the scene is randomised. Scoring keeps a
little state per trial (the tug test) and resets it when the busbar lies untouched at its start.

## The demo

`demo()` (for `--policy yam_scripted`) plays open-loop waypoints with IK on the URDF, right arm only:

1. Gripper pointing down, fingers closing along the busbar: it grips the busbar by its two end caps, with a limited
   squeeze (finger target 10 mm inside the busbar's length), lifts it and turns it forks-up, the gripper now
   pointing forward.
2. It lines the forks up under the four terminals and pushes up, commanding the busbar 16 mm past home: the arm's
   wrist is compliant (the station's i2rt gains), so the position error becomes push force. The forks go in and the
   springs catch them, about 1 mm short of home, where the fingertips would meet the outer breakers.
3. It lets go, closes its fingers, and presses the busbar fully home from below with the top edge of its
   fingertips at the busbar's middle, then lowers and backs away.

Two constants come from runs, not from the model: `GRIP_ROLL` (the busbar sits 0.9 degrees rolled in the grip after
the pick, so the hand rolls back by as much and all four forks meet their terminals together) and the off-centre grip
`GRIP_HIGH`.

## Fidelity and simplifications

- **Geometry.** Sizes are the published 17.5 mm module, a DIN 43880 size-1 breaker (about 85 mm tall) and a 10 mm2
  fork busbar; the rest (terminal depth and position, fork and body sizes) is estimated from product photos. All
  parts are boxes; nothing is from CAD.
- **Terminals.** Each is a 10 mm wide, 14 mm deep slot with 1.5 mm lead-ins, a fixed contact wall at its back and a
  push-in clamping spring at its front: a leaf hinged at the slot's mouth, leaning up toward the wall at 20 degrees,
  on a sprung revolute joint (about 0.8 N on a tine). A real push-in spring holds what it clamps until it is opened;
  friction alone does not reproduce that in PhysX, so each tine has a 0.8 mm notch 0.6 to 2.6 mm below its tip that
  the spring's tip drops into once the fork is home. Real tines are plain. The spring is 10 g, heavier than a real
  one, so the solver holds its stop under load.
- **Friction.** 0.5 between tines and the spring or contact wall (tinned copper), 0.3 against the plastic housing.
- **Breakers** are fixed in place (no rail latch), so the scoring rule for a breaker moved on its rail is not
  simulated.
- **Tug test.** In the simulation it is a force applied to the busbar after the robot has let go, off camera; in a
  real trial it is a spring scale.

# Replace a worn pulley and refit its long and short screws from memory

This task tests memory. A robot arm's worn timing pulley sits on a bench stand, held by four screws with identical heads: two long and two short. A screw's length shows only once it is lifted out, and then while it hangs in the open rack. Nothing in view shows which hole is deep, so the arms must remember which hole gave which length. They lift out the screws, swap the pulley, and drop each screw back in a hole of its own length. A long screw does not fit a shallow hole: it stops 2.8 cm proud. With two of each length, any wrong layout puts a long screw in a shallow hole, so every wrong layout shows physically. The instruction does not say that the screws differ. A blind guess of the layout succeeds 1 time in 6.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source <ARENA_DIR>/yam.env`).
- `refitkit.py`, `refit_score.py`, `refit_usd.py`, `refit_mesh.py`, `moveo/` and `textures/` are in this folder. Importing `refitkit` hooks `yam_arena.scene` without modifying it, so the parts, declared as subclasses of `Box`, spawn from USD files the kit writes at start-up.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --policy dream -P rows_per_request=20 --scorer task_score --max-steps 2400 \
    --instruction "Replace this worn timing pulley with the new one: lift out its four screws and hang them in the rack, put the worn pulley on its mat, set the new pulley on the two pins, then drop each screw back into the hole it came from, and move both arms away." \
    --store-frames --log-dir out
```

## Scripted demo

```bash
REFIT_DEMO=safe inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=640 -E cam_height=360 --policy yam_scripted -P hold_s=2 --scorer task_score --max-steps 2300 \
    --instruction "Replace this worn timing pulley with the new one: lift out its four screws and hang them in the rack, put the worn pulley on its mat, set the new pulley on the two pins, then drop each screw back into the hole it came from, and move both arms away." \
    --store-frames --log-dir demo
```

`REFIT_DEMO=safe` (the default) reaches 1.0. Each arm hangs its two screws in the rack, the arms taking turns. The left arm lifts the worn pulley by its hub, with its fingers through the windows, and sets it on its mat. The right arm sets the new pulley on the pins. The screws then go back. The plan is open loop, so run it with `-E object_jitter=0`, and it holds the layout as numbers. `REFIT_DEMO=swapped` returns the left arm's two screws to each other's holes. `REFIT_START=open` starts with the pulley off the stand and the screws in the rack, for looking at the holes.

## Scoring

`--scorer task_score` prints the total. Per-step `info` is not saved, so the kit also writes the state to `refit_scores/score_*.json` (or `$REFIT_SCORE_DIR`) every 10 control steps and whenever the score changes.

- The new pulley is seated when its axis is within 5 mm of the stand's centre, its underside within 3 mm of the stand's top, and it lies within 5 degrees of flat.
- The worn pulley is on its mat when its axis is within 5 cm of the mat's centre, its underside is within 1 cm of the mat's top, and it moves slower than 1 cm/s.
- A screw is seated in a hole when its shank is within 4 mm of the hole's axis, it stands within 10 degrees of upright, and the underside of its head is within 3 mm of the pulley's top.
- A screw has hung in the rack once, at some moment, its shank has been within 6 mm of a slot's axis, it has stood within 10 degrees of upright, the underside of its head has been within 3 mm of the rack's top, and no gripper's fingertip has been within 3 cm of it.
- A hole is right when the new pulley is seated, a long screw is seated in a deep hole or a short screw in a shallow hole, that screw has hung in the rack, and no gripper's fingertip is within 3 cm of it. The two screws of each length are interchangeable, since a tape measure cannot tell them apart.
- A long screw does not fit a shallow hole: it stops with its head 2.8 cm proud, so it cannot seat. If its tip comes within 2 mm of that floor, held or not, that hole is lost for the rest of the run, so trying a hole does not pay.
- A short screw reaches the seat in a deep hole, since depth cannot stop a shorter screw. It looks seated and earns nothing. It also leaves a long screw with nowhere deep to go, so with all four screws in, a wrong layout always has a long screw standing proud.
- Points: 0.1 for the worn pulley on its mat, 0.1 for the new pulley seated, 0.2 for each right hole. A total of 1.0 also needs both grippers' fingertips at least 5 cm from the pulleys and screws and every pulley and screw moving slower than 1 cm/s, held for 1 s. Without that hold it scores 0.9.

- If the model refuses at any point, the trial ends there and scores as at the end of a run, so never 1.0. The harness then prints no `task_score`, so read `score` under `final` in the last `refit_scores/score_*.json`.

`python test_refit_score.py` runs these rules on made-up positions, without the simulator.

To check by hand, lift each screw out and measure it with a tape, and measure each hole's depth with a rod.

## What the cameras show

Measured from frames, as shank length in pixels, long / short. The wrist rows come from scripted runs of the demo plan, at steps it shares with the filmed demo, and the top-camera row from a `REFIT_START=open` run:

| Camera and view | 320 x 176 | 640 x 360 |
|---|---|---|
| Wrist camera, screw in the grip | 45 / 25 | 93 / 51 |
| Wrist camera, screw hanging in the rack as the arm passes | about 30 / 22 | about 60 / 38 |
| Top camera, screw hanging in the rack | 4 / 2 | 9 / 4 or 5 |

At 320 x 176 the top camera shows a shank 1 pixel wide against the rack's shadow, so the wrist cameras show length best. An arm over the stand can hide the rack from the top camera.

Nothing shows depth. With the pulley off, every hole in the stand is the same black dot, and its contrast against the stand follows position, not depth. In grey levels:

| | Deep far-left | Deep near-right | Shallow far-right | Shallow near-left |
|---|---|---|---|---|
| 640 x 360 | 28 | 42 | 23 | 53 |
| 320 x 176 | 14 | 23 | 17 | 26 |

## Assets

| Part | Source | Licence |
|---|---|---|
| Pulley, `moveo/1M1.STL` | [BCN3D Moveo](https://github.com/BCN3D/BCN3D-Moveo/blob/master/STL%20files/Articulation%201/1M1.STL), the arm's base timing pulley; mesh, scale and hole positions unchanged | MIT, `moveo/LICENSE` |
| Aluminium, `textures/machined_aluminium_*.jpg` | [ambientCG Metal051A](https://ambientcg.com/view?id=Metal051A) | CC0 |
| `textures/printed_lines.jpg`, `textures/printed_worn.jpg` | generated by `make_textures.py` | ours |

These parts are ours, not from the arm's CAD:

- The bench stand. The pulley's real mating part, the arm's yoke, stands 200 mm tall and would put the screws out of a straight-down grip's reach.
- The screws. The real holes recess their screw heads where fingers cannot reach, so these knurled heads sit on top of the pulley. They lift out and drop in, without turning.
- The hole depths in the stand, the pins in two of the pulley's six holes, the rack and the mats.
- The worn copy's flattened teeth and faded surface.

## Why a repair guide cares

[iFixit's list of common repair mistakes](https://www.ifixit.com/News/the-10-things-likely-to-go-wrong-when-fixing-a-phone-or-laptop) warns about screws of different lengths and the damage a long one can do in the wrong hole.

## Physics check

`REFIT_CHECK=1` measures, at every physics step, how far a shank goes past the flats of a hole in a pulley on the stand, and how far a head goes into that pulley. The report gives both in mm under `deepest`.

```bash
REFIT_CHECK=1 REFIT_DEMO=safe inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=240 \
    -E cam_width=64 -E cam_height=36 --policy yam_scripted -P hold_s=2 --scorer task_score --max-steps 25000 \
    --instruction "check" --log-dir check
```

## Runtime and seeds

- A model run, 2,400 steps at 20 rows per request: about 7 minutes on an RTX 5090, including Isaac Sim start-up. A scripted run: about 6 minutes. The first start on a new machine adds several minutes while Isaac Sim compiles its shaders.
- The slowest 30 steps took 7.9 s in the model run, so a call of 20 rows waits about 5.3 s for the simulator, within the model session's 8 s idle limit.
- The layout is fixed with `-E object_jitter=0`, so every trial starts identically.

## Licence

MIT.

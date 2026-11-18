# Set a circular saw's depth of cut to 25 mm for 3/4 in boards: free the depth lever, swing the foot plate, lock it

A corded 7-1/4 in circular saw (DEWALT DWE575 class, unplugged) lies on its motor side on the bench, set to its full
65 mm depth of cut. The robot is told to set the saw to cut 25 mm deep and lock it there.

Before ripping 3/4 in (19 mm) boards a carpenter sets the blade to stand about 6 mm past the board: deeper cuts are
slower and kick back more. The foot plate pivots at the front; the depth lever at the rear clamps it. The lever must
be freed, the plate swung on its pivot until the blade stands 25 mm past it, and the lever clamped again. The task
tests a measured adjustment of a pivoting part, held between a release and a lock.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the saw, its foot plate, depth lever and lower guard, `success()` and `score()` |
| `partkit.py` | Rigid parts built from boxes, with hinges, for yam_arena task files |
| `demo.py` | The scripted full-score demo and its negative variants |

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Set this saw to cut 25 mm deep and lock it there." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir dream_out
```

## Scripted demo

```bash
python demo.py task.py demo_out                     # full score: writes demo_*_cam.mp4, setup.png, result.json
DEMO_VARIANT=nolock python demo.py task.py nolock   # set to 25 mm, lever left out: 0.5
DEMO_VARIANT=deep python demo.py task.py deep       # set to 35 mm and locked: 0.25
```

The right arm pushes the depth lever's thumb tab out round its pivot to free the plate, pinches the plate's top edge
near the rear and swings the plate out on its front pivot by the angle that leaves 25 mm of blade past it (from the
saw's geometry), lets go, and pushes the lever back home. The demo reads only the layout and its own joints.

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`).

- 0 if the saw is moved 30 mm or turned 15 degrees. This is permanent.
- 1.0 (success, which must hold for 1 s): the blade 25 mm (within 1.5 mm) past the foot plate's outer face, the depth
  lever home (clamped), both grippers 30 mm or more from the saw, nothing moving.
- 0.75: set and locked, but a gripper is near or something still moves.
- 0.5: set to 25 mm, lever not home.
- 0.25: the depth was changed by 10 mm at some point.
- Otherwise 0.

## Runtime and seeds

About 1 minute per trial for the full 1,200 steps, plus Isaac Sim start-up. The layout is fixed (`object_jitter=0`);
there are no seeds to choose.

## Fidelity

Rigid bodies at true size. The foot plate is a hinge on the saw at its front pivot; the depth clamp is task code on that hinge: freed, the plate sticks where it is put until pushed past 0.3 N m of friction; with the lever home it is held hard where it stood. The lever and the spring-shut lower guard are hinges. The saw is unplugged; no motor is simulated.

## Real-world build

- A corded 7-1/4 in circular saw of the DEWALT DWE575 class, unplugged, its plug taped, laid on its motor side on the
  bench in two outline marks.

Check the result with a small square or depth gauge against the foot plate: 25 mm of blade past it, and the depth
lever home. The saw model shares its geometry with the team's guard task.

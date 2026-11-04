# Pass a garden hose over a fence

A white fence board (20 mm thick, 12 cm tall) stands on the table's midline between the two arms and runs the whole
length of the table, so there is no way round or under it. Left of it is a green grass mat, right of it a brown soil
mat. A green 9 mm garden hose (55 cm) is fixed to a grey tap block on the grass near the fence and lies straight
across the grass, ending in an orange 50 x 22 x 22 mm nozzle. On the soil side stands an open grey trough (inner
12 x 8 cm, walls 4 cm tall). The robot must lift the nozzle over the fence with the left arm, take it on the far side
with the right arm, lay it in the trough with the hose draped over the fence top, and let go. Neither arm can do it
alone: the right arm cannot reach the nozzle's start on the grass, and the left arm cannot reach the trough over the
fence (its reach on the soil side ends about 9-10 cm from the fence; the trough starts 19 cm from it).

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the hose simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "Pass the green garden hose over the white fence: lift its orange nozzle over the fence with the left arm, take it on the far side with the right arm and lay it in the grey trough, leaving the hose draped over the top of the fence. Then let go of the hose."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

Closed loop (reads the live hose state): the left arm grips the nozzle at its hose end, turns it to point across the
fence while bringing it in, lifts it over the fence top (nozzle ~2.5 cm above it) and lowers it on the soil side,
lets go and backs off over the fence. The nozzle hangs against the fence from the hose draped over the top. The right
arm picks the nozzle up where it actually is, carries it into the trough, lets go and moves clear. Success holds 1 s
and ends the episode at step ~410 (~20 s); task_score 1.0. `demo/` holds the videos, `setup.png` and `result.json`.

Shortcut variants (`HOSE_CHEAT=...`, each scores exactly 0):
- `nothing`: no motion.
- `leftonly`: the left arm alone carries the nozzle over the fence and swings it out towards the trough as far as it
  reaches, opening its fingers during the swing (a throw); the nozzle ends hanging against the fence, ~15 cm short of
  the trough.
- `hold`: the full hand-over, but the right arm keeps holding the nozzle in the trough (not released).
- `around`: the left arm carries the nozzle low along the fence to the far end of its reach and offers it to the
  right arm across the fence; the fence runs to the table's far edge, so the hose never gets round.

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- **0** unless both grippers' TCPs (centre between the finger pads) are at least 3.5 cm from every point of the hose
  and nozzle, and the nozzle centre is inside the trough's inner walls (x 24-36 cm, y -19 to -27 cm) below the 4 cm
  wall top.
- Then **0.7**, or **1.0** if the hose's centre line crosses the fence's mid-plane exactly once and there lies
  between 2 cm below and 3 cm above the fence top (resting on the top, not looped over twice or wedged low).
- Real trials: 0 if the fence, trough or tap is moved, or the hose is pulled off the tap.
- `distance()`: horizontal distance from the nozzle centre to the trough centre (m).

## Runtime and seeds

- Run limit 1,200 steps (1 min at 20 Hz). One trial takes ~5-10 min wall time on one GPU including the ~1-2 min
  Isaac Sim start-up.
- Layout is fixed with `object_jitter=0`; the hose is never jittered (it always starts straight), so every trial
  starts identically.

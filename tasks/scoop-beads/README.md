# Move more than four beads into the bowl

A tray of 16 mm beads sits on one side of the bench and an empty bowl on the other. Neither the
gripper nor the flat spatula on the rack can move them: the jaws are two flat plates closing on
spheres, and a blade with no walls spills whatever rides onto it. The scoop beside it has a pan
with three walls, and it works -- but only drawn into the tray's own wall, because a lip cannot
lift a sphere off open floor.

The instruction names neither the tool nor the method: **"Move more than four beads into the bowl."**

## Environment

| | |
|---|---|
| `yam-isaac-lab-arena` | `0059d02` |
| Isaac Lab-Arena | 0.3.1 (installed: `isaaclab_arena` 0.3.0) |
| Isaac Sim / Isaac Lab | 6.1.0 / 3.0 |
| GPU used for the runs below | one RTX-class GPU, `cuda:0` |
| Control | 20 Hz, `object_jitter=0` |

Start every shell with `source <ARENA_DIR>/yam.env`.

## Files

| File | What |
|---|---|
| `scoop_beads.py` | the task file: `SCENE`, `OBJECTS`, `success`, `distance`, `score`, `demo` |
| `bench.py` | the scene's geometry and the measurements the scorer takes |
| `simkit.py` | the spawn helper: materials, compound bodies, per-surface friction, smooth motion |
| `shortcuts.py` | four scripted shortcut policies |
| `check_layout.py` | offline checks: the geometry, the clearances, the demo's IK, the dig mechanics |

No asset files ship with the task; the bowl and the spatula are models from Arena's own library.

## Running it

```bash
cd <this repo>

# the scripted demo (reaches full score)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/scoop-beads/scoop_beads.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Move more than four beads into the bowl." --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/sb-demo

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/scoop-beads/scoop_beads.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Move more than four beads into the bowl." --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/sb-dream

# one shortcut policy (hold | gripper | spatula | shove)
SCOOP_SHORTCUT=spatula inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/scoop-beads/shortcuts.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Move more than four beads into the bowl." --scorer task_score \
  --max-steps 2400 --log-dir ~/yam-eval/sb-sc-spatula

# offline, no GPU
cd tasks/scoop-beads
python check_layout.py
```

`SCOOP_TRACE=1` on any run prints the scoop's pose and tilt, how low its lip is sitting, where the
heap has been pushed to and where every undelivered bead is, every twentieth control step; that is
how the numbers below were measured.

**Seeds and determinism.** The layout is fixed: with `-E object_jitter=0` nothing is moved at reset,
so there is one starting state and no seed to set. The scripted demo is open loop, so it replays
identically. PhysX is not bit-deterministic (`deterministic=False` in Arena's default `PhysxCfg`),
so individual beads take different paths between runs; the demo's first scoopful has delivered
between five and seven beads against a threshold of five.

**Runtime per trial.** The plan is 108 s of motion, 2,170 control steps at 20 Hz, but the episode
ends on success as soon as the fifth bead settles -- about 28 s in, on the first scoopful. Wall
clock: roughly 1 minute per trial once Isaac Sim is up, 3 to 6 for the first in a fresh process.

## Results

| Policy | `task_score` | Notes |
|---|---|---|
| `yam_scripted` (the demo) | **1.0** | 7 beads delivered on the first scoopful; ends with `termination_reason="success"` |
| `dream` (Dream-1 beta) | **0.0** | ran the full 2,400 steps, no bead delivered |
| shortcut `hold` | 0.0 | both arms hold the ready pose |
| shortcut `gripper` | 0.0 | open jaws driven into the heap and closed: two flat plates on 16 mm spheres |
| shortcut `spatula` | 0.0 | the flat spatula drawn through the beads the way the scoop is; nothing stays on it |
| shortcut `shove` | 0.0 | both arms lean on the tray, which is a fixed collider |

## Why a scoop, and why the tray has to be full

Three measurements decided the design, all of them in `check_layout.py` as checks:

- **A lip cannot lift a sphere off a flat floor.** The push lands below the bead's centre, so it
  rolls ahead of the blade instead of riding up it: the heap was driven from +83 mm to +135 mm
  across the dish without a single bead being collected. What loads the pan is the charge behind
  the lip wedging the beads in front of it up into it, which is why the tray is filled rather than
  sparsely loaded -- four beads alone were picked up by nothing at all.
- **The wall has to be taller than a bead and shorter than the handle's clearance.** Against the
  27 mm rim this started with, 30 mm beads rode straight over and finished on the bench. Against a
  45 mm wall the hand could not get inside. 16 mm beads in a 24 mm tray satisfy both.
- **The pan has to be drawn into the wall, not stopped short of it.** Ending the draw at +20 mm
  left the lip at +55 while the heap's far edge was at +110: the pan was stopping in the middle of
  what it had pushed and lifting out empty.

## Why the pour is aimed short of the bowl

Tipping rotates the scoop about the jaws, 170 mm behind the pan, so the lip swings through a long
arc as it goes over and throws the beads about 100 mm past wherever the pan is held -- measured, 81
to 122 mm beyond the bowl's middle. The pan is therefore held 70 mm short of the bowl. At the
300 mm separation this started with, no such pose was reachable on any heading; drawing the pair to
240 mm brought it inside the arm's envelope.

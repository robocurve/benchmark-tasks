# Take a plugged cord out of a board hole and bin it

An e-waste strip-out bench. A dark-brown wooden upright board (240 x 12 x 50 mm) stands on the table's midline between the
two arms, with a 14 x 14 mm square hole low in its middle. A grey 6 mm cord (30 cm) passes straight through the hole:
on the left-arm side it ends in a black moulded plug (40 x 24 x 18 mm, far too big for the hole), on the right-arm
side in a bare end with a small copper crimp (10 x 8 x 8 mm, fits through). A blue open bin stands on the right-arm
side. The robot must take the cord out of the board, drop it into the bin and let go of it. This is a toy-scale
mock-up of stripping external cables from e-waste.

The obvious move, pulling the bare end (next to the bin) towards the bin, jams the plug against the hole. The cord has
to come out towards the plug side. The intended solution: the left arm draws it out by the plug, lifts it over the
board and sets the plug down on the bin side, and the right arm carries it into the bin (the plug starts on the left
side, 20 cm from the board; the bin is on the right side, 16-32 cm from the board). A single arm could also push the
bare end back through the hole and then drag the cord over the board into the bin; the scorer would give that the same credit (not run).

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1600 \
    --instruction "Take the grey cord out of the hole in the wooden board, drop it into the blue bin and let go of it."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1600
```

Closed-loop (reads the live cord state). The left arm pinches the plug at its middle (wrist camera looking back at
the board) and pulls along the hole axis until the crimp is 3 cm clear of the board, turns the plug to lie along the
board, lifts it over the board top and sets it down 5 cm beyond the board on the right-arm side, lets go and moves
away. The right arm picks the plug up, lifts it to 16 cm above the table, carries it over the bin (drawing the rest of the cord over the
board) and sets it in the bin's far corner. The crimp end still trails over the bin's near wall, so the right arm
picks up the middle of the part outside the bin and lays it in (repeated if needed), then moves away (success holds 1 s and ends the episode at step 721, ~36 s; task_score 1.0). `demo/` holds
the recorded run (videos, `setup.png`, `result.json`).

Shortcut variants (env var `CORD_CHEAT`, each must score 0): `nothing` (no motion); `bare` (the right arm pulls the
bare end into the bin: the plug jams in the hole, the cord still passes through it); `hold` (full demo, but the right
arm keeps holding the cord over the bin). `onearm` (the left arm alone withdraws the cord, carries the plug over the board
to y -10 cm and drops it short of the bin) scores the partial 0.3 for a correct extraction (nothing lands in the bin).
The shortcut runs were made before a colour-only change of the board (light wood to dark brown); the scene
configuration was otherwise identical, and the demo was re-run after the change (1.0, step 721).

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- **0** if the cord's centre line still crosses the board's mid-plane inside the hole (3 mm tolerance), if either
  gripper's TCP is within 3 cm of any point of the cord, plug or crimp (still holding it), or if the state is invalid.
  Damage is not scored (the sim cord cannot tear and the board and bin are fixed).
- Otherwise **0.3 + 0.7 x f**, where f is the fraction of the cord's length (crimp to plug tip, sampled every 5 mm)
  inside the bin's inner walls and below its 4 cm rim; **1.0** when all of it is in the bin.
- `progress()` reports the stages (plug moved, out of the hole, fraction on the bin side, fraction in the bin,
  released); it does not change the score.

## Runtime and seeds

- Run limit 1,600 steps (80 s at 20 Hz). One trial takes ~4-6 min wall time on one GPU including the ~1-2 min Isaac
  Sim start-up.
- Layout is fixed with `object_jitter=0`; the cord is never jittered (it always starts straight through the hole), so
  every trial starts identically.

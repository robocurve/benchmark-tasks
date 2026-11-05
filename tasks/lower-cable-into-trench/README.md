# Lower a cable into a trench without touching the edge

A toy cable trench runs left-right in front of the robot: two grey slabs 4 cm thick, laid on the table (not dug),
leave a 6 cm wide, 4 cm deep slot between them, closed at both ends by 2 cm blocks. The rim (the trench edge) is
orange: a 1 cm strip along each long side and the tops of both end blocks. A free black 8 mm
cord, 50 cm long with a rigid end cap on each end, lies straight on the near slab, parallel to the trench and 8 cm in
front of it. The robot must lower the whole cord into the trench **without any part of it ever touching the trench
edge**, then let go. The edge is checked over the whole episode, not only at the end: pushing the cord over the edge,
dragging it in end-first, or laying it in bit by bit across the rim all fail. It is a toy-scale stand-in for laying
underground cable directly in a trench (O*NET 49-9051.00, Electrical Power-Line Installers and Repairers). The
sources are about pulling cable over edges, not about lowering: a Transpower NZ quality alert (2018-007) says "The
majority of cable outer sheath damage occurs during installation when the cable is being pulled and rubs against
sharp objects such as rocks or the edge of ducts", and Belden's fiber cable installation guideline (2020) says "Avoid
pulling cables over edges." This task applies the same no-contact-with-the-edge rule while lowering a cord in.

**Why two arms.** With a straight-down grip the highest TCP over the slab near the middle is about 20 cm above the
table (16 cm above the slab tops), so one arm holding the cord by its middle leaves both 28.5 cm halves (cap
included) resting on the slab; carrying it over the slot drags them towards and over the edge (shortcut `onearm`
below). Neither arm reaches the far cap (straight-down IK: near the slab the left arm reaches no further than
about y -16 cm; the caps are at y ±27 cm), so one arm cannot reach both caps. The demo lifts the cord by both caps, held nearly straight, and lowers it in level. A two-arm sequence that
lays one end in first is also checked (`oneend`, scores 0). Grasps with a tilted wrist were not checked.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "Lower the whole black cable, end caps included, down into the trench without letting any part of it touch the orange trench edge, then let go of it."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

Closed loop (reads the live states): each arm pinches the end cap on its side (fingers across the cap), both lift the
cord 15 cm above the slab with it held nearly straight, carry it over the slot's centre line (aiming the measured
caps, not the grippers), wait until it stops swinging, lower it straight down until the caps are 5 mm above the rim,
let go (the cord drops about 4 cm onto the trench floor) and back away. Success holds 1 s and ends the episode at
step 326 (~16 s); task_score 1.0. `demo/` holds the videos, `setup.png` and `result.json`.

Shortcut variants (test only, `TRENCH_DEMO=<name> python cabledemo.py task.py out`), all score exactly 0:

| variant | what it does | why 0 |
|---|---|---|
| `nothing` | waits | cord not in the trench |
| `push` | both arms push the cord sideways across the slab and over the edge | middle falls in; 32.6 cm of cord touched the edge, caps still on the slab |
| `onearm` | the left arm lifts the cord by its middle as high as it reaches, carries it over the slot, lowers and lets go | the halves drag on the slab; the middle is in, the caps stay on the slab across the rim (2.4 cm touched) |
| `oneend` | the left cap is laid in the trench first, then the right cap is carried over 3 cm above the slab | both caps in, middle still across the rim; 10.4 cm touched |
| `hold` | the full demo, but the grippers never open | the cord is held just above rim height (lowest point 3 mm above), not in the trench, and gripped |

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- **Touched length**: in a real trial the rim edge (the corner where the slab top meets the slot wall) is rubbed
  with chalk before the trial and the total length of
  chalk marks on the cord and caps is measured with a tape. In the sim, the cord and caps are sampled every 2 mm (caps
  every 4.4 mm) after every physics step; a sample whose centre comes within its radius + 2 mm of the rim line (where
  the slab top meets the slot wall, all four sides) is marked, and the marks are latched for the episode. Cap radius:
  10 mm (half-diagonal of its cross-section).
- **0** if more than 1 cm of the cord touched the rim (dragged over it or rested across it); if at the end any point on
  the centre line of the cord or caps is not inside the slot below the rim; or if a gripper centre (TCP) is within 3 cm of the cord or
  caps (not released; other arm parts are not checked).
- Otherwise **1.0** if nothing touched the rim, **0.5** if 1 cm or less did (a graze).
- Release height is not scored: the demo lets go with the caps 5 mm above rim height and the cord drops about 4 cm.
  A sample passing within 2 mm of the corner without touching it still counts as touched (conservative).

`progress()` reports edge_clean, not_dragged, lifted_clear, in_trench (fraction of length) and released.

## Runtime and seeds

About 16 s of simulated time for the demo (326 steps); the run limit is 1,200 steps (1 minute at 20 Hz). The layout
is fixed (`object_jitter=0`); the cable is not jittered. `VARIATION` in `task.py` lists the ranges a seeded runner may
apply (cord start ±1.5 cm, yaw ±0.05 rad).

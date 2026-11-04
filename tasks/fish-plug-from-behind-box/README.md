# Fish a charger out from behind a box by its cord

An unplugged phone charger (45 x 26 x 16 mm brick) has fallen into the 35 mm gap between a free 2 kg storage box
(300 x 120 x 200 mm) and a fixed back panel. It is hidden behind the box; only its white 6 mm cord (25 cm, with a
small USB-C plug on the far end) comes out past the box's left end onto the open table. The box and panel are too
tall for a gripper to reach down into the gap, so the charger can only be fished out by its cord. The gap is
narrower than the charger is long, so the charger cannot turn inside it: pulling the cord towards the robot wraps
it round the box's back corner, wedges the charger between that corner and the panel, and drags the box (about 5 cm
in the sim). The cord must be drawn out lengthwise along the gap until the charger is past the panel's end, then
brought round and put in a green tray in front of the right arm. Only the left arm reaches the cord and the gap's
exit, and only the right arm reaches the tray, so the charger is handed across.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner). The task
  uses only kit features (a free cable with a rigid plug on each end, free and fixed boxes).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "The unplugged phone charger has fallen into the gap behind the brown box. Fish it out by its white cord and put it in the green tray, then let go. Don't move the box: keep it in place (within 2 cm) and upright."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

Closed-loop (reads the live cord state). The left arm grips the USB-C plug, its wrist camera looking back along the
gap at the box corner, and draws the cord 17.5 cm straight along the gap line until the charger has slid out past
the panel's end (it retries with a new grasp if the charger is not yet clear). It then picks the charger, carries
it round the box's left end (not over the box) and sets it down in front of the box at the midline. The right arm
picks it up, its wrist camera facing the tray, lays it in the tray and lets go. Success holds for 1 s and ends the
episode at step 554 (~28 s); task_score 1.0. `demo/` holds the videos, `setup.png` and `result.json`.

Unintended strategies (all score exactly 0), selected with `FISH_DEMO=<mode>`:

| `FISH_DEMO=` | what it does | result |
|---|---|---|
| `nothing` | waits | 0 (charger still behind the box) |
| `yank` | grips the USB-C plug and pulls the cord straight towards the robot | 0 (charger wedges at the box's back corner; the box is dragged ~5 cm) |
| `shove` | pushes the box's left end to the right first (the box moves ~4 cm), then does the correct retrieval | 0 (box moved more than 2 cm; the charger is fished out, which alone would earn 0.5) |
| `hold` | correct retrieval, but the right arm keeps holding the charger in the tray | 0 (gripper not released) |

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- **0** if at any control step a box corner moved more than 2 cm horizontally from its start, the box tilted more
  than 10°, or the box or charger fell off the table (tracked over the whole episode, reset at every episode start).
- **0** if at the end either gripper (point between the finger pads) is within 3 cm of the cord, the charger or the
  USB-C plug.
- **1.0** if the charger lies in the tray: centre over the inner floor at least 5 mm in from the walls, resting on
  the floor.
- **0.5** if the charger is fished out but not in the tray: centre at least 5 cm nearer the robot than the box's
  back face (x ≤ 38 cm), lying on the table.
- Otherwise 0.
- `distance()`: charger-to-tray-centre distance (m), 1.0 once the box has been disturbed.
- `progress()` (diagnostic only): charger out of the gap, fished out, in tray, box kept, released.

## Runtime and seeds

- Run limit 1,200 steps (60 s at 20 Hz); the demo needs 554. One trial takes ~5–10 min wall time on one GPU,
  including the ~1–2 min Isaac Sim start-up.
- Layout is fixed with `object_jitter=0`; the cable is never jittered (it always starts straight in the gap). With
  jitter > 0 the box would be shifted but the cord not, so always run with `object_jitter=0`.

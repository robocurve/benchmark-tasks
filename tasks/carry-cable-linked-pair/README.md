# Carry two cord-linked devices onto a shelf without pulling the cord

An action camera (5 x 4 x 4 cm, 80 g) sits in front of the left arm and a power bank (6 x 3.5 x 2.8 cm, 120 g) in
front of the right arm. A 28 cm cord, fixed into both devices, lies straight and just taut between them. Both
devices and the cord must end up on a raised wooden shelf (12 x 40 x 3 cm) farther from the robot, released. Each
device can be reached only by its own arm, and because the cord is already taut, moving one device towards the shelf
on its own drags the other one along the table by the cord. The robot has to treat the pair as one tethered load:
lift both, keep the cord slack while carrying (the scripted demo carries them together in lockstep with each device
eased 8 mm towards the other), and set them down on the shelf with the cord lying on it. Dragging, sliding or
hanging either device by the cord is checked at every control step, not only at the end.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (cable simulation and scripted-demo runner). `task.py` adds
  two fixed joints on top of the kit that weld the cord's first and last segments to the two free devices (the kit
  itself can only weld a cable end to the world).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "Put the action camera, the power bank and the cord joining them on the wooden shelf, then let go. The cord is already taut: lift and carry the devices so the cord never pulls on either one. Never drag, slide or hang a device by the cord, and leave no part of the cord hanging off the shelf."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

Closed-loop (reads where the devices actually are): both arms grip their device from above at the same time (yaw 0,
so both wrist cameras look forward at the shelf), lift it while easing 8 mm towards the other device (16 mm of cord
slack), carry both 21 cm straight forward in lockstep with the cord hanging in a shallow loop between them, set them
down on the shelf side by side and release. Success holds for 1 s and ends the episode at step ~216 (~11 s);
task_score 1.0. `demo/` holds its videos, `setup.png` and `result.json`.

Unintended strategies (all score exactly 0), selected with `PAIR_DEMO=<mode>`:

| `PAIR_DEMO=` | what it does | result |
|---|---|---|
| `nothing` | waits | 0 (nothing on the shelf) |
| `drag` | left arm carries the camera onto the shelf alone | 0 (the cord drags the power bank along the table) |
| `lone` | right arm carries the power bank onto the shelf alone | 0 (the cord drags the camera along the table) |
| `hold` | both carried and set on the shelf correctly, but the grippers never let go | 0 (not released) |

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- **0** if at any control step a device slid more than 3 cm along the table or shelf from where it rested (its start
  spot, then wherever it was last set down after being lifted), hung more than 15 mm above any surface with no gripper
  TCP within 15 mm of its box (pulled up by the cord), leaned more than 30° or fell off the table. Tracked in module
  state that resets at every episode start.
- **0** if at the end any gripper TCP is within 2 cm of either device or the cord (not released).
- Otherwise **1.0** if both devices are on the shelf (whole footprint over the shelf, bottom within 8 mm of the shelf
  top, tilt ≤ 10°) and every point of the cord is over the shelf and at most 12 mm above its top; **0.5** if both
  devices are on the shelf but part of the cord is not; **0.25** if exactly one device is on the shelf; else 0.
- `distance()`: summed distance of the two devices from the shelf centre line (m); 1.0 once a rule is broken.
- `progress()`: stage diagnostic (both lifted, each device on the shelf, cord on shelf, released, never dragged).

## Runtime and seeds

- Run limit 1,200 steps (1 min at 20 Hz). One trial takes ~3–6 min wall time on one GPU including the ~1–2 min
  Isaac Sim start-up.
- Layout is fixed with `object_jitter=0`; the cable is never jittered (it always starts straight). With jitter > 0 the
  devices would be shifted but the cord not, breaking the welds' geometry, so always run with `object_jitter=0`.

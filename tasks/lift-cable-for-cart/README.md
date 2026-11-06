# Lift a cable so a cart can pass

A black 8 mm power cord crosses a cart lane on the table. It runs straight across, 28 cm in front of the robot,
between two spring-loaded cord retractors (a grey floor box on the right, a yellow reel behind a grey partition on
the left; the cord passes the partition through a notch at its bottom edge). A white inline switch box sits on the
cord near the middle. A light blue cart block stands in the lane in front of the left arm, 2.4 cm before the cord;
two orange marks 6 cm beyond the cord line are the finish line. The robot must get the cart fully past the finish
line, passing under the cord inside the lane, without it ever touching the cord and without lifting it off the table
(a cart is rolled, not carried), then lay the cord back down on its line and let go.

The task is bimanual by construction: a released cord falls straight back onto the floor (the retractors pull it
taut along its line), so one arm has to keep the cord lifted while the other moves the cart under it, and the cart
pusher cannot follow the cart under the cord (the gripper is taller than the clearance), so it has to release the
cart, reach over the held cord and draw the cart on from its far end.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "Get the blue cart past the orange finish marks without it ever touching the black power cord: lift the cord with one arm and slide the cart under it with the other. Keep the cart on the table. Then lay the cord back down along its line and let go of it."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

Closed-loop (reads the live cart and cord states): the right arm takes the cord by its switch box and lifts it to
~14 cm above the table (cord ~6 cm above the table where it crosses the cart's path; the retractors pay out
~8 and ~3 cm and keep the cord taut). The left arm, fingers closed as a pusher, pushes the cart from behind until its
front 9 cm are past the cord, stopping 4.5 cm short of the cord line; it then rises to 17 cm, moves over the held
cord, grips the cart's front end and slides it on until the rear edge is 2 cm past the finish line, and lets go.
The right arm lowers the cord back onto its line and lets go; the retractors take the slack back in. Minimum
cart-cord gap during the run: 25 mm; the cord's underside is 28 mm above the cart's top as the cart's centre crosses x 28. Success holds for 1 s and ends the episode at step 451 (~23 s);
task_score 1.0. `demo/` holds its videos, `setup.png` and `result.json`.

Shortcut / failure variants (test only, `CART_DEMO=<name> python cabledemo.py task.py out`), all score exactly 0:
`nothing` (do nothing: cart never past the line), `through` (lift nothing, push the cart straight across the
cord lying on the floor: touch), `carry` (pick the cart up and carry it over the cord through the air: lifted),
`lifthold` (lift the cord and keep holding it, cart unmoved: cart never past the line), `round` (never lift the cord:
slide the cart round the floor outlet's open end (cart centre beyond y -30 cm), past x 28 cm and back into the lane
beyond the finish line: crossed the cord line outside the lane), `low` (grip the cord but raise it only ~1 cm, below
the cart's top, and push the cart across in the lane: the cart does not pass under the cord).
`hold` (full demo, but the cord is kept lifted) is the partial-credit case: cart past, cord not back = 0.7.

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- **The cart passes under the cord.** Each time the cart's centre crosses the cord line (x 28 cm), in either
  direction, it must be within 5 cm of the lane centre line (y 12 cm), and the underside of the cord (switch box
  included) everywhere over the cart's width plus 1 cm on each side must be at or above the cart's top (3 cm above
  the table). A crossing that fails either check is a foul (score 0); a cart taken round either end of the cord fails
  the lane check.
- **0** if, at any control step of the episode, the cart's surface came within 2 mm of the cord or the switch box
  (touching), the cart's bottom rose more than 1 cm above the table (lifted / carried), the cart tilted more
  than 15 deg, or a crossing of the cord line was not under the cord (above). These latch for the whole episode and
  are cleared only when a new episode starts (embodiment reset), never by putting things back.
- **0** if the cart never passed under the cord (no forward crossing that met both checks), or if the cart is not fully past the finish
  line at the end: its rear edge beyond x 34 cm, its centre within 5 cm of the lane centre line (y 12 cm), upright on
  the table. Doing nothing therefore scores 0, and the cord being back in its line earns nothing on its own.
- **0.7** at the end: cart fully past the line after passing under the cord, no contact ever, cord not back on its
  line or still held.
- **1.0** at the end: the same, and the cord between the right retractor and the partition lies on the table (centre at
  most 1.5 cm above it) within 3 cm of its line (x 28 cm) and both grippers' TCPs are at least 3 cm from every
  point of the cord, switch box included (let go).
- `success()` additionally requires the cart and cord at rest. `distance()`: cart rear edge to the finish line (m),
  1 m once a disqualifier has fired.

## Runtime and seeds

- Run limit 1,200 steps (60 s at 20 Hz). One trial takes ~5-10 min wall time on one GPU including the ~1-2 min
  Isaac Sim start-up.
- Layout is fixed with `object_jitter=0`; the cord is never jittered (it always starts straight), so every trial
  starts identically.

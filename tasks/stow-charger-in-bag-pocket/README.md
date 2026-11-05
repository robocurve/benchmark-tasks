# Hold a bag's pocket flap open and stow a charger inside

A flat navy travel bag lies fixed on the table. Its top has a pocket (opening 5 x 20 cm, 5 cm deep) closed by a
stiff blue flap hinged along the opening's far edge. The hinge stops at 80 degrees, so the flap falls shut under its
own weight as soon as nothing holds it up. A white phone charger (30 cm cable, charger brick on one end, USB-C plug
on the other) lies straight on the bag top in front of the pocket. The robot must hold the flap open by its grey
knob, put the brick, the whole cable and the plug into the pocket, let the flap fall shut over it and let go.

The task needs both arms at the same time (simultaneous hold): one arm holds the flap up for the whole insertion
while the other packs the brick and the cable, a deformable object that has to be guided into a narrow opening
without leaving a loop or the plug hanging out. Pushing the cable against the closed flap, packing only the brick
or keeping hold of the charger all score 0.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).
  `task.py` adds two joints on top of the kit: a revolute hinge (0 to 80 degrees) between the world and the flap,
  and a fixed joint welding the knob to the flap.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
    --instruction "Pack the white phone charger into the bag's top pocket: hold the pocket flap open by its grey knob, put the brick, the whole cable and the plug inside, then let the flap fall shut over it and let go of everything."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 2400
```

Closed loop on the live object states. The left arm pinches the knob and swings it up along the hinge arc to 70
degrees (targets corrected by the measured knob offset in the grip) and holds the flap there. Meanwhile the right
arm picks up the brick across its width (wrist camera on the opening side), waits for the flap, carries the brick
over the right half of the opening, lowers it in and lets go. It then takes the USB-C plug where it now lies,
carries it low to just above the opening, raises it as high as it reaches so the cable hangs from the plug down
into the pocket (where the brick holds its other end), lowers it to the pocket floor and lets go, then presses down
anything still sticking up. The left arm lowers the flap back along its arc to 8 degrees, opens and moves away; the
flap drops shut. Success holds for 1 s and ends the episode at step 713 (~36 s); task_score 1.0. `demo/` holds the
videos, `setup.png` and `result.json`.

Unintended strategies (each scores exactly 0), selected with `STOW_DEMO=<mode>`:

| `STOW_DEMO=` | what it does | result |
|---|---|---|
| `nothing` | waits | 0 (charger on the bag top) |
| `brick_only` | flap held open, brick put in the pocket, cable and plug left on the bag top, flap closed | 0 (cable and plug outside the pocket) |
| `hold` | everything packed, but the right hand keeps holding the plug inside the pocket while the flap is closed | 0 (gripper still on the plug; the flap rests on the arm) |
| `onearm_push` | one arm only, flap never held: brick set against the closed flap's front edge and shoved in under it, pressed down; then the same with the USB-C plug | 0 (the brick slides over the shut flap onto the bag behind it; the flap never opens) |
| `prop` | brick and cable packed, USB-C plug left on the opening's front rim, flap closed onto it | 0 (flap propped open at 11 degrees on the plug) |
| `drape` | brick packed, cable laid over the front edge with the plug outside on the bag top, flap closed | 0 (cable and plug outside the opening) |

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- **1.0** if at the end both grippers are at least 3.5 cm from the charger, the flap and the knob, the flap is shut
  (within 3 degrees of flat), and every point of the cable centre line, the brick centre and the plug centre lie
  inside the opening's footprint (x 25 to 30 cm, y -10 to +10 cm) and at least 3 mm below the bag top.
- **0** otherwise. In a real trial also 0 if the bag was moved more than 1 cm (it is fixed in simulation).
- There is no partial rung: the pocket is 5 cm deep, so a charger that is fully inside never keeps the flap from
  shutting; a flap resting on the charger means part of it sticks out of or lies outside the opening.

`progress()` reports stages for analysis only (brick in pocket, fraction of the charger inside, flap shut, released).

Runtime per trial: 2,400 control steps (2 minutes at 20 Hz). Seeds: the layout is fixed with `object_jitter=0`;
the cable is never jittered.

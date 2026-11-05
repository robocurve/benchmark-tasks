# Keep a condensate drain hose sloping downhill

An air conditioner's white indoor-unit box stands on the robot's left. Its 8 mm off-white PVC condensate hose
(56 cm) leaves an outlet 14 cm above the table, hangs down to the table and lies straight towards the robot's right,
ending in a small grey nozzle. On the right, a drain tundish (open box, rim 4.5 cm above the table) stands on a
plinth. Between them, further from the robot, two grey steps of decreasing height (10.5 and 8 cm) form a bent
path from the unit to the drain. The robot must lay the hose over both steps and put the
nozzle inside the tundish so that the hose **falls continuously** from the outlet to the drain, then let go.
Carried straight across, the hose sags onto the table and has to climb back up into the raised tundish: a water
trap, which the International Mechanical Code (sec. 307.2: uniform slope, at least 1/8 in per ft) and AC
installation manuals (drain pipe inclined downward, no trap, not run up and down) rule out. Laid over the steps,
it falls all the way. The steps are off the straight line between the unit and the drain, so a free-hanging hose
does not find them by itself.

Two arms are natural (simultaneous hold): one lays the hose's middle on the low step (the part from the outlet
drops onto the high step) and holds it there while the other carries the nozzle to the drain and seats it; one arm
alone can't hold the hose up on the steps while seating the nozzle, and the left arm can't reach the drain well
while the right arm can't reach the high step.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (hose simulation and scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "Route the air conditioner's white condensate hose into the drain tundish so that it slopes downhill all the way from the unit to the drain, with no dip that would trap water: lay it over the two grey steps and put the nozzle inside the tundish. Then let go of the hose."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

Closed loop (reads the live hose): the left arm pinches the hose at the arclength that belongs on the low step
(straight-line distance from the outlet plus 4 mm), the right arm pinches the nozzle. Both lift; the left carries
its point over the low step and lowers it onto it (the hose from the outlet settles onto the high step)
while the right carries the nozzle, wrist camera ahead towards the tundish, to 1.2 cm above the rim. The right
lets go (nozzle drops in), then the left lets go and both move clear. task_score 1.0 (success held 1 s, episode
ends at step 286, ~14 s). `demo/` holds its videos, `setup.png` and `result.json`.

Shortcut variants (`DRAIN_CHEAT=...`, each scores exactly 0):
- `nothing`: no motion.
- `direct`: the right arm alone carries the nozzle into the tundish; the hose sags to the table and climbs 45 mm.
- `onestep`: the hose is picked where it lies on the table, laid on the high step only (skipping the low step),
  then the nozzle is put in the tundish: 0 by the both-steps rule (and its total rise is 22 mm anyway).
- `onestep_taut`: tries to pick the hose where it hangs near the outlet to lay it taut over the high step; the
  hanging hose slips out of the open fingers, so it ends as `direct` (45 mm rise).
- `hold`: the full demo, but the right gripper keeps holding the nozzle down in the tundish.

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- **0** if either gripper's TCP (centre between the finger pads) is within 3.5 cm of the hose or nozzle.
- **0** if the nozzle centre is not inside the tundish (within its 6.5 x 6.5 cm inner opening, below the rim).
- **0** unless the hose rests on both steps: for each step, some hose segment centre lies inside the step's
  footprint and 0-15 mm above its top (the 8 mm hose's underside touching it or at most 1.1 cm above it).
- Otherwise the hose centre line's height profile from the outlet to the nozzle (outlet tip, every 2 cm segment
  centre, hose end) gives the **total rise**: walking from the outlet, a climb counts once the line is more than
  2 mm above the lowest point since the last counted climb, and then counts in full from that low point (wiggles
  under 2 mm are ignored, real climbs are summed). **1.0** if total rise <= 5 mm, **0.5** if <= 15 mm, else **0**.
- Real trials: 0 if the unit, a step or the drain is moved.
- `distance()`: nozzle-to-tundish-centre distance plus total rise (m). `progress()`: stage diagnostic (nozzle
  grasped, on_step_high, on_step_low, nozzle in drain, rise <= 15 mm, rise <= 5 mm, released).

## Runtime and seeds

- Run limit 1,200 steps (1 min at 20 Hz). One trial takes a few minutes wall time on one GPU including the
  ~1-2 min Isaac Sim start-up.
- Layout is fixed with `object_jitter=0`; the hose always starts straight (it is never jittered), so every trial
  starts identically. `VARIATION` lists offsets a later seeded runner may apply.

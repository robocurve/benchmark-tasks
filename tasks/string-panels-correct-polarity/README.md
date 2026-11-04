# String three solar panels in series with correct polarity

Three small dark-blue "PV module" blocks stand in a row at the far side of the table (left, middle, right). Each has
a short red (+) and a short black (-) 7 mm lead lying on the table towards the robot, ending in a rigid connector
body. Two fixed grey in-line couplers (a U-channel with a 2 mm centre stop) sit between the modules, closer to the
robot. The robot must string the modules in series like a PV installer mating MC4 leads: left module red + middle
module black in one coupler, middle red + right black in the other, one connector entering from each side until it
touches the centre stop. The left module's black lead and the right module's red lead are the string's home-run
ends and must stay free. The layout is a derangement: on each coupler's outer side the nearest module lead is the
home-run lead (left black, right red), and pairing "any red with any black" at each coupler gives a reversed string, so a
policy that pairs by position or colour alone scores 0. The task is bimanual: only the left arm reaches the left
module and only the right arm the right module; the middle module's leads go to the opposite couplers, and the demo
has each arm reach across the midline for one of them while the other arm stays clear.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1400 \
    --instruction "Wire the solar panels in series, left to right: in each grey coupler, push a panel's red (+) lead and the black (-) lead of the panel to its right in from opposite sides until both touch the centre stop. Leave the left panel's black and the right panel's red lead free for the inverter, then let go."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1400
```

Closed-loop (reads the live connector poses). Each grip is turned so the wrist camera looks along the connector
towards the coupler's centre stop while it is carried in and inserted. Both arms at once: each picks its outer
panel's correct lead (left red, right black) near the connector's lead end, turns it in place to point at its
coupler, sets it down and grips it again from the other side (the wrist can't reach a camera-forward grip on a
connector pointing at the robot), carries it to 2 cm outside the channel, lines it up on the channel axis, slides it
against the centre stop (backing off and retrying if it stalls at the mouth) and lets go. Then, one at a time with
the other arm parked, the right arm reaches across to plug the middle black lead into the left coupler and the left
arm the middle red lead into the right coupler (gripping each along its tip directly). `demo/` holds the videos,
`setup.png` and `result.json`: success holds for 1 s and ends the episode at step 927 (~46 s); task_score 1.0.

Shortcut / failure variants (test only, `STRING_DEMO=<name> python cabledemo.py task.py out`):

| variant | what it does | score |
|---|---|---|
| `nothing` | does nothing | 0 |
| `nearest` | position only: plugs the lead nearest each coupler side (left black + middle black; right red + middle red) | 0 |
| `redred` | left coupler correct, right coupler gets the two red leads (middle red + right red) | 0 (one wrong pair zeroes everything, although the left pair alone would earn 0.5) |

Also available, not part of the shortcut set: `hold` (full demo, then the right arm re-grips a connector and keeps
holding it: 0 by the release rule) and `half` (left coupler only, correctly: expected partial credit 0.5).

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- **0** if a gripper centre is within 3 cm of any connector centre (still holding/touching).
- **0** if a home-run connector (left black, right red) is at a coupler (centre within 3 cm across / 6 cm along the
  coupler from its stop, below 4 cm), or if a coupler zone holds two connectors that are not one of the correct
  series pairs (red-red, black-black, a module's own red + black, or a red/black pair from the wrong modules).
- Otherwise **0.5 per correct pair** (left red + middle black; middle red + right black) seated in one coupler: one
  connector on each side of the stop, centre within 8 mm of the channel line, lying on the table (centre at most
  6 mm above lying flat), pointing at the stop (within ~30 deg of the channel axis), front end within 5 mm of the
  stop face. Either pair may use either coupler. Both pairs = 1.0.
- `success()` additionally requires all leads at rest. `distance()`: summed distance of the four connectors to their
  seated poses, 1 m once a wrong connection exists.

## Runtime and seeds

- Run limit 1,400 steps (70 s at 20 Hz). One trial takes ~5–10 min wall time on one GPU including the ~1–2 min
  Isaac Sim start-up.
- Layout is fixed with `object_jitter=0`; the leads are never jittered (they always start straight from their
  anchors), so every trial starts identically.

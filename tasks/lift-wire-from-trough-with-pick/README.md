# Lift a wire out of a deep trough with a hook pick

A loose yellow wire offcut (6 mm PVC, 24 cm) lies on small support ribs at the bottom of a white PLA wiring trough
(28 mm wide inside, 60 mm deep, 30 cm long) that runs left-right in front of the robot. The gripper cannot reach the
wire: the fingers widen from 8 mm at their tips to 36 mm 5 cm above them. Closed, they wedge between the walls with
their tips just above the wire, where they cannot grasp it. Opened, they jam even higher up. A blue L-shaped hook pick (10 x 10 x 100 mm shank with a 14 mm foot at
the bottom) stands in a holder on the right. A grey tray sits on the left. The robot must use the pick to lift the
wire out of the trough, lay the whole wire in the tray and let go of everything.

This is a small version of a common maintenance job: retrieving a dropped wire offcut, clipping or lacing cord from
a narrow wire run where fingers do not fit, so that no debris is left in the wiring. It tests tool use (fetching the
tool, putting its foot under a thin deformable object at the bottom of a narrow slot by turning the wrist, and
lifting it out) and a two-arm sequence. The pick holder is on the right and the tray on the left, so the natural split is one arm for each (reach was not
checked for every other route). The right arm hooks the wire out and lays it in front of the trough,
and the left arm carries it to the tray. A handoff of the pick to the left arm would also work, and it still uses
both arms.

## Context

- O*NET 49-2091.00 Avionics Technicians, task: "Adjust, repair, or replace malfunctioning components or assemblies,
  using hand tools or soldering irons." (https://www.onetonline.org/link/summary/49-2091.00)
- EASA AMC 20-21: "This AMC promotes a housekeeping philosophy of \"protect, clean as you go\" when performing
  maintenance, repair, or alterations on or around aircraft EWIS." (https://www.easa.europa.eu/en/downloads/1715/en)
  The task is a toy-scale stand-in; neither source describes hooking a wire from a trough.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
    --instruction "A loose wire offcut has fallen into the deep wiring trough, where the grippers do not fit. Use the hook pick to lift the wire out of the trough, then lay the whole wire in the grey tray. Leave nothing in the grippers."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 2400
```

The demo is closed loop: it reads the live pick and wire states.

1. The right arm pinches the pick's shank near its top and lifts it out of the holder.
2. It servoes the shank into the rib gap beside the wire's middle, against the far wall, with the foot pointing
   along the trough.
3. It lowers the foot below the wire and turns the wrist 90 degrees, so the foot swings under the wire.
4. It lifts the wire's middle above the rim, carries it over the near wall and lays it on the table in front of the
   trough. Then it slides the pick back out and puts the pick down.
5. The left arm grasps the wire near its middle and lifts it. The wire's ends slide up over the wall.
6. The left arm lowers the doubled wire into the tray, lays it out along the tray and lets go.

`demo/` holds the videos, `setup.png` and `result.json`.

Each shortcut variant scores exactly 0:
- `TROUGH_CHEAT=none`: do nothing.
- `TROUGH_CHEAT=fingers`: try to fish the wire out with the bare closed fingers in both orientations, then grab the
  wire's highest point and carry it to the tray. The closed fingertips cannot get down to the wire, which stays on the ribs, and nothing is lifted.
- `TROUGH_CHEAT=hold`: the full demo, but the left gripper keeps holding the wire over the tray.

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

The wire is measured along its length, sampled every 4 mm.

- **0** if at the end either gripper centre (TCP) is within 3 cm of the wire or of the pick's shank, i.e. it still
  holds one of them. In a real trial a moved trough or tray also scores 0. In sim they are fixed.
- **1.0** if the whole wire lies inside the tray's inner outline, less than 2.5 cm above the table.
- **0.5 + 0.5 x (fraction of the wire inside the tray)** if no part of the wire is within the trough's outline
  (walls included) lower than 1 cm above the rim. A wire lying across a wall top still counts as in the trough.
- **0.25** if at least half of the wire's length is out of the trough.
- Otherwise **0**.

`distance()` is the fraction of the wire not in the tray. `progress()` reports pick lifted, wire raised above the
rim, fraction out of the trough, fraction in the tray, and whether everything has been released.

## How this differs from similar tasks

The wire is a flexible offcut lying in a trough deeper than the gripper can enter, so it is hooked with a pick, lifted out and laid whole in a tray. Unlike tool-retrieval tasks that sweep or slide a rigid part along a surface, the wire is lifted vertically out of the trough.

## Runtime and seeds

- Run limit 2,400 steps (2 min at 20 Hz). The demo finishes in about 900 steps. One trial takes about 5-10 min of
  wall time on one GPU, including Isaac Sim start-up.
- The layout is fixed with `object_jitter=0`. The wire always starts straight on the ribs, so every trial starts
  identically.

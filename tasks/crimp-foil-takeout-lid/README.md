# Crimp the board lid onto a filled foil take-out pan by folding its rim down all the way round

A filled 2.25 lb oblong aluminium foil take-out pan stands on a cutting board in front of the robot, its white
foil-laminated board lid lying seated on the pan's top flange. Round the lid, the pan's 8 mm rim lip stands turned
straight up all the way round. The robot must fold the lip down flat onto the lid all the way round, so the lid is
crimped on and stays on when the pan is carried or turned on its side, and then let go.

**Ability tested:** folding a soft metal rim flat over a lid section by section all the way round, while the other
hand keeps the lid seated and the free, light pan from sliding away under each push.

This is everyday take-out packing work: food preparation workers "Package take-out foods or serve food to
customers." (O*NET 35-2021.00), and fast food and counter workers "Wrap menu items such as sandwiches, hot entrees,
and desserts for serving or for takeout." (O*NET 35-3023.00). Foil pans with board lids are closed by folding the
pan's soft rim over the lid edge. Pan (8 3/4 x 6 1/8 x 1 11/16 in) and lid (8 3/8 x 6 in) are the catalogue sizes; the
upturned 8 mm lip is the prepared start state this task assumes (the catalogue only says the pan has hemmed edges), and
the yield moment is an estimate, not a measurement.

Why two hands: the pan is free and light (425 g with its rice) on a smooth board, and every inward push on the lip
also pushes the pan; one hand presses the lid down near one end (which keeps the lid on its seat and the pan in
place) while the other folds the lip, then the hands swap ends. Reach also splits the work: each arm reaches only
its own half of the rim (the far short end is out of reach of the opposite arm).

## Closest tasks

- TK-S2WMM8 "Fold carton flaps closed" (pilot) scores four large cardboard flaps turned onto a box top; this task
  scores a narrow soft-metal rim bent flat over a separate lid in 24 short sections all the way round, including the
  corners, with the lid held seated and the pan held still.
- TK-BUAYX9 "Close a snap-fit case lid" (accepted) scores two elastic latches clicking over a lid; here nothing
  clicks: the rim stays only where it is pushed and pressed, so every section must be folded and pressed flat.
- TK-T6U364 "Secure a clevis pin with a cotter pin by bending its legs" scores bending two steel legs on a pin; this
  task scores a continuous perimeter fold over a flat lid and lid retention.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- Everything the scene needs is in `task.py` (the pan with its hinged rim is authored there as one articulation);
  `cabledemo.py` is the scripted-demo runner.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 4000 \
    --instruction "Crimp the board lid onto the filled foil take-out pan: fold the pan's rim down flat onto the lid all the way round, so the lid stays on when the pan is carried or turned on its side. Then let go."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 4000
```

The demo reads the simulated pan pose and lip angles to plan its moves. The right hand presses the lid down near
the right end of the pan (about 9 cm from the centre) while the left hand works round its own half of the rim:
for each lip section it comes down 2 cm outside the lip, pushes it inward past upright with the fingertips, then
presses it flat onto the lid and checks the section's angle (up to three presses). Then the hands swap: the left
hand presses the lid near the left end and the right hand crimps its half. Any section the demo reads back
from the simulated hinge angle as short of flat gets up to two more presses straight down on it (a little lower each
time), then both hands slide off the lid and park beside their bases. (Scripted demos may read simulator state; the
policy under test sees only the cameras and its own joints.)

The recorded demo folds all 24 sections and scores 1.0 in 1458 steps (73 s), with the lid seated well inside the 4 mm
tolerance (about 1 mm off centre at the last logged check) and both hands parked clear.

Shortcut and partial-credit checks (scripted policies run through the same scorer; evidence kept with the task):

| Policy (same scorer, `FOIL_CHEAT=<name>` in `task.py`) | Score | Evidence |
|---|---|---|
| none: do nothing | 0.0 (100 steps) | `scratch/sc_none.json` |
| press: press straight down round the whole rim, never pushing the lip inward | 0.0 (406 steps) | `scratch/sc_press.json` |
| corners: fold only the four cut corners, with the lid held | 0.0 (498 steps) | `scratch/sc_corners.json` |
| onearm: left arm alone, nobody holding the pan, folds what it can reach | 0.25 (742 steps) | `scratch/sc_onearm.json` |
| hold: fold all 24 sections, then keep a hand pressing on the lid | 0.0 (1554 steps) | `scratch/sc_hold.json` |
| tier50: leave two far-side sections and one right-end section open | 0.5 (1388 steps) | `scratch/sc_tier50.json` |
| tier75: leave one far-side section open | 0.75 (1474 steps) | `scratch/sc_tier75.json` |
| full demo | 1.0 (1458 steps) | `demo/result.json` |

The scoring ladder is also checked offline against synthetic states in `scratch/score_test.py`.

## Scoring

Measured at the end with a ruler. The 24 lip sections are marked on the rim. A section is **folded** when its free
edge lies within 1.4 mm of the lid, i.e. within 10 deg of flat (in the sim: its hinge angle is at least 80 deg).

- **0** if any of: the lid is off its seat (centre moved more than 4 mm from the pan centre, raised more than 2 mm,
  or tilted more than 3 deg relative to the pan); the pan is tipped more than 10 deg or off the board; the pan is
  crushed (under 40 mm tall; real trials only, the simulated pan is rigid); a gripper centre (TCP) is within 10 cm
  of the lid's top surface (not let go); or less than 25% of the rim length is folded.
- **0.25**: at least 25% folded, but some side has less than half its length folded (the lid is not caught all
  round).
- **0.5**: every side at least 50% folded (the lid is caught on all four sides).
- **0.75**: at least 90% of the rim length folded.
- **1.0**: all 24 sections folded.

`success()` is the 1.0 condition with the pan and lid at rest, held 1 s. `distance()` is the unfolded fraction of
the rim (+1 if a zero condition holds). `progress()` reports lid_seated, pan_upright_on_board, folded_fraction,
every_side_half_folded, all_folded and released.

## Runtime and seeds

- Run limit 4000 steps at 20 Hz. The layout is fixed with `object_jitter=0` (the pan is an articulation and is
  never jittered); every trial starts identically.
- Real trials need a new pan and lid each time (a crimped rim can't be reset); see reset notes.

## Fidelity

The rim lip is 24 rigid sections hinged to the pan along the lid edge. Each hinge has static joint friction equal
to the section's plastic yield moment (0.25 N*m per metre of rim, so about 1 N pushed at the top of the 8 mm lip
for a 32 mm section; the cut corners, where real foil gathers, are twice as stiff per metre). A section therefore
stays wherever it is bent, with no springback, which is how soft aluminium foil behaves; the foil between sections
does not stretch or pleat. The yield value follows the plastic moment of a single foil ply (sigma_y * t^2 / 4 with
t about 0.08 mm and sigma_y about 150 MPa gives about 0.24 N*m/m). The pan body is rigid (walls do not dent), the
rice is represented only by the pan's mass (425 g), and the board lid is a rigid 1.5 mm board (real lids are thinner board). The arms' own joint compliance is the robot's real gains, so pushing and pressing need commanded
overshoot, as on hardware.

# Shut an overpacked suitcase with a sleeve hanging out: tuck it in, press the lid down, latch both

An 18 in aluminium-frame hard-shell case with two lever latches lies open on its back, packed too full. Two overfilled packing cubes and a
chunky jumper folded on top of them hold the let-down lid about 17 deg open. One sleeve of the jumper hangs out over the
front rim, down the front face, and its cuff lies on the table. If the lid is pressed down as it is, the sleeve gets
pinched in the seam. The robot has to notice the sleeve and work it back inside under the raised lid, then close the
case. One hand holds the springy lid down (about 7 N net at its front edge) while the other swings a latch over its
strike. Then it fastens the second latch and lets go.

Instruction: `Close the suitcase with nothing caught in it, latch it shut, then let go.`

## What makes it hard

- **The sleeve.** Its cuff can be carried in under the lid's raised front edge. But a sleeve pulled in only by its
  cuff leaves a loop hanging over the rim, so the robot has to look again and push that loop in as well. Any fabric
  left on the rim or outside the case is caught when the lid comes down.
- **The lid pushes back.** Each latch is a lever hinged on the base below the seam, with a hook at its top. The hook
  passes over the strike on the lid's front edge only when the lid is already shut; a latch swung in while the lid is
  up sits in the strike's path and stops the lid closing. So one hand has to hold the lid down while the other
  swings the first latch shut. Once the first latch holds the lid, the second can be swung shut on its own.

## Real-world setup

| item | real part | size / position |
|---|---|---|
| suitcase | 18 in aluminium-frame hard-shell case with two lever latches (no zip), 46 x 34 x 21 cm, halves 10.5 cm deep | on its back, long side along y, centred at y 0, front face at x 20 cm, hinge along the far side (x 54), taped to the table at its marks |
| lid | the case's own lid half, packed full behind its zipped divider (flat-folded clothes) | let down onto the load, resting about 17 deg open, 10 cm gap at the front |
| packing cubes | two packing cubes, nylon, about 31 x 21 cm, overfilled with folded T-shirts so they bulge in the middle | side by side in the base; their front edges sit 4.5 cm below the rim |
| jumper | a red chunky-knit jumper, folded to 20 x 34 cm | on the cubes, front edge near x 31, about 7 cm proud of the rim |
| sleeve | the jumper's right sleeve, about 9 cm wide laid flat | out of the fold at its front, across the cubes, over the front rim at y -6 to +3, down the front face; the cuff lies on the table out to about x 14 (tape mark) |
| latches | the case's two lever latches on the base's front face, each with a hook over a strike on the lid's front edge | at y +11 and -11, swung open to their stop |

Positions are in cm from the midpoint between the arm bases: x forward, y left, heights above the table top. The lid
top is 21 cm up when shut, above the straight-down reach, so the lid is pressed near its front edge with the arm's
45 deg tilted grip. The sleeve is pinched and carried in under the lid with a 60 deg tilted grip.

Once per session, check the push-back. Press the middle of the lid's front edge down with a force gauge until the lid
is just shut: it should read 6-8 N. If not, add or remove a T-shirt in the cubes.

## Scoring (real trial)

End photos of the closed case all round, and the trial video.

- **Score 0** if any of the jumper is outside the case or pinched between lid and base at the end (any fabric
  visible at the seam or outside), if the jumper or a cube has been taken out of the case, or if a gripper is within
  5 cm of the case at the end.
- **Latched:** the lever is flat against the front with its hook over the strike, and the lid stays shut on its own
  when its front edge is lifted gently by hand after the trial.

| score | end state |
|---|---|
| 1.0 | nothing caught, both latches latched, grippers clear |
| 0.5 | nothing caught, one latch latched, grippers clear |
| 0.25 | nothing caught (the sleeve inside the case, lid open or shut), no latch latched, grippers clear |
| 0 | otherwise |

In simulation the case and cubes are fixed, so the scorer reads the jumper, sleeve, latches and grippers. The sleeve
is inside when every node of the cloth sheet lies within the inner faces of the shell walls (1.5 mm in from the front
wall) and below the lid's underside (4 mm tolerance). A latch is
latched when it is within 3 deg of shut and the lid is within 2 mm of shut (the hook over the strike is what holds
it there). The jumper is in the case when its centre is inside the shell. Grippers are clear when no pad tip, fingertip
point or tool-centre point is within 5 cm of the case's bounding volume (case, lid and open latches).

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1.
- `task.py` and `sleeve.py` (the cloth sheet) in the same folder.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
    --instruction "Close the suitcase with nothing caught in it, latch it shut, then let go."
```

Runtime: about 15 min for the full 2,400 steps (about 3 steps/s on one 48 GB GPU, longer while recording; the cloth sheet costs about 0.1 s per step). Layout fixed (`object_jitter=0`); nothing is randomised.

## Fidelity

The case shell, cubes and latches are rigid; the case is fixed (taped down on the real table). The lid is one rigid
body on a hinge joint along the back edge. The load (the cubes' bulge and the jumper) is one rigid block that turns on
the same hinge axis with a linear torsion spring: about 7 N net push-back at the lid's front edge when shut (12.6 N from the spring, less
the lid's and the load's weight). Real clothes compress unevenly and stiffen as they compress. The sleeve is a reduced position-based cloth sheet (1.25 cm grid, distance and shear constraints, no bending
stiffness, no self-collision). It is stepped every control step from the measured gripper state and the parts' poses,
and collides with the shell, cubes, jumper, lid, latches, fingers and hands. It does not push back on them. A closing jaw holds
the sleeve only if it closes across a free edge of the sleeve; it gathers the cloth it sweeps between its pads; opening the jaw or pulling the cloth 3.5 cm
away from the pads lets go. Its parameters are not calibrated to a real knit. Each latch is one rigid colliding lever
and hook on a hinge with an open stop at 30 deg. A spring pulls it shut within 15 deg of shut.

## Reset

Open the lid, refold the jumper on the cubes at its marks, lay the sleeve out across the cubes and over the rim to the
cuff mark on the table, let the lid down, and swing both latches open. The case stays taped at its marks.

# Peel a cup-noodle lid back halfway

> Peel back the lid of the cup noodles halfway, ready for the hot water.

## An everyday skill

The directions on a cup of instant noodles start the same way the world over: peel the lid back halfway, pour in
boiling water, then press the lid down again while the noodles steep. The paper-foil lid is heat-sealed only on the
rim. At the tab the peel front runs along the rim's curve, so the seal holds hardest there; once it gives, only two
short crossings of the rim hold the front, and the lid runs across the cup with a jolt. A pull hard enough to start
it carries on unless it is eased off at once, and a lid peeled almost off can't be closed down for the steeping. The
cup weighs 75 g, much less than the pull, so it lifts or tips unless the other hand holds it. The robot has to pull
until the seal gives, ease off, and stop the peel front (not its hand) at the middle. Nothing happens during the
trial that the robot doesn't cause.

## Scene (station frame: x forward from the arms, y to the robot's left, cm)

| Object | Details |
| --- | --- |
| Cup noodles | one unopened Nissin Cup Noodles, Chicken, 2.25 oz / 64 g, in its paper cup (rim 9.4 cm across, 10.6 cm tall, base 6.6 cm; 75 g in all), dry, at room temperature; standing upright on the bare table, base centred at (36, -2); the tab points to the robot's right (-y). Before the trial the tab is bent up to 30 +-5 deg at the rim's edge, once, with a finger; no part of the seal may open. |

Both arms start at the ready pose. One fixed layout, as in the screenshot. Real trials are dry (no water).

## Scoring (task.py)

d = how far the heat seal is peeled, measured along the line from the tab through the cup's centre, from the rim's
edge on the tab side to the peel front (where the lid still sticks to the rim), as a percentage of the rim's diameter
(top photo at the end of the trial, with a ruler laid across the rim; in simulation, the number of 5 mm strips let
go). A torn edge is not a peel front: a lid torn through scores 0.

- **0, latched for the trial:** the cup's base rises more than 1 cm off the table, or the cup tips past 30 deg
  (judged on a side video); the cup's wall is dented or creased (checked on the cup after the trial; in simulation,
  where the cup is rigid, a grip harder than 20 N on each finger with the fingertips on the cup's wall counts as a
  dent); the lid tears through; or d passes 85 % (the lid can no longer be pressed down over the cup).
- Otherwise, at the end:
  - **1.0** if 40 % <= d <= 60 % and the cup stands upright (within 10 deg) on the table, let go (no gripper within
    6.5 cm of its axis, or within 4 cm of the tab), for 1 s;
  - **0.5** if 16 % <= d < 40 % or 60 % < d <= 85 %, or 40 to 60 % while the cup or tab is still held;
  - **0.25** if the tab's seal has opened but d is under 16 %;
  - **0** otherwise (nothing peeled).

Holding the cup is up to the robot; lifting it to peel in the air is not allowed (the cup must stay on the table),
which is how the directions are followed at a table before the boiling water goes in.

## Running it

    source <IsaacLab-Arena>/yam.env
    python make_assets.py                          # writes assets/cup.usda (pxr only)
    python demo.py task.py out                     # scripted full-score demo
    python demo.py task.py out --variant onehand   # control: peels without holding the cup
    python demo.py task.py out --variant squeeze   # control: as the demo, but clamps the cup hard (dents it)
    python demo.py task.py out --variant rip       # control: one steady pull, as for taking a lid right off
    python demo.py task.py out --variant nowatch   # control: a steady pull to the halfway hand pose, not watching
    python demo.py task.py out --variant idle      # control: does nothing

A policy run (Dream-1) uses the same task file through Inspect Robots:

    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 --policy dream \
        --max-steps 1200 --instruction "$(python -c 'import task; print(task.INSTRUCTION)')" --scorer task_score

Run limit: 1,200 steps (60 s at 20 Hz); the demo takes 304. A trial takes about 4 minutes of wall time on one GPU.
No randomness.

## Results (simulation)

| Run | Score |
| --- | --- |
| demo | 1.0 in 304 steps (the tab's seal opens at 6.0 N; d = 47 %; the cup rises at most 6.5 mm; grip 14 N) |
| onehand: the same pull without holding the cup | 0 (the cup comes up off the table, 11 mm, before the seal opens) |
| squeeze: the demo with a clamping grip on the cup | 0 (29 N on each finger: a dented cup) |
| rip: one steady 2 s pull back past the far rim | 0 (the peel runs to 79 % and the pull drags the cup up out of the hold, 11 mm) |
| nowatch: a steady pull to where the hand would be at halfway | 0 (the soft arm never builds the 6 N the tab needs) |
| idle | 0 |
| Dream-1 (dream-1-beta, 1 trial, 1,200 steps) | 0 (its right gripper comes near the cup only late in the trial and never takes the tab; the lid stays sealed) |

The demo: the left gripper takes the cup round its body 5 cm above the table, coming in from the robot's side so its
fingers lie either side of the tab's line (a grip across the other way lets the pull pivot the cup), closes to about
14 N on each finger, and presses the held cup down onto the table (its command 2 cm below the hold), as a hand does.
The right gripper comes in from outside the tab, tilted down along it, and pinches it. It pulls the lid back up and
away at 45 deg, a millimetre further each step, until the seal gives; the moment the front moves it puts its command
back 1 cm behind the front, so the pull stored in the arm doesn't run the peel on, then follows the front and stops
once it is 40 mm in. It lets go of the tab, then of the cup. The front's position is read from the task state (a
camera sees the same edge, where the folded-back lid meets the part still on the rim). The arm's soft joints sag, so
the demo corrects its command from the measured tool point on the way in, and holds that correction during the pull.

## Fidelity

Rigid bodies, 240 Hz physics, 20 Hz control. The lid (`make_assets.py`) is one articulation with the cup: 19 rigid
strips, 5 mm wide, hinged across the cup, and the paper tab on a limp hinge at the rim's edge (so pinching the tab
can't open the seal; only pulling it can). A strip still sealed is held flat on the rim by its hinge's limits. Each
control step the task reads the force through the hinge at the peel front (PhysX's joint force), takes its lift and
outward-pull part in the cup's frame, and lets the strip go if that exceeds its seal: 200 N/m (3 N per 15 mm) times
the length of the 3 mm sealed ring the front crosses in that strip, and 1.5 times that for the first strip, where the
peel starts at the tab. That gives about 6 N at the tab, 2.6 N for the next strip and 1.2 N across the middle. The
front can run up to 3 strips (15 mm) per control step. Peeled strips bend freely, and the one at the front keeps a
crease, so the peeled lid stays folded back up on the tab's side (it can't fold down under the part still sealed). The cup is rigid, with the noodles as mass. The squeeze on it is
read from the YAM finger servos (5,000 N/m, 50 N limit: stiffness times how far each finger is held back from its
target); over 20 N on both fingers with the fingertips on the wall counts as a dent (an estimate for a paper cup). Seal strength varies between cases: the reset notes ask for
a measurement each session.

## Files

`task.py` (scene, seal rule, scoring), `demo.py` (scripted demo and controls), `make_assets.py` (USD generator),
`assets/` (generated USD), `dynkit.py` and `demokit.py` (shared helpers: USD bodies, the episode clock, Cartesian IK
demo runner). All assets are made by this task's code; no third-party assets.

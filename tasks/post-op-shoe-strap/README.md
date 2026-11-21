# Fasten a post-op shoe on a practice leg

A practice lower leg sits as if seated, its foot in a padded dressing on the table, already wearing an open-toe
post-operative shoe: a rigid rocker sole, a navy padded upper and two hook-and-loop straps through D-rings, like the
shoes worn for weeks after foot surgery or a forefoot fracture. The forefoot strap is already fastened. The instep
strap is threaded through its D-ring on the outer side and opened, and the outer wing of the shoe's opening has
sprung open. The robot must draw the strap back through the D-ring until the opening closes, lay it back over and
smooth its hook patch down onto the strap's loop patch so that it holds a pull, and let go. Putting the shoe on is
not part of the task.

What makes it hard: the strap slides through the D-ring only by contact, against about 5 N of springback from the
upper; its tail has to be folded back round the ring's bar onto its own loop patch; hook and loop fastens only where
it is pressed at 2 N or more, so a strap only laid over does not hold and must be smoothed down along its length, and
the result is pull-tested. The leg hangs from sprung, damped knee hinges with the sole resting on the table: dragging
it more than 5 cm off its mark scores zero. One arm does the strap work; the other may hold the heel, which the task
neither requires nor scores.

Category: Societal relatability. Foot and toe fractures and bunion surgery are common, the post-op shoe is worn for
weeks, and because bending to the foot is hard after surgery a family member or carer often fastens it.

A task file for the `yam_arena` embodiment (Isaac Lab-Arena 0.3.1).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fasten the post-op shoe on the practice leg: draw its instep strap back through the D-ring until the shoe is snug, then lay the strap back over and smooth it down so that it holds. Then let go of the shoe." \
  --scorer task_score --max-steps 1600 --store-frames --log-dir runs/dream
```

Scripted reference demo (scores 1.0): the same command with `--policy yam_scripted`, which plays `demo()`.
`TASK_DEMO` picks one of two scripted failures: `failure` (the strap is drawn snug and its end held down, then let
go without being smoothed down along its length: 0.5) and `loose` (the tab is laid over and pressed down
without first being drawn up, so the opening is not closed: 0).

Offline scorer tests (no simulator): `python test_shoescore.py`.

## Files

- `task.py`: the scene, the per-step hook (hook and loop, scorer, the scorer's tab pull) and `demo()`.
- `shoekit.py`: the leg, shoe and strap as one articulation, and the hook-and-loop joints.
- `shoescore.py`: the scorer as plain functions; `test_shoescore.py` tests every tier and zero on hand-built states.
- `shoedemo.py`: the scripted reference demo, open loop: Cartesian waypoints played by `yam_scripted`; it reads
  nothing from the simulator.
- `scenekit.py`, `robotpts.py`, `steptime.py`: step and reset hooks, robot sample points for the hands-off check,
  step timing.

## The scripted demo

The right hand pinches the heel's pull tab and holds it while the left hand works (the task neither requires nor
scores this). The left hand pinches the strap's pull tab, draws it straight up (the strap runs back through its
D-ring and the opening closes against its spring), swings it round the D-ring at about the tail's length so that the
tail lies back over the strap, and presses the tab's end down. It lets go, lifts clear, comes down with its closed
fingertips on the strap right at the D-ring and smooths it down in one slide from the fold to the strap's end, square
to the upper, as a person lays hook and loop down from the ring outward. Both hands then let go and go home.

## How it is scored

`score()`: 0.25 once at least 10 mm more of the strap has been drawn through its D-ring than at the start (still
through it); 0.5 if the opening's edges under the strap are also 12 mm or less apart; 0.75 if at least 35 mm of the
hook patch is also fastened onto the loop patch (from the first to the last fastened point); 1.0 if the fastening also
holds a 5 N pull on the tab along the strap, away from the D-ring (the tab moves under 3 mm). 35 mm is 1.4 strap
widths: the first 11 mm or so of hook past the ring lies on the fold round its bar and cannot be pressed flat, so a
well smoothed strap fastens about 39 mm. Everything is measured
with every part of the robot at least 3 cm from the shoe for 1 s; the 5 N pull is applied by the scorer then, with
the arms still, as a capped spring drive on the tab that holds 5 N for the whole second (until the tab has
moved 10 mm). Zero, held, if the leg moves more than 5 cm off its mark or the strap comes out of its D-ring at any
time, or if the robot has not let go at the end. Only the instep strap is scored; the forefoot strap is fastened
from the start.

In simulation the drawn length is the strap's arclength at the D-ring's plane, the gap is between the flap's edge and
the inner edge, and the fastened length is the span of engaged hook plates. In a real trial the same tiers are read
with a pen line put on the strap at the D-ring's outer bar at reset, a ruler beside the strap's toe-side edge, the
spring scale on the tab (pull first, then measure the fastened length); task.json "scoring" gives the procedure.

## Model

See `task.json` "fidelity". Hook and loop: for every (hook plate, loop plate) pair a point joint is authored
disabled; it is switched on, at the plates' pose at that moment, only when PhysX reports at least 2 N of contact
force between those two plates with their faces opposed, and switched off when its spring force exceeds 2.5 N of
peel (1.0 N per cm of width over 25 mm) or 17 N of shear per plate (about 7 N/cm2 over 5.6 x 25 mm, rounded up).
Typical published strengths of sew-on nylon hook and loop, measured by ASTM D5170 (peel) and ASTM D5169 (shear). The
joints act at plate centres with free rotation, so they add no bending stiffness to the strap's hinges. The strap is
34 thin plates, so it bends round the D-ring's round bar and slides through it by contact. The forefoot strap, the
dressing and the padding are visual only. Strap width (25 mm) and the D-ring's opening (29 x 7.5 mm) are assumptions in
the range of real post-op shoes. The rubber pull tab is
stiff in itself and joins the webbing with a webbing hinge.

## Runtime and seeds

- Demo: 1.0 in 487 steps (24.4 s simulated); run limit 1,600 steps (80 s at 20 Hz).
- `object_jitter=0` keeps the layout fixed; the demo is open loop and deterministic.

## Sources

All geometry is built from primitives and convex prisms in `shoekit.py`; no meshes or textures. Sizes follow a men's
Large open-toe post-op shoe (e.g. DARCO MedSurg or Medline post-op shoe, size L: outsole about 29 x 11 cm; two D-ring
hook-and-loop straps as on the Rolyan post-op shoe) and an adult lower leg.

## Licence

MIT

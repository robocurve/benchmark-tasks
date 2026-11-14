# Flex a 3D printer's build plate to pop a printed robot part free

A 257 mm spring-steel build plate with a textured PEI face lies on a 180 mm stand on the bench, just taken off the
printer, with a freshly printed robot arm link (170 mm long) stuck across it near the edge facing the arms. A parts bin stands beside it. Holding the
plate by its two side edges, the hands lift it and bend both edges down, so the plate curves away under the link's ends
and the link pops free; they flatten the plate and set it back on the stand; then one hand takes the link off and drops
it in the bin, and both arms go back to where they started.

What it tests: bending one springy object with both hands at once, the right way and far enough to release the part,
but not so sharply that it kinks; then picking the freed part.

Why two hands: the part is released only when the plate under both of its ends has turned away from it at once. One
hand lifting one edge tips the whole plate on the other edge, which bends nothing. Bending the edges up (the other way)
presses the plate into the part and frees nothing, as with a real plate.

## How it is modelled

`yam_arena` task files place single primitives; the plate, the part, the stand and the bin here are rigid bodies built
with `bodykit.py`, reused unchanged from the accepted ice-cube tray task (`tasks/twist-an-ice-cube-tray-and-tip-the-cubes-into-a-dish`).

- The plate: five rigid strips (51.4 mm wide, 0.2 mm apart) joined along x by four hinges with springs standing in
  for the steel's stiffness (0.04 N m per degree at the inner pair, 0.08 at the outer pair; together about a quarter of
  real 0.5 mm steel, so a parallel grip holds the bend). It is 2 mm thick in the sim (0.5 mm real) so the fingers can
  hold its edge. The PEI face runs to each strip's edges, so the seams barely show.
- The bond: the part is held to the middle strip by a fixed joint (a `Glue`), with contact between the two left on
  (`task.py` sets the joint's `collisionEnabled`, which USD turns off by default; otherwise the freed part fell through
  the plate). After every control step the task file reads the strips' poses; once the edge strips under both of the
  part's ends have turned 6 degrees or more from the middle strip, edges down, it switches the joint off for good (as
  the ice tray switches its frozen cubes off at a 12 degree twist).
- Kink: a bend past 20 degrees at any one hinge (a bend radius under about 15 cm), at any time, is latched as a kinked
  plate (score 0).

## Run and score a trial

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/x2d-plate-flex/task.py \
      -E object_jitter=0 -E control_hz=20 --instruction "$(python -c "import json;print(json.load(open('tasks/x2d-plate-flex/task.json'))['instruction'])")" \
      --scorer task_score --max-steps 1200

The scorer is `task_score` (see `score()` in `task.py` and `scoring` in `task.json`). One trial is 1,200 control steps
(1 minute at 20 Hz); the scripted demo takes about 28 s. No seeds: the layout is fixed (`object_jitter=0`).

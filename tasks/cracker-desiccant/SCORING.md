# Serve all six crackers, excluding the non-food packet

Instruction: **Please put all the crackers from the tray into the basket.**

Each correctly delivered cracker contributes **1/6**. Thus 0, 1, 2, 3, 4, 5,
or 6 delivered crackers score 0, 0.167, 0.333, 0.5, 0.667, 0.833, or 1.0.
Fractions are exact internally; decimals above are rounded for display.

If the DO NOT EAT packet enters or is placed on the basket at any time, the
**entire trial permanently scores 0**, even if the packet is later removed.
Removing the packet alone earns no credit. It may stay on the source tray or
be set aside. A cracker left on the tray, held or dropped outside the basket
earns no credit individually; it does not erase credit for other crackers.
No particular arm, transfer method or order is required.

## Measurement

The basket is fixed. All coordinates below are in its local metre frame.

- Each original cracker is counted once. All collision vertices must be
  inside the tapered inner ellipse and between Z=0.008 and 0.063. For each
  vertex, f=clamp((z-0.008)/0.048,0,1), with ellipse radii
  a=0.109+0.020f+0.002 and b=0.064+0.020f+0.002. Require
  (x/a)^2+(y/b)^2 <= 1. The 2 mm outward tolerance accommodates contact
  rounding; checking every collider vertex rejects substantial overhang.
- Support is inferred geometrically. The lowest collider point must be
  within 4 mm of the basket floor at Z=0.012, or within 4 mm of the top of
  another contained cracker already supported by the basket. Stacked
  crackers need at least 20 square millimetres of projected footprint overlap.
  The support graph must ultimately reach the basket floor. Food on a tray
  held above the basket does not qualify. This is a rigid-body support
  approximation, not a force sensor or a prescribed target pose.
- Release requires both measured tool-centre points to be at least 45 mm
  from the cracker's oriented local collision bounding box. This conservative
  clearance is the gripper-release proxy; no particular gripper opening or
  resting arm configuration is required.
- Linear speed must be at most 0.020 m/s, angular speed at most 0.20 rad/s.
  Containment, support, release and speed conditions must hold for 20
  consecutive 20 Hz control steps (1 second). Each cracker has its own timer.
  Removing a delivered cracker immediately removes its credit; the score is
  not the maximum reached earlier.
- The packet veto fires when its projected convex collider footprint overlaps
  the basket's outer-rim ellipse, radii (0.135,0.090), and its vertical span
  intersects Z=[0.008,0.064]. This catches entry below the rim or resting on
  it. Passing above the rim without entering does not trigger the veto.
  The flag is checked every control step and clears only on a new trial reset.
- Nonfinite state, an incorrect set of cracker identities, or a decreasing
  control-step index permanently invalidates the trial and gives zero.
  Reading the score repeatedly at the same step does not advance any timer.

The six crackers and sealed packet are rigid surrogates. Crumbling, paper
or cloth deformation, packet opening, ingestion and toxicity are not simulated.
The task tests separating food from non-food packaging when serving snacks.

## Validation

CPU tests cover each cracker count, packet veto and recovery, reset, dwell,
duplicate reads, held/hovering food, stacks, rounded-mesh overlap, overhang,
and packet positions around the rim. The contact demonstration and fresh
joint-command replay both reached 1.0. A physical packet-entry control scored
0.0. See README.md and task.json for the Dream outcome and reproduction commands.

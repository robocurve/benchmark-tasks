# Raise a hinged mast with its two guy lines and peg each so the mast stands plumb

A small guyed mast stands on the table in front of the robot. A white mast is hinged at its foot so it can tip only
sideways, towards either arm. It starts leaning 30 degrees to the robot's right, resting on its hinge stop. Two blue
guy lines are tied to a yellow collar at the top of the mast, one on each side. Each guy ends in a rigid white ring eye
with a grip bail, and both lie loose on the table. Three hooked ground pegs stand on each side in line with the mast.
The robot must haul the mast upright with the guys alone and put each ring on the peg that holds the mast plumb, then
let go. Only one peg per side gives a plumb mast (left: the outer peg; right: the inner peg). Any other peg leaves its
guy slack or too short, and the mast then stands out of plumb or falls back onto a stop.

The tested ability is setting a structure by two lines that pull against each other. Hauling on one guy brings the
mast up but pulls it past plumb once it is over the top, so the other guy must already be holding. The robot must then
choose, for each guy, the anchor that leaves both lines taut with the mast vertical. A two-guy tilt-up mast is erected
this way in the field. A mast-erection manual says to "Erect the mast carefully by pulling guy wire 1 and lifting the
mast", to "Connect guy wire 1 (opposite the tilting direction) with fasteners to its foundation", and "If the mast is
not vertical, adjust the guy wires, and check the position of the mast again." O*NET 49-9052.00 (Telecommunications
Line Installers and Repairers) lists "Participate in the construction or removal of telecommunication towers or
associated support structures." and "String cables between structures and lines from poles, towers, or trenches, and
pull lines to proper tension."

Sources (each opened and quoted verbatim):
- Vaisala, "Erecting mast - DKP110" (mast erection procedure),
  https://docs.vaisala.com/r/M211625EN-H/en-US/GUID-6299CA52-BF47-4A3B-830B-3F2C2FAD6E51: "To eliminate excess slack,
  pre-tighten the guy wires." "This prevents the mast from tilting over." "Erect the mast carefully by pulling guy wire
  1 and lifting the mast." "Connect guy wire 1 (opposite the tilting direction) with fasteners to its foundation." "If
  the mast is not vertical, adjust the guy wires, and check the position of the mast again." "Make sure that each guy
  wire is tight."
- 3 Star Inc., "Gripple 1/8" inch Wire Rope Grip for 618/620 for Antenna Mast Down Guy",
  https://www.3starinc.com/gripple_18_inch_cable_rope_grip_for_618_telescopic_mast_down_guy_wire: "by hand you can
  tension the Gripple ends at all Anchor Points until Mast is relatively Plumb".
- O*NET 49-9052.00 Telecommunications Line Installers and Repairers, https://www.onetonline.org/link/summary/49-9052.00:
  "Participate in the construction or removal of telecommunication towers or associated support structures." and
  "String cables between structures and lines from poles, towers, or trenches, and pull lines to proper tension."

The instruction states only the goal ("Stand the white mast up plumb using only its two blue guy lines: put each guy's ring eye on the ground peg on its own side that holds the mast plumb, then let go of both rings."). It does not say which pegs are right, which guy to haul,
that the other guy must hold the mast as it passes the top, or that the mast must not be pushed: that is what the
task tests.

Skills: constraint-keeping (two opposed lines hold a load at a set point), placement/pinning (ring eyes onto hooked
pegs), measurement (choosing the anchor that leaves the mast plumb). Industry: telecommunications / utility
installation (antenna and instrument masts). Two arms: reach-forced (below), with the two lines working against each
other.

## Scene (station table; cm from the midpoint between the arm bases, x forward, y left, heights above the table)

| object | type | size | material (real rebuild) | position |
|---|---|---|---|---|
| mast foot | fixed block | 5 x 5 x 2 cm | grey PLA | centred at x 30, y 0 |
| mast | rigid bar on a hinge, 30 g | 2 x 2 x 22 cm | white PLA or square aluminium tube | hinge axis along x at x 30, y 0, 2 cm up; tips only sideways; stops at 30 deg each way; slight hinge damping; starts leaning 30 deg to the robot's right on its stop |
| guy collar | block sliding on the mast top, 10 g | 3.4 x 3.4 x 2 cm | yellow PLA | rests against a top stop 21 cm above the hinge; a 24 N/m spring under it lets the guys pull it down up to 6 cm |
| left guy | 6 mm cord, 29.6 cm, 15 g | blue braided polyester cord | tied to the collar's left face; lies loose from the collar down to the table and along it to about y +14 |
| right guy | 6 mm cord, 26.1 cm | blue braided polyester cord | tied to the collar's right face; hangs from the collar and lies on the table towards the robot, ending near x 17, y -19 |
| ring eyes | one rigid ring on each guy end, 8 g | 44 x 44 x 5 mm square ring, 6 mm bars, 32 x 32 mm hole; a grip bail over the hole (legs 50 mm tall, 8 mm cross bar) | white PLA, glued to the guy | lying flat on the table at the guy ends |
| hooked pegs | 6 fixed posts | 8 x 8 x 30 mm, with a 9 x 8 x 6 mm nose at the top pointing away from the mast | black PLA | at x 30; left y 20, 24, 28; right y -23, -27, -31 |

A ring dropped over a peg is pulled in by its guy until the post bears on the hole's outer edge, and the nose keeps
the ring from riding up the post. Lengths are set so that the left guy on its outer peg (y 28) and the right guy on
its inner peg (y -23) stand the mast plumb with the collar pulled about 2.5 cm down its spring.

Static outcome of each peg pair (measured in the sim, mast released and settled):

| left peg \ right peg | y -23 | y -27 | y -31 |
|---|---|---|---|
| y 20 | -26.3 deg (`mirror`) | not run (expected worse than -26.3, 0) | not run (expected worse than -26.3, 0) |
| y 24 | -10.1 deg (`pair10`) | -11.8 deg (`middle`) | -17.9 deg (`pair12`) |
| y 28 | plumb, -0.02 deg (demo) | -5.2 deg (`tier05`, 0.5) | left ring dragged off its peg, -28.7 deg (`pair22`) |

Negative = leaning to the robot's right. A left peg nearer the mast leaves the left guy too long, and a right peg
further out leaves the right guy too short; both lean the mast to the right, so the errors add up. The y 20 pairs with right -27 or -31 were not run: the left guy is already 8 cm too long and the right guy shorter still, so both errors lean the same way and they should be at least as bad as the mirror run (0). Only the
neighbouring right peg keeps the lean within 8 degrees (0.5); every pair that uses the same peg position on both sides
(inner, middle, outer) scores 0.

## How this differs from similar tasks

- Tending a boat's bow and stern lines in a filling lock, then making them fast at the toggle that leaves them taut
  (board, the nearest task): two lines hold a load that is moved by an outside lift, and each line is laid into a
  slotted cleat. Here nothing moves the load but the robot: the mast is raised by hauling one line, it would fall past
  plumb without the other, and the two lines end up pulling against each other to make a free-standing structure.
  The end state is an angle (plumb), set jointly by the two anchor choices, not a load kept against a stop.
- Tensioning a messenger wire into a far notch and hanging a ring at a mark (board): one wire, anchored, is tensioned
  by one move and a slider is placed. Here there are two lines whose tensions must balance at the mast top.
- Striking free lines from a pin rail while leaving loaded ones (board): the robot reads which line carries a load
  and removes the free ones. Here the robot creates the load path, raising a structure and anchoring both lines.
- Raising a pen gate by its rope (earlier task): one rope lifts a hinged flap while the other arm works below it, and the gate
  falls shut when let go. Here the hinged part must be left standing by two lines on their own.
- Tandem pipe lift and batter-board lines over a corner pin (screened earlier): two lines lift a load together, or
  lines are crossed over a pin at a mark. Neither has lines that oppose each other or a structure that stands by them.
- No board task or public set (LIBERO, RoboCasa, SimplerEnv, BEHAVIOR, DataCenterBench, RoboDojo) raises a structure
  with opposed guy lines and anchors them to set it plumb.

## Why two arms

Reach-forced, and the two lines work against each other. Offline IK (`scratch/ik.py`, output `scratch/ik.out`) with
the gripper pointing down, TCP over the ring's hole centre at x 30 cm, y 12, 16, 20, 24, 28 and 32 cm on each side,
6.5, 10 and 14 cm above the table (pinching a ring lying on the table, carrying it over the pegs): the arm on that side
reaches all 18 points at yaw 0 with 0 mm error; the other arm finds no solution at any of them for any yaw from -90 to
+90 degrees (7 yaws, 3 seeds each). Every peg is at
|y| >= 20 cm, so each ring can be pegged only by the arm on its side. Physics adds the second reason: a single guy
pegged alone pulls the mast past plumb onto the opposite stop (the `onearm_left` run below), so the mast stands only with
both lines holding.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py`, `cableshape.py` and `cabledemo.py` are in this folder (the cable simulation, the shaped start of the
  guys and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 900 \
    --instruction "Stand the white mast up plumb using only its two blue guy lines: put each guy's ring eye on the ground peg on its own side that holds the mast plumb, then let go of both rings."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 900
```

Result (`demo/`, recorded after the last change to `task.py`): score 1.0, success after 356 of 900 steps. Both arms
take their ring by its bail. The right arm holds its ring over its inner peg (y -23) as the catch. The left arm hauls
the left guy out over its outer peg (y 28); the mast comes up from 30 degrees and stops at +1.3 degrees on the right
guy. The left arm lowers its ring over its peg and lets go, then the right arm does the same. The collar ends 24 mm
down its spring, both guys are taut, and the mast stands at -0.02 degrees (largest lean in the 2 s after release
0.09 degrees). The arms never touch the mast. Wrist-camera mean brightness (0-255, every 2 s): left 144-172, right
148-168. Both wrist cameras look down on the ring, the peg row and the guy while placing, and the top camera shows the
whole mast, both guys and both peg rows.

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

Score 0 if any of these holds:
- a robot link touched the mast or the guy collar at any time (contact monitor on every physics step, latched for the
  trial): only the guys may move the mast;
- either ring is not on a peg on its own side (the post passes through the ring's hole below the nose);
- at the end a gripper is within 3 cm of either guy or ring, or both were let go less than 2 s ago;
- the mast leaned more than 8 degrees from plumb at any moment in the time since both were let go.

Otherwise 1.0 if the mast stayed within 3 degrees of plumb since both rings were let go, and 0.5 if it stayed within 8
degrees.

| score | end state |
|---|---|
| 1.0 | both rings on pegs (left outer, right inner), mast within 3 deg of plumb for at least 2 s after both grippers let go |
| 0.5 | both rings on pegs and let go, mast standing 3-8 deg out of plumb (left ring on y 28 with the right ring one peg out, on y -27: 5.2 deg) |
| 0 | any zero condition above, or the mast more than 8 deg out (any other peg pair, or a ring off its peg) |

Touching, grasping or hauling a guy earns nothing by itself: the lowest tier needs both rings hooked on pegs, let go,
and the mast standing by its guys within 8 degrees of plumb.

## Shortcuts (scripted variants of the demo, `MAST_DEMO=<variant>`; evidence in `scratch/sc_<variant>.json/.log`)

| variant | what it does | score | why |
|---|---|---|---|
| none | waits | 0 | the mast stays on its stop at 30 deg; no ring on a peg |
| hold | hauls the mast plumb and holds both rings over their correct pegs, never pegging them | 0 | no ring on a peg (mast at +0.8 deg while held) |
| holdend | correct, but keeps pinching both rings after lowering them onto their pegs | 0 | grippers still on the rings at the end |
| onearm | the right arm pegs its ring on its correct peg; the left guy is left on the table | 0 | the right guy is slack with the mast still on its stop (30 deg); the left ring is not on a peg |
| onearm_left | the left arm alone hauls the mast up and pegs its ring on its correct peg | 0 | with nothing holding the other side, the left guy pulls the mast over onto the far stop (+30 deg); the right ring is not on a peg |
| mirror | both rings on the inner pegs (y 20 and -23) | 0 | the left guy is 8 cm too long: the mast leans 26.3 deg |
| middle | both rings on the middle pegs (y 24 and -27) | 0 | the mast leans 11.8 deg |
| pair10 | left ring on its middle peg (y 24), right correct | 0 | left guy too long: the mast leans 10.1 deg |
| push | a gripper pushes the mast towards upright (to +10 deg), then the demo pegging is attempted | 0 | robot-mast contact latched (and the left grasp then missed, mast back on its stop) |
| pair12 | left middle (y 24), right outer (y -31) | 0 | the mast leans 17.9 deg |
| pair22 | both rings on the outer pegs (y 28 and -31) | 0 | the right guy is 8 cm too short: with the right ring pegged the left ring is dragged off its peg and the mast lies at 28.7 deg |
| tier05 | left ring correct, right ring on its middle peg (y -27) | 0.5 | the right guy is too short: the mast stands 5.2 deg out of plumb, both lines taut (partial tier check) |

The shortcut runs were made on the final physics, layout and scoring, but before the last edits of `task.py` (the result files are older than it); afterwards only demo-mode switches and comments
in `task.py` changed (the `tier05` switch was repointed from the left-middle pair, kept as `pair10`, to the right-middle
pair; its evidence is the run of that pair).

## Limit (chance and replay)

The layout is fixed, so a policy that replays the demo's pegs blind (left outer, right inner) would score 1.0, and one
that pegs left outer with right middle scores 0.5. A random pick of one peg per side scores 1.0 with probability 1/9
and 0.5 with probability 1/9 (the other seven pairs score 0). The task still needs both guys hauled and pegged with
the mast raised by the guys alone, and the peg choice follows from the guy lengths and peg spacing.

## Runtime and seeds

- Run limit 900 steps (45 s at 20 Hz); the demo finishes in about 360 steps. One trial takes about 4-7 minutes
  wall time on one GPU including the 1-2 minute Isaac Sim start-up.
- The layout is fixed with `object_jitter=0`: at every reset the mast is set on its stop and both guys are laid in the
  same shape and settled for 2.5 s, so every trial starts identically.

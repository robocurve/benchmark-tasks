# Fit a Mini-ITX server board into a 1U chassis, flat on its four standoffs with its connectors through the I/O shield

A Supermicro A2SDi-8C-HLN4F Mini-ITX server board lies on an antistatic foam pad beside an open Supermicro
CSE-505-203B 1U chassis, the pair that makes up the SuperServer SYS-5019A-FTN4. The robot is told: "Fit the server
board into the chassis, seated on its four standoffs with its connectors through the I/O shield, and let go of it."

The board has to go in the way it does in the real chassis: lifted over the 43 mm walls, lowered between the power
supply and the fan holder, with its connector edge toward the rear wall, then moved back so its D-sub, USB and
network connectors pass through the cut-outs of the chassis's I/O shield until the D-sub flanges meet the shield.
Only then do its four mounting holes sit over the four standoffs, ready for screws. A board set down anywhere else
rests on the standoff tops but cannot be fastened.

## Credit: the original task

This task takes up the idea of PR #2747, "Seat a Mini-ITX server board on four locating standoffs" (TK-3Y5TDY),
which itself continued PR #2029 (TK-KH4T4W). This task is submitted only after PR #2747's 48-hour revision hold has
ended with no revision. Everything
here is a new implementation written from scratch: the chassis, the shield, the board model, the scoring and the
demo. No code or files from either PR are used. The reviews of both PRs are answered below.

## How each point of the review is addressed

Review of PR #2747 (TK-3Y5TDY), every sentence:

| Review point | What this task does |
|---|---|
| "Four projecting locating pins and an open plate replace the named server chassis and its mounting arrangement, changing the claimed installation procedure." | No pins and no plate. The named chassis is modelled at its catalogue size (437 x 249 x 43 mm) with floor, walls, power supply, fan holder, drive bracket and the rear-wall window holding the board's 1U I/O shield. The board rests on four plain flat-topped M3 standoffs at the Mini-ITX C, F, H, J positions, as in the real chassis. The installation is the real one: in over the walls, connectors through the shield, holes over the standoffs (`_chassis_shapes()`, `openings()` in `task.py`). |
| "The code awards 0.5 with hole-axis errors up to 2 mm, although the specified tier requires all four pins to have entered their holes." | Every tier is a state the code checks exactly as the text states it, and every credited tier also needs the board flat on all four standoffs (`on_standoffs()`). Full credit needs every hole axis within 0.48 mm of its standoff's, the clearance at which an M3 screw starts by hand (`centred()`; tested at 0.47 and 0.49 mm). The 0.5 tier needs both D-sub shells at least 1 mm into their cut-outs (`entered()`; tested at 0.9 and 1.1 mm). |
| "Please revise the setup to use a representative server chassis, its actual standoff arrangement and rear-I/O opening, so the robot performs the installation in its real approach constraints." | As above. The approach constraints are the real ones: the 43 mm walls, the power supply 63 mm to the right of the board's place, the fan holder 42 mm ahead of it, and a shield whose D-sub cut-outs leave 0.45 mm each side (the cut-out sizes are estimates; the real shield adds an EMI gasket, not modelled). The board cannot be dropped straight down into place, because its connectors overhang the PCB edge past the shield's inner face; it must go in short of the shield and move back. |
| "Make the 0.5 score require the stated four-pin engagement, including vertical engagement, and add tests for misaligned and unengaged cases." | With real standoffs the engagement is: flat on all four standoffs (the PCB within 1 mm of every standoff top, each standoff top fully under the PCB, tilt at most 2 degrees: the 0.25 tier), connectors into the shield (0.5), home with every hole centred (1.0). `near_misses.py` tests misaligned cases (0.49 mm and 6 mm sideways, turned 6 and 90 degrees, flanges 0.31 mm off the shield) and unengaged ones (held 1.5 mm above the standoffs, tilted 2.5 degrees, set down short of the shield, shells only 0.9 mm in, left on the pad, still gripped, gripper state unknown). |
| "List the reused scene and its license in the README." | See "Assets and licences" below. |
| "Rerun Dream-1 and provide a full-score demo for the revised scene within the action limit." | Both were run on this `task.py`; see "Results". |

Earlier review of the same idea (PR #2029, TK-KH4T4W), the points that apply to any version of it:

| Review point | What this task does |
|---|---|
| Use the standard Mini-ITX hole pattern and realistic hole sizes. | Holes C, F, H, J at the Mini-ITX positions (154.94 and 132.08 mm front to back, 157.48 mm side to side), 3.96 mm diameter. |
| Ground it in Impact on AI timelines with a real data-centre server board and a representative assembly setting. | Category Impact on AI timelines; the board and chassis are the maker's own parts list for the SYS-5019A-FTN4 1U server, built on a bench by system integrators and data-centre technicians. The shield is the one the board's product page lists as its optional 1U shield for SC505/SC504 chassis (MCP-260-00085-0B). |
| Replace a sub-millimetre hole-to-pin threshold that cannot be checked with simple tools. | Each level is read with a feeler gauge, a straightedge, an inclinometer, a protractor, a ruler or a photo (see Scoring). The hole tolerance is a functional check: an M3 screw starts by hand in every hole without moving the board (0.48 mm, half the difference between the 3.96 mm hole and the 3.0 mm screw). |
| Specify protective handling for the real board and the gripper pads. | ESD wrist strap and gloves for the person resetting, ESD-safe foam tape on both gripper pads, board unpowered (`fixture/README.md`). |

## Run and score

The task runs on yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena) with the stock
`inspect-robots` command, from this folder's parent:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=fit-the-mini-itx-server-board-into-its-1u-chassis/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=640 -E cam_height=360 \
  --instruction "Fit the server board into the chassis, seated on its four standoffs with its connectors through the I/O shield, and let go of it." \
  --scorer task_score --max-steps 1200 --epochs 2
```

Use `--policy dream -E cam_width=320 -E cam_height=176` for Dream-1. `--epochs 2` runs one trial in each layout lot:
trial k uses lot k mod 2 (lot A first, as in the screenshot); `TASK_LOT=A` or `TASK_LOT=B` in the environment pins
every trial to one lot. Without `--epochs 2` only lot A is run. Submissions use `object_jitter=0`; the layout has no
random seed. A trial takes at most 1 minute of simulated time (1,200 steps at 20 Hz) plus Isaac Sim start-up; the
scripted demo reaches full score and ends the trial at step 533 (about 27 s) in each lot.

Offline checks (no simulator): `python near_misses.py` cases are run by the program's offline checker against
`score()` and `success()`.

## The scene

All positions in metres; x forward from the line through the two arm bases, y to the robot's left, z up; bench top
z 0.7475. The full list is in `task.json` `setup`; the essentials:

- Chassis (fixed): outer faces x 0.199 to 0.448, y -0.291 to +0.146, 1 mm steel, walls 43 mm high. Standoffs:
  brass, 5.5 mm across, 6.35 mm high (estimates), tops at z 0.7549, at the Mini-ITX pattern around the board's place (centre
  x 0.360, y 0.050). I/O shield: 1 mm steel, inner face x 0.447, spanning y -0.025 to +0.125, with five cut-outs.
- Foam pad (fixed): 150 x 150 x 10 mm, centre x 0.272, y 0.330.
- Board (loose): 170 x 170 x 1.6 mm PCB with heat sink, four DIMM slots and five rear connectors; 0.45 kg (estimate).
- Lot A: board centre x 0.272, y 0.330, connector edge facing +x. Lot B: centre x 0.284, y 0.338, connector edge
  facing -y (toward the chassis), so a replay of lot A's motions puts the board in with its connectors facing a side
  wall.

## Scoring

Read at the end of each trial (`score()`), trials averaged over lots A and B. The text in `task.json` `scoring` is
the same rule; the code names are given here.

Zero first, for the whole trial (it stays 0 once triggered; cleared only by a scene reset, the board back at its
start, at rest, after a jump of over 20 mm in one control step):
- the board moves downward faster than 0.6 m/s at any moment, held or not (a drop or a slam; a free fall of about
  2 cm reaches that speed), judged from the video;
- any corner of the board comes within 3 mm of the chassis floor (it slipped off the standoffs);
- the board leaves the bench.

Every credited level also needs all of these (`_value()`, `on_standoffs()`):
- let go: no point of either arm's links 3 to 5, gripper or fingertips within 5 mm of any part of the board (arm hull
  faces sampled every 3 mm; no credit while the arm state is unknown);
- still for 1 s after the arms are clear (within 0.2 mm and 0.2 degrees per control step, 20 steps);
- flat on all four standoffs in its place: the gap under the PCB at each standoff at most 1 mm (a 1 mm feeler
  gauge does not enter), every standoff top fully under the PCB (seen from above, none shows past the PCB edge),
  tilt at most 2 degrees (phone inclinometer), connector edge toward the shield and turned at most 5 degrees
  (protractor against the rear wall), every mounting hole within 10 mm of its standoff (ruler).

| Score | State | How it is read in a real trial |
|---|---|---|
| 1.0 | Home (`home()`, `centred()`): both D-sub flanges bear on the shield (gap at most 0.3 mm) and every hole axis is within 0.48 mm of its standoff's | A 0.3 mm feeler gauge does not pass between either flange and the shield; an M3 x 5 mm screw starts by hand in all four holes without moving the board |
| 0.5 | Both D-sub shells at least 1 mm into their cut-outs (`entered()`), but not home | The shells are flush with or past the 1 mm shield's outer face: a straightedge laid on the outer face touches each shell |
| 0.25 | Flat on its standoffs as above, but a D-sub shell is less than 1 mm into its cut-out | The checks in the list above |
| 0 | Anything else: on the pad, held, tilted, turned round, off its place, dropped | |

`near_misses.py` holds 40-odd cases, run in order with the whole-trial record: full success and the hold, still
gripped, arm state unknown, an arm face 4.5 mm and 6 mm from the heat sink with no corner near, each level and the
states either side of its limits (shells 0.9 and 1.1 mm in, flanges 0.29 and 0.31 mm off, 0.47 and 0.49 mm to the
side), turned 6 and 90 degrees, tilted 2.5 degrees, held 1.5 mm up, 6 mm to the side, left on the pad, a drop then a
good finish (0), slipping onto the floor then a good finish (0), and a scene reset that clears the record.

## The demonstration

`demo()` (left arm only, open loop from the visible layout: where the board lies and which way its connector edge
faces; nothing is read from the simulator during the run):

1. Grip the heat sink from above at the board's balance point (a person would hold the board by its edges; the
   YAM's parallel fingers can close only on the heat sink, which is fastened to the board), lift the board off the
   foam and turn it so its connector edge faces the rear wall.
2. Carry it over the left wall and lower it into the chassis about 12 mm short of home, holding it about
   13 mm above the standoffs (18 mm commanded; the loaded arm sags 4-5 mm) so the D-sub shells pass above their
   cut-outs; move it back until the D-sub shells meet the shield's inner face just above
   their cut-outs, then put it down on the standoffs (the shells slide down the shield to their cut-outs) and let
   go: it settles flat on the four standoffs. Lift off.
3. Grip the heat sink again and slide the board back along the standoff tops: the shells enter their cut-outs, the
   flanges stop the board against the shield and square it, and every hole comes to rest over its standoff. Ease
   off, let go and move away.

`CARRY_FIX` and `SLIDE_FIX` are the left arm's repeatable sideways tracking offsets at the chassis, loaded (3.6 mm)
and unloaded (2.2 mm), measured once in a trace and the same in both lots. Where the board comes down along x is set by the shield face,
not by the arm. `DEMO_VARIANT=short|off_side|no_push` give shortcut variants of the same plan (left short of the
shield, re-gripped 2 mm to the side, no push) for traces.

## Results

Both runs used this `task.py` (SHA-256 begins 73b167ac), `--epochs 2`, `object_jitter=0`:

- Scripted demo (`--policy yam_scripted`): 1.0 in lot A and 1.0 in lot B; each trial reached full score and ended
  at step 533 of 1,200. The video and screenshot show lot A.
- Dream-1 (`--policy dream`, same instruction and limit): 0.0 in lot A and 0.0 in lot B; both trials ran to the
  1,200-step limit. Dream-1 did not refuse the instruction.
- During development, a trace in which the board was left flat on the standoffs with its D-sub shells against the
  shield beside their cut-outs scored 0.25, as the scoring intends.

## Fidelity and limits

- Rigid bodies at true size, PhysX at 240 Hz, 20 Hz control.
- The board, heat sink and connectors are one rigid body made of boxes. The board's outline, thickness and hole
  pattern follow the Mini-ITX standard; the connector positions along the edge, the heat sink's size and the
  0.45 kg mass are estimates.
- The PCB has no hole geometry: the 5.5 mm standoff tops are wider than the 3.96 mm holes, so the board rests on
  the standoff tops whether or not a hole is over a standoff, as the real board does. Holes and the standoffs'
  threaded holes are drawn on top for the camera.
- The shield's cut-outs are plain rectangles sized from the modelled connectors (estimates); the real shield's EMI
  gasket and fingers are not modelled. The D-sub flanges bearing on the shield's inner face are the depth stop; on
  the real board the flanges sit against the inside of the shield when the board is in place, but the exact gap is
  an estimate.
- The chassis's interior parts are boxes at estimated positions; they matter only as obstacles.
- Friction between the board and the standoffs and shield is 0.45 static, 0.35 dynamic (estimate for solder-masked
  FR-4 on brass and steel).
- Two lots stop one fixed habit (replaying one layout's motions); they cannot stop a replay of both layouts.

## Assets and licences

- Reused scene: the program's station scene with the two YAM arms, cameras and lights, from the yam-isaac-lab-arena
  environment (the program's YAM embodiment for Isaac Lab-Arena). Its YAM robot model comes from i2rt under the MIT
  licence; the environment repository itself states that it has no licence yet (placeholder). Isaac Lab-Arena:
  Apache-2.0; Isaac Lab: BSD-3-Clause; Isaac Sim: NVIDIA's Isaac Sim licence. Nothing else is reused.
- Every object in this task is built from primitives in `task.py`; no third-party meshes, scans or textures.
- `parts.py` and `lots.py` are task-local helpers for the program's YAM embodiment (articulated parts, layout lots);
  they patch only this task's own process.
- Product names (Supermicro A2SDi-8C-HLN4F, CSE-505-203B, MCP-260-00085-0B, SYS-5019A-FTN4) identify the real
  parts for the physical trial; sizes come from the maker's catalogue pages (board 6.7 x 6.7 in, chassis
  17.2 x 9.8 x 1.7 in).

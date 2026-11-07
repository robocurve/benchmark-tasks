# Run a layered hose out of its bed from the end on top and lay it to a hydrant pad

A fire-service hose bed on the table. A black 8 mm hose lies in an open-ended bed in three stacked layers, with a
brass coupling on each end; the two couplings look the same. One coupling lies on top of the load at the bed's
closed (far left) end. The other comes out from under the load at the open end, the one nearer the middle of the
table and the pad, so it is the convenient one. A yellow hydrant pad lies far out on the right. The robot must run
the hose out of the bed and put its end on the pad, leaving it lying clear, and let go.

Only the top end pays out cleanly: taken from the top, the layers peel off one after another. Taken from the bottom,
the layers above ride out on top of it and the load comes out in a tangle. The instruction does not say which end to
take; the robot has to read it from the layering.

## Objects

| object | type | size | material / colour |
|---|---|---|---|
| hose bed | 3 fixed walls (box) | inner 12 x 22 cm, walls 1 cm thick, 5 cm tall; open at the y 22 cm end | painted wood, dark red |
| hose | deformable cable | 70 cm, 8 mm diameter, 50 g | black rubber hose |
| couplings (2) | rigid boxes on the hose ends | 30 x 16 x 12 mm, 12 g each | brass |
| hydrant pad | fixed flat box | 12 x 12 cm, 2 mm thick | yellow |

Positions (cm from the midpoint between the arm bases, x forward, y left): bed inner floor x 24-36, y 22-44. Hose
along x 30: the bottom layer runs from the open end (its coupling sticks out to y 18.5) to y 36.4, turns in a 5 cm
loop at the closed end up onto itself (middle layer, back to y 29.6), turns in a second loop near the open end up
onto the top layer, which ends with its coupling at y 42 on top of the load. Pad centre x 25, y -40.

## Two arms (offline IK, gripper pointing down)

- The bed (y 22-44) is out of the right arm's reach: no IK solution at (30, 15), (30, 22), (30, 40) cm; the
  nearest it reaches is (24, 15).
- The pad (x 19-31, y -34 to -46) and everything 30 cm or more right of the midline are out of the left arm's
  reach: no solution at (19, -34), (31, -34), (5, -30), (10, -30), (20, -30), (30, -30) or (10, -25) cm, at 1.4 or
  4 cm above the table. The right arm reaches all four pad corners.
- So the top coupling is carried by the left arm to the middle of the table, set down, and taken on by the right
  arm. While the right arm holds the end on the pad, the left arm lays the last of the hose (the bottom coupling
  trails out of the bed doubled back on itself) in an open curve towards the robot. The 0.3 tier (end at least
  30 cm right of the midline) is also out of the left arm's reach.

## Scoring (`score()`, at the end; `success()` must hold 1 s with the hose at rest)

- **0** if: any part of the hose that started in the bed left it (2 cm beyond its inner outline) while the top
  coupling was still over the bed floor: the load was pulled from the wrong end or dragged out by its middle
  (latched for the run); a gripper centre is within 3 cm of the hose or a coupling at the end; the hose already out
  of the bed lies across or against itself (two parts at least 8 cm apart along the hose within 1.2 cm of each
  other, top view) or bends tighter than 3 cm radius; or the top coupling is not down on the table at least 30 cm
  right of the midline.
- **0.3** otherwise: the top end has been run out, handed over and laid down on the right, clear and released.
- **0.6** the top coupling's centre is on the pad, but some hose is still over the bed floor.
- **1.0** the top coupling is on the pad and no part of the hose or couplings is over the bed floor.

Why the order rule: a stored line runs out cleanly only when "the line going out first is on top of the line going
out last" (Sailing World), and a flat load pulled any other way ends "tangled up like a pile of spaghetti" (Fire
Engineering). The instruction does not state the rule (it tests whether the robot reads the layering); the scoring
checks it directly. In the sim, pulling the bottom coupling does tangle the hose (the `bottom` run ends with the hose
lying across itself in many places), but a load dragged out by its middle and then pulled by its top end straightens
again (the light 8 mm sim hose has no twist), so for that case only the order rule gives the 0. A real hose keeps
the twists and kinks the sim leaves out.

## Shortcut and partial variants (test only, `HOSE_CHEAT=<name>`)

Evidence in `scratch/sc_<name>.json` and `.log`, all run on the final `task.py`.

| variant | what it does | score |
|---|---|---|
| `none` | no motion | 0 |
| `hold` | full demo, but the right arm keeps holding the coupling on the pad | 0 (not released) |
| `onearm` | left arm alone runs the top end out to the middle and sets it down | 0 (end not 30 cm right of the midline; the left arm cannot reach there) |
| `bottom` | the convenient wrong end: bottom coupling run out, handed over, carried to the pad | 0 (order latch; the load also came out lying across itself in many places) |
| `dump` | left arm drags the load out of the bed by its middle, then the top end is run to the pad as in the demo | 0 (order latch; the final lay was clean, see "Why the order rule") |
| `short` | top end set down right of the midline short of the pad, tail laid back in the bed | 0.3 (tier check) |
| `near` | top end on the pad, bottom coupling laid back in the bed | 0.6 (tier check) |
| demo | full task | 1.0 |

## How this differs

- Nearest board task: TK-4US7Z8, pay a multicore off a free-turning drum (ours). There the line comes off a
  turning drum and the skill is controlling the drum. Here nothing turns: the line comes out of a static layered
  pile, and the skill is reading which end lies on top, since the two ends look alike and the wrong one is closer.
- Nearest of our tasks: flake-cable-into-crate is the reverse act (laying a cable into a crate in zigzags).
  TK-J79PY6 (pass a garden hose over a fence) and drain-hose-slopes-downhill route a hose that is already laid out.

## Paper fields

- Skill tags: extraction, sequencing, handoff, placement/pinning.
- Industry: fire service / emergency response (also marine and rigging, where flaked lines are run out the same way).
- Two-arm need: reach-forced (bed left-only, pad right-only, one midline handover), then a simultaneous hold (right
  arm holds the end on the pad while the left lays the tail).

## Sources (opened; quotes verbatim)

- Sailing World, "Coiling Lines and Flaking Halyards" (www.sailingworld.com/how-to/how-to-coil-a-line): "When it
  comes to flaking halyards, the most important thing is that the line going out first is on top of the line going
  out last." and "To ensure the coiled line runs smoothly, lay it with its bitter end at the bottom or away from the
  bulk of the coil."
- Fire Engineering, "Back to Basics: Pulling the Flat Load Off the Rig"
  (www.fireengineering.com/apparatus-equipment/back-to-basics-pulling-off-the-flat-load/): a badly pulled flat load
  leaves the hose "tangled up like a pile of spaghetti", and "if we do not flake out and charge the hose correctly,
  kinks in the hose will occur, the hose will twist itself into knots". The article is about pulling the nozzle end off and walking
  away without flaking; it does not single out the bottom end, so the context cites it only for unflaked pulls.
- Fire Engineering, "Engine Company Basics: The Minuteman Hose Load"
  (www.fireengineering.com/firefighting/engine-company-basics-the-minuteman-hose-load/): "as the firefighter
  advances the hoseline, the top part of the line starts to flake off as he advances." (opened and checked: the top section comes off first.)
- O*NET 33-2011.00 Firefighters, task (opened): "Operate pumps connected to high-pressure hoses." This is an occupation
  link only; it does not describe hose loading, and the context labels it so.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py`, `cableshape.py` and `cabledemo.py` are in this folder (hose simulation, layered start layout,
  scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1000 \
    --instruction "Run the black hose out of its bed and put its end on the yellow pad, then let go."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1000
```

Closed loop on the live couplings. The left arm pinches the top coupling, lifts it, turns it to point the way the
hose will run, draws it over the open end to the middle of the table and sets it down. The right arm picks it up,
carries it to the pad and draws it 4 cm along the pad so the hose behind it lies straight. While the right arm
holds it there, the left arm lifts the bottom coupling (the last of the bottom layer trails out folded back on
itself) and lays it towards the robot in an open curve. Then both let go. `demo/` holds the videos, `setup.png` and
`result.json`.

Runtime: the demo takes 752 steps (about 38 s at 20 Hz); a 1,000-step trial takes several minutes on one GPU. Seeds: the layout is
fixed (`object_jitter=0`); the layered start is written at every reset (`cableshape`) and is deterministic.

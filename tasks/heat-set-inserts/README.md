# Install two M3 heat-set inserts in a 3D-printed robot part

A 3D-printed PLA robot link plate lies on the table with a brass M3 heat-set insert sitting loose in the mouth of each of
its two pilot holes. A soldering iron with an M3 heat-set tip rests on its stand, and a steel 1-2-3 block stands beside the
plate. The robot must install both inserts the way CNC Kitchen's guide describes: melt each one most of the way in with
the iron, keeping the iron square to the plate and pressing lightly, then press the last part with a flat tool (the
block) while the plastic is still soft, so the insert ends flush and square. It then lays the hot iron back on its
stand.

A heat-set insert is a small brass sleeve, knurled outside and threaded inside (M3 here). Pressed into a hole in a 3D-printed plastic part with a soldering iron, it heats, melts the plastic round it and sinks in; the plastic sets again in its knurls and holds it, so the part has a metal thread that survives being screwed and unscrewed many times, where a thread cut in the plastic would strip. Open-source robot arms are built this way: the Instructables "Giant 3D Printed Robot Arm" joins most of its parts with heat-set inserts and machine screws, set with a soldering iron at 230 C; USC's SPRITE robot, the RBX1 arm and the PAROL6 desktop arm include the same step in their build guides. CNC Kitchen's guide ("Tips & Tricks for Heat-Set Inserts used in 3D printing") gives the method used here: iron 10-20 C above the print temperature (225 C for PLA), melt the insert to about 90 % of its depth, finish with a flat tool. The difficulty is physical: the iron must come down on a 4.4 mm insert and stay vertical while it sinks
it with light pressure (a tilted iron sets it crooked), stop short of flush (the tip's shoulder melts the plastic round
the hole if it goes past), and the block must reach the insert while it is still soft and come down flat on the
plate, which is what leaves it flush and square.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: scene, the melting rule, `score()`, `success()`, `self_test()` and the scripted two-arm `demo()` |
| `contactkit.py` | Reads the PhysX contact forces between the iron or the block and each insert or the plate |
| `touch.py` | Reads whether any robot link touches the iron (for "let go") |
| `scenekit.py` | USD bodies and fixed shapes for yam_arena task files (hooks yam_arena without changing it) |
| `tiltkit.py` | Any gripper orientation for the scripted demo (the iron is held tip-down from the side) |
| `assets/plate.usdc` | The plate (30 x 60 x 10 mm) with its two inserts, each on a prismatic joint, a fixed-base articulation, 277 KB |
| `assets/plate.stl` | The plate with its holes, for printing, 605 KB |
| `assets/iron.usdc` | The soldering iron with its heat-set tip, a free body, 26 KB |
| `assets/block.usdc` | The 1-2-3 block, a free body, 1 KB |

No third-party assets; `work/heatset/make_parts.py` builds all four from the dimensions below.

## Setup

Positions in cm from the midpoint between the two arm bases (x forward, y to the robot's left); heights above the
table top. Same layout every trial.

| Object | Where | What |
|---|---|---|
| Plate | centred at (33.0, -3.0), lying flat, its 60 mm side along y, held still (taped to the table) | 3D-printed PLA plate, 30 x 60 x 10 mm (`assets/plate.stl`), with two blind pilot holes 4.0 mm across and 8 mm deep at (33.0, -4.5) and (33.0, -1.5) (the M3 pilot hole from Accu-Components' hole chart) |
| Inserts | one in each hole's mouth, set 0.8 mm in, upright | Brass M3 heat-set inserts, 5.7 mm long (McMaster-Carr 94459A140, M3 x 5.7 mm installed length), 4.4 mm across (Accu-Components, non-flanged M3) |
| Soldering iron | lying on its stand along x, tip towards the robot at (20.0, -17.0), its axis 4.0 cm above the table; switched on at 225 C | Pen-style soldering iron with an M3 heat-set insert tip (a 5 mm shoulder over a pilot pin), about 175 mm long, its grip about 15 mm across, 45 g |
| Iron stand | under the iron | Two supports, each a bar with a post either side: one under the shaft 3.5 cm from the tip, one under the grip 16 cm from the tip |
| 1-2-3 block | standing on its 25.4 x 50.8 mm end at (25.0, 10.0), its 50.8 mm side along x | Hardened steel 1-2-3 block, 25.4 x 50.8 x 76.2 mm, 0.77 kg |

## Scoring

Check each insert after the trial with a straightedge and a 0.2 mm feeler gauge, a 30 mm M3 screw and a small
square, and a photo of the plate.

- Each insert scores **0.5** when it is **flush** (a straightedge across it rests on the plate and a 0.2 mm feeler
  gauge does not pass under it, and it is not more than 0.2 mm below the surface), **square** (a 30 mm M3 screw
  threaded into it stands within 3 degrees of perpendicular: 1.6 mm at its head against a small square), and the PLA
  round it carries **no melt mark** from the iron's shoulder.
- **0.25** if it is at least half way in (2.45 mm or more below where it was set) but not flush, not square, or marked.
- **0** if it is less than half way in, or sunk more than 0.2 mm below the surface.
- The two inserts' scores are added; full credit (1.0) also needs the iron lying back on its stand, let go, for 1 s;
  until then the score is at most 0.75.

## Run a trial

From this folder, in a shell with the yam_arena environment active (`source <ARENA_DIR>/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "This 3D-printed robot part needs metal threads for the M3 screws that will hold it. Install the two brass heat-set inserts sitting loose in its holes: melt each one most of the way in with the hot soldering iron, keeping the iron straight, then press it flush with the steel block while the plastic is still soft. Lay the iron back on its stand when you are done." \
  --scorer task_score --max-steps 3600 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/heatset-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. Seed 0; the
scene has no random placement (`-E object_jitter=0`). On an RTX 3060 the scripted demo (60 s of simulated time) takes
about 6 minutes including start-up; a full 3,600-step trial about 15 minutes.

Offline checks (no GPU): `python task.py` runs the melting rule and the scoring on hand-made pushes and replays both
arms' plans through their kinematics.

## Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 --instruction "..." \
  --scorer task_score --max-steps 3600 --store-frames --log-dir ~/yam-eval/heatset-demo
```

The right arm picks the iron off its stand by the grip, lifts it clear and turns it tip-down; for each insert it comes
down onto it and sinks it to 80 % of its depth at 1 mm/s, lifts and moves off to the right. The left arm, holding the
block by its top, brings it over the insert once the iron is clear and presses it down flat on the plate, then lifts
it aside. Finally the iron is turned level and laid back on its stand, and the block set back where it stood.
`HEATSET_LOG=1` prints each insert's depth, tilt and state and the contact forces every 0.5 s.

## Reset

Use a fresh printed plate with two fresh inserts set in its holes for each trial (or reprint the plate); let the block
cool and wipe it; lay the iron back on its stand.

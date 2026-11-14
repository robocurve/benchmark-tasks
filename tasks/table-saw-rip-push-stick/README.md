# Rip a short pine board on a running table saw (push stick, hold-down hand)

**Category:** Impact on jobs. Ripping stock on a table saw is routine carpenter work (O*NET 47-2031.00, about 700,000 US workers, BLS OEWS May 2023). The board is fed with a push stick so the hand stays clear of the blade. Two hands are needed because they do different jobs: one holds the board down on the table and tight to the fence against the cutting loads (which push it away from the fence and lift it), the other feeds it forward.

**Strengths.** Real-size build (600 x 1070 mm cast-iron saw table, 10 in blade, 3.2 mm kerf, 400 mm push stick, 19 x 89 x 220 mm pine board). The scorer is an end-state ruler check (strip width, flat, still, hands clear). The scripted demo scores 1.0. Four ablations (one arm only, bare hand, early release) each fail on a real gate.

**Contrast with other tasks.** #586 uses a relief block at a saw fence; #1245 is a paper guillotine with the hand kept clear. Here one workpiece is fed past the blade with a tool by one arm while the other arm holds it down and to the fence.

## What it tests
Two arms, two different jobs on one workpiece: the right arm feeds the board with a push stick (fork-heeled, so it straddles the blade), the left arm keeps the board down on the table and tight against the fence while the cut starts. The instruction gives the goal only; the score checks the hand-to-blade distance a carpenter is taught (100 mm, below 150 mm above the table).

Instruction: "Cut the pine board lengthwise along the fence so that a 40 mm strip comes off, then leave both pieces lying flat on the table."

## Scene (station frame: x forward, y left, z up, metres; table top z = 0.7475)
| Object | Size (mm) | Material, mass | Pose |
|---|---|---|---|
| saw table | 600 (x) x 1070 (y) x 30 (z) slab, x 0.120..0.720, y -0.62..0.45 | cast iron, with fence and blade one rigid body, 142.6 kg (cannot be shoved) | on the station table |
| rip fence | 30 x 1070 x 45 high, face at x = 0.185 | aluminium (in the 142.6 kg) | along the table |
| blade | 10 in (254 mm) disc drawn as two steps: 2.4 mm plate, 150 mm long at the table, 77 mm long at the top, 25 mm above the table, red teeth strip | steel | x = 0.2266 (centre of the kerf), y 0.105..0.255 |
| stop line | 4 mm red strip, flush, y = 0.005 (100 mm before the blade) | paint | across the table |
| pine board (strip) | 40 x 220 x 19 | pine 500 kg/m3, 0.0836 kg, friction 0.30 | x 0.185..0.225, y -0.12..0.10 (against the fence, front end 5 mm before the blade) |
| pine board (offcut) | 45.8 x 220 x 19 | pine, 0.0957 kg, friction 0.30 | x 0.2282..0.274, same y |
| push stick | forked heel (two blocks 20 x 26 mm tall, 14 mm gap for the blade), 400 mm bar 30 x 10 mm riding 3 mm above the blade top, upright grip fin 50 x 12 x 50 mm 330 mm behind the heel | pine, 0.115 kg, friction 0.5 | lying on the saw table at x 0.36, heel front face y -0.20 |

The 3.2 mm kerf (1/8 in) between the pieces is the real saw kerf; the 89 mm board is a real 1x4. The saw table is a real size benchtop/contractor saw with in- and outfeed.

## Scoring (end state; every number from task.py)
- Zero gate (latched, a real hazard): any gripper fingertip, gripper link or TCP within 100 mm in plan of the blade while below 150 mm above the saw table (the shop rule: hand 4 in from the blade, push stick inside 15 cm); the push stick touching the blade plate; a piece falling or thrown off the table; the board kicked up more than 30 mm off the table (kickback).
- 0.25: board front 30 mm into the blade, or the board was lifted more than 4 mm over the table while over the blade (lifted over, not cut).
- 0.5: cut complete (both rear ends at least 12 mm past the blade's back edge) but a piece not lying flat on the saw table, or the strip width at the blade centre off 40 mm by more than 1/4 in (6.4 mm).
- 0.75: complete, flat, width within 6.4 mm, but a gripper within 30 mm of a piece, a piece still moving, or width not within 3.2 mm.
- 1.0: complete, strip 40 +- 3.2 mm (1/8 in, a rough-cut tolerance) wide where the blade is, pieces flat (tilt <= 8 degrees) and still (<= 10 mm/s) on the saw table, both grippers >= 30 mm from the pieces and >= 100 mm from the blade (below 150 mm). All of this true for 20 consecutive steps (1 s) before 1.0 is returned. The push stick may lie anywhere.
- Hand-to-blade and 150 mm height: shop teaching (push stick inside 15 cm of the blade; hand never within 4 in, 100 mm).

## Design: what is code-driven, what is not modelled
- **The blade and the cut are code.** The blade is drawn but has no collision with the board (a 0.4 mm clearance between a 2.4 mm plate and a 3.2 mm kerf cannot be threaded by a pushed free board in the solver). The board is two pieces lying with the real kerf gap between them; the cut is complete when both rear ends are past the blade. The strip width is measured from the strip's fence-side edge at the blade centre while the board is in the cut (latched min and max).
- **The side, lift and yaw loads are code.** While the board is in the cut and its rear end is more than 170 mm before the blade (the stretch a hold-down hand can cover while staying 100 mm from the blade), persistent external forces act on the strip, the piece guided between fence and blade, which carries its 45 % width share of the board loads: base side force 1.0 N away from the fence, lift 2.0 N, yaw torque 0.02 N m (so about 0.45 N, 0.9 N and 0.009 N m on the strip). These are modelled well below real feed forces (25-60 N) because the simulated arms stall near 7 N; they are set so that an unheld board leaves the fence and lifts. The lift stands for the kickback tendency of rising rear teeth: the strip weighs 0.82 N, so an unheld strip floats up. The side load stops once the strip is 12 mm off the fence; the loads stop once the strip is out of blade reach. With the pad pressing on top the strip stays within about 2 mm of the fence. After the rear end is within 170 mm of the blade no loads act.
- **Not modelled:** blade rotation and teeth, sawdust, kickback dynamics, the pinch of the kerf closing, the blade guard and riving knife (a real saw has them), noise, vibration, the dust port. The offcut is unloaded; the pieces are plain rigid boxes.

**Limitations.** Coded cut and loads; small force magnitudes; the stick is gripped by a fin; fixed layout (object_jitter=0); no guard or riving knife.

## Files
`task.py` (scene, scoring, loads), `partkit.py` (shared compound-part helper, unchanged), `demo.py` (scripted demo), `tests/test_scoring.py`, `task.json`.

## Run
```
inspect-robots run --embodiment yam_arena --policy dream -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<instruction above>" --scorer task_score --max-steps 1500 --store-frames --log-dir <dir>
python demo.py task.py demo_out [--no-record]     # DEMO_VARIANT: settle, one_right, one_left, bare_hand, early_lift
python -m pytest tests -q
```
Run limit 1,500 steps (75 s at 20 Hz). The demo takes 703 steps. Seeds: object_jitter=0, no randomness.

## The demo (open loop in the fixed layout)
It reads no object pose. Each arm's own joint positions (proprioception) are integrated into the command. Right arm: takes the push stick by its grip fin (fingers open along y, pinching the 12 mm fin), lifts it 7 mm, carries it to 30 mm behind the board, then feeds at 20 mm/s for the first 60 mm and 35 mm/s after, until the board's rear end is 6 mm past the cut-complete line. Left arm: goes high over the saw (above 150 mm), lowers a closed fist onto the board's top over the strip and kerf, presses (about 30 mm of commanded travel below contact, a few N), rides with the board for the stretch where the loads act, then lifts away and returns to its ready pose. Right arm releases and backs away 120 mm.

## Real-world build and reset
A table saw with a 254 mm blade, a rip fence set 40 mm from the blade's face, a 19 x 89 x 220 mm pine board marked with a 3.2 mm kerf line, a 400 mm push stick, a painted line 100 mm before the blade. Reset: pieces back against the fence, front end 5 mm before the blade, stick at the right of the board. Measure strip width at both ends with a ruler and fingertip-to-blade distance against the painted line.

## Shortcuts and what stops each
1. Bare-hand push the whole way: gripper enters the 100 mm zone: score 0.
2. Lift the board over the blade: board bottom more than 4 mm above the table while over the blade caps the score at 0.25.
3. Fling the board: off the table or more than 30 mm up: 0.
4. Push the stick through the blade: touching the blade plate: 0.
5. No hold-down hand: the loads push the board off the fence and lift it: see the ablations.
6. Hold-down hand riding into the zone: 0.

## Measured in the simulator
| Run | Score | What happened |
|---|---|---|
| settle (no action) | 0.0 | pieces at rest, no gate |
| demo (both arms) | 1.0 | 703 steps; strip width 37.7-39.4 mm (within 1/8 in); held 1 s |
| right arm only (stick, no hold-down) | 0.0 | strip yawed 9 deg and swung 40 mm off the fence, then lifted more than 30 mm: kickback gate |
| left arm only (bare closed hand pushes the rear) | 0.0 | board kicked up (gate) |
| both arms, right pushes with a bare fist | 0.0 | fist within 100 mm of the blade: hand gate |
| both arms, left lifts away after 10 mm | 0.0 | unheld board kicked up: kickback gate |
| Dream-1 run | 0.0 | 1,500 steps, the arms did not reach the stick or board (frame at 40 s: stick and board untouched) (the first wording naming the machine was refused by the service before any step and was not scored) |
Runtime: the demo takes about 35 s of simulated time (703 steps at 20 Hz); run limit 1,500 steps.

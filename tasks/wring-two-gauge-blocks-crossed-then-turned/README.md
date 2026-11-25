# Wring two gauge blocks crossed, then turned into line

Two Mitutoyo rectangular steel gauge blocks (section 35 x 9 mm; 50 mm and 25 mm) stand on a measuring face on a black
granite surface plate. The maker's quick guide for thick blocks: cross the blocks at 90 degrees in the middle of the
measuring faces, rotate them while applying slight force, then align the measuring faces
(https://assets.testequity.com/te1/Documents/pdf/mitutoyo/Mitutoyo_Gauge-Blocks-Quick-Guide_Manual_0625.pdf).

## Run and score a trial

In the simulator's environment (yam-isaac-lab-arena's `setup.sh`), from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=wring_two_gauge_blocks_crossed_then_turned.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Wring the 25 mm gauge block onto the top face of the 50 mm block so the two make one stack with their faces in line, then let go of both." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir logs/wring-dream
```

For the scripted demo use `--policy yam_scripted` (and `-E cam_width=640 -E cam_height=352`).

- One fixed layout (`object_jitter=0`); the seed changes nothing. Runtime per trial: up to 1,800 steps (90 s at 20 Hz);
  the scripted demo takes 1,220 steps (61 s).
- Demo modes (`KIT_DEMO`): `demo`; `slam` (set down in line); `hover` (turned in the air); `held` (never let
  go); `wiggle` (no net turn); `light` (about 1 N); `leftpress`, `platepress` (the left hand pushes the
  50 mm block, or the plate, while the right turns a resting block); `none`.
- Tests: `PYTHONPATH=<yam-isaac-lab-arena> PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q` in this folder.

## Runs

- MuJoCo physics check, this task file: demo 1.0; `held` 0.75; `slam`, `hover`, `wiggle`, `light` and
  `leftpress` 0.25; `platepress` 0; `none` 0.
- Isaac (GPU), this task file: scripted demo 1.0 in 61.0 s (half-second trace: `demo_trace.txt`). The counted turn
  was one unbroken spell from 22.5 s: face to face at every control step, the scale at 3.7-4.4 N, no arm link within
  5 mm of the 50 mm block, plate or scale. Corner gaps at the steps that added the 60 deg: 0.038 mm at most, in the
  spell's first steps as the block settled under the press; 0.003 mm median, 0.025 mm at most, at the trace samples
  after. Wrung by 33.5 s, 4.2 mm from the face's middle (limit 6); squaring up then lifted a corner 0.43 mm at most
  (limit 1), the block always over the face; released at 48.5 s, within 0.4 mm and 0.6 deg, still.
- Dream-1 0.0: in 90 s (1,800 steps) it never gripped either block, and neither block moved more than 1 mm.

## Scoring (`grade()` in the task file; task.json states every number)

| Score | End state (first that fits) |
|---|---|
| 0 | at any time, even if put back: a block fell over or dropped off the plate, or the 50 mm block was lifted |
| 1.0 | wrung, in line, released and still |
| 0.75 | wrung and in line, still held or not yet still |
| 0.5 | wrung, not in line |
| 0.25 | the 25 mm block on the 50 mm block's top face, not wrung |
| 0 | anything else, including the untouched start |

Wrung (simulator): the 25 mm block gripped and turned a net 60 deg or more (turning back subtracts) in one unbroken
spell in which, at every control step, the two faces are face to face (every corner of the patch where they overlap
within 0.05 mm of the 50 mm block's face), the bench scale under the plate reads at least 3 N, and no part of either
arm is within 5 mm of the 50 mm block, the plate or the scale; any step that fails restarts the turn. Leaving the face afterwards loses it:
lifted or peeled more than 1 mm at any of its four lower corners, or slid until nothing is over the face (tests 13, 15, 17).
Wrung (real bench) needs both: the video shows that turn with the filmed scale at least 306 g (3 N) above its tare
and no hand on the 50 mm block, the plate or the scale, and afterwards lifting the 25 mm block 20 mm by hand lifts
the 50 mm block with it. `revision.json`: each earlier review point with its code line, test and trace number.

## Fidelity

Rigid, free bodies: nothing in code holds a block. The oil film is not simulated: the scorer credits wringing from the
work that makes it. After the counted turn the simulator only checks that the block was not taken off the face
(1 mm); on a real bench the lift test decides whether it stayed wrung.

- Press: the maker says "slight force" and publishes no figure, so the floor is a chosen one: 3 N, about five times
  the 25 mm block's weight (0.6 N), so a resting block or a brushing touch never counts. A real trial checks the same
  floor on the filmed scale (at least 306 g above tare): simulator and bench need the same press. It is read on
  a bench platform scale under the plate (in simulation a 20 kN/m platform, tared in the first second with both arms
  clear), and it can only come through the faces: no arm link may be near anything else the scale carries (tests 11,
  14).
- Face to face: both faces are flat, so the corners of the overlap patch bound every point between them. One edge down
  and the other up does not count: at 1.5 deg the far edge of the 9 mm face is 0.24 mm clear, and
  `test_8` turns such a block a pressed 90 deg and counts nothing (so do a
  0.06 mm one-edge gap, a 0.2 mm lift, no press and a hand on the 50 mm block: tests 9-12).
  Why 0.05 mm (0.32 deg across the 9 mm face) and not zero: rigid bodies have no oil film or surface give, so a
  block pressed and turned by a compliant arm never sits at exactly zero. The demo's corners read 0.003 mm typically
  and up to 0.045 mm for a step, so a 0.02 mm limit would break even its levelled turn. The simulator's own contact
  test is far looser (contact points from 1 mm away). 0.05 mm is the thinnest leaf of a common feeler set.
- Demo: the left hand steadies the 50 mm block while the 25 mm block is set down, then lets go and backs out. The
  right hand servos its press on the scale to 4.2 N and turns 90 deg over 16 s with no sideways pull, its wrist
  levelling the block from its live tilt; then it squares the stack up.
- Masses from steel density at catalogue size (124 g, 62 g); friction static = dynamic (blocks 0.3, plate 0.5).

## Real trials

Safe to run: two small steel blocks, light forces, no sharp edges or heat. Before each trial
prepare the two meeting faces as in task.json setup (alcohol wipe, a trace of oil wiped almost off).

Parts (equivalents work):
- Mitutoyo rectangular steel gauge blocks, 50 mm and 25 mm, grade 2 (workshop grade): about $58 each (611675-551,
  marssupply.com).
- Granite surface plate, grade B, 12 x 12 x 3 in. A stock 12 x 9 x 3 in grade B plate (HHIP 4401-0011, about $80,
  penntoolco.com) also works with its 12 in side along y.
- Bench platform scale, 30 kg x 1 g, platform at least 30 cm square (Insize 8001-30, about $439, zoro.com), its display
  facing the side camera.
- Isopropyl alcohol, lint-free wipes, gauge-block oil (about $15), a protractor, a steel rule.

## Assets

None from third parties: everything is built in the task file.

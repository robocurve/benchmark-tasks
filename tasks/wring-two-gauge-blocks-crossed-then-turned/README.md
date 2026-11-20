# Wring two gauge blocks crossed, then turned into line

**Instruction:** "Wring the 25 mm gauge block onto the top face of the 50 mm block so the two make one stack with their
faces in line, then let go of both."

Two Mitutoyo rectangular steel gauge blocks (section 35 x 9 mm; 50 mm and 25 mm nominal) stand on a measuring face on
a black granite surface plate. Gauge blocks are joined into a stack by wringing: the maker's quick guide for thick
blocks says to cross the blocks at 90 degrees in the middle of the measuring faces, rotate them while applying slight
force, then align the measuring faces
(https://assets.testequity.com/te1/Documents/pdf/mitutoyo/Mitutoyo_Gauge-Blocks-Quick-Guide_Manual_0625.pdf).
Set one block squarely on the other and it only rests there: it looks like a stack, but the top block lifts off
alone. The robot steadies the tall, narrow 50 mm block with one hand (as a person's fingers do, against the film's
drag) and, with the other, sets the 25 mm block's face crossed on the middle of its top face, keeps the two 9 mm wide
faces flat in contact under a light press through a 90 degree turn into line, then lets go of both.

## Run and score a trial

In the simulator's environment (the Robocurve Brev launchable, or yam-isaac-lab-arena's `setup.sh`), with a Dream API
key configured, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=wring_two_gauge_blocks_crossed_then_turned.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Wring the 25 mm gauge block onto the top face of the 50 mm block so the two make one stack with their faces in line, then let go of both." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir logs/wring-dream

# The scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=wring_two_gauge_blocks_crossed_then_turned.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Wring the 25 mm gauge block onto the top face of the 50 mm block so the two make one stack with their faces in line, then let go of both." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir logs/wring-demo
```

- One fixed layout (`object_jitter=0`); the seed changes nothing. Runtime per trial: up to 1,800 steps (90 s of sim
  time at 20 Hz); the scripted demo is about 950 steps (47.5 s).
- Demo modes (`KIT_DEMO`): `demo` (default), `slam` (the natural move: the 25 mm block set straight down on the face
  already in line, let go), `hover` (turned into line 5 mm above the face, then set down and let go), `held` (wrung
  and in line, still held), `wiggle` (set down in line, twisted 35 deg each way three times under the press: no net
  turn), `light` (the full turn into line but at a light touch of about 1 N on the scale, under the 3 N floor),
  `none` (arms stay home).
- Tests: `PYTHONPATH=<yam-isaac-lab-arena> PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q` in this folder.

## Runs

- MuJoCo physics check (CPU), this task file: demo 1.0; no arm-to-arm contact; the 50 mm block never moved more than
  5 mm or 10 deg.
- Cheat modes in the same check: `none` 0, `slam` 0.25, `hover` 0.25, `wiggle` 0.25, `light` 0.25 (the full turn with
  the faces touching but pressed with about 1 N), `leftpress` 0.25 (the left hand pushes the 50 mm block down, about
  5 N on the scale, while the right turns a merely resting 25 mm block: a hand on the 50 mm block, so not counted),
  `platepress` 0.25 (the left hand pushes on the plate beside the blocks instead), `held` 0.75.
- Isaac (GPU, yam-isaac-lab-arena), this task file: scripted demo 1.0. The 25 mm block was set on the face at 16.5 s;
  the left fingers opened off the 50 mm block and the hand rose clear at 18 s; the scale read 5.8-6.2 N through the
  whole counted turn (tilt 0.0-1.3 deg, no gripper within 5 mm of the 50 mm block, the plate or the scale); wrung at 26.0 s after a 67 deg turn; the left hand closed on
  the 50 mm block again at 29 s for the squaring up; released at 35 s in line within 0.8 mm, and still with both hands
  clear at the end (47.5 s). Dream-1 0.0: in 90 s it never gripped either block, and neither block moved more than 1 mm.

## Scoring (the scorer is `grade()` near the top of the task file)

| Score | End state (first that fits) |
|---|---|
| 0 | a block fell over (tilted more than 30 deg) or dropped off the plate, or the 50 mm block was lifted more than 5 mm off the plate, at any time, even if put back |
| 1.0 | wrung, in line (centres within 3 mm across both axes, long sides within 4 deg), released (no gripper within 30 mm of either block) and still (neither moves more than 1 mm over the last 1 s) |
| 0.75 | wrung and in line, still held or not yet still |
| 0.5 | wrung, not in line |
| 0.25 | the 25 mm block on the 50 mm block's top face (within 1 mm of it, centres within 20 mm, tilted under 3 deg), not wrung |
| 0 | anything else, including the untouched start |

Wrung (simulator): the 25 mm block was gripped (both pads of one gripper within 3 mm of it, closed to under
37 mm apart) and turned a net at least 60 deg in one unbroken gripped, flat contact (turning back subtracts, so a
wiggle earns nothing) with its lower
face touching the 50 mm block's top face (no gap over 0.05 mm at their closest point and tilt under 1.5 deg) and pressed
onto it with at least 3 N on the bench scale under the plate, with no gripper within 5 mm of the 50 mm block, the plate or the scale, about a point within 6 mm of the face's middle, and has not since been lifted more than 1 mm off the face or slid more than 20 mm off its centre. Wrung (real bench) needs both: the video shows the
counted turn made with the filmed scale at least 0.31 kg above its tare and no hand or arm on the 50 mm block, the plate or the scale, and after the
end photo, lifting the 25 mm block 20 mm by hand lifts the 50 mm block with it.

## Fidelity

Rigid bodies. The oil film's hold of a wrung pair is not simulated: the scorer credits wringing from the work that
makes it, and a real trial checks it by the lift test.

The press (revision): Mitutoyo's gauge block guide says to cross the blocks at 90 degrees in the middle of the
measuring faces and "rotate the gauge blocks while applying slight force to them"; no maker or standard we found
publishes a figure. We declare 3 N as the floor for "slight force": about five times the 25 mm block's own weight
(0.6 N), so resting the block on the face, or a light touch, never counts, while a gentle fingertip press does. The
press is measured, not inferred: the plate stands on a bench platform scale (in simulation a stiff platform, 20 kN/m,
0.15 mm at 3 N, with the plate's weight tared), and every counted degree of the turn needs the faces touching (no gap
over 0.05 mm at their closest point, tilt under 1.5 deg) and the scale at 3 N or more. A rigid block held in rigid pads
rocks up to about 1.5 deg about the axis through its pads under the press (a real block seats flat on its oil film), so
across the 9 mm face the far edge stands at most 0.24 mm off while the press passes through the touching edge (the
demo turns at 0.0-1.3 deg); the 3 N on the scale can only
come through that contact, because the turn counts only while no gripper is within 5 mm of the 50 mm block, the plate or the scale
(the hand that steadied the 50 mm block while the 25 mm block was set down has let go and risen clear): a hand pushing
on any of them would load the scale too, and the `leftpress` and `platepress` cheats check that this earns nothing.
The floor stays at 3 N because this rig cannot press much harder: pinched by its 35 mm ends, the 25 mm block pivots
in the pads above about 6 N (at 13 N it turned 90 deg), and the free-standing 50 mm block starts to turn with it. The demo's left hand steadies the 50 mm block while the 25 mm block is set down at a
light touch, then lets go and backs out; the right hand then servos its press on the scale to 6 N through
the turn, squares the block up at 2 N, then eases to 0.5 N before letting go. Block masses come from steel density at catalogue size (124 g
and 62 g). The 21 kg plate is a free body too heavy to move. Friction static = dynamic (blocks 0.3, plate 0.5). Both blocks are free bodies: nothing in code holds them. The film's drag as it takes is not simulated.

## Real trials

Safe to run: two small steel blocks on a granite plate, light forces, no sharp edges, heat or liquids. Wipe the
measuring faces that meet (the 50 mm block's upper face, the 25 mm block's lower face) with a lint-free cloth and
isopropyl alcohol, give them a trace of gauge-block oil and wipe it almost off before each trial; keep the other faces, the plate
and the gripper pads dry. Use grade 2 (workshop) blocks kept for practice, not calibration blocks; a block that is dropped or knocked over is
nicked and must be checked before use.

## Assets

None from third parties: every shape is a box built in the task file.

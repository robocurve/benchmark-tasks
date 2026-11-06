# Close a carton's flaps so it can be taped

An open corrugated shipping carton stands between the two arms with its four top flaps open. The robot has to close
it the way a packer does before taping: both end flaps folded down first, then both long flaps folded down over them.
The end flaps spring back open when let go, as new board does, so an end flap stays down only while something holds
it: a hand, or a long flap lying on top of it.

Instruction: *Close the carton's flaps so it is ready to tape.*

Files: `task.py` (the task file: objects, hinges, `success`, `distance`, `score`, `demo`), `robocurve_ext.py` (adds
hinge joints to loose `Box` objects; see "Hinges" below), `task.json`, `test_task.py` (offline checks).

## What is on the table

Positions are in metres. x runs forward from the arm bases, y runs left from the midline, and the table top is
z = 0.7475.

- **Carton body (fixed):** a regular slotted carton of kraft corrugated board, about 9 x 7 x 2 in: outer 180 mm (x)
  x 220 mm (y) x 50 mm tall, centred at x 0.29, y 0, built from five 4 mm boards (floor, front wall with its outer
  face at x 0.200, back wall at x 0.380, side walls with their outer faces at y ±0.110).
- **End flaps (minor flaps, loose, hinged):** two 172 x 90 x 3 mm boards, 20 g, light kraft, hinged along the side
  walls' top outer edges. Each has a hinge spring pulling it open (1.3e-4 N·m per degree toward 100°, damping 3e-5
  N·m·s/deg). Closed, they lie on the side walls with a 40 mm gap between their free edges.
- **Long flaps (major flaps, loose, hinged):** two 220 x 89.5 x 3 mm boards, 60 g, darker kraft, hinged along the
  front and back walls' top outer edges, with no spring (damping only): they stay where they are put. Closed, they lie
  on top of the end flaps and meet in the middle with a 1 mm gap.
- All four flaps start open at 100° from closed, standing and leaning 10° outward on their stops. As on a real
  slotted carton, every flap is half the carton's 180 mm width deep.

## Run and score a trial

Built against the YAM Isaac Lab-Arena embodiment at commit `bd62771`. Copy this folder to `~/close-carton-flaps` on
the GPU machine with `inspect-robots` installed. Always run with `-E object_jitter=0`: a seeded reset shifts loose objects
by up to 3 cm, and each flap has to start on its hinge.

```bash
export INSTR="Close the carton's flaps so it is ready to tape."

# a policy (here Dream-1), 800 steps = 40 s at 20 Hz
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$HOME/close-carton-flaps/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score --max-steps 800 --store-frames --log-dir ~/yam-eval/carton-flaps-dream

# the scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$HOME/close-carton-flaps/task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "$INSTR" --scorer task_score --max-steps 800 --store-frames --log-dir ~/yam-eval/carton-flaps-demo

# one MP4 per camera, in each log's frames/ folder
for log in ~/yam-eval/carton-flaps-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. Offline checks: `YAM_ARENA=~/yam-isaac-lab-arena python -m pytest -q
test_task.py` (NumPy and pytest only; a few minutes, most of it the URDF-mesh clearance sweeps).

## Scoring

- A flap's angle is its rotation from its closed pose; a flap is **closed** when that angle is under 10°.
- An end flap counts 0.25 when it is closed and its centre is lower than both long flaps' centres.
- A long flap counts 0.25 when it is closed and both end flaps count (each closed and lower than both long flaps).
- The score, taken at the trial's last step, is the sum: 0, 0.25, 0.5, 0.75 or 1.0. Both end flaps held down with
  the long flaps open scores 0.5; one end flap left open scores at most 0.25, because neither long flap then has both
  end flaps under it; long flaps closed first with the end flaps folded on top of them scores 0.
- The trial succeeds and ends early with 1.0 when all four flaps are closed in that order and all four are resting
  (under 1 cm/s and 0.5 rad/s) for 1 s. The carton body is fixed and nothing else is scored.

## Hinges

`task.py` lists every flap as a loose `Box`. `robocurve_ext.py`, shipped in this folder, wraps
`yam_arena.scene.task_object` when the task file is loaded and adds a PhysX revolute joint (with its limits, spring
and damping) between each flap and the world at the flap's crease, then spawns the flap at its start angle. It
changes nothing in `yam_arena` on disk.

## Design numbers

- **Spring-back:** an end flap's weight about its crease is at most 0.0088 N·m (flat), and its spring gives 0.013 N·m
  at closed and more than its weight at every angle up to 100°, so a released end flap swings back open. The two long
  flaps (120 g together) are what keeps the closed end flaps down: in the demo nothing else touches the end flaps for
  the last 6 s of the trial, and they end closed under the long flaps.
- **Demo:** each arm pushes its end flap past vertical from outside the carton and presses it flat with closed
  fingers. The right arm then slides to the middle while opening, so that its two pads (47 mm either side of the
  gripper's centre) rest on both end flaps at once, and holds them. The left arm leaves outward, goes round the back
  corner and pushes the far long flap past vertical; it falls shut over the end flaps. The right arm then lets go,
  goes round the front corner, pushes the near long flap just past vertical and moves out to the side as it falls
  shut. The left arm goes round the back corner because reaching over the standing far flap is out of the arm's
  reach.
- **Reach and clearances (offline tests):** both plans track the IK within 0.5 mm / 0.1° at 20 Hz inside the joint
  limits, with every TCP at most 14 cm above the table; the arms stay at least 3 mm apart; no arm passes through a
  standing flap before pushing it, nor below the wall tops over the walls.

## What makes this task distinctive

Two of the four parts fight back: an end flap returns to open the moment nothing holds it, and the only lasting way to
hold it is the job's own order of folds, with the long flaps on top. So the end flaps have to be kept down while a
long flap is folded over them, and whatever keeps them down has to get out from under without letting them lift; in
the demo one gripper, opened wide, holds both end flaps while the other arm folds the far long flap. Folding in the
wrong order scores nothing even when all four flaps end flat.

## Verification

Run on an RTX 5090 with `yam-isaac-lab-arena` at `bd62771`, using the commands above (`object_jitter=0`, seed 0, the default):

- Scripted demo: `task_score` 1.0, ended by success at step 491 (two runs, the same both times); 60-63 s wall clock
  including Isaac start-up.
- Dream-1: `task_score` 0.0 after the full 800-step run limit (it moved both grippers around the carton without
  closing a flap); 90 s wall clock.
- Offline tests: 23 passed.
- Top camera: in the first frame the carton and its four standing flaps are visible between the arms; in the demo's
  last frame the closed top is visible, with the right arm beside the carton.

## Third-party assets

None. The carton and its flaps are `Box` primitives defined in `task.py`; no meshes, textures or library assets are
used.

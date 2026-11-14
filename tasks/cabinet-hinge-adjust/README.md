# Even out a cabinet flap door's gaps with the hinge's side screw, then close it

The robot picks up a screwdriver, finds which of three look-alike slotted screws on a cup-hinge arm moves a drop-down
cabinet door sideways, turns it by the right amount, then closes the door so the gaps either side are even, and lets
go. Carpenters and general maintenance and repair workers (O*NET 47-2031.00 and 49-9071.00, over 2 million US
workers together) re-centre cabinet doors this way after installation and whenever a door starts to rub.

The ability is turning a slot-head adjuster to move a hung door by a measured amount, with two decoys beside it. The
door starts open, lying flat, 1.9 mm off centre (reveals 6.4 mm on the right and 2.6 mm on the left). The side cam is
an eccentric: turned back to neutral it centres the door. The depth cam beside it pushes the door proud of its
neighbours, and the clamp screw lets the hinge arm slip so the door sags for good. Nothing marks which is which, and
the instruction names none of them. The gaps can only be read with the door closed, so the job is turn, close, look.

Physics, not the instruction, defeats the natural plan. The screw heads are 7 mm across with a 1.8 mm slot, so the
blade has to be centred on the slot to within about half a millimetre and turned by contact alone, and a head only
moves while the blade is in the slot. A wrong screw gives the wrong result in the sim: the depth cam leaves the gaps as
they were and the face proud, and loosening the clamp screw even 25 deg lets the arm slip 2.5 mm and the whole run
scores 0, however well it is corrected afterwards. The flap weighs 1.05 kg and swings on a lift stay, so it has to be
pinched at its edge and lifted shut.

Instruction given to the policy (23 words; it states the goal only):

`Even out the gaps either side of the open cabinet flap door with the red screwdriver, then close the door and let go.`

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one L40S). Versions used for the
submitted runs:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: ff25af11a60e (`sha256sum task.py | cut -c1-12`; equals `eval.environment_revision` in the
  log)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn the cabinet run as one fixed PhysX articulation (a carcass link with three slotted heads on
revolute joints, a three-axis slide of prismatic joints that carries the hinge arm, and the flap door on a revolute
hinge), and the screwdriver and its rack as compound rigid bodies, built from box and cylinder colliders in `task.py`.
It also calls the task's per-step record after every control step. No fork of yam_arena and no third-party assets.
`linkkit.py` and `linkdemo.py` are the program's own helpers, written for these tasks, and carry their own licence text.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

29 tests feed synthetic records to the scorer: the finished state scores 1.0 and the start state 0; the side-cam rung
at 29.29 deg vs 29.31 deg (either side of neutral); reveals differing by 1.0 mm are even and 1.1 mm are not; each reveal
at 3.0 and 6.0 mm are in band, 2.9 and 6.1 are not; the door at 3.0 deg is closed and at 3.1 deg is not; proud by 0.6 mm
is flush and by 0.7 mm is not (0.5 at the end); sag of 1.2 mm at any time, even with the door open or after correct work,
gives 0, and 1.1 mm does not; a pad 29 mm from the door or driver restarts the 1 s release timer and 30 mm does not; a
hand at 6 mm/s or 4 mm/s; 0.95 s of release is not enough; a driver tip left in a slot blocks the release; a door still
moving at 4 mm/s is not steady (0.5); closed-then-reopened scores 0.5; an overturned cam or the depth cam alone never
scores above 0.25 or 0; the record is clean after a reset; and measure() on door poses built from the hinge's kinematics
reads reveals 6.4 and 2.6 mm at door angles 0, 30 and 90 deg, the proud and sag values, the cam's 1.9 mm at 39.3 deg
and the engaged flag.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Even out the gaps either side of the open cabinet flap door with the red screwdriver, then close the door and let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/cabinet-hinge-adjust-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`; otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). Repeat trials: add `--epochs K`; with `object_jitter=0` the seed changes nothing in the scene.

```bash
inspect-robots inspect adhoc_69b6539c.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' adhoc_69b6539c.json
```

## Scored runs

| Run | yam-isaac-lab-arena | Commit | task.py sha256 (12) | Steps | Wall time | Log | task_score | Reason |
|---|---|---|---|---|---|---|---|---|
| Dream-1 | 1.0.0 | 0059d02a1a91 | ff25af11a60e | 1200 | 4 min | adhoc_69b6539c.json | 0.0 | max_steps |
| Scripted demo, recorded (`linkdemo.py`) | 1.0.0 | 0059d02a1a91 | ff25af11a60e | 865 (success held 1 s) | about 9 min | result.json | 1.0 | reveals 4.44 / 4.56 mm, flush, released |
| Scripted demo, repeat (`--no-record`) | 1.0.0 | 0059d02a1a91 | ff25af11a60e | 865 (success held 1 s) | about 8 min | result.json | 1.0 | same state to 0.01 mm |

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                          # scripted demo
python linkdemo.py task.py cheat_out --variant depth_screw   # a cheat variant (or DEMO_VARIANT=depth_screw)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json` (score, steps, IK error and the scorer's record). The script reads only
what a camera could measure: link and object poses (the door, the three heads, the driver), each head's angle computed
from its link poses, and the robot's own gripper state. It never reads the hinge's joint values, the slide, the latch
or the record. It uses the left arm only.

1. Pick: round the cabinet's wall to a point above the rack, down on the driver's handle (pads on its top 20 mm, 4 mm
   squeeze), straight up 115 mm so the shaft clears the rack's hole.
2. The door's sideways offset is read from its seen pose (1.9 mm) and the cam turn needed is worked out from the head's
   seen angle (39.3 deg back to 0). The blade is carried over the side cam's head, aligned with the seen slot, brought
   to 4 mm above it and nulled laterally and in yaw from the seen driver and head poses (to under 0.3 mm), then lowered
   into the slot (2.5 mm engaged).
3. Turn: in steps of at most 14 deg, the wrist rolls the driver and the seen head angle is read after each, until it is
   within 1 deg of the goal. Out of the slot, up to 128 mm, over the rack, down, let go, up.
4. Close: the door's left edge is pinched (pads 40 mm apart on approach, measured pad error nulled, then closed 8 mm tighter
   than the slab's 18 mm), 100 mm along the edge from the hinge, and swung about the hinge line until the seen door
   angle is inside the self-closing range; the pads open and the arm returns to its ready pose while the door finishes
   closing on the stay's catch.

The demo scores 1.0 (`success`, episode ended at step 865 of 1,200, 43 s): reveals 4.44 mm and 4.56 mm, door face 0.0
mm proud, the side cam at -1.1 deg, the clamp and depth cams untouched, the door peaked at 0.77 mm of sag (the gate is
1.2 mm), both grippers 116 and 171 mm clear and still for 1.95 s. Worst IK error 20 mm and 5.5 deg (the pinch approach
at the edge of the arm's reach).

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file; every one scores exactly 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | Nothing moves; the reveals stay 6.4 / 2.6 mm and the side cam at 39.3 deg. | 0.0 |
| `depth_screw` | The demo's whole procedure on the wrong screw: the depth cam, turned 51 deg (the door goes 1.6 mm proud), then close. | Closes uneven and proud: reveals unchanged at 6.4 / 2.6 mm, proud 1.56 mm, side cam still at 39.3 deg. | 0.0 |
| `clamp_first` | Loosens the clamp screw 60 deg first, then does the demo's side-cam turn and close. | The arm slips and the door sags 2.5 mm for good (peak 3.4 mm at 20.6 s): the gate. The reveals end even (4.46 / 4.54 mm) and flush, which would score 1.0 without the gate. | 0.0 |
| `wrong_way` | Turns the side cam the wrong way, 23 deg further from neutral (to 62 deg), then closes. | Reveals 7.15 / 1.85 mm, uneven; the cam is not nearer neutral. | 0.0 |
| `overturn` | Turns the side cam to its far stop (-86 deg), then closes. | Reveals 1.51 / 7.49 mm, uneven the other way; the cam is far from neutral. | 0.0 |

## Scoring

Measured in the world frame every control step from reset, from the door link's pose and the three heads' link poses:
psi = door opening angle (about the hinge line), gR and gL = reveal to the right and left neighbour (mm, from the
hinge line's lateral position and the 297 mm door width), proud = door face forward of flush (mm), sag = hinge line
below nominal (mm), th = angle of each head from its link pose. The record keeps flags, peaks and held times for the
whole episode and is cleared only by the simulator reset, so loosening the clamp screw and tightening it again never
clears the gate.

- Gate, any time in the run, gives 0: sag of 1.2 mm or more. The clamp screw was loosened more than 25 deg and the arm
  slipped 2.5 mm; it stays slipped when the screw is retightened. A door that is only pressed or bumped moves its hinge
  line by under 0.8 mm (the demo's peak is 0.77 mm).
- Closed = psi at most 3 deg. Even = closed, |gR - gL| at most 1.0 mm and each reveal 3.0 to 6.0 mm. Flush = |proud| at
  most 0.6 mm.
- 0.25: at the end the side cam is at least 10 deg nearer neutral than it started (|th| at most 29.3 deg), any way it
  got there.
- 0.5: the door was closed with even reveals at some time, but not at the end.
- 0.75: at the end the door is closed, even and flush, and still (under 3 mm/s) for the last 1 s.
- 1.0: also the release gate: every pad point of both grippers at least 30 mm from the door and the driver, both
  grippers under 5 mm/s for the last 1 s, and the driver's tip not in any slot.

`score()` returns the rung, `success()` is true on the 1.0 rung (yam_arena ends the episode once it has held for
`SUCCESS_HOLD_S = 1.0` s), and `distance()` is 1 - score.

Real trial (observable with these tools): the door's reveals with the door closed and the hinge's stop on, a feeler
gauge set at mid height on both sides (3.0 and 6.0 mm go and no-go, and the two readings compared for a 1.0 mm
difference); a steel straightedge laid across the neighbouring door faces with a 0.6 mm feeler for flush; a dial
indicator or steel rule on the door's bottom edge against a fixed mark for sag (1.2 mm); an angle finder on the door
for 3 deg of opening; scribe marks on the three head faces and a protractor for the side cam's angle (29.3 deg for the
first rung); the video for the door being still for 1 s and for the pads being 30 mm away and still. The clamp screw's
slip is read from the scribed mark: a head turned past 25 deg counter-clockwise means the gate.

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s simulated), `--max-steps 1200`. The demo succeeds at step 865.
- Runtime on one L40S: a demo run takes about 7 min wall clock on the shared VM (about 2.5 control steps/s with other
  simulations running, plus a few minutes of Isaac Sim start-up). A Dream-1 trial is slower, since Dream round trips
  pause the simulator.
- Seeds: `--seed 0` for every submitted run. `-E object_jitter=0` fixes the layout. The scripted demo needs no seed.

## Materials and how each fit and force is built

Positions in metres, x forward from the arm bases, y to the robot's left, heights above the table top.

- Cabinet run (fixed, 30 kg): 18 mm oak-veneer chipboard, a plinth 300 x 470 x 96 mm at x 0.20 to 0.50; bottom board
  300 x 470 x 18 mm at z 0.100 to 0.118; dividers at y +-0.162 and outer walls at y +-0.244 (300 x 18 x 288 mm); back
  board at x 0.482 to 0.500; a 60 mm back stretcher; no top board (an open-topped base unit with the worktop not yet
  fitted, so an arm can reach in).
- Flap door: 297 x 18 x 270 mm MDF with grey laminate, 1.05 kg with its hinge-cup boss (35 x 35 x 12 mm steel), hinged
  along its bottom outer edge, axis along y at x 0.182, z 0.100. Closed it fills x 0.182 to 0.200 and y +-0.1485 (4.5
  mm reveal each side); open 90 deg it lies flat on the bottom board's level. Two neighbouring flaps of the same size
  are closed and fixed beside it (modelled as 82 mm strips; they continue beyond the strip).
- Lift stay and self-closing hinge: a stay with a 0.7 N m/rad spring takes about 80 percent of the flap's weight at 90
  deg (gravity 1.4 N m, spring 1.1 N m), and a soft-close catch of 3.0 N m/rad, damped, pulls it shut over the last 50
  deg. To verify on the bench, read the force at 0.1 m from the hinge line with a spring scale: the flap rests open at 90
  deg with about 3 N pushing it open and 5 N at 60 deg (within 20 percent), and below 50 deg it closes itself with about
  14 N at 45 deg and 6 N at 20 deg. Hinge friction 0.1 N m.
- Hinge arm and slide: a cup-hinge arm 64 x 39 x 10 mm, 50 g, whose adjusters are the three slotted heads. The side
  adjuster is an eccentric cam with 3.0 mm of throw (door moves 3.0 mm x sin of the cam angle, with stops at +-90 deg);
  the depth cam is an eccentric of 2.0 mm throw; the clamp screw clamps the arm to its plate and, when loosened more
  than 25 deg (a quarter turn), lets the arm drop 2.5 mm in its slotted holes onto a stop. Verify with a dial indicator
  on the door's edge: 0.52 mm per 10 deg near neutral on the side cam, 0.35 mm per 10 deg on the depth cam, and 2.5 mm
  of drop with the clamp loosened.
- Slotted heads: zinc, 7 x 7.8 x 5 mm, 5 g, slot 1.8 mm wide and 3 mm deep widening over a 2 mm chamfer to 3.8 mm at
  the top, on a 60 x 40 x 6 mm steel mounting tab at x 0.212 to 0.252, y 0.040 to 0.100. Heads at y 0.070: clamp screw
  at x 0.220, side cam at x 0.232, depth cam at x 0.244 (12 mm apart). Each head turns against 0.004 N m of friction
  (check with a 0.1 to 1 N cm torque screwdriver); the clamp screw only loosens (counter-clockwise from above).
- Screwdriver: stubby slotted, 75 mm overall, 140 g, red PP handle 22 mm in diameter and 45 mm long, steel shaft 5 mm in
  diameter and 22 mm long, blade 5.0 x 0.9 x 8 mm (a blade that fits the 1.8 mm slot with 0.45 mm of play each side).
  Friction 0.8 / 0.7.
- Holder rack (fixed, 1.0 kg): an 80 x 80 x 6 mm steel plate at z 0.0365 to 0.0425 with a 16 mm square hole, on four 8
  mm legs at +-0.036 m from (0.070, 0.210). The driver hangs by its handle's shoulder, blade down through the hole.
- Positions, colours and friction for every body are in `task.json` (setup) and at the top of `task.py`.

## Reset between trials

Nothing is consumed. Open the door 90 deg onto its stop. Turn the side cam to 39.3 deg counter-clockwise from neutral
(a scribed mark), the depth cam to neutral, and tighten the clamp screw to its stop. Check the reveals with feeler
gauges (6.4 mm right, 2.6 mm left, flush, no sag). Stand the screwdriver blade-down in the rack. The cabinet and the
rack are fixed on taped outlines.

## Fidelity

Rigid bodies. The cup hinge's four-bar linkage is one pivot on a three-axis slide; the cam angles drive the slide in
software with the eccentric law, the clamp slip is a latched 2.5 mm drop, and the lift stay and soft-close are joint
springs. The run is cut to a 470 mm strip with 82 mm neighbour flaps and no top board so the arm can reach in. The
heads are a simplified slotted shape with a chamfered mouth. Blade-in-slot torque, the door's weight on the pads and the
lift are real contact.

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code (SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text). They use only public
Isaac Lab and USD Physics techniques, copy no code from other tasks in this repository, and the scene uses no
third-party assets or brands.

## Revision

Reviewer point (TK-QZ2S3Z, feasibility, not met): the recorded demo listed a task.py hash different from the submitted
Dream-1 run, so the full-score demonstration of the submitted version was unverified.

- Hashes: every task-file hash in this README (the Dream-1 row and both demo rows) is now the single value ff25af11a60e, the
  sha256 of the one filed `task.py`; the yam-arena commit is 0059d02a1a91. `task.py` itself is unchanged in this revision,
  so the scoring, the scene and the 1.0 demonstration are the same file as the Dream-1 trial.
- Demonstration: the scripted demo and every cheat variant were re-run in the simulator on this exact `task.py`: the
  demo scores 1.0 (success held 1 s, inside the 1,200-step limit) and `nothing`, `depth_screw`, `clamp_first`,
  `wrong_way` and `overturn` each score 0. The Dream-1 log's `eval.environment_revision` equals the demo's ff25af11a60e.
- Left unchanged on purpose: the reviewer named no loophole, so no new cheat was needed; the five existing cheats stay.

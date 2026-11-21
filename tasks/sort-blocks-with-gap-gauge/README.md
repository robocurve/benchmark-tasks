# Sort machined blocks with a go/no-go gap gauge

Six bare aluminium blocks, 60 mm long and 20 mm tall, lie on the station table. Their widths differ by a few
millimetres: three are within the specified 40.0 +- 1.3 mm, two are undersize and one is oversize. A fixed steel gap
gauge stands between the arms: a base plate with two gaps across it, each between a pair of anvils. The GO gap
(green anvils, 41.3 mm) takes any block that is not oversize; the NO-GO gap (red anvils, 38.7 mm) takes only an
undersize block. Two open trays stand nearer the robot: green (accept) on the left, red (reject) on the right. The
robot must check every block on the gauge and sort it.

Checking parts against fixed-limit gauges and rejecting those out of specification is core work for inspectors,
testers, sorters, samplers and weighers. A block is set on the GO gap, where it must drop in, and if it does, on the
NO-GO gap, where it must not. It is let go there because a fixed gap gauge reads by the part's own weight: a held part
can be forced in or held off. Unlike measuring a part with calipers and reading a value, nothing is read here: the
fixed gaps decide, and the robot must carry each block by its two ends, set it squarely onto a gap only 1.6 mm wider
than the widest block it takes, let go, take it again from wherever it came to rest, and act on whether it dropped in.
Unlike plain sorting into bins, the right tray is unknown until the block has been tried on the gauge, and only gauged
blocks earn credit. Which block starts where changes with a layout seed, so a policy cannot sort by start place.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: blocks, gauge, trays, the scorer (`score()`, `success()`), the contact and gripper readers, and the scripted `demo()` plan |
| `test_task.py` | Offline tests (plain Python, no simulator): geometry, scoring rules, failure cases, and a kinematic replay of the demo plan |
| `task.json` | The submission text |
| `variants.py` | The cheap strategies (same scene and scorer, other scripted plans), for the table under "Cheap strategies" |
| `build/` | STL files, a dimensioned drawing and build notes (`BUILD.md`) for the real gauge and blocks |
| `LICENSE` | MIT |

`task.py` hooks `yam_arena.scene` and the embodiment's step without modifying yam_arena (through import
hooks), so it runs with the standard `yam_arena` embodiment, the `yam_scripted` policy and the `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment, from this folder:

```bash
GAUGE_SORT_LAYOUT=0 inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Check each of the six aluminium blocks on the gauge, setting it on the green GO gap and, only if it drops in, then on the red NO-GO gap. Put each block that drops into GO but not into NO-GO in the green tray and every other block in the red tray." \
  --scorer task_score --max-steps 9600 --store-frames --log-dir ~/yam-eval/gauge-sort
```

The scripted demo is the same command with `--policy yam_scripted -P hold_s=2`. `task_score` reports 1.0 when every
block is credited (held 1 s), otherwise the score at the last step. `GAUGE_SORT_LOG=checks.jsonl` writes one line per
block and step whenever a block's pose shows a reading on the gauge, with its pose, both grippers' TCP and opening,
the per-part gauge forces, the robot force and the decision.

Offline tests (no simulator; Python 3.10 or newer with NumPy):

```bash
PYTHONPATH=<path to yam-isaac-lab-arena> python test_task.py
```

## Layouts

`GAUGE_SORT_LAYOUT=<int>` (default 0) shuffles the six blocks over the six start places (x 18, 28, 38 cm at y +23 and
y -23 cm). Seeds 0 to 3:

| Seed | Left row, near to far | Right row, near to far |
|---|---|---|
| 0 | F, C, A | B, E, D |
| 1 | E, F, B | A, D, C |
| 2 | B, A, D | F, C, E |
| 3 | E, A, B | C, D, F |

Always run with `-E object_jitter=0` and `-E control_hz=20`: the start places are fixed by the seed, and at any other
control rate both readers give nothing (the 6-step hold assumes 20 Hz), so no check counts and no block counts as
in a tray: the whole trial scores 0, after one warning.

## Scene

Positions are in cm from the midpoint between the two arm bases on the table top: x forward, y to the robot's left.

- Blocks (loose): 6.0 x W x 2.0 cm, aluminium (2700 kg/m3, 119 to 136 g). W: A 3.93, B 3.95, C 3.97 (in
  tolerance); D 3.71, E 3.68 (undersize); F 4.19 (oversize). Letters on the undersides.
- Gauge (fixed): base plate 5.0 x 17.0 x 1.0 cm at x 32, y 0. Anvils 1.0 x 1.2 x 1.5 cm on the base at x 32: GO gap
  4.13 cm between inner faces, centred at y +4.5; NO-GO gap 3.87 cm, centred at y -4.5. Each anvil's inner top edge has a 2 mm x 45 degree lead-in chamfer; the gauge is steel (friction 0.4). The anvils are short (x) so
  that the fingers, which hold a block by its two ends, pass beside them; the base is 1 cm shorter than a block, so
  the fingertips clear it too. A seated block stands 5 mm above the anvil tops.
- A block that must drop into a gap clears it by at least 0.8 mm a side (C in GO, D in NO-GO); a block that must
  stay out overlaps each anvil's inner face by at least 0.3 mm a side (A on NO-GO, F on GO). `check_geometry()`
  asserts both at import. The nearest in- and out-of-tolerance blocks differ by 2.2 mm in width; the in-tolerance
  blocks differ from each other by 0.2 mm. These are the tightest margins the scripted demo handled reliably in
  single-gap probes: dropping in with 0.75 mm a side worked 12 of 12 times, with 0.6 mm 2 of 6 and with 0.3 mm 2 of 6;
  staying out with 0.3 mm of overlap 6 of 6.
- Trays (fixed): inside 12 x 16 cm, floor 4 mm, walls 3 cm tall and 6 mm thick; green at x 17, y +9; red at x 17,
  y -9.

## Scoring

Evaluated every control step; the score at the last step counts. A block earns credit only when it was validly
checked, in order, and ends in the tray its checks call for.

A **check** at a gap counts when all of the following hold for 6 control steps in a row (0.3 s at 20 Hz):

1. **Released**: every gripper whose TCP (the point between the finger pads, 13 mm short of their tips) is within
   5 cm of the block is open wider than 6.5 cm (the block's 6.0 cm length plus 5 mm). Read from the finger joints
   and the gripper link pose in the simulator.
2. **Supported by the gauge, measured as contact force**: the upward contact force from the gauge (base plate and
   that gap's two anvils) on the block, normal plus friction, is at least 0.8 of the block's weight, and the total
   contact force from all robot links on the block is at most 0.02 N. Both come from a PhysX rigid contact view
   with the six blocks as sensors (friction matters for a block tipped against an anvil edge, which friction partly
   carries). A block held a fraction of a millimetre above the gauge, resting on a finger, or pressed by a link does
   not pass, however close it is.
3. **Pose shows the reading** (pose decides only which reading, never support). In every reading the block's long
   side runs along the gap's slot (front to back, within 10 degrees), so its width spans the gap, and its centre is
   within 12 mm of the gap's centre along the slot. Seated (level within 3 degrees, bottom within 1.5 mm of the base
   top, wholly between the anvils) means "drops in". Not seated but resting on the anvils, level or tipped against
   one up to 30 degrees (a tipped block may reach down to the base), with its centre over the gap and its lowest
   point at most 1.5 mm above the anvil tops, means "stays out". The pose test does not see what the block touches; that the gauge carries it is what
   condition 2 checks.
4. **The reading is one the block's width allows**: a narrow block balanced across a gap is no reading.

Order: a NO-GO reading counts only after the block has a GO reading of "drops in". The first reading at each gap
stands. Drops into GO and stays out of NO-GO: green tray. Stays out of GO, or drops into NO-GO: red tray. A block
that stays out of GO needs no NO-GO check.

**In a tray**: the block's whole footprint is inside the walls, it is still, it is released (the same gripper rule
as a check), and its bottom rests on the tray floor or on a block that is itself in that tray (worked out from the
floor up). Held above the tray, gripped while touching the floor, lying on the rim, or stacked on a block that is
held up does not count. If the grippers cannot be read, no block is in a tray.

**Score** = credited blocks / 6. If any out-of-tolerance block is anywhere over the green tray's outer footprint (in
it, on its rim or walls, on blocks in it, or held above it) at the end (an escape), the score is capped at 0.25.
Correct trays without checks score 0.

If the finger joints or contact forces cannot be read, or a reading is malformed (missing arm or block, wrong
shape, NaN), no check counts that step (one warning per reader, no exception); without a gripper reading no block
counts as in a tray either, so a trial in which the grippers can never be read (for example at a control rate other
than 20 Hz) scores 0. Checks are cleared at every reset of
the embodiment, when the episode clock goes back, and when all six blocks are at their start places at rest.

### Real-trial scoring

Needed: the uninterrupted video of the trial, the layout table (which block starts where) and, at the end, a photo of
both trays from above. No measuring instrument is needed.

1. Checks, from the video. Follow each block from its start place. A check counts when the block is seen left alone on
   a gap, the gripper visibly open and clear of it, for at least 0.3 s (6 frames of the 20 frames-per-second video), and it is either seated between the
   anvils (dropped in) or resting on the anvils, level or tipped against one (stayed out). Write down, per block, each
   gap and what it showed. The first reading at each gap stands. A NO-GO reading counts only if the block already
   dropped into GO. A block balanced across a gap it is too narrow for, or held up by anything other than the gauge,
   gives no reading.
2. What the checks call for: dropped into GO and stayed out of NO-GO: green tray; stayed out of GO, or dropped into
   NO-GO: red tray. A block with no GO reading, or that dropped into GO with no NO-GO reading, calls for nothing.
3. Trays, from the end photo. A block is in a tray when its whole footprint is inside the walls (not on the rim), it
   is no longer held or moving, and it rests on the tray floor or on a block that is itself in that tray. Turn each
   block over and read its letter.
4. A block is credited when its checks call for a tray and it is in that tray. Score = credited blocks / 6.
5. Escape cap: if D, E or F (out of tolerance) is anywhere over the green tray at the end (in it, on its rim or held
   above it), the score is at most 0.25.

## Scripted demo

`demo()` is a per-arm waypoint plan for `--policy yam_scripted`. It reads the layout seed to know which block is
where and how wide it is, and, while it runs, the blocks' poses from the simulator (a scripted demo may use simulator
state); it acts only through the arms. A few waypoints are worked out at the moment the plan reaches them: above a gap
the held block's seen offset from the gap's centre corrects the target across the slot (the arm backs out 3 mm
and comes in again, so that it really moves), and
every regrasp goes to where the block is seen to lie (it may have shifted or tipped on the gauge). Each arm
handles the three blocks in front of it, the two arms taking turns (the idle arm waits at its park pose). For each
block: open, descend, grasp it by its two ends (fingers closing along x, commanded 4 mm inside the ends so that the
finger drives centre the block instead of saturating), lift, carry to 15 mm beside the GO gap on the arm's own side
and move in over it, the wrist turned 1 degree against the measured yaw of a held block (every gauge visit ends with this same short sideways move, so the arm's friction lag is the
same each time; the target is offset against that lag by a fixed amount per arm, `RELEASE_DY`), lower until the
block is 4 mm above the anvil tops, wait 1 s, align it, then either let go 2 mm above the anvils (a block that will
drop in falls through the chamfered entry on its own; held, it would pivot between the fingers and jam) or set it
down on the anvils (a block that will stay out), wait 0.4 s with the fingers open, back off 5 cm, wait 0.8 s for the
check, take it again where it lies; if it dropped into GO, the same on NO-GO; then carry it to
the next free place in its tray, let go 2 mm above the floor, rise and park. With `GAUGE_SORT_PROBE=wedge`,
`demo()` instead only lets block F go 3.5 mm off the GO centre so that it tips against an anvil, for force
calibration; it is not the full-score demo. All heights, gaps and dwell times are named
constants in `task.py`.

The plan is 229.8 s long for every layout (4,596 control steps at 20 Hz; 4,636 with the policy's 2 s hold); the
proposed run limit of 9,600 steps is 2.07 times that (4,964 steps, 4.1 minutes, of margin).
`test_task.py` checks it without a simulator: every waypoint within reach bounds, the SDK's own IK reaching every
interpolated target within 1 mm and 1 degree, never both arms in the gauge zone, the TCPs always at least 12 cm
apart, each block's sequence of releases (GO, NO-GO if needed, tray) right for its width, and an idealised
kinematic replay scoring 1.0 for four layouts. This is not a physics result.

## Cheap strategies

`variants.py` loads this task (same scene, scorer and hooks) and replaces only the scripted plan:
`GAUGE_SORT_VARIANT=<name> inspect-robots run ... -E task=variants.py ...` with the same options as a trial.

| Strategy | What the arms do | Score by the rules |
|---|---|---|
| `nothing` | go to their park poses and stay for the whole run | 0 |
| `no_checks` | every block straight to its correct tray, no gauge checks | 0 (no block was checked) |
| `all_green` | every block into the green tray, no gauge checks | 0 (no checks; the escapes would cap it at 0.25 anyway) |
| `held_checks` | every block set on GO (and NO-GO where needed) but never let go there, then to its correct tray | 0 (a held block gives no reading) |

`test_task.py` replays the four strategies kinematically with the scorer and checks that each scores 0.

## Fidelity

- Blocks, gauge and trays are rigid boxes. Real blocks have small edge breaks; here the block edges are sharp, and each
  anvil has a 2 mm x 45 degree lead-in chamfer (a flat bevel, where real anvils may have a radius).
- The tolerance is scaled up from real gauging practice. Real fixed-limit gauges work to hundredths of a millimetre;
  here a block clears a gap by at least 0.8 mm a side or overlaps its anvils by at least 0.3 mm a side, so that rigid
  contacts in the simulator resolve every case reliably.
- Friction is the station's default for task objects (1.2 static, 1.0 dynamic), higher than bare aluminium on steel.
- Contact forces are PhysX's per-substep values at the last physics substep of each control step. The two force
  thresholds are 0.8 of the weight and 0.02 N. In the demo runs a released block resting on the gauge read 0.94 to
  1.02 of its weight (friction carrying up to 43 % when tipped) and the robot force on it 0 N; held blocks read
  about 38 N of robot force.

## Real-world build

Six aluminium (6061) bars 60 x 20 mm, milled to the six widths (+- 0.05 mm) and lettered underneath; a steel base
plate 50 x 170 x 10 mm with four hardened anvils 10 x 12 x 15 mm screwed on to give 41.3 and 38.7 mm gaps, each with a 2 mm x 45 degree lead-in chamfer on its inner top edge (set with
gauge blocks), GO anvils painted green, NO-GO red; two open plastic trays of about 12 x 16 cm inside, one green, one
red. Place them as in the scene, taped to the table.

Dimensioned drawing, STL files for the gauge and the blocks, the checks to make before a session and the set-up and
reset per layout: `build/BUILD.md`.

## Third-party assets and licenses

The SDK is yam-isaac-lab-arena, the program's official dependency, installed in the user's environment. Nothing from
it is redistributed here. Licenses are given as each source states them.

| Asset | Used for | Source | License as stated by the source | Bundled |
|---|---|---|---|---|
| YAM arm and gripper (URDF, meshes, collision hulls, converted USD) | both arms | SDK `yam_arena/assets/yam/`, from i2rt | MIT, Copyright (c) I2RT Robotics | no |
| Station floor and table | room | SDK `station.py` / `scene.py`: boxes with flat colours generated in code | SDK README: "This repository has no license yet" | no |
| Top-camera stand | room | SDK `station.camera_stand`: boxes and capsules generated in code | as the row above | no |
| Dome light and lamp | lighting | SDK `scene.py` | as the row above | no |
| top_cam, left_cam, right_cam | policy RGB | SDK pinhole camera sensors at the calibrated poses | as the row above | no |
| Station fruit, bowl, plate (Arena object library) and water bottle | not used | - | - | no |
| Blocks, gauge, trays | task objects; `build/` files | this folder (`task.py`, `build/`) | the authors' own geometry generated in code, MIT (this folder's LICENSE) | yes |

Isaac Lab-Arena ships the Apache License 2.0. Isaac Sim and Isaac Lab are runtime dependencies under their own
licenses; none of them is redistributed here. The build script (`build/export_gauge.py`) uses NumPy (BSD-3-Clause)
and Matplotlib (Matplotlib license, PSF-based); neither is bundled. All task-specific code and geometry in this folder is the authors' own
work under the MIT License.

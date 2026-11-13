# Sort machined blocks with a go/no-go gap gauge

Instruction given to the policy:
`Check each of the seven aluminium blocks with the gauge. A block that drops into the green GO gap but not into the red NO-GO gap is in tolerance: put it in the green tray. Put every other block in the red tray.`

Files: `task.json`, `gauge_sort.py` (scene, success, partial credit and the scripted demo), `check_demo.py`
(offline checks), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena, Isaac Sim, yam-arena), with the launchable's `yam.env` sourced
in every shell (`source <ARENA_DIR>/yam.env`). Versions used for the submitted runs:

- inspect-robots: 1.0.0
- yam-arena commit: 0059d02a1a91
- task file SHA-256, first 12 hex: d1c3e071721e (equals `eval.environment_revision` in the logs)

## Offline check, no GPU

From the root of this repository:

```bash
python tasks/go-no-go-gauge-sort/check_demo.py     # about 75 s
```

- the starting layout: every block at x >= 0.16 (nearer the arm bases, a pick tipped blocks up on end), in its
  own arm's row, with room between neighbours for the open fingers, and at least 20 mm clear of the gauge and
  the trays;
- scoring on hand-built states: start 0; all sorted 1.0 and success; a second layer still counts; a block still
  moving, held 10 cm above a tray or lying on a tray wall is not counted; every block in one tray 0 (either
  tray); one false reject 4/7; one escape 0.25 (capped); two escapes 1/7; only the in-tolerance blocks sorted
  3/7, only the out-of-tolerance blocks 4/7; random sorting, all 2^7 ways: expected score 0.052;
- the gauge reading on blocks seated in a gap, lying on the anvils, jammed tilted, held, or over the other gap;
- the demo, replayed through `yam_arena.demo.ArmPlan` at 20 Hz with the blocks' resting places tracked, for the
  physical outcomes (a block drops into a gap exactly when it is narrower) and for every block staying out of GO,
  every block dropping through both gaps and every block passing (every branch, for both arms): 3,977 steps
  (199 s), a 0.5 s hold with the fingers shut before every let-go over a gap, worst IK error 0.083 mm / 0.015 deg, every joint inside the URDF limits, the arms' link-3/4/5,
  gripper and TCP frames never closer than 165 mm, every block ending in the tray its gauge readings call for,
  and the physical outcomes scoring 1.0.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/go-no-go-gauge-sort/gauge_sort.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Check each of the seven aluminium blocks with the gauge. A block that drops into the green GO gap but not into the red NO-GO gap is in tolerance: put it in the green tray. Put every other block in the red tray." \
  --scorer task_score --max-steps 6000 --seed 0 --no-prompt --store-frames --log-dir logs/gauge-dream
```

`-E object_jitter=0` is required: the blocks are 40 mm apart along x, and the default 3 cm jitter can overlap them.

Scoring is automatic. `task_score` is 1.0 when `success()` (all seven blocks in the right trays, at rest) holds for
1 s; otherwise `score()` at the end of the episode: (correct - 2 x false rejects - 2 x escapes) / 7, at least 0,
and at most 0.25 if any out-of-tolerance block is in the green tray. Putting every block in the red tray, or
every block in the green tray, scores 0; sorting at random scores about 0.052 on average. The CLI prints `task_score: <value>`; the log
keeps it at `results.metrics.task_score`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/go-no-go-gauge-sort/gauge_sort.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Check each of the seven aluminium blocks with the gauge. A block that drops into the green GO gap but not into the red NO-GO gap is in tolerance: put it in the green tray. Put every other block in the red tray." \
  --scorer task_score --max-steps 6000 --seed 0 --no-prompt --store-frames --log-dir logs/gauge-demo
```

The demo is `demo()`, a scripted (not learned) policy run through `yam_scripted`. It gauges every block and sorts
it by what the gauge did; it never reads the widths, but it does read the simulator's block poses (see Looking).
For each block, one arm picks it up and sets it square over the GO gap, holds still for 0.5 s, lets go, backs
off and looks. If the block is seated in the gap, the arm grips it by its ends, lifts it out and tests it over the
NO-GO gap the same way. If the block stayed out of GO, it goes to the red tray. After a NO-GO test, a block seated in NO-GO goes to
the red tray and one that stayed out goes to the green tray.

- **Looking:** the `yam_scripted` policy sees only images and joint angles, so the demo reads the simulator's
  pose of the block (through the task file's `score()` hook, which runs after every control step). A block counts
  as seated when it lies flat on the gauge base between the anvils (lowest point within 3 mm of the base,
  highest under 8 mm above the anvil tops). Each waypoint after a look works out its target when the arm reaches
  it.
- **Grip:** straight down, turned 90 degrees, so the fingers close along x on the block's 50 mm ends. The anvils are
  only 10 mm long along x, so the fingers stand clear of them and reach down beside a block that has dropped
  into a gap. A grip across the width would not fit, with 0.75 to 4 mm between the block and each anvil.
  Each block is gripped with the pad tips 1 mm above its bottom. The fingers approach and descend open 88 mm
  (19 mm clear of each end) and release at 68 mm, which fits inside the 80 mm trays.
- **Sharing the gauge:** the arms take turns, one block at a time: left block 1, right block 5, left 2, right 6,
  left 3, right 7, left 4; the right arm parks after block 7. An arm
  holds its next block above its starting place until the other arm has finished its delivery. Each arm uses
  its own half of each tray; a third block in the same half goes on top of the first.
- **Settling:** a carried block sways a few millimetres across the gap while the arm lowers it, more than the
  1.25 mm a side that the widest in-tolerance block (30.5 mm) has in GO. The arm holds still for 0.5 s, fingers shut,
  before each let-go over a gap, so the sway dies out before the block is released.
- About 3,980 of the 6,000 steps; the episode ends 1 s after the last block comes to rest in its tray.

## Run limit, runtime and seeds

- Run limit: 6,000 control steps at 20 Hz (5 minutes). The demo gauges all seven blocks one at a time in
  about 3,980 steps.
- Seeds: `--seed 0`, `-E object_jitter=0`. The demo is deterministic; Dream-1 is not seeded.

## Submitted runs

- Scripted demo, seed 0, `object_jitter=0`: `task_score` 1.0 (succeeded), 3,943 of the 6,000 steps.
- Dream-1, same seed and settings, all 6,000 steps: `task_score` 0.0.
- `task.json` `scores` records both, and `media` holds the uploaded videos and setup screenshot.

## Reset between trials

Nothing is consumed. Return each block, by the number on its underside, to its starting place, lying along x.

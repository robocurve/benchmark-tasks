# Put up a wall drop-leaf table

A solid-pine wall drop-leaf table (the 50 x 30 cm size) is mounted, leaf 380 mm above the table top, on a wall
section standing across the far end of the station table. Folded, its 0.98 kg leaf hangs straight down in front of its two triangular support brackets, which
lie flat against the wall. The leaf only rises to level (its back edge meets the fixed strip) and falls back if let
go; a bracket can only swing out while the leaf is held up level, because the hanging leaf is in its way. The robot
must hold the leaf level with one hand while the other swings a bracket out to its 90 degree stop under it, then let
the leaf down onto the brackets with both out, and take its hands away.

## Run and score a trial

```
inspect-robots run --embodiment yam_arena -E task=drop_leaf_table.py -E object_jitter=0 -E control_hz=20 \
  --instruction "The wall-mounted drop-leaf table is folded down. Put it up so the leaf rests level on both of its support brackets, swung fully out, and stays there when you let go." \
  --scorer task_score --max-steps 900 --policy <policy> --seed <seed>
```

`dev/run.sh <name> <policy> <seed> 900 [LEAF_FAIL=...]` does the same and writes a per-step log (leaf angle from
level, each bracket's swing, the fingers' clearance from the leaf and brackets, the score so far). `--scorer
task_score` reports the best score of the trial (`SCORE_OVER = "best"`).

- Run limit: 900 steps (45 s at 20 Hz). The scripted demo takes 630 steps (the leaf is level at 9.6 s).
- Runtime per trial: about 8 to 15 minutes wall clock on the shared GPU machine (including about 2 minutes of
  start-up), depending on the other jobs on it.
- Seeds: any. Each seed moves the wall section and table together up to 30 mm along x and 25 mm along y, turns them
  up to 5 degrees about the vertical, and starts each bracket 0 to 2 degrees open (`layout()`).

## Scripted demo

`--policy yam_scripted` plays the demo. It **reads object poses from the simulator** to plan (the wall section's
pose, which carries the table); it moves the parts only through the grippers, by contact. The left hand pinches the
leaf's left end 130 mm from the hinge and raises it through 90 degrees to level, pressing it into its stop. The right
hand pinches the right bracket's diagonal brace and swings the bracket out to its stop under the held leaf, lets go
and backs off. The left hand lets go of the leaf, which now rests on the right bracket, goes under it, and swings the
left bracket out the same way. Both hands then go back to their ready pose.

Wrong outcomes on purpose (`LEAF_FAIL=`): `fixed` (the plan for the reference layout of the setup text, not the
poses), `early` (the leaf let go as soon as it is level, before a bracket is out), `one` (the left bracket never swung
out), `half` (the right bracket swung only 50 degrees), `holding` (the left hand keeps holding the leaf to the end). `one` and `half` were not run.

## Results (final code)

All on the same code (sha256 of the three Python files in each run's `sha256.txt`). The scripted runs were started
with `--max-steps 1200` and all ended by step 756, inside the 900-step limit; the Dream-1 run used 900. Logs without
frames are in `runs/`. "Leaf" is the leaf's angle from level, "right"/"left" each bracket's swing (90 at its stop),
"clear" the fingers' smallest distance from the leaf and brackets, all at the end of the run.

| Run | Seed | Score | Steps | Leaf | Right | Left | Clear | What happened |
|---|---|---|---|---|---|---|---|---|
| a6-s0 | 0 | 1.0 | 630 | -1.24 deg | 89.8 deg | 90.0 deg | 80 mm | leaf held level, right bracket swung to its stop, leaf let down on it, left bracket swung out |
| a6-s1 | 1 | 1.0 | 630 | -1.24 deg | 90.0 deg | 90.0 deg | 112 mm | same |
| a6-s2 | 2 | 1.0 | 630 | -1.24 deg | 90.0 deg | 90.0 deg | 113 mm | same |
| a6-demo-s0 (video) | 0 | 1.0 | 630 | -1.24 deg | 89.8 deg | 90.0 deg | 80 mm | as a6-s0, frames stored |
| a6-fixed-s0 | 0 | 0.0 | 630 | -90 deg | 0.4 deg | 1.2 deg | 176 mm | the plan for the reference layout: the left hand misses the leaf's end; the leaf never leaves hanging, so neither bracket can be reached |
| a6-early-s0 | 0 | 0.0 | 426 | -90 deg | 0.3 deg | 1.2 deg | 185 mm | the leaf let go as soon as it is level: it falls back; the right hand meets the falling leaf and cannot get to the bracket behind it |
| a6-holding-s0 | 0 | 0.0 | 756 | +0.01 deg | 90.0 deg | 1.2 deg | 2 mm | the left hand keeps holding the leaf level while the right bracket is swung out: never hands-off (0.3 if the hands-off rule were dropped) |
| dream-try1 (Dream-1, video) | 0 | 0.0 | 900 | -90 deg | 0.2 deg | 1.2 deg | 339 mm | model run for the full run limit: the leaf was never lifted |

The leaf resting on the brackets sits 1.24 degrees below level (the brackets' tops are 3 mm under the hinge line).
Each seed moves the wall section and table (seeds 0, 1, 2: x +12, +21, +15 mm; y -23, +11, +15 mm; turn +3.2,
+0.6, +0.3 degrees).

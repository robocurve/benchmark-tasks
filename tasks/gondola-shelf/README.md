# Install two hook-in gondola brackets at a marked height and seat the steel shelf

A two-column slotted steel retail gondola upright stands at the back of the station table with four numbered bracket
levels (`1` to `4`), with level `3` marked in yellow across both standards. Two stamped-steel hook-in shelf brackets
sit upright in side cradles on the left and right, and a white flanged steel shelf pan sits flat on front risers. Each
bracket has an upper rear J-hook and a lower alignment tongue one slot pitch below it, so pushing a level bracket
straight at the upright drives its lower tongue into the solid steel bridge between slots. Each arm pitches its bracket
~28 degrees nose-up, passes the upper hook through slot `3`, drops the tooth behind the 4.5 mm web, and swings the
bracket down flat so the lower tongue seats into slot `2`. Both arms then grasp the shelf's upright side fences, lift
the 320 mm shelf together, and seat its captive front and rear lips across both level-3 brackets.

## Files

- `task.py`: scene geometry, `on_step` latched violation and stillness monitor, `score()`, and observable-pose scripted `demo()`.
- `task.json`: task metadata, setup specification, numeric scoring rubric, fidelity notes, and trial scores.
- `labkit.py` / `arena_assets_fix.py`: multi-box rigid body builder and `on_step` / `on_reset` hooks for `yam_arena`.

## Run a trial

Inside the Isaac Lab-Arena environment with `yam-isaac-lab-arena` installed:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "$(python -c 'import json;print(json.load(open("task.json"))["instruction"])')" \
  --scorer task_score --max-steps 1800 --store-frames --log-dir logs
```

For the scripted demonstration, pass `--policy yam_scripted` (completes in ~650 steps).

## Runtime and seeds

- Run limit: 1,800 control steps (90 s at 20 Hz). The scripted demonstration completes in ~650 steps (~32.5 s).
- Fixed starting layout: `-E object_jitter=0`, no random seeds required.

## Scoring a real trial

- Bracket seating (`0.25` each): verify with a ruler and protractor that each bracket's upper hook tooth sits behind the 4.5 mm web in the yellow level-3 slot and its lower tongue sits inside the 116 mm slot below, with the rear shoulder within 7 mm of the upright face and orientation within 7 degrees of upright and square, and all fingertips at least 8 cm clear of every bracket surface.
- Shelf seating (`1.0` total): verify with a tape measure and inclinometer that the white shelf rests centred across both level-3 brackets (within 12 mm of `x = 0.329 m`, 15 mm of `y = 0.000 m`, and 10 mm of `157 mm` above the table, tilt <= 5 degrees, yaw <= 6 degrees) with both bracket flanges captured inside the 138 mm front/rear lip pocket within 5 mm vertically of the deck underside, all fingertips >= 8 cm from every bracket and shelf surface, and all vertices moving < 1 mm (3D) over the final 1.0 s, with no bracket or shelf making contact with the table or floor at > 0.50 m/s pre-impact downward vertex speed in the trial video (harmless tilts, near-misses above the surface, and gentle temporary placement on the table at <= 0.50 m/s are permitted).

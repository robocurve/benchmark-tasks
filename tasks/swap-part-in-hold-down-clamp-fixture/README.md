# Change parts in a swing-clamp fixture: swing the clamp off, unload, load the blank, swing it back over

> Swing the clamp open, put the finished part in the output tray, load the blank into the nest and swing the clamp back over it.

Changing parts in a hold-down clamp fixture is a core, repeated operation for miscellaneous assemblers and fabricators (about 1.46 million US workers, BLS OEWS May 2024, 51-2090): open the clamp, unload the finished part, load the next blank into the nest, and clamp it. Loading over the finished part, or leaving the blank unclamped, scraps the next part.

**Physical difficulty (one fixed level):** the clamp is a working swing arm on a stiff vertical pivot (0.12 N m), held where it is left: it lies 3 mm over the part, so nothing can be lifted out or put in until its knob has been pushed round through 90 degrees, and it is closed by pushing it back round until it lies over the new blank. Parts come out of and go into a nest with 6 mm of play. The left arm works the clamp, the right arm the parts, in order: open, unload, load, clamp.

## Run a trial

From the repository root, in the simulator's environment (Isaac Lab-Arena with the `yam_arena` embodiment), with a
Dream Labs key in `~/.dreamlabs/credentials`:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks//task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Swing the clamp open, put the finished part in the output tray, load the blank into the nest and swing the clamp back over it." --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/clamp-dream
```

The scripted full-score demonstration: the same command with `--policy yam_scripted` and
`--log-dir ~/yam-eval/clamp-demo`. Render videos with `inspect-robots video <log>.json`.

Offline checks, without the simulator (the demo's inverse kinematics and the scoring rules on fake states):
`python tasks//check_task.py`. Diagnostics: set `CLAMP_TRACE=/path/trace.jsonl` before a run to log,
every control step, every part's pose and joint positions and both grippers' tool points.

## Runtime and seeds

The scripted demonstration takes about 33 s; a Dream-1 trial runs until the finished state has held for
1 s or the run limit of 2,400 steps (2 minutes at 20 Hz). No seeds: `-E object_jitter=0` keeps every object exactly
where `task.py` places it, matching the setup screenshot.

## The layout

Positions are measured on the table from the point midway between the two arm bases, which stand 61 cm apart:
forward means away from the arms, left and right as seen from behind the arms.

- **Fixture** (`fixture`): steel plate 12 x 24 cm x 1.5 cm, centred 22 cm forward on the centre line, fixed. On it, a part nest (inside 52 x 62 mm, 12 mm walls) centred 4 cm right, and beside it, 3.5 cm left, a pivot post carrying the swing clamp (`clamp` joint, hinge about the vertical): a blue arm 7.5 cm long lying 3 mm over the nested part with a pad at its end, and a red knob standing 3 cm up on the arm, 5 cm out from the pivot. Pushed round, the arm swings forward up to 92 degrees; pivot friction 0.12 N m holds it where it is left.
- **Finished part** (`finished_part`): bright machined block 40 (forward) x 50 x 30 mm, 120 g, in the nest under the closed clamp.
- **Blank** (`blank`): the same block, saw-cut dull grey, on the table at 13 cm forward, 20 cm right.
- **Output tray** (`output_tray`): green, inside 11 x 11 cm with a 1.5 cm lip, centred 27 cm forward, 25 cm right, fixed.

Nothing else is on the table.

## Scoring

A part is **in the nest** when its centre lies inside the nest's inside rectangle seen from above and less than 1.5 cm above where it rests on the plate; **in the tray** likewise for the tray. **Still**: moved under 5 mm over the last 0.3 s. The clamp is **open** once its arm has swung more than 60 degrees off the nest, and **closed on the blank** when its arm is back within 5 degrees of lying over the nest with the blank in it.

- **Milestones, 0.2 each** (0 to 0.8), at the end of the trial: the clamp opened past 60 degrees (at any time); the finished part in the tray and still; the blank in the nest and still, with the finished part out; the clamp closed over the blank.
- **1.0:** all four hold, nothing moves, and no gripper tool point is within 4 cm of the blank, the finished part or the clamp pad, for 1 s. The trial then ends. Completed less than 1 s before the run limit scores 0.8.
- **0, defects:** the blank and the finished part in the nest at the same time (loading over the finished part), or a part falling off the table.

## Scoring a real trial

Film from above and the side. 0.2 each: clamp opened; finished part in the tray; blank in the nest; clamp closed down on the blank. Full score when all four hold with the grippers clear for 1 s. Both parts in the nest at once, or a part on the floor, scores 0.

## The demonstration

Scripted, open loop. The left arm, fingers closed, lowers them behind the clamp's knob and pushes it round 110 degrees (finger tips trailing 1 cm behind it) until the arm stands forward against its stop, and moves clear. The right arm (pads closing across the 40 mm width) lifts the finished part out of the nest into the output tray, then the blank from the table into the nest, letting go 1 cm above the plate. The left arm then pushes the knob back round until the arm lies over the blank against its stop, and lifts clear.

## Files

- `task.py`: the scene, the scoring and the scripted demonstration.
- `check_task.py`: offline checks (no simulator).
- `task.json`: the task's description for the task board.
- `labkit.py`, `arena_assets_fix.py`, `taskbase.py`: helpers. `labkit.py` and `arena_assets_fix.py` are copied unchanged from the accepted task `tasks/glovebox-filter` (labkit itself follows the technique of `tasks/bearing-press/partkit.py`, PR #40); they build multi-shape parts with hinged and sliding joints for yam_arena without modifying it. `taskbase.py` holds this task's per-trial scoring memory and small geometry helpers.
